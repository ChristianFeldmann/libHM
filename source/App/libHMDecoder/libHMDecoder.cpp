
#include "libHMDecoder.h"

#include "TLibCommon/CommonDef.h"
#include "TLibCommon/TComTU.h"
#include "TLibDecoder/TDecTop.h"
#include "TLibDecoder/NALread.h"
#include "TLibCommon/TComRom.h"

using namespace TComRom;

// TODO: (isNaluWithinTargetDecLayerIdSet) The target layer file is not supported (yet)
bool isNaluWithinTargetDecLayerIdSet(InputNALUnit* nalu) { return true; }

#define HMDECODERWRAPPER_MAXBUFFERSIZE 1000

class hmDecoderWrapper
{
public:
  hmDecoderWrapper()
  {
    loopFiltered = false;
    maxTemporalLayer = -1; ///< maximum temporal layer to be decoded
    iPOCLastDisplay = -MAX_INT;
    iSkipFrame = 0;
    pcListPic = NULL;
    pcListPic_readIdx = 0;
    numPicsNotYetDisplayed = 0;
    dpbFullness = 0;
    lastNALTemporalID = 0;
    flushOutput = false;
    sheduleFlushing = false;

    // Initialize the decoder
    decTop.create();
    decTop.init();
    decTop.setDecodedPictureHashSEIEnabled(true);
    iPOCLastDisplay += iSkipFrame; // set the last displayed POC correctly for skip forward.

    internalsBlockDataValues = 0;
    pauseInternalsPUPartIdx = -1;
    pauseInternalsPUSubPartIdx = -1;
    pauseInternalsCUIdx = -1;
    for (int i = 0; i < 5; i++)
    {
      pauseInternalsCUPartIdxRecursive[i] = -1;
      pauseInternalsTUIdxRecursive[i] = -1;
    }
  }
  ~hmDecoderWrapper() { decTop.destroy(); };

  bool loopFiltered;
  int  maxTemporalLayer; ///< maximum temporal layer to be decoded
  int iPOCLastDisplay;
  int iSkipFrame;
  TComList<TComPic*>* pcListPic;
  int pcListPic_readIdx;
  int numPicsNotYetDisplayed;
  unsigned int numReorderPicsHighestTid;
  unsigned int maxDecPicBufferingHighestTid;
  int dpbFullness;
  int lastNALTemporalID;
  bool flushOutput;
  bool sheduleFlushing; // After the normal output function is finished, we will perform flushing.

  // The local memory for the global variable
  bool md5_mismatch;

  // Add the internals block value to the array. Check if the data array is full first.
  void addInternalsBlockData(libHMDec_BlockValue val);
  bool internalsBlockDataFull() { return internalsBlockDataValues >= HMDECODERWRAPPER_MAXBUFFERSIZE;  }
  void clearInternalsBlockData() { internalsBlockDataValues = 0; }

  TDecTop decTop;

  // The array that is filled when internals are returned.
  // The array is defined, filled and cleared only in this library so that no chaos is created
  // between the heap of the shared library and the caller programm.
  libHMDec_BlockValue internalsBlockData[HMDECODERWRAPPER_MAXBUFFERSIZE];
  unsigned int internalsBlockDataValues;

  // If the array is full, we will save which CU, PU or TU we were processing when the array did overflow
  // so that we can continue the next time libHMDEC_get_internal_info() is called.
  int pauseInternalsPUPartIdx;
  int pauseInternalsPUSubPartIdx;
  int pauseInternalsCUIdx;
  int pauseInternalsCUPartIdxRecursive[5];  // The part index (per depth)
  int pauseInternalsTUIdxRecursive[5];      // The index (per trDepth)
};

void hmDecoderWrapper::addInternalsBlockData(libHMDec_BlockValue val)
{
  // Append the value
  internalsBlockData[internalsBlockDataValues] = val;
  internalsBlockDataValues++;
}

extern "C" {

  HM_DEC_API const char *libHMDec_get_version(void)
  {
    return NV_VERSION;
  }

  HM_DEC_API libHMDec_context* libHMDec_new_decoder(void)
  {
    hmDecoderWrapper *decCtx = new hmDecoderWrapper();
    if (!decCtx)
      return NULL;

    return (libHMDec_context*)decCtx;
  }

  HM_DEC_API libHMDec_error libHMDec_free_decoder(libHMDec_context* decCtx)
  {
    hmDecoderWrapper *d = (hmDecoderWrapper*)decCtx;
    if (!d)
      return LIBHMDEC_ERROR;

    delete d;
    return LIBHMDEC_OK;
  }

  HM_DEC_API void libHMDec_set_SEI_Check(libHMDec_context* decCtx, bool check_hash)
  {
    hmDecoderWrapper *d = (hmDecoderWrapper*)decCtx;
    if (!d)
      return;

    d->decTop.setDecodedPictureHashSEIEnabled(check_hash);
  }
  HM_DEC_API void libHMDec_set_max_temporal_layer(libHMDec_context* decCtx, int max_layer)
  {
    hmDecoderWrapper *d = (hmDecoderWrapper*)decCtx;
    if (!d)
      return;

    d->maxTemporalLayer = max_layer;
  }

  HM_DEC_API libHMDec_error libHMDec_push_nal_unit(libHMDec_context *decCtx, const void* data8, int length, bool eof, bool &bNewPicture, bool &checkOutputPictures)
  {
    hmDecoderWrapper *d = (hmDecoderWrapper*)decCtx;
    if (!d)
      return LIBHMDEC_ERROR;

    if (length <= 0)
      return LIBHMDEC_ERROR_READ_ERROR;

    // Check the NAL unit header
    uint8_t *data = (uint8_t*)data8;
    if (length < 4 && !eof)
      return LIBHMDEC_ERROR_READ_ERROR;

    // Do not copy the start code (if present)
    int copyStart = 0;
    if (data[0] == 0 && data[1] == 1 && data[2] == 1)
      copyStart = 3;
    else if (data[0] == 0 && data[1] == 0 && data[2] == 0 && data[3] == 1)
      copyStart = 4;

    // Create a new NAL unit and put the payload into the nal units buffer
    InputNALUnit nalu;
    TComInputBitstream &bitstream = nalu.getBitstream();
    vector<uint8_t>& nalUnitBuf = bitstream.getFifo();
    for (int i = 0; i < copyStart; i++)
      data++;
    for (int i = 0; i < length - copyStart; i++)
    {
      nalUnitBuf.push_back(*data);
      data++;
    }

    // Read the NAL unit
    read(nalu);

    if( (d->maxTemporalLayer >= 0 && nalu.m_temporalId > d->maxTemporalLayer) || !isNaluWithinTargetDecLayerIdSet(&nalu)  )
    {
      bNewPicture = false;
    }
    else
    {
      bNewPicture = d->decTop.decode(nalu, d->iSkipFrame, d->iPOCLastDisplay);
      if (bNewPicture)
      {
        // We encountered a new picture in this NAL unit. This means: we will filter the now complete former
        // picture. There might also be pictures to be output/read. After reading these pictures, this function
        // must be called again with the same NAL unit.
        // 
        // The original TAppDecoder will rewind the bitstream for this. We don't have a bitstream like this.
        // This also means that eof is false for this call. Only in the next call it is set.
        eof = false;
      }
    }

    // Filter the picture if decoding is complete
    if (bNewPicture || eof || nalu.m_nalUnitType == NAL_UNIT_EOS)
    {
      int poc;
      if (!d->loopFiltered || !eof)
        d->decTop.executeLoopFilters(poc, d->pcListPic);
      d->loopFiltered = (nalu.m_nalUnitType == NAL_UNIT_EOS);
    }

    // Check if we might be able to read pictures
    checkOutputPictures = false;
    d->flushOutput = false;
    bool fixCheckOutput;
    if ( bNewPicture &&
      (   nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL
        || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP
        || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA_N_LP
        || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA_W_RADL
        || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA_W_LP ) )
    {
      checkOutputPictures = true;
      d->flushOutput = true;
    }
    if (nalu.m_nalUnitType == NAL_UNIT_EOS)
    {
      checkOutputPictures = true;
      fixCheckOutput = true;
    }

    // FIX_WRITING_OUTPUT
    fixCheckOutput = (!bNewPicture && nalu.m_nalUnitType >= NAL_UNIT_CODED_SLICE_TRAIL_N && nalu.m_nalUnitType <= NAL_UNIT_RESERVED_VCL31);

    // next, try to get frames from the deocder
    if((bNewPicture || fixCheckOutput) && d->pcListPic != NULL)
    {
      checkOutputPictures = true;

      d->lastNALTemporalID = nalu.m_temporalId;

      // This is what xWriteOutput does before iterating over the pictures
      const TComSPS* activeSPS = &(d->pcListPic->front()->getPicSym()->getSPS());
      unsigned int maxNrSublayers = activeSPS->getMaxTLayers();
      d->numPicsNotYetDisplayed = 0;
      d->dpbFullness = 0;

      if(d->maxTemporalLayer == -1 || d->maxTemporalLayer >= maxNrSublayers)
      {
        d->numReorderPicsHighestTid = activeSPS->getNumReorderPics(maxNrSublayers-1);
        d->maxDecPicBufferingHighestTid =  activeSPS->getMaxDecPicBuffering(maxNrSublayers-1);
      }
      else
      {
        d->numReorderPicsHighestTid = activeSPS->getNumReorderPics(d->maxTemporalLayer);
        d->maxDecPicBufferingHighestTid = activeSPS->getMaxDecPicBuffering(d->maxTemporalLayer);
      }

      TComList<TComPic*>::iterator iterPic = d->pcListPic->begin();
      while (iterPic != d->pcListPic->end())
      {
        TComPic* pcPic = *(iterPic);
        if(pcPic->getOutputMark() && pcPic->getPOC() > d->iPOCLastDisplay)
        {
          d->numPicsNotYetDisplayed++;
          d->dpbFullness++;
        }
        else if(pcPic->getSlice( 0 )->isReferenced())
        {
          d->dpbFullness++;
        }
        iterPic++;
      }
    }

    if (eof)
    {
      // At the end of the file we have to use the normal output function once and then the flushing
      checkOutputPictures = true;
      d->sheduleFlushing = true;
    }

    if (checkOutputPictures)
      // Reset the iterator over the output images
      d->pcListPic_readIdx = 0;

    return LIBHMDEC_OK;
  }

  HM_DEC_API libHMDec_picture *libHMDec_get_picture(libHMDec_context* decCtx)
  {
    hmDecoderWrapper *d = (hmDecoderWrapper*)decCtx;
    if (!d)
      return NULL;

    if (d->pcListPic == NULL)
      return NULL;
    if (d->pcListPic->size() == 0)
      return NULL;
    if (d->pcListPic_readIdx < 0 || d->pcListPic_readIdx > d->pcListPic->size())
      return NULL;

    // Get the pcListPic_readIdx-th picture from the list
    TComList<TComPic*>::iterator iterPic = d->pcListPic->begin();
    for (int i = 0; i < d->pcListPic_readIdx; i++)
      iterPic++;

    if ((*(iterPic))->isField())
      // TODO: Field output not supported (YET)
      return NULL;

    // Go on in the list until we run out of frames or find one that we can output
    while (iterPic != d->pcListPic->end())
    {
      TComPic* pcPic = *(iterPic);

      if ((d->flushOutput && (pcPic->getOutputMark())) ||
        (pcPic->getOutputMark() && pcPic->getPOC() > d->iPOCLastDisplay && (d->numPicsNotYetDisplayed > d->numReorderPicsHighestTid || d->dpbFullness > d->maxDecPicBufferingHighestTid)))
      {
        if (!d->flushOutput)
          // Output picture found
          d->numPicsNotYetDisplayed--;

        if(pcPic->getSlice(0)->isReferenced() == false)
          d->dpbFullness--;

        // update POC of display order
        d->iPOCLastDisplay = pcPic->getPOC();

        // erase non-referenced picture in the reference picture list after display
        if ( !pcPic->getSlice(0)->isReferenced() && pcPic->getReconMark() == true )
        {
#if !DYN_REF_FREE
          pcPic->setReconMark(false);

          // mark it should be extended later
          pcPic->getPicYuvRec()->setBorderExtension( false );

#else
          pcPic->destroy();
          pcListPic->erase( iterPic );
          iterPic = pcListPic->begin(); // to the beginning, non-efficient way, have to be revised!
          continue;
#endif
        }
        pcPic->setOutputMark(false);

        // Return the picture
        return (libHMDec_picture*)pcPic;
      }

      iterPic++;
      d->pcListPic_readIdx++;
    }

    // We reached the end of the list wothout finding an output picture
    if (d->flushOutput)
    {
      // Flushing over
      d->pcListPic->clear();
      d->iPOCLastDisplay = -MAX_INT;
      d->flushOutput = false;
    }
    if (d->sheduleFlushing)
    {
      // The normal output function is over. In the next call to this function, we will start flushing.
      d->flushOutput = true;
      d->sheduleFlushing = false;
      d->pcListPic_readIdx = 0;   // Iterate over all items again
    }

    return NULL;
  }

  HM_DEC_API int libHMDEC_get_POC(libHMDec_picture *pic)
  {
    if (pic == NULL)
      return -1;
    TComPic* pcPic = (TComPic*)pic;
    if (pcPic == NULL)
      return -1;

    return pcPic->getPOC();
  }

  HM_DEC_API int libHMDEC_get_picture_width(libHMDec_picture *pic, libHMDec_ColorComponent c)
  {
    if (pic == NULL)
      return -1;
    TComPic* pcPic = (TComPic*)pic;
    if (pcPic == NULL)
      return -1;

    // Conformance window
    const Window &conf = pcPic->getConformanceWindow();
    
    if (c == LIBHMDEC_LUMA)
    {
      int subtract = (conf.getWindowLeftOffset() >> pcPic->getComponentScaleX(COMPONENT_Y)) + (conf.getWindowRightOffset() >> pcPic->getComponentScaleX(COMPONENT_Y));
      return pcPic->getPicYuvRec()->getWidth(COMPONENT_Y) - subtract;
    }
    if (c == LIBHMDEC_CHROMA_U)
    {
      int subtract = (conf.getWindowLeftOffset() >> pcPic->getComponentScaleX(COMPONENT_Cb)) + (conf.getWindowRightOffset() >> pcPic->getComponentScaleX(COMPONENT_Cb));
      return pcPic->getPicYuvRec()->getWidth(COMPONENT_Cb) - subtract;
    }
    if (c == LIBHMDEC_CHROMA_V)
    {
      int subtract = (conf.getWindowLeftOffset() >> pcPic->getComponentScaleX(COMPONENT_Cr)) + (conf.getWindowRightOffset() >> pcPic->getComponentScaleX(COMPONENT_Cr));
      return pcPic->getPicYuvRec()->getWidth(COMPONENT_Cr) - subtract;
    }
    return -1;
  }

  HM_DEC_API int libHMDEC_get_picture_height(libHMDec_picture *pic, libHMDec_ColorComponent c)
  {
    if (pic == NULL)
      return -1;
    TComPic* pcPic = (TComPic*)pic;
    if (pcPic == NULL)
      return -1;

    // Conformance window
    const Window &conf = pcPic->getConformanceWindow();
    
    if (c == LIBHMDEC_LUMA)
    {
      int subtract = (conf.getWindowTopOffset() >> pcPic->getComponentScaleY(COMPONENT_Y)) + (conf.getWindowBottomOffset() >> pcPic->getComponentScaleY(COMPONENT_Y));
      return pcPic->getPicYuvRec()->getHeight(COMPONENT_Y) - subtract;
    }
    if (c == LIBHMDEC_CHROMA_U)
    {
      int subtract = (conf.getWindowTopOffset() >> pcPic->getComponentScaleY(COMPONENT_Cb)) + (conf.getWindowBottomOffset() >> pcPic->getComponentScaleY(COMPONENT_Cb));
      return pcPic->getPicYuvRec()->getHeight(COMPONENT_Cb) - subtract;
    }
    if (c == LIBHMDEC_CHROMA_V)
    {
      int subtract = (conf.getWindowTopOffset() >> pcPic->getComponentScaleY(COMPONENT_Cr)) + (conf.getWindowBottomOffset() >> pcPic->getComponentScaleY(COMPONENT_Cr));
      return pcPic->getPicYuvRec()->getHeight(COMPONENT_Cr) - subtract;
    }
    return -1;
  }

  HM_DEC_API int libHMDEC_get_picture_stride(libHMDec_picture *pic, libHMDec_ColorComponent c)
  {
    if (pic == NULL)
      return -1;
    TComPic* pcPic = (TComPic*)pic;
    if (pcPic == NULL)
      return -1;

    if (c == LIBHMDEC_LUMA)
      return pcPic->getPicYuvRec()->getStride(COMPONENT_Y);
    if (c == LIBHMDEC_CHROMA_U)
      return pcPic->getPicYuvRec()->getStride(COMPONENT_Cb);
    if (c == LIBHMDEC_CHROMA_V)
      return pcPic->getPicYuvRec()->getStride(COMPONENT_Cr);
    return -1;
  }

  HM_DEC_API short* libHMDEC_get_image_plane(libHMDec_picture *pic, libHMDec_ColorComponent c)
  {
    if (pic == NULL)
      return NULL;
    TComPic* pcPic = (TComPic*)pic;
    if (pcPic == NULL)
      return NULL;

    // Conformance window
    const Window &conf = pcPic->getConformanceWindow();
    ComponentID compID = (c == LIBHMDEC_LUMA) ? COMPONENT_Y : (c == LIBHMDEC_CHROMA_U) ? COMPONENT_Cb : COMPONENT_Cr;
    const UInt csx = pcPic->getComponentScaleX(compID);
    const UInt csy = pcPic->getComponentScaleY(compID);
    const Int planeOffset = (conf.getWindowLeftOffset() >> csx) + (conf.getWindowTopOffset() >> csy) * pcPic->getStride(compID);
    
    if (c == LIBHMDEC_LUMA)
      return pcPic->getPicYuvRec()->getAddr(COMPONENT_Y) + planeOffset;
    if (c == LIBHMDEC_CHROMA_U)
      return pcPic->getPicYuvRec()->getAddr(COMPONENT_Cb) + planeOffset;
    if (c == LIBHMDEC_CHROMA_V)
      return pcPic->getPicYuvRec()->getAddr(COMPONENT_Cr) + planeOffset;
    return NULL;
  }

  HM_DEC_API libHMDec_ChromaFormat libHMDEC_get_chroma_format(libHMDec_picture *pic)
  {
    if (pic == NULL)
      return LIBHMDEC_CHROMA_UNKNOWN;
    TComPic* pcPic = (TComPic*)pic;
    if (pcPic == NULL)
      return LIBHMDEC_CHROMA_UNKNOWN;

    ChromaFormat f = pcPic->getChromaFormat();
    if (f == CHROMA_400)
      return LIBHMDEC_CHROMA_400;
    if (f == CHROMA_420)
      return LIBHMDEC_CHROMA_420;
    if (f == CHROMA_422)
      return LIBHMDEC_CHROMA_422;
    if (f == CHROMA_444)
      return LIBHMDEC_CHROMA_444;
    return LIBHMDEC_CHROMA_UNKNOWN;
  }

  HM_DEC_API int libHMDEC_get_internal_bit_depth(libHMDec_picture *pic, libHMDec_ColorComponent c)
  {
    TComPic* pcPic = (TComPic*)pic;
    if (pcPic == NULL)
      return -1;

    const BitDepths &bitDepths = pcPic->getPicSym()->getSPS().getBitDepths();
    if (c == LIBHMDEC_LUMA)
      return bitDepths.recon[CHANNEL_TYPE_LUMA];
    if (c == LIBHMDEC_CHROMA_U || c == LIBHMDEC_CHROMA_V)
      return bitDepths.recon[CHANNEL_TYPE_CHROMA];
    return -1;
  }

  // --------- internals --------

  typedef enum
  {
    LIBHMDEC_CTU_SLICE_INDEX = 0,     ///< The slice index of the CTU
    LIBHMDEC_CU_PREDICTION_MODE,      ///< Does the CU use inter (0) or intra(1) prediction?
    LIBHMDEC_CU_TRQ_BYPASS,           ///< If transquant bypass is enabled, is the transquant bypass flag set?
    LIBHMDEC_CU_SKIP_FLAG,            ///< Is the CU skip flag set?
    LIBHMDEC_CU_PART_MODE,            ///< What is the partition mode of the CU into PUs? 0: SIZE_2Nx2N, 1: SIZE_2NxN, 2: SIZE_Nx2N, 3: SIZE_NxN, 4: SIZE_2NxnU, 5: SIZE_2NxnD, 6: SIZE_nLx2N, 7: SIZE_nRx2N
    LIBHMDEC_CU_INTRA_MODE_LUMA,      ///< If the CU uses intra prediction, get the intra mode for luma
    LIBHMDEC_CU_INTRA_MODE_CHROMA,    ///< If the CU uses intra prediction, get the intra mode for chroma
    LIBHMDEC_CU_ROOT_CBF,             ///< In the CU is inter, get the root coded block flag of the TU
    LIBHMDEC_PU_MERGE_FLAG,           ///< If the PU is inter, is the merge flag set?
    LIBHMDEC_PU_MERGE_INDEX,          ///< If the PU is merge, what is the merge index?
    LIBHMDEC_PU_UNI_BI_PREDICTION,    ///< Does the PU use uni- (0) or biprediction (1)? Also called interDir.
    LIBHMDEC_PU_REFERENCE_POC_0,      ///< If the PU uses inter prediction, what is the reference POC of list 0?
    LIBHMDEC_PU_MV_0,                 ///< If the PU uses inter prediction, what is the motion vector of list 0?
    LIBHMDEC_PU_REFERENCE_POC_1,      ///< If the PU uses bi-directions inter prediction, what is the reference POC of list 1?
    LIBHMDEC_PU_MV_1,                 ///< If the PU uses bi-directions inter prediction, what is the motion vector of list 1?
    LIBHMDEC_TU_CBF_Y,                ///< Get the coded block flag for luma
    LIBHMDEC_TU_CBF_CB,               ///< Get the coded block flag for chroma U
    LIBHMDEC_TU_CBF_CR,               ///< Get the coded block flag for chroma V
    LIBHMDEC_TU_COEFF_TR_SKIP_Y,      ///< Get the transform skip flag for luma
    LIBHMDEC_TU_COEFF_TR_SKIP_Cb,     ///< Get the transform skip flag for chroma U
    LIBHMDEC_TU_COEFF_TR_SKIP_Cr,     ///< Get the transform skip flag for chroma V
    LIBHMDEC_TU_COEFF_ENERGY_Y,       ///< If the root CBF of the TU is not 0, get the coefficient energy of the TU for luma
    LIBHMDEC_TU_COEFF_ENERGY_CB,      ///< If the root CBF of the TU is not 0, get the coefficient energy of the TU for chroma U
    LIBHMDEC_TU_COEFF_ENERGY_CR,      ///< If the root CBF of the TU is not 0, get the coefficient energy of the TU for chroma V
    LIBHMDEC_NUM_TYPES
  } libHMDec_info_types_idx;

  // These types are supported here
  HM_DEC_API unsigned int libHMDEC_get_internal_type_number()
  {
    return LIBHMDEC_NUM_TYPES;
  }

  HM_DEC_API const char *libHMDEC_get_internal_type_name(unsigned int idx)
  {
    switch (idx)
    {
    case LIBHMDEC_CTU_SLICE_INDEX:      return "CTU Slice Index";
    case LIBHMDEC_CU_PREDICTION_MODE:   return "CU Pred Mode";
    case LIBHMDEC_CU_TRQ_BYPASS:        return "CU TrQuant Bypass";
    case LIBHMDEC_CU_SKIP_FLAG:         return "CU Skip";
    case LIBHMDEC_CU_PART_MODE:         return "CU Part Mode";
    case LIBHMDEC_CU_INTRA_MODE_LUMA:   return "CU Intra Mode Y";
    case LIBHMDEC_CU_INTRA_MODE_CHROMA: return "CU Intra Mode C";
    case LIBHMDEC_CU_ROOT_CBF:          return "CU Root CBF";
    case LIBHMDEC_PU_MERGE_FLAG:        return "PU Merge";
    case LIBHMDEC_PU_MERGE_INDEX:       return "PU Merge Idx";
    case LIBHMDEC_PU_UNI_BI_PREDICTION: return "PU Uni/Bi Pred";
    case LIBHMDEC_PU_REFERENCE_POC_0:   return "PU Ref POC 0";
    case LIBHMDEC_PU_MV_0:              return "PU MV 0";
    case LIBHMDEC_PU_REFERENCE_POC_1:   return "PU Ref POC 1";
    case LIBHMDEC_PU_MV_1:              return "PU MV 1";
    case LIBHMDEC_TU_CBF_Y:             return "TU CBF Y";
    case LIBHMDEC_TU_CBF_CB:            return "TU CBF Cb";
    case LIBHMDEC_TU_CBF_CR:            return "TU CBF Cr";
    case LIBHMDEC_TU_COEFF_TR_SKIP_Y:   return "TU TrSkip Y";
    case LIBHMDEC_TU_COEFF_TR_SKIP_Cb:  return "TU TrSkip Cb";
    case LIBHMDEC_TU_COEFF_TR_SKIP_Cr:  return "TU TrSkip Cr";
    case LIBHMDEC_TU_COEFF_ENERGY_Y:    return "TU Coeff Energy Y";
    case LIBHMDEC_TU_COEFF_ENERGY_CB:   return "TU Coeff Energy Cb";
    case LIBHMDEC_TU_COEFF_ENERGY_CR:   return "TU Coeff Energy Cr";
    default: return "";
    }
  }

  HM_DEC_API libHMDec_InternalsType libHMDEC_get_internal_type(unsigned int idx)
  {
    switch (idx)
    {
    case LIBHMDEC_CTU_SLICE_INDEX:      return LIBHMDEC_TYPE_RANGE;
    case LIBHMDEC_CU_PREDICTION_MODE:   return LIBHMDEC_TYPE_FLAG;
    case LIBHMDEC_CU_TRQ_BYPASS:        return LIBHMDEC_TYPE_FLAG;
    case LIBHMDEC_CU_SKIP_FLAG:         return LIBHMDEC_TYPE_FLAG;
    case LIBHMDEC_CU_PART_MODE:         return LIBHMDEC_TYPE_RANGE;
    case LIBHMDEC_CU_INTRA_MODE_LUMA:   return LIBHMDEC_TYPE_INTRA_DIR;
    case LIBHMDEC_CU_INTRA_MODE_CHROMA: return LIBHMDEC_TYPE_INTRA_DIR;
    case LIBHMDEC_CU_ROOT_CBF:          return LIBHMDEC_TYPE_FLAG;
    case LIBHMDEC_PU_MERGE_FLAG:        return LIBHMDEC_TYPE_FLAG;
    case LIBHMDEC_PU_MERGE_INDEX:       return LIBHMDEC_TYPE_RANGE;
    case LIBHMDEC_PU_UNI_BI_PREDICTION: return LIBHMDEC_TYPE_FLAG;
    case LIBHMDEC_PU_REFERENCE_POC_0:   return LIBHMDEC_TYPE_RANGE_ZEROCENTER;
    case LIBHMDEC_PU_MV_0:              return LIBHMDEC_TYPE_VECTOR;
    case LIBHMDEC_PU_REFERENCE_POC_1:   return LIBHMDEC_TYPE_RANGE_ZEROCENTER;
    case LIBHMDEC_PU_MV_1:              return LIBHMDEC_TYPE_VECTOR;
    case LIBHMDEC_TU_CBF_Y:             return LIBHMDEC_TYPE_FLAG;
    case LIBHMDEC_TU_CBF_CB:            return LIBHMDEC_TYPE_FLAG;
    case LIBHMDEC_TU_CBF_CR:            return LIBHMDEC_TYPE_FLAG;
    case LIBHMDEC_TU_COEFF_TR_SKIP_Y:   return LIBHMDEC_TYPE_FLAG;
    case LIBHMDEC_TU_COEFF_TR_SKIP_Cb:  return LIBHMDEC_TYPE_FLAG;
    case LIBHMDEC_TU_COEFF_TR_SKIP_Cr:  return LIBHMDEC_TYPE_FLAG;
    case LIBHMDEC_TU_COEFF_ENERGY_Y:    return LIBHMDEC_TYPE_RANGE;
    case LIBHMDEC_TU_COEFF_ENERGY_CB:   return LIBHMDEC_TYPE_RANGE;
    case LIBHMDEC_TU_COEFF_ENERGY_CR:   return LIBHMDEC_TYPE_RANGE;
    default: return LIBHMDEC_TYPE_UNKNOWN;
    }
  }

  HM_DEC_API unsigned int libHMDEC_get_internal_type_max(unsigned int idx)
  {
    switch (idx)
    {
    case LIBHMDEC_CTU_SLICE_INDEX:      return 10;
    case LIBHMDEC_CU_PART_MODE:         return 7;
    case LIBHMDEC_PU_MERGE_INDEX:       return 6;
    case LIBHMDEC_PU_REFERENCE_POC_0:   return 16;
    case LIBHMDEC_PU_REFERENCE_POC_1:   return 16;
    case LIBHMDEC_TU_COEFF_ENERGY_Y:    return 1000;
    case LIBHMDEC_TU_COEFF_ENERGY_CB:   return 1000;
    case LIBHMDEC_TU_COEFF_ENERGY_CR:   return 1000;
    default: return LIBHMDEC_TYPE_UNKNOWN;
    }
  }

  HM_DEC_API unsigned int libHMDEC_get_internal_type_vector_scaling(unsigned int idx)
  {
    if (idx == LIBHMDEC_PU_MV_0 || idx == LIBHMDEC_PU_MV_1)
      return 4;
    return 1;
  }

  HM_DEC_API const char *libHMDEC_get_internal_type_description(unsigned int idx)
  {
    switch (idx)
    {
    case LIBHMDEC_CTU_SLICE_INDEX:      return "The slice index of the CTU"; break;
    case LIBHMDEC_CU_PREDICTION_MODE:   return "Does the CU use inter (0) or intra(1) prediction?"; break;
    case LIBHMDEC_CU_TRQ_BYPASS:        return "If transquant bypass is enabled, is the transquant bypass flag set?"; break;
    case LIBHMDEC_CU_SKIP_FLAG:         return "Is the CU skip flag set?"; break;
    case LIBHMDEC_CU_PART_MODE:         return "What is the partition mode of the CU into PUs? 0: SIZE_2Nx2N, 1: SIZE_2NxN, 2: SIZE_Nx2N, 3: SIZE_NxN, 4: SIZE_2NxnU, 5: SIZE_2NxnD, 6: SIZE_nLx2N, 7: SIZE_nRx2N"; break;
    case LIBHMDEC_CU_INTRA_MODE_LUMA:   return "If the CU uses intra prediction, get the intra mode for luma"; break;
    case LIBHMDEC_CU_INTRA_MODE_CHROMA: return "If the CU uses intra prediction, get the intra mode for chroma"; break;
    case LIBHMDEC_CU_ROOT_CBF:          return "In the CU is inter, get the root coded block flag of the TU"; break;
    case LIBHMDEC_PU_MERGE_FLAG:        return "If the PU is inter, is the merge flag set?"; break;
    case LIBHMDEC_PU_MERGE_INDEX:       return "If the PU is merge, what is the merge index?"; break;
    case LIBHMDEC_PU_UNI_BI_PREDICTION: return "Does the PU use uni- (0) or biprediction (1)? Also called interDir."; break;
    case LIBHMDEC_PU_REFERENCE_POC_0:   return "If the PU uses inter prediction, what is the reference POC of list 0?"; break;
    case LIBHMDEC_PU_MV_0:              return "If the PU uses inter prediction, what is the motion vector of list 0?"; break;
    case LIBHMDEC_PU_REFERENCE_POC_1:   return "If the PU uses bi-directions inter prediction, what is the reference POC of list 1?"; break;
    case LIBHMDEC_PU_MV_1:              return "If the PU uses bi-directions inter prediction, what is the motion vector of list 1?"; break;
    case LIBHMDEC_TU_CBF_Y:             return "Get the coded block flag for luma"; break;
    case LIBHMDEC_TU_CBF_CB:            return "Get the coded block flag for chroma U"; break;
    case LIBHMDEC_TU_CBF_CR:            return "Get the coded block flag for chroma V"; break;
    case LIBHMDEC_TU_COEFF_TR_SKIP_Y:   return "Get the transform skip flag for luma"; break;
    case LIBHMDEC_TU_COEFF_TR_SKIP_Cb:  return "Get the transform skip flag for chroma U"; break;
    case LIBHMDEC_TU_COEFF_TR_SKIP_Cr:  return "Get the transform skip flag for chroma V"; break;
    case LIBHMDEC_TU_COEFF_ENERGY_Y:    return "If the root CBF of the TU is not 0, get the coefficient energy of the TU for luma"; break;
    case LIBHMDEC_TU_COEFF_ENERGY_CB:   return "If the root CBF of the TU is not 0, get the coefficient energy of the TU for chroma U"; break;
    case LIBHMDEC_TU_COEFF_ENERGY_CR:   return "If the root CBF of the TU is not 0, get the coefficient energy of the TU for chroma V"; break;
    default: return ""; break;
    }
  }

  bool addValuesForPUs(hmDecoderWrapper *d, TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, unsigned int type)
  {
    PartSize ePartSize = pcCU->getPartitionSize( uiAbsPartIdx );
    UInt uiNumPU = ( ePartSize == SIZE_2Nx2N ? 1 : ( ePartSize == SIZE_NxN ? 4 : 2 ) );
    UInt uiPUOffset = ( g_auiPUOffset[UInt( ePartSize )] << ( ( pcCU->getSlice()->getSPS()->getMaxTotalCUDepth() - uiDepth ) << 1 ) ) >> 4;

    TComRom::TComRomScan *scan = pcCU->getRomScan();
    const int cuWidth = pcCU->getSlice()->getSPS()->getMaxCUWidth() >> uiDepth;
    const int cuHeight = pcCU->getSlice()->getSPS()->getMaxCUHeight() >> uiDepth;
    const int cuX = pcCU->getCUPelX() + scan->auiRasterToPelX[ scan->auiZscanToRaster[uiAbsPartIdx] ];
    const int cuY = pcCU->getCUPelY() + scan->auiRasterToPelY[ scan->auiZscanToRaster[uiAbsPartIdx] ];

    UInt uiPartIdx = 0;
    UInt uiSubPartIdx = uiAbsPartIdx;
    if (d->pauseInternalsPUPartIdx != -1 && d->pauseInternalsPUSubPartIdx != -1)
    {
      // We are continuing parsing
      uiPartIdx = d->pauseInternalsPUPartIdx;
      uiSubPartIdx = d->pauseInternalsPUSubPartIdx;
      d->pauseInternalsPUPartIdx = -1;
      d->pauseInternalsPUSubPartIdx = -1;
    }

    for (; uiPartIdx < uiNumPU; uiPartIdx++, uiSubPartIdx += uiPUOffset)
    {
      // Before we get the block data, check if the value cache is full
      if (d->internalsBlockDataFull())
      {
        // Cache is full, save the position of the PU so that we can continue in the next call
        d->pauseInternalsPUPartIdx = uiPartIdx;
        d->pauseInternalsPUSubPartIdx = uiSubPartIdx;
        return false;
      }

      // Set the size and position of the PU
      libHMDec_BlockValue b;
      switch (ePartSize)
      {
      case SIZE_2NxN:
        b.w = cuWidth;
        b.h = cuHeight >> 1;
        b.x = cuX;
        b.y = (uiPartIdx == 0) ? cuY : cuY + b.h;
        break;
      case SIZE_Nx2N:
        b.w = cuWidth >> 1;
        b.h = cuHeight;
        b.x = (uiPartIdx == 0) ? cuX : cuX + b.w;
        b.y = cuY;
        break;
      case SIZE_NxN:
        b.w = cuWidth >> 1;
        b.h = cuHeight >> 1;
        b.x = (uiPartIdx == 0 || uiPartIdx == 2) ? cuX : cuX + b.w;
        b.y = (uiPartIdx == 0 || uiPartIdx == 1) ? cuY : cuY + b.h;
        break;
      case SIZE_2NxnU:
        b.w = cuWidth;
        b.h = (uiPartIdx == 0) ? (cuHeight >> 2) : ((cuHeight >> 2) + (cuHeight >> 1));
        b.x = cuX;
        b.y = (uiPartIdx == 0) ? cuY : cuY + (cuHeight >> 2);
        break;
      case SIZE_2NxnD:
        b.w = cuWidth;
        b.h = (uiPartIdx == 0) ? ((cuHeight >> 2) + (cuHeight >> 1)) : (cuHeight >> 2);
        b.x = cuX;
        b.y = (uiPartIdx == 0) ? cuY : cuY + (cuHeight >> 2) + (cuHeight >> 1);
        break;
      case SIZE_nLx2N:
        b.w = (uiPartIdx == 0) ? (cuWidth >> 2) : ((cuWidth >> 2) + (cuWidth >> 1));
        b.h = cuHeight;
        b.x = (uiPartIdx == 0) ? cuX : cuX + (cuWidth >> 2);
        b.y = cuY;
        break;
      case SIZE_nRx2N:
        b.w = (uiPartIdx == 0) ? ((cuWidth >> 2) + (cuWidth >> 1)) : (cuWidth >> 2);
        b.h = cuHeight;
        b.x = (uiPartIdx == 0) ? cuX : cuX + (cuWidth >> 2) + (cuWidth >> 1);
        b.y = cuY;
        break;
      case SIZE_2Nx2N:
        b.w = cuWidth;
        b.h = cuHeight;
        b.x = cuX;
        b.y = cuY;
        break;
      default:
        assert(false);
      }

      // Get the value that we want to save for this PU
      if (type == LIBHMDEC_PU_MERGE_FLAG)
        b.value = pcCU->getMergeFlag(uiSubPartIdx) ? 1 : 0;
      if (type == LIBHMDEC_PU_MERGE_INDEX && pcCU->getMergeFlag(uiSubPartIdx))
        b.value = (int)pcCU->getMergeIndex(uiSubPartIdx);
      if (type == LIBHMDEC_PU_UNI_BI_PREDICTION)
        b.value = (int)pcCU->getInterDir(uiSubPartIdx);
      if (type == LIBHMDEC_PU_REFERENCE_POC_0 || type == LIBHMDEC_PU_REFERENCE_POC_1)
      {
        RefPicList list = (type == LIBHMDEC_PU_REFERENCE_POC_0) ? REF_PIC_LIST_0 : REF_PIC_LIST_1;
        if (!(pcCU->getInterDir(uiSubPartIdx) & (1 << list)))
          // We are looking for the reference POC for list 0/1 but this PU does not use list 0/1
          return true;

        Int refIdx = pcCU->getCUMvField(list)->getRefIdx(uiSubPartIdx);
        Int refPOC = pcCU->getSlice()->getRefPic(list, refIdx)->getPOC();
        Int curPOC = pcCU->getPic()->getPOC();
        b.value = refPOC - curPOC;
        if (b.value < -16)
          b.value = -16;
        else if (b.value > 16)
          b.value = 16;
      }
      if (type == LIBHMDEC_PU_MV_0 || type == LIBHMDEC_PU_MV_1)
      {
        RefPicList list = (type == LIBHMDEC_PU_MV_0) ? REF_PIC_LIST_0 : REF_PIC_LIST_1;
        if (!(pcCU->getInterDir(uiSubPartIdx) & (1 << list)))
          // We are looking for motion vectors for list 0/1 but this PU does not use list 0/1
          return true;

        b.value  = pcCU->getCUMvField(list)->getMv(uiSubPartIdx).getHor();
        b.value2 = pcCU->getCUMvField(list)->getMv(uiSubPartIdx).getVer();
      }

      // Add the value
      d->addInternalsBlockData(b);
    }
    return true;
  }

  bool addValuesForTURecursive(hmDecoderWrapper *d, TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt trDepth, unsigned int typeIdx)
  {
    UInt trIdx = pcCU->getTransformIdx(uiAbsPartIdx);
    if (trDepth < trIdx)
    {
      // Split
      UInt uiNextDepth = uiDepth + trDepth + 1;
      UInt uiQNumParts = pcCU->getTotalNumPart() >> (uiNextDepth<<1);

      int i = 0;
      if (d->pauseInternalsTUIdxRecursive[trDepth] != -1)
      {
        // Continue parsing from here
        i = d->pauseInternalsTUIdxRecursive[trDepth];
        d->pauseInternalsTUIdxRecursive[trDepth] = -1;
      }
      for (; i < 4; i++)
      {
        if (!addValuesForTURecursive(d, pcCU, uiAbsPartIdx + i * uiQNumParts, uiDepth, trDepth + 1, typeIdx))
        {
          // The cache is full
          d->pauseInternalsTUIdxRecursive[trDepth] = i;
          return false;
        }
      }
    }

    // Is there still space in the cache?
    if (d->internalsBlockDataFull())
      return false;

    // We are not at the TU level
    TComRom::TComRomScan *scan = pcCU->getRomScan();
    UInt uiLPelX = pcCU->getCUPelX() + scan->auiRasterToPelX[ scan->auiZscanToRaster[uiAbsPartIdx] ];
    UInt uiTPelY = pcCU->getCUPelY() + scan->auiRasterToPelY[ scan->auiZscanToRaster[uiAbsPartIdx] ];

    libHMDec_BlockValue b;
    b.x = uiLPelX;
    b.y = uiTPelY;
    b.w = (pcCU->getSlice()->getSPS()->getMaxCUWidth() >> (uiDepth + trDepth));
    b.h = (pcCU->getSlice()->getSPS()->getMaxCUHeight() >> (uiDepth + trDepth));
    if (typeIdx == LIBHMDEC_TU_CBF_Y)
      b.value = (pcCU->getCbf(uiAbsPartIdx, COMPONENT_Y, trDepth) != 0) ? 1 : 0;
    else if (typeIdx == LIBHMDEC_TU_CBF_CB)
      b.value = (pcCU->getCbf(uiAbsPartIdx, COMPONENT_Cb, trDepth) != 0) ? 1 : 0;
    else if (typeIdx == LIBHMDEC_TU_CBF_CR)
      b.value = (pcCU->getCbf(uiAbsPartIdx, COMPONENT_Cr, trDepth) != 0) ? 1 : 0;
    else if (typeIdx == LIBHMDEC_TU_COEFF_TR_SKIP_Y)
      b.value = (pcCU->getTransformSkip(uiAbsPartIdx, COMPONENT_Y) != 0) ? 1 : 0;
    else if (typeIdx == LIBHMDEC_TU_COEFF_TR_SKIP_Cb)
      b.value = (pcCU->getTransformSkip(uiAbsPartIdx, COMPONENT_Cb) != 0) ? 1 : 0;
    else if (typeIdx == LIBHMDEC_TU_COEFF_TR_SKIP_Cr)
      b.value = (pcCU->getTransformSkip(uiAbsPartIdx, COMPONENT_Cr) != 0) ? 1 : 0;
    else if (typeIdx == LIBHMDEC_TU_COEFF_ENERGY_Y || typeIdx == LIBHMDEC_TU_COEFF_ENERGY_CB || typeIdx == LIBHMDEC_TU_COEFF_ENERGY_CB)
    {
      ComponentID c = (typeIdx == LIBHMDEC_TU_COEFF_ENERGY_Y) ? COMPONENT_Y : (typeIdx == LIBHMDEC_TU_COEFF_ENERGY_CB) ? COMPONENT_Cb : COMPONENT_Cr;

      const int nrCoeff = (typeIdx == LIBHMDEC_TU_COEFF_ENERGY_Y) ? b.w * b.h : b.w/2 * b.h/2;

      TCoeff* pcCoef = pcCU->getCoeff(c);
      int64_t e = 0;
      for (int i = 0; i < nrCoeff; i++)
      {
        TCoeff co = pcCoef[i];
        e += co * co;
      }
      if (e > MAX_INT)
        b.value = MAX_INT;
      else
        b.value = (int)e;
    }
    d->addInternalsBlockData(b);
    return true;
  }

  bool addValuesForCURecursively(hmDecoderWrapper *d, TComDataCU* pcLCU, UInt uiAbsPartIdx, UInt uiDepth, unsigned int typeIdx)
  {
    TComSlice * pcSlice = pcLCU->getSlice();
    const TComSPS &sps = *(pcSlice->getSPS());
    const TComPPS &pps = *(pcSlice->getPPS());

    Bool bBoundary = false;
    TComRom::TComRomScan *scan = pcLCU->getRomScan();
    UInt uiLPelX = pcLCU->getCUPelX() + scan->auiRasterToPelX[scan->auiZscanToRaster[uiAbsPartIdx]];
    UInt uiRPelX = uiLPelX + (sps.getMaxCUWidth() >> uiDepth) - 1;
    UInt uiTPelY = pcLCU->getCUPelY() + scan->auiRasterToPelY[scan->auiZscanToRaster[uiAbsPartIdx]];
    UInt uiBPelY = uiTPelY + (sps.getMaxCUHeight() >> uiDepth) - 1;

    if ((uiRPelX >= sps.getPicWidthInLumaSamples()) || (uiBPelY >= sps.getPicHeightInLumaSamples()))
    {
      bBoundary = true;
    }

    if (((uiDepth < pcLCU->getDepth(uiAbsPartIdx)) && (uiDepth < sps.getLog2DiffMaxMinCodingBlockSize())) || bBoundary)
    {
      UInt uiNextDepth = uiDepth + 1;
      UInt uiQNumParts = pcLCU->getTotalNumPart() >> (uiNextDepth<<1);
      UInt uiPartIdx = 0;
      if (d->pauseInternalsCUPartIdxRecursive[uiDepth] != -1)
      {
        // Continue retriveal from this point
        uiPartIdx = d->pauseInternalsCUPartIdxRecursive[uiDepth];
        d->pauseInternalsCUPartIdxRecursive[uiDepth] = -1;
      }
      for (; uiPartIdx < 4; uiPartIdx++)
      {
        UInt uiIdx = uiAbsPartIdx + uiPartIdx * uiQNumParts;
        uiLPelX = pcLCU->getCUPelX() + scan->auiRasterToPelX[scan->auiZscanToRaster[uiIdx]];
        uiTPelY = pcLCU->getCUPelY() + scan->auiRasterToPelY[scan->auiZscanToRaster[uiIdx]];

        if ((uiLPelX < sps.getPicWidthInLumaSamples()) && (uiTPelY < sps.getPicHeightInLumaSamples()))
        {
          if (!addValuesForCURecursively(d, pcLCU, uiIdx, uiNextDepth, typeIdx))
          {
            // The cache is full. Save the current part index in the current depth so we can continue from here.
            d->pauseInternalsCUPartIdxRecursive[uiDepth] = uiPartIdx;
            return false;
          }
        }
      }
      return true;
    }

    // We reached the CU
    if (typeIdx == LIBHMDEC_CU_PREDICTION_MODE || typeIdx == LIBHMDEC_CU_TRQ_BYPASS || typeIdx == LIBHMDEC_CU_SKIP_FLAG || typeIdx == LIBHMDEC_CU_PART_MODE || typeIdx == LIBHMDEC_CU_INTRA_MODE_LUMA || typeIdx == LIBHMDEC_CU_INTRA_MODE_CHROMA || typeIdx == LIBHMDEC_CU_ROOT_CBF)
    {
      if ((typeIdx == LIBHMDEC_CU_TRQ_BYPASS && !pps.getTransquantBypassEnabledFlag()) ||
          (typeIdx == LIBHMDEC_CU_INTRA_MODE_LUMA && !pcLCU->isIntra(uiAbsPartIdx)) ||
          (typeIdx == LIBHMDEC_CU_INTRA_MODE_CHROMA && !pcLCU->isIntra(uiAbsPartIdx)) ||
          (typeIdx == LIBHMDEC_CU_ROOT_CBF && pcLCU->isInter(uiAbsPartIdx)))
        // There is no data for this CU of this type
        return true;

      // Is there more space in the cache?
      if (d->internalsBlockDataFull())
        return false;

      libHMDec_BlockValue b;
      b.x = uiLPelX;
      b.y = uiTPelY;
      b.w = (sps.getMaxCUWidth() >>uiDepth);
      b.h = (sps.getMaxCUHeight() >>uiDepth);
      if (typeIdx == LIBHMDEC_CU_PREDICTION_MODE)
        b.value = int(pcLCU->getPredictionMode(uiAbsPartIdx));
      else if (typeIdx == LIBHMDEC_CU_TRQ_BYPASS)
        b.value = pcLCU->getCUTransquantBypass(uiAbsPartIdx) ? 1 : 0;
      else if (typeIdx == LIBHMDEC_CU_SKIP_FLAG)
        b.value =  pcLCU->isSkipped(uiAbsPartIdx) ? 1 : 0;
      else if (typeIdx == LIBHMDEC_CU_PART_MODE)
        b.value = (int)pcLCU->getPartitionSize(uiAbsPartIdx);
      else if (typeIdx == LIBHMDEC_CU_INTRA_MODE_LUMA)
        b.value = (int)pcLCU->getIntraDir(CHANNEL_TYPE_LUMA, uiAbsPartIdx);
      else if (typeIdx == LIBHMDEC_CU_INTRA_MODE_CHROMA)
        b.value = (int)pcLCU->getIntraDir(CHANNEL_TYPE_CHROMA, uiAbsPartIdx);
      else if (typeIdx == LIBHMDEC_CU_ROOT_CBF)
        b.value = (int)pcLCU->getQtRootCbf(uiAbsPartIdx);
      d->addInternalsBlockData(b);
    }
    else if (pcLCU->isInter(uiAbsPartIdx) && (typeIdx == LIBHMDEC_PU_MERGE_FLAG || typeIdx == LIBHMDEC_PU_UNI_BI_PREDICTION || typeIdx == LIBHMDEC_PU_REFERENCE_POC_0 || typeIdx == LIBHMDEC_PU_MV_0 || typeIdx == LIBHMDEC_PU_REFERENCE_POC_1 || typeIdx == LIBHMDEC_PU_MV_1))
      // Set values for every PU
      return addValuesForPUs(d, pcLCU, uiAbsPartIdx, uiDepth, typeIdx);
    else if (typeIdx == LIBHMDEC_TU_CBF_Y || typeIdx == LIBHMDEC_TU_CBF_CB || typeIdx == LIBHMDEC_TU_CBF_CR || typeIdx == LIBHMDEC_TU_COEFF_ENERGY_Y || typeIdx == LIBHMDEC_TU_COEFF_ENERGY_CB || typeIdx == LIBHMDEC_TU_COEFF_ENERGY_CR || typeIdx == LIBHMDEC_TU_COEFF_TR_SKIP_Y || typeIdx == LIBHMDEC_TU_COEFF_TR_SKIP_Cb || typeIdx == LIBHMDEC_TU_COEFF_TR_SKIP_Cr)
      return addValuesForTURecursive(d, pcLCU, uiAbsPartIdx, uiDepth, 0, typeIdx);

    // This code line should never be reached.
    return true;
  }

  HM_DEC_API libHMDec_BlockValue *libHMDEC_get_internal_info(libHMDec_context *decCtx, libHMDec_picture *pic, unsigned int typeIdx, unsigned int &nrValues, bool &callAgain)
  {
    nrValues = 0;
    callAgain = false;

    hmDecoderWrapper *d = (hmDecoderWrapper*)decCtx;
    if (!d)
      return NULL;

    // Clear the internals before adding new ones
    d->clearInternalsBlockData();

    if (pic == NULL)
      return NULL;
    TComPic* pcPic = (TComPic*)pic;
    if (pcPic == NULL)
      return NULL;
    TComPicSym *s = pcPic->getPicSym();
    if (s == NULL)
      return NULL;

    int nrCU = s->getNumberOfCtusInFrame();
    int i = 0;
    if (d->pauseInternalsCUIdx != -1)
    {
      // Continue from the given index.
      i = d->pauseInternalsCUIdx;
      d->pauseInternalsCUIdx = -1;
    }
    for (; i < nrCU; i++)
    {
      TComDataCU *pcLCU = s->getCtu(i);

      if ((typeIdx == LIBHMDEC_TU_COEFF_TR_SKIP_Y || typeIdx == LIBHMDEC_TU_COEFF_TR_SKIP_Cb || typeIdx == LIBHMDEC_TU_COEFF_TR_SKIP_Cr) && pcLCU->getSlice()->getPPS()->getUseTransformSkip())
        // Transform skip not enabled for this slice
        continue;

      if (typeIdx == LIBHMDEC_CTU_SLICE_INDEX)
      {
        if (d->internalsBlockDataFull())
        {
          // Cache is full. Remember the CU index so we can continue from here.
          d->pauseInternalsCUIdx = i;
          nrValues = d->internalsBlockDataValues;
          callAgain = true;
          return d->internalsBlockData;
        }

        libHMDec_BlockValue b;
        b.x = pcLCU->getCUPelX();
        b.y = pcLCU->getCUPelY();
        b.w = pcLCU->getSlice()->getSPS()->getMaxCUWidth();
        b.h = pcLCU->getSlice()->getSPS()->getMaxCUHeight();
        b.value = (int)pcLCU->getPic()->getCurrSliceIdx();
        d->addInternalsBlockData(b);
      }
      else
      {
        if (!addValuesForCURecursively(d, pcLCU, 0, 0, typeIdx))
        {
          // Cache is full. Remember the CU index so we can continue from here.
          d->pauseInternalsCUIdx = i;
          nrValues = d->internalsBlockDataValues;
          callAgain = true;
          return d->internalsBlockData;
        }
      }
    }

    // Processing of all values is finished. The cache is not full.
    nrValues = d->internalsBlockDataValues;
    callAgain = false;
    return d->internalsBlockData;
  }

  HM_DEC_API libHMDec_error libHMDEC_clear_internal_info(libHMDec_context *decCtx)
  {
    hmDecoderWrapper *d = (hmDecoderWrapper*)decCtx;
    if (!d)
      return LIBHMDEC_ERROR;

    // Clear the internals
    d->clearInternalsBlockData();

    return LIBHMDEC_OK;
  }

} // extern "C"
