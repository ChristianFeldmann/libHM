
#include "libHMDecoder.h"

#include "TLibCommon/CommonDef.h"
#include "TLibCommon/TComTU.h"
#include "TLibDecoder/TDecTop.h"
#include "TLibDecoder/NALread.h"

// The HEVC reference software uses global variables for some things.
// This is not a good idea for a shared library so we have to work around this by saving/setting these variables
// in case multiple decoders are used at the same time.
bool g_md5_mismatch = false; ///< top level flag that indicates if there has been a decoding mismatch

// TODO: (isNaluWithinTargetDecLayerIdSet) The target layer file is not supported (yet)
bool isNaluWithinTargetDecLayerIdSet(InputNALUnit* nalu) { return true; }

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

    md5_mismatch = false;

    // Initialize the decoder
    decTop.create();
    decTop.init();
    decTop.setDecodedPictureHashSEIEnabled(true);
    iPOCLastDisplay += iSkipFrame; // set the last displayed POC correctly for skip forward.
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

  // The vector that is filled when internals are returned.
  // The vector is defined, filled and cleared only in this library so that no chaos is created 
  // between the heap of the shared library and the caller programm.
  std::vector<libHMDec_BlockValue> internalsBlockData;

  TDecTop decTop;
};

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

    // Recieve the NAL units data and put it into a vector so that we can parse it using the InputNALUnit.
    vector<uint8_t> nalUnit;
    for (int i = 0; i < copyStart; i++)
      data++;
    for (int i = 0; i < length-copyStart; i++)
    {
      nalUnit.push_back(*data);
      data++;
    }

    // Read the NAL unit
    InputNALUnit nalu;
    read(nalu, nalUnit);
    
    if( (d->maxTemporalLayer >= 0 && nalu.m_temporalId > d->maxTemporalLayer) || !isNaluWithinTargetDecLayerIdSet(&nalu)  )
    {
      bNewPicture = false;
    }
    else
    {
      // Restore the global variable
      g_md5_mismatch = d->md5_mismatch;

      bNewPicture = d->decTop.decode(nalu, d->iSkipFrame, d->iPOCLastDisplay);
      if (bNewPicture)
      {
        // We encountered a new picture in this NAL unit. This means: we will filter the now complete former
        // picture. There might also be pictures to be output/read. After reading these pictures, this function
        // must be called again with the same NAL unit.
      }

      // Save the global variable
      d->md5_mismatch = g_md5_mismatch;
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
      TComSPS* activeSPS = d->decTop.getActiveSPS();
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
        pcPic->getOutputMark() && pcPic->getPOC() > d->iPOCLastDisplay && (d->numPicsNotYetDisplayed > d->numReorderPicsHighestTid || d->dpbFullness > d->maxDecPicBufferingHighestTid))
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
      // The normal output function is over, now let's flush
      d->flushOutput = true;
      d->sheduleFlushing = false;
      d->pcListPic_readIdx = 0;   // Iterate over all items again
      return libHMDec_get_picture(decCtx);
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

    if (c == LIBHMDEC_LUMA)
      return pcPic->getPicYuvRec()->getWidth(COMPONENT_Y);
    if (c == LIBHMDEC_CHROMA_U)
      return pcPic->getPicYuvRec()->getWidth(COMPONENT_Cb);
    if (c == LIBHMDEC_CHROMA_V)
      return pcPic->getPicYuvRec()->getWidth(COMPONENT_Cr);
    return -1;
  }
  HM_DEC_API int libHMDEC_get_picture_height(libHMDec_picture *pic, libHMDec_ColorComponent c)
  { 
    if (pic == NULL) 
      return -1;
    TComPic* pcPic = (TComPic*)pic;
    if (pcPic == NULL)
      return -1;

    if (c == LIBHMDEC_LUMA)
      return pcPic->getPicYuvRec()->getHeight(COMPONENT_Y);
    if (c == LIBHMDEC_CHROMA_U)
      return pcPic->getPicYuvRec()->getHeight(COMPONENT_Cb);
    if (c == LIBHMDEC_CHROMA_V)
      return pcPic->getPicYuvRec()->getHeight(COMPONENT_Cr);
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

    if (c == LIBHMDEC_LUMA)
      return pcPic->getPicYuvRec()->getAddr(COMPONENT_Y);
    if (c == LIBHMDEC_CHROMA_U)
      return pcPic->getPicYuvRec()->getAddr(COMPONENT_Cb);
    if (c == LIBHMDEC_CHROMA_V)
      return pcPic->getPicYuvRec()->getAddr(COMPONENT_Cr);
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


  HM_DEC_API int libHMDEC_get_internal_bit_depth(libHMDec_ColorComponent c)
  {
    if (c == LIBHMDEC_LUMA)
      return g_bitDepth[COMPONENT_Y];
    if (c == LIBHMDEC_CHROMA_U)
      return g_bitDepth[COMPONENT_Cb];
    if (c == LIBHMDEC_CHROMA_V)
      return g_bitDepth[COMPONENT_Cr];
    return -1;
  }

  // --------- internals --------

  void addValuesForPUs(hmDecoderWrapper *d, TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, libHMDec_info_type type)
  {
    PartSize ePartSize = pcCU->getPartitionSize( uiAbsPartIdx );
    UInt uiNumPU = ( ePartSize == SIZE_2Nx2N ? 1 : ( ePartSize == SIZE_NxN ? 4 : 2 ) );
    UInt uiPUOffset = ( g_auiPUOffset[UInt( ePartSize )] << ( ( pcCU->getSlice()->getSPS()->getMaxCUDepth() - uiDepth ) << 1 ) ) >> 4;

    const int cuWidth = g_uiMaxCUWidth >> uiDepth;
    const int cuHeight = g_uiMaxCUHeight >> uiDepth;
    const int cuX = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
    const int cuY = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];

    for (UInt uiPartIdx = 0, uiSubPartIdx = uiAbsPartIdx; uiPartIdx < uiNumPU; uiPartIdx++, uiSubPartIdx += uiPUOffset)
    {
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
      if (type == LIBHMDEC_PU_REFERENCE_POC_0)
        b.value = pcCU->getCUMvField(REF_PIC_LIST_0)->getRefIdx(uiSubPartIdx);
      if (type == LIBHMDEC_PU_MV_0)
      {
        b.value  = pcCU->getCUMvField(REF_PIC_LIST_0)->getMv(uiSubPartIdx).getHor();
        b.value2 = pcCU->getCUMvField(REF_PIC_LIST_0)->getMv(uiSubPartIdx).getVer();
      }
      if (type == LIBHMDEC_PU_REFERENCE_POC_1 && pcCU->getInterDir(uiSubPartIdx) == 2)
        b.value = pcCU->getCUMvField(REF_PIC_LIST_1)->getRefIdx(uiSubPartIdx);
      if (type == LIBHMDEC_PU_MV_1 && pcCU->getInterDir(uiSubPartIdx) == 2)
      {
        b.value  = pcCU->getCUMvField(REF_PIC_LIST_1)->getMv(uiSubPartIdx).getHor();
        b.value2 = pcCU->getCUMvField(REF_PIC_LIST_1)->getMv(uiSubPartIdx).getVer();
      }

      // Add the value
      d->internalsBlockData.push_back(b);
    }
  }

  void addValuesForTURecursive(hmDecoderWrapper *d, TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt trDepth, libHMDec_info_type type)
  {
    UInt trIdx = pcCU->getTransformIdx(uiAbsPartIdx);
    if (trDepth < trIdx)
    {
      // Split
      UInt uiNextDepth = uiDepth + trDepth + 1;
      UInt uiQNumParts = pcCU->getTotalNumPart() >> (uiNextDepth<<1);

      for (int i = 0; i < 4; i++)
        addValuesForTURecursive(d, pcCU, uiAbsPartIdx + i * uiQNumParts, uiDepth, trDepth + 1, type);
    }

    // We are not at the TU level
    UInt uiLPelX = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
    UInt uiTPelY = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];

    libHMDec_BlockValue b;
    b.x = uiLPelX;
    b.y = uiTPelY;
    b.w = (g_uiMaxCUWidth >> (uiDepth + trDepth));
    b.h = (g_uiMaxCUHeight >> (uiDepth + trDepth));
    if (type == LIBHMDEC_TU_CBF_Y)
      b.value = (pcCU->getCbf(uiAbsPartIdx, COMPONENT_Y, trDepth) != 0) ? 1 : 0;
    else if (type == LIBHMDEC_TU_CBF_CB)
      b.value = (pcCU->getCbf(uiAbsPartIdx, COMPONENT_Cb, trDepth) != 0) ? 1 : 0;
    else if (type == LIBHMDEC_TU_CBF_CR)
      b.value = (pcCU->getCbf(uiAbsPartIdx, COMPONENT_Cr, trDepth) != 0) ? 1 : 0;
    else if (type == LIBHMDEC_TU_COEFF_TR_SKIP_Y)
      b.value = (pcCU->getTransformSkip(uiAbsPartIdx, COMPONENT_Y) != 0) ? 1 : 0;
    else if (type == LIBHMDEC_TU_COEFF_TR_SKIP_Cb)
      b.value = (pcCU->getTransformSkip(uiAbsPartIdx, COMPONENT_Cb) != 0) ? 1 : 0;
    else if (type == LIBHMDEC_TU_COEFF_TR_SKIP_Cr)
      b.value = (pcCU->getTransformSkip(uiAbsPartIdx, COMPONENT_Cr) != 0) ? 1 : 0;
    else if (type == LIBHMDEC_TU_COEFF_ENERGY_Y)
      b.value = -1;
    else if (type == LIBHMDEC_TU_COEFF_ENERGY_CB)
      b.value = -1;
    else if (type == LIBHMDEC_TU_COEFF_ENERGY_CR)
      b.value = -1;
  }

  void addValuesForCURecursively(hmDecoderWrapper *d, TComDataCU* pcLCU, UInt uiAbsPartIdx, UInt uiDepth, libHMDec_info_type type)
  {
    TComPic* pcPic = pcLCU->getPic();

    Bool bBoundary = false;
    UInt uiLPelX   = pcLCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
    UInt uiRPelX   = uiLPelX + (g_uiMaxCUWidth >> uiDepth)  - 1;
    UInt uiTPelY   = pcLCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
    UInt uiBPelY   = uiTPelY + (g_uiMaxCUHeight >> uiDepth) - 1;

    UInt uiCurNumParts = pcPic->getNumPartInCU() >> (uiDepth<<1);
    TComSlice *pcSlice = pcLCU->getPic()->getSlice(pcLCU->getPic()->getCurrSliceIdx());
    Bool bStartInCU = (pcLCU->getSCUAddr() + uiAbsPartIdx + uiCurNumParts > pcSlice->getSliceSegmentCurStartCUAddr()) && (pcLCU->getSCUAddr() + uiAbsPartIdx < pcSlice->getSliceSegmentCurStartCUAddr());
    if (bStartInCU || (uiRPelX >= pcSlice->getSPS()->getPicWidthInLumaSamples()) || (uiBPelY >= pcSlice->getSPS()->getPicHeightInLumaSamples()))
      bBoundary = true;

    if(((uiDepth < pcLCU->getDepth(uiAbsPartIdx)) && (uiDepth < g_uiMaxCUDepth - g_uiAddCUDepth)) || bBoundary)
    {
      UInt uiNextDepth = uiDepth + 1;
      UInt uiQNumParts = pcLCU->getTotalNumPart() >> (uiNextDepth<<1);
      UInt uiIdx = uiAbsPartIdx;
      for (UInt uiPartIdx = 0; uiPartIdx < 4; uiPartIdx++)
      {
        uiLPelX = pcLCU->getCUPelX() + g_auiRasterToPelX[g_auiZscanToRaster[uiIdx]];
        uiTPelY = pcLCU->getCUPelY() + g_auiRasterToPelY[g_auiZscanToRaster[uiIdx]];

        if ((uiLPelX < pcSlice->getSPS()->getPicWidthInLumaSamples()) && (uiTPelY < pcSlice->getSPS()->getPicHeightInLumaSamples()))
          addValuesForCURecursively(d, pcLCU, uiIdx, uiNextDepth, type);

        uiIdx += uiQNumParts;
      }
      return;
    }

    // We reached the CU 
    if (type == LIBHMDEC_CU_PREDICTION_MODE || type == LIBHMDEC_CU_TRQ_BYPASS || type == LIBHMDEC_CU_SKIP_FLAG || type == LIBHMDEC_CU_PART_MODE || type == LIBHMDEC_CU_INTRA_MODE_LUMA || type == CHANNEL_TYPE_LUMA || type == LIBHMDEC_CU_INTRA_MODE_CHROMA || type == LIBHMDEC_CU_ROOT_CBF)
    {
      if ((type == LIBHMDEC_CU_TRQ_BYPASS && !pcLCU->getSlice()->getPPS()->getTransquantBypassEnableFlag()) ||
          (type == LIBHMDEC_CU_INTRA_MODE_LUMA && !pcLCU->isIntra(uiAbsPartIdx)) ||
          (type == LIBHMDEC_CU_INTRA_MODE_CHROMA && !pcLCU->isIntra(uiAbsPartIdx)) ||
          (type == LIBHMDEC_CU_ROOT_CBF && pcLCU->isInter(uiAbsPartIdx)))
        return;

      libHMDec_BlockValue b;
      b.x = uiLPelX;
      b.y = uiTPelY;
      b.w = (g_uiMaxCUWidth>>uiDepth);
      b.h = (g_uiMaxCUHeight>>uiDepth);
      if (type == LIBHMDEC_CU_PREDICTION_MODE)
        b.value = int(pcLCU->getPredictionMode(uiAbsPartIdx));
      else if (type == LIBHMDEC_CU_TRQ_BYPASS)
        b.value = pcLCU->getCUTransquantBypass(uiAbsPartIdx) ? 1 : 0;
      else if (type == LIBHMDEC_CU_SKIP_FLAG)
        b.value =  pcLCU->isSkipped(uiAbsPartIdx) ? 1 : 0;
      else if (type == LIBHMDEC_CU_PART_MODE)
        b.value = (int)pcLCU->getPartitionSize(uiAbsPartIdx);
      else if (type == LIBHMDEC_CU_INTRA_MODE_LUMA)
        b.value = (int)pcLCU->getIntraDir(CHANNEL_TYPE_LUMA);
      else if (type == LIBHMDEC_CU_INTRA_MODE_CHROMA)
        b.value = (int)pcLCU->getIntraDir(CHANNEL_TYPE_CHROMA);
      else if (type == LIBHMDEC_CU_ROOT_CBF)
        b.value = (int)pcLCU->getQtRootCbf(uiAbsPartIdx);
      d->internalsBlockData.push_back(b);
    }
    else if (pcLCU->isInter(uiAbsPartIdx) && (type == LIBHMDEC_PU_MERGE_FLAG || type == LIBHMDEC_PU_UNI_BI_PREDICTION || type == LIBHMDEC_PU_REFERENCE_POC_0 || type == LIBHMDEC_PU_MV_0 || type == LIBHMDEC_PU_REFERENCE_POC_1 || type == LIBHMDEC_PU_MV_1))
      // Set values for every PU
      addValuesForPUs(d, pcLCU, uiAbsPartIdx, uiDepth, type);
    else if (type == LIBHMDEC_TU_CBF_Y || type == LIBHMDEC_TU_CBF_CB || type == LIBHMDEC_TU_CBF_CR || type == LIBHMDEC_TU_COEFF_ENERGY_Y || type == LIBHMDEC_TU_COEFF_ENERGY_CB || type == LIBHMDEC_TU_COEFF_ENERGY_CR || type == LIBHMDEC_TU_COEFF_TR_SKIP_Y || type == LIBHMDEC_TU_COEFF_TR_SKIP_Cb || type == LIBHMDEC_TU_COEFF_TR_SKIP_Cr)
      addValuesForTURecursive(d, pcLCU, uiAbsPartIdx, uiDepth, 0, type);
  }

  HM_DEC_API std::vector<libHMDec_BlockValue> *libHMDEC_get_internal_info(libHMDec_context *decCtx, libHMDec_picture *pic, libHMDec_info_type type)
  {
    hmDecoderWrapper *d = (hmDecoderWrapper*)decCtx;
    if (!d)
      return NULL;
    
    // Clear the internals before adding new ones
    d->internalsBlockData.clear();

    if (pic == NULL)
      return NULL;
    TComPic* pcPic = (TComPic*)pic;
    if (pcPic == NULL)
      return NULL;
    TComPicSym *s = pcPic->getPicSym();
    if (s == NULL)
      return NULL;

    int nrCU = s->getNumberOfCUsInFrame();
    for (int i = 0; i < nrCU; i++)
    {
      TComDataCU *pcLCU = s->getCU(i);

      if ((type == LIBHMDEC_TU_COEFF_TR_SKIP_Y || type == LIBHMDEC_TU_COEFF_TR_SKIP_Cb || type == LIBHMDEC_TU_COEFF_TR_SKIP_Cr) && pcLCU->getSlice()->getPPS()->getUseTransformSkip())
        // Transform skip not enabled for this slice
        continue;
      
      if (type == LIBHMDEC_CTU_SLICE_INDEX)
      {
        libHMDec_BlockValue b;
        b.x = pcLCU->getCUPelX();
        b.y = pcLCU->getCUPelY();
        b.w = g_uiMaxCUWidth;
        b.h = g_uiMaxCUHeight;
        b.value = (int)pcLCU->getPic()->getCurrSliceIdx();
        d->internalsBlockData.push_back(b);
      }
      else
        addValuesForCURecursively(d, pcLCU, 0, 0, type);
    }

    return &d->internalsBlockData;
  }

  HM_DEC_API libHMDec_error libHMDEC_clear_internal_info(libHMDec_context *decCtx)
  {
    hmDecoderWrapper *d = (hmDecoderWrapper*)decCtx;
    if (!d)
      return LIBHMDEC_ERROR;

    // Clear the internals
    d->internalsBlockData.clear();

    return LIBHMDEC_OK;
  }

} // extern "C"