
#include "libHMDecoder.h"

#include "TLibCommon/CommonDef.h"
#include "TLibDecoder/TDecTop.h"
#include "TLibDecoder/NALread.h"

#include <vector>

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
    lastNALTemporalID = 0;
    flushOutput = false;

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
  int lastNALTemporalID;
  bool flushOutput;
  
  // The local memory for the global variable
  bool md5_mismatch;

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

    if (length <= 0 && !eof)
      return LIBHMDEC_ERROR_READ_ERROR;
    
    // Check the NAL unit header
    uint8_t *data = (uint8_t*)data8;
    if (length < 4)
      return LIBHMDEC_ERROR_READ_ERROR;

    // Do not copy the start code (if present)
    int copyStart = 0;
    if (data[0] == 0 && data[1] == 1 && data[2] == 1)
      copyStart = 3;
    else if (data[0] == 0 && data[1] == 0 && data[2] == 0 && data[3] == 1)
      copyStart = 4;

    // Recieve the NAL units data and put it into a vector so that we can parse it using the InputNALUnit.
    vector<uint8_t> nalUnit;
    if (!eof)
    {
      for (int i = 0; i < copyStart; i++)
        data++;
      for (int i = 0; i < length-copyStart; i++)
      {
        nalUnit.push_back(*data);
        data++;
      }
    }

    InputNALUnit nalu;
    if (!eof)
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
      if (!d->loopFiltered || eof)
      {
        int poc;
        d->decTop.executeLoopFilters(poc, d->pcListPic);
      }
      d->loopFiltered = (nalu.m_nalUnitType == NAL_UNIT_EOS);
    }

    // Check if we might be able to read pictures
    checkOutputPictures = false;
    d->flushOutput = false;
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
      d->flushOutput = true;
    }
    // write reconstruction to file
    if(bNewPicture)
    {
      checkOutputPictures = true;

      d->lastNALTemporalID = nalu.m_temporalId;

      // Calculate the number of not yet displayed pictures
      d->numPicsNotYetDisplayed = 0;
      TComList<TComPic*>::iterator iterPic = d->pcListPic->begin();
      while (iterPic != d->pcListPic->end())
      {
        TComPic* pcPic = *(iterPic);
        if(pcPic->getOutputMark() && pcPic->getPOC() > d->iPOCLastDisplay)
        {
          d->numPicsNotYetDisplayed++;
        }
        iterPic++;
      }
      iterPic = d->pcListPic->begin();
      if (d->numPicsNotYetDisplayed>2)
      {
        iterPic++;
      }
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

    TComPic* pcPic = *(iterPic);
    if (pcPic->isField())
      // TODO: Field output not supported (YET)
      return NULL;

    // Go on in the list until we run out of frames or find one that we can output
    while (iterPic != d->pcListPic->end())
    {
      if ((d->flushOutput && (pcPic->getOutputMark())) ||
        (pcPic->getOutputMark() && (d->numPicsNotYetDisplayed > pcPic->getNumReorderPics(d->lastNALTemporalID) && pcPic->getPOC() > d->iPOCLastDisplay)))
      {
        if (!d->flushOutput)
          // Output picture found
          d->numPicsNotYetDisplayed--;

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
      d->pcListPic->clear();
      d->iPOCLastDisplay = -MAX_INT;
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


  int libHMDEC_get_internal_bit_depth(libHMDec_ColorComponent c)
  {
    if (c == LIBHMDEC_LUMA)
      return g_bitDepth[COMPONENT_Y];
    if (c == LIBHMDEC_CHROMA_U)
      return g_bitDepth[COMPONENT_Cb];
    if (c == LIBHMDEC_CHROMA_V)
      return g_bitDepth[COMPONENT_Cr];
    return -1;
  }

} // extern "C"