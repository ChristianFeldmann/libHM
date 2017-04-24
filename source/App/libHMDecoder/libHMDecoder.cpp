
#include "libHMDecoder.h"

#include "TLibCommon/CommonDef.h"
#include "TLibDecoder/TDecTop.h"
#include "TLibDecoder/NALread.h"

#include <vector>

bool g_md5_mismatch = false; ///< top level flag that indicates if there has been a decoding mismatch
bool check_sei_hash = true;
bool loopFiltered = false;
int  maxTemporalLayer = -1; ///< maximum temporal layer to be decoded
int iPOCLastDisplay = -MAX_INT;
int iSkipFrame = 0;
int poc;
TComList<TComPic*>* pcListPic = NULL;
int pcListPic_readIdx = 0;
int numPicsNotYetDisplayed;
int lastNALTemporalID = 0;
bool flushOutput = false;

// TODO: (isNaluWithinTargetDecLayerIdSet) The target layer file is not supported (yet)
bool isNaluWithinTargetDecLayerIdSet(InputNALUnit* nalu) { return true; }

extern "C" {
  
  HM_DEC_API const char *libHMDec_get_version(void)
  {
    return NV_VERSION;
  }

  HM_DEC_API libHMDec_context* libHMDec_new_decoder(void)
  {
    TDecTop *decTop = new TDecTop();
    if (!decTop)
      return NULL;

    // Initialize the decoder
    decTop->create();
    decTop->init();
    decTop->setDecodedPictureHashSEIEnabled(check_sei_hash);
    iPOCLastDisplay += iSkipFrame; // set the last displayed POC correctly for skip forward.

    return (libHMDec_context*)decTop;
  }

  HM_DEC_API libHMDec_error libHMDec_free_decoder(libHMDec_context* decCtx)
  {
    TDecTop *decTop = (TDecTop*)decCtx;
    decTop->destroy();
    delete decTop;
    return LIBHMDEC_OK;
  }

  HM_DEC_API void libHMDec_set_SEI_Check(bool check_hash)
  {
    check_sei_hash = check_hash;
  }
  HM_DEC_API void libHMDec_set_max_temporal_layer(int max_layer)
  {
    maxTemporalLayer = max_layer;
  }

  HM_DEC_API libHMDec_error libHMDec_push_nal_unit(libHMDec_context *decCtx, const void* data8, int length, bool eof, bool &bNewPicture, bool &checkOutputPictures)
  {
    if (length <= 0 && !eof)
      return LIBHMDEC_ERROR_READ_ERROR;
    if (decCtx == NULL)
      return LIBHMDEC_ERROR;

    TDecTop *decTop = (TDecTop*)decCtx;
    if (decTop == NULL)
      return LIBHMDEC_ERROR;

    // Recieve the NAL units data and put it into a vector so that we can parse it using the InputNALUnit.
    vector<uint8_t> nalUnit;
    if (!eof)
    {
      uint8_t *data = (uint8_t*)data8;
      for (int i=0; i<length; i++)
      {
        nalUnit.push_back(*data);
        data++;
      }
    }

    InputNALUnit nalu;
    if (!eof)
      read(nalu, nalUnit);
    
    if( (maxTemporalLayer >= 0 && nalu.m_temporalId > maxTemporalLayer) || !isNaluWithinTargetDecLayerIdSet(&nalu)  )
    {
      bNewPicture = false;
    }
    else
    {
      bNewPicture = decTop->decode(nalu, iSkipFrame, iPOCLastDisplay);
      if (bNewPicture)
      {
        // We encountered a new picture in this NAL unit. This means: we will filter the now complete former
        // picture. There might also be pictures to be output/read. After reading these pictures, this function
        // must be called again with the same NAL unit.
      }
    }

    // Filter the picture if decoding is complete
    if (bNewPicture || eof || nalu.m_nalUnitType == NAL_UNIT_EOS)
    {
      if (!loopFiltered || eof)
      {
        decTop->executeLoopFilters(poc, pcListPic);
      }
      loopFiltered = (nalu.m_nalUnitType == NAL_UNIT_EOS);
    }

    // Check if we might be able to read pictures
    checkOutputPictures = false;
    flushOutput = false;
    if ( bNewPicture &&
      (   nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL
        || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP
        || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA_N_LP
        || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA_W_RADL
        || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA_W_LP ) )
    {
      checkOutputPictures = true;
      flushOutput = true;
    }
    if (nalu.m_nalUnitType == NAL_UNIT_EOS)
    {
      checkOutputPictures = true;
      flushOutput = true;
    }
    // write reconstruction to file
    if(bNewPicture)
    {
      checkOutputPictures = true;

      lastNALTemporalID = nalu.m_temporalId;

      // Calculate the number of not yet displayed pictures
      numPicsNotYetDisplayed = 0;
      TComList<TComPic*>::iterator iterPic = pcListPic->begin();
      while (iterPic != pcListPic->end())
      {
        TComPic* pcPic = *(iterPic);
        if(pcPic->getOutputMark() && pcPic->getPOC() > iPOCLastDisplay)
        {
          numPicsNotYetDisplayed++;
        }
        iterPic++;
      }
      iterPic   = pcListPic->begin();
      if (numPicsNotYetDisplayed>2)
      {
        iterPic++;
      }
    }
    
    if (checkOutputPictures)
      // Reset the iterator over the output images
      pcListPic_readIdx = 0;

    return LIBHMDEC_OK;
  }

  HM_DEC_API libHMDec_picture *libHMDec_get_picture()
  {
    if (!pcListPic)
      return NULL;
    if (pcListPic->size() == 0)
      return NULL;
    if (pcListPic_readIdx < 0 || pcListPic_readIdx > pcListPic->size())
      return NULL;

    // Get the pcListPic_readIdx-th picture from the list
    TComList<TComPic*>::iterator iterPic = pcListPic->begin();
    for (int i = 0; i < pcListPic_readIdx; i++)
      iterPic++;

    TComPic* pcPic = *(iterPic);
    if (pcPic->isField())
      // TODO: Field output not supported (YET)
      return NULL;

    // Go on in the list until we run out of frames or find one that we can output
    while (iterPic != pcListPic->end())
    {
      if ((flushOutput && (pcPic->getOutputMark())) ||
        (pcPic->getOutputMark() && (numPicsNotYetDisplayed > pcPic->getNumReorderPics(lastNALTemporalID) && pcPic->getPOC() > iPOCLastDisplay)))
      {
        if (!flushOutput)
          // Output picture found
          numPicsNotYetDisplayed--;

        // update POC of display order
        iPOCLastDisplay = pcPic->getPOC();

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
      pcListPic_readIdx++;
    }

    // We reached the end of the list wothout finding an output picture

    if (flushOutput)
    {
      pcListPic->clear();
      iPOCLastDisplay = -MAX_INT;
    }

    return NULL;
  }

  HM_DEC_API int libHMDEC_get_picture_width(libHMDec_picture *pic, libHMDec_ColorComponent c)
  { 
    if (pic == NULL) 
      return -1;
    TComPic* pcPic = (TComPic*)pic;
    if (pcPic == NULL)
      return -1;

    if (c == LIBHMDEC_LUMA)
      return pcPic->getPicYuvRec()->getWidth();
    if (c == LIBHMDEC_CHROMA_U || c == LIBHMDEC_CHROMA_V)
      return pcPic->getPicYuvRec()->getWidth() / 2;
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
      return pcPic->getPicYuvRec()->getHeight();
    if (c == LIBHMDEC_CHROMA_U || c == LIBHMDEC_CHROMA_V)
      return pcPic->getPicYuvRec()->getHeight() / 2;
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
      return pcPic->getPicYuvRec()->getStride();
    if (c == LIBHMDEC_CHROMA_U || c == LIBHMDEC_CHROMA_V)
      return pcPic->getPicYuvRec()->getStride() / 2;
    return -1;
  }

  HM_DEC_API uint8_t* libHMDEC_get_image_plane(libHMDec_picture *pic, libHMDec_ColorComponent c)
  {
    if (pic == NULL) 
      return NULL;
    TComPic* pcPic = (TComPic*)pic;
    if (pcPic == NULL)
      return NULL;

    if (c == LIBHMDEC_LUMA)
      return (uint8_t*)pcPic->getPicYuvRec()->getLumaAddr();
    if (c == LIBHMDEC_CHROMA_U)
      return (uint8_t*)pcPic->getPicYuvRec()->getCbAddr();
    if (c == LIBHMDEC_CHROMA_V)
      return (uint8_t*)pcPic->getPicYuvRec()->getCrAddr();
    return NULL;
  }

  int libHMDEC_get_internal_bit_depth(libHMDec_ColorComponent c)
  {
    if (c == LIBHMDEC_LUMA)
      return g_bitDepthY;
    if (c == LIBHMDEC_CHROMA_U || c == LIBHMDEC_CHROMA_V)
      return g_bitDepthC;
    return -1;
  }

} // extern "C"