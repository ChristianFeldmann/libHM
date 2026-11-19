/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2026, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "TLibCommon/CommonDef.h"
#include "TLibCommon/SEI.h"
#if JVET_AK0194_DSC_SEI
#include "TLibCommon/SEIDigitallySignedContent.h"
#endif
#include "TEncGOP.h"
#include "TEncTop.h"

//! \ingroup TLibEncoder
//! \{

Void SEIEncoder::initSEIActiveParameterSets (SEIActiveParameterSets *seiActiveParameterSets, const TComVPS *vps, const TComSPS *sps)
{
  assert (m_isInitialized);
  assert (seiActiveParameterSets!=NULL);
  assert (vps!=NULL);
  assert (sps!=NULL);

  seiActiveParameterSets->activeVPSId = vps->getVPSId(); 
  seiActiveParameterSets->m_selfContainedCvsFlag = false;
  seiActiveParameterSets->m_noParameterSetUpdateFlag = false;
  seiActiveParameterSets->numSpsIdsMinus1 = 0;
  seiActiveParameterSets->activeSeqParameterSetId.resize(seiActiveParameterSets->numSpsIdsMinus1 + 1);
  seiActiveParameterSets->activeSeqParameterSetId[0] = sps->getSPSId();
}

Void SEIEncoder::initSEIFramePacking(SEIFramePacking *seiFramePacking, Int currPicNum)
{
  assert (m_isInitialized);
  assert (seiFramePacking!=NULL);

  seiFramePacking->m_arrangementId = m_pcCfg->getFramePackingArrangementSEIId();
  seiFramePacking->m_arrangementCancelFlag = 0;
  seiFramePacking->m_arrangementType = m_pcCfg->getFramePackingArrangementSEIType();
  assert((seiFramePacking->m_arrangementType > 2) && (seiFramePacking->m_arrangementType < 6) );
  seiFramePacking->m_quincunxSamplingFlag = m_pcCfg->getFramePackingArrangementSEIQuincunx();
  seiFramePacking->m_contentInterpretationType = m_pcCfg->getFramePackingArrangementSEIInterpretation();
  seiFramePacking->m_spatialFlippingFlag = 0;
  seiFramePacking->m_frame0FlippedFlag = 0;
  seiFramePacking->m_fieldViewsFlag = (seiFramePacking->m_arrangementType == 2);
  seiFramePacking->m_currentFrameIsFrame0Flag = ((seiFramePacking->m_arrangementType == 5) && (currPicNum&1) );
  seiFramePacking->m_frame0SelfContainedFlag = 0;
  seiFramePacking->m_frame1SelfContainedFlag = 0;
  seiFramePacking->m_frame0GridPositionX = 0;
  seiFramePacking->m_frame0GridPositionY = 0;
  seiFramePacking->m_frame1GridPositionX = 0;
  seiFramePacking->m_frame1GridPositionY = 0;
  seiFramePacking->m_arrangementReservedByte = 0;
  seiFramePacking->m_arrangementPersistenceFlag = true;
  seiFramePacking->m_upsampledAspectRatio = 0;
}

Void SEIEncoder::initSEISegmentedRectFramePacking(SEISegmentedRectFramePacking *seiSegmentedRectFramePacking)
{
  assert (m_isInitialized);
  assert (seiSegmentedRectFramePacking!=NULL);

  seiSegmentedRectFramePacking->m_arrangementCancelFlag = m_pcCfg->getSegmentedRectFramePackingArrangementSEICancel();
  seiSegmentedRectFramePacking->m_contentInterpretationType = m_pcCfg->getSegmentedRectFramePackingArrangementSEIType();
  seiSegmentedRectFramePacking->m_arrangementPersistenceFlag = m_pcCfg->getSegmentedRectFramePackingArrangementSEIPersistence();
}

Void SEIEncoder::initSEIDisplayOrientation(SEIDisplayOrientation* seiDisplayOrientation)
{
  assert (m_isInitialized);
  assert (seiDisplayOrientation!=NULL);

  seiDisplayOrientation->cancelFlag = false;
  seiDisplayOrientation->horFlip = false;
  seiDisplayOrientation->verFlip = false;
  seiDisplayOrientation->anticlockwiseRotation = m_pcCfg->getDisplayOrientationSEIAngle();
}

Void SEIEncoder::initSEIToneMappingInfo(SEIToneMappingInfo *seiToneMappingInfo)
{
  assert (m_isInitialized);
  assert (seiToneMappingInfo!=NULL);

  seiToneMappingInfo->m_toneMapId = m_pcCfg->getTMISEIToneMapId();
  seiToneMappingInfo->m_toneMapCancelFlag = m_pcCfg->getTMISEIToneMapCancelFlag();
  seiToneMappingInfo->m_toneMapPersistenceFlag = m_pcCfg->getTMISEIToneMapPersistenceFlag();

  seiToneMappingInfo->m_codedDataBitDepth = m_pcCfg->getTMISEICodedDataBitDepth();
  assert(seiToneMappingInfo->m_codedDataBitDepth >= 8 && seiToneMappingInfo->m_codedDataBitDepth <= 14);
  seiToneMappingInfo->m_targetBitDepth = m_pcCfg->getTMISEITargetBitDepth();
  assert(seiToneMappingInfo->m_targetBitDepth >= 1 && seiToneMappingInfo->m_targetBitDepth <= 17);
  seiToneMappingInfo->m_modelId = m_pcCfg->getTMISEIModelID();
  assert(seiToneMappingInfo->m_modelId >=0 &&seiToneMappingInfo->m_modelId<=4);

  switch( seiToneMappingInfo->m_modelId)
  {
  case 0:
    {
      seiToneMappingInfo->m_minValue = m_pcCfg->getTMISEIMinValue();
      seiToneMappingInfo->m_maxValue = m_pcCfg->getTMISEIMaxValue();
      break;
    }
  case 1:
    {
      seiToneMappingInfo->m_sigmoidMidpoint = m_pcCfg->getTMISEISigmoidMidpoint();
      seiToneMappingInfo->m_sigmoidWidth = m_pcCfg->getTMISEISigmoidWidth();
      break;
    }
  case 2:
    {
      UInt num = 1u<<(seiToneMappingInfo->m_targetBitDepth);
      seiToneMappingInfo->m_startOfCodedInterval.resize(num);
      Int* ptmp = m_pcCfg->getTMISEIStartOfCodedInterva();
      if(ptmp)
      {
        for(Int i=0; i<num;i++)
        {
          seiToneMappingInfo->m_startOfCodedInterval[i] = ptmp[i];
        }
      }
      break;
    }
  case 3:
    {
      seiToneMappingInfo->m_numPivots = m_pcCfg->getTMISEINumPivots();
      seiToneMappingInfo->m_codedPivotValue.resize(seiToneMappingInfo->m_numPivots);
      seiToneMappingInfo->m_targetPivotValue.resize(seiToneMappingInfo->m_numPivots);
      Int* ptmpcoded = m_pcCfg->getTMISEICodedPivotValue();
      Int* ptmptarget = m_pcCfg->getTMISEITargetPivotValue();
      if(ptmpcoded&&ptmptarget)
      {
        for(Int i=0; i<(seiToneMappingInfo->m_numPivots);i++)
        {
          seiToneMappingInfo->m_codedPivotValue[i]=ptmpcoded[i];
          seiToneMappingInfo->m_targetPivotValue[i]=ptmptarget[i];
        }
      }
      break;
    }
  case 4:
    {
      seiToneMappingInfo->m_cameraIsoSpeedIdc = m_pcCfg->getTMISEICameraIsoSpeedIdc();
      seiToneMappingInfo->m_cameraIsoSpeedValue = m_pcCfg->getTMISEICameraIsoSpeedValue();
      assert( seiToneMappingInfo->m_cameraIsoSpeedValue !=0 );
      seiToneMappingInfo->m_exposureIndexIdc = m_pcCfg->getTMISEIExposurIndexIdc();
      seiToneMappingInfo->m_exposureIndexValue = m_pcCfg->getTMISEIExposurIndexValue();
      assert( seiToneMappingInfo->m_exposureIndexValue !=0 );
      seiToneMappingInfo->m_exposureCompensationValueSignFlag = m_pcCfg->getTMISEIExposureCompensationValueSignFlag();
      seiToneMappingInfo->m_exposureCompensationValueNumerator = m_pcCfg->getTMISEIExposureCompensationValueNumerator();
      seiToneMappingInfo->m_exposureCompensationValueDenomIdc = m_pcCfg->getTMISEIExposureCompensationValueDenomIdc();
      seiToneMappingInfo->m_refScreenLuminanceWhite = m_pcCfg->getTMISEIRefScreenLuminanceWhite();
      seiToneMappingInfo->m_extendedRangeWhiteLevel = m_pcCfg->getTMISEIExtendedRangeWhiteLevel();
      assert( seiToneMappingInfo->m_extendedRangeWhiteLevel >= 100 );
      seiToneMappingInfo->m_nominalBlackLevelLumaCodeValue = m_pcCfg->getTMISEINominalBlackLevelLumaCodeValue();
      seiToneMappingInfo->m_nominalWhiteLevelLumaCodeValue = m_pcCfg->getTMISEINominalWhiteLevelLumaCodeValue();
      assert( seiToneMappingInfo->m_nominalWhiteLevelLumaCodeValue > seiToneMappingInfo->m_nominalBlackLevelLumaCodeValue );
      seiToneMappingInfo->m_extendedWhiteLevelLumaCodeValue = m_pcCfg->getTMISEIExtendedWhiteLevelLumaCodeValue();
      assert( seiToneMappingInfo->m_extendedWhiteLevelLumaCodeValue >= seiToneMappingInfo->m_nominalWhiteLevelLumaCodeValue );
      break;
    }
  default:
    {
      assert(!"Undefined SEIToneMapModelId");
      break;
    }
  }
}

Void SEIEncoder::initSEISOPDescription(SEISOPDescription *sopDescriptionSEI, TComSlice *slice, Int picInGOP, Int lastIdr, Int currGOPSize)
{
  assert (m_isInitialized);
  assert (sopDescriptionSEI != NULL);
  assert (slice != NULL);

  Int sopCurrPOC = slice->getPOC();
  sopDescriptionSEI->m_sopSeqParameterSetId = slice->getSPS()->getSPSId();

  Int i = 0;
  Int prevEntryId = picInGOP;
  for (Int j = picInGOP; j < currGOPSize; j++)
  {
    Int deltaPOC = m_pcCfg->getGOPEntry(j).m_POC - m_pcCfg->getGOPEntry(prevEntryId).m_POC;
    if ((sopCurrPOC + deltaPOC) < m_pcCfg->getFramesToBeEncoded())
    {
      sopCurrPOC += deltaPOC;
      sopDescriptionSEI->m_sopDescVclNaluType[i] = m_pcEncGOP->getNalUnitType(sopCurrPOC, lastIdr, slice->getPic()->isField());
      sopDescriptionSEI->m_sopDescTemporalId[i] = m_pcCfg->getGOPEntry(j).m_temporalId;
      sopDescriptionSEI->m_sopDescStRpsIdx[i] = m_pcEncTop->getReferencePictureSetIdxForSOP(sopCurrPOC, j);
      sopDescriptionSEI->m_sopDescPocDelta[i] = deltaPOC;

      prevEntryId = j;
      i++;
    }
  }

  sopDescriptionSEI->m_numPicsInSopMinus1 = i - 1;
}

Void SEIEncoder::initSEIBufferingPeriod(SEIBufferingPeriod *bufferingPeriodSEI, TComSlice *slice)
{
  assert (m_isInitialized);
  assert (bufferingPeriodSEI != NULL);
  assert (slice != NULL);

  UInt uiInitialCpbRemovalDelay = (90000/2);                      // 0.5 sec
  bufferingPeriodSEI->m_initialCpbRemovalDelay      [0][0]     = uiInitialCpbRemovalDelay;
  bufferingPeriodSEI->m_initialCpbRemovalDelayOffset[0][0]     = uiInitialCpbRemovalDelay;
  bufferingPeriodSEI->m_initialCpbRemovalDelay      [0][1]     = uiInitialCpbRemovalDelay;
  bufferingPeriodSEI->m_initialCpbRemovalDelayOffset[0][1]     = uiInitialCpbRemovalDelay;

  Double dTmp = (Double)slice->getSPS()->getVuiParameters()->getTimingInfo()->getNumUnitsInTick() / (Double)slice->getSPS()->getVuiParameters()->getTimingInfo()->getTimeScale();

  UInt uiTmp = (UInt)( dTmp * 90000.0 );
  uiInitialCpbRemovalDelay -= uiTmp;
  uiInitialCpbRemovalDelay -= uiTmp / ( slice->getSPS()->getVuiParameters()->getHrdParameters()->getTickDivisorMinus2() + 2 );
  bufferingPeriodSEI->m_initialAltCpbRemovalDelay      [0][0]  = uiInitialCpbRemovalDelay;
  bufferingPeriodSEI->m_initialAltCpbRemovalDelayOffset[0][0]  = uiInitialCpbRemovalDelay;
  bufferingPeriodSEI->m_initialAltCpbRemovalDelay      [0][1]  = uiInitialCpbRemovalDelay;
  bufferingPeriodSEI->m_initialAltCpbRemovalDelayOffset[0][1]  = uiInitialCpbRemovalDelay;

  bufferingPeriodSEI->m_rapCpbParamsPresentFlag = 0;
  //for the concatenation, it can be set to one during splicing.
  bufferingPeriodSEI->m_concatenationFlag = 0;
  //since the temporal layer HRD is not ready, we assumed it is fixed
  bufferingPeriodSEI->m_auCpbRemovalDelayDelta = 1;
  bufferingPeriodSEI->m_cpbDelayOffset = 0;
  bufferingPeriodSEI->m_dpbDelayOffset = 0;
}

#if NNPFC_SEI_MESSAGE
Void SEIEncoder::initSEINeuralNetworkPostFilterCharacteristics(SEINeuralNetworkPostFilterCharacteristics *sei, Int filterIdx, const TComSlice *slice)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(sei != nullptr), "Unspecified error");
  sei->m_purpose = m_pcCfg->getNNPostFilterSEICharacteristicsPurpose(filterIdx);
  sei->m_id = m_pcCfg->getNNPostFilterSEICharacteristicsId(filterIdx);
  sei->m_baseFlag = m_pcCfg->getNNPostFilterSEICharacteristicsBaseFlag(filterIdx);
  sei->m_modeIdc = m_pcCfg->getNNPostFilterSEICharacteristicsModeIdc(filterIdx);
  if (sei->m_modeIdc == POST_FILTER_MODE::URI)
  {
    sei->m_uriTag = m_pcCfg->getNNPostFilterSEICharacteristicsUriTag(filterIdx);
    sei->m_uri    = m_pcCfg->getNNPostFilterSEICharacteristicsUri(filterIdx);
  }
  sei->m_propertyPresentFlag = m_pcCfg->getNNPostFilterSEICharacteristicsPropertyPresentFlag(filterIdx);
  if (sei->m_propertyPresentFlag)
  {
    sei->m_numberInputDecodedPicturesMinus1 = m_pcCfg->getNNPostFilterSEICharacteristicsNumberInputDecodedPicturesMinus1(filterIdx);
    CHECK(sei->m_numberInputDecodedPicturesMinus1 > 63, "m_numberInputDecodedPicturesMinus1 shall be in the range of 0 to 63");

    sei->m_inputPicOutputFlag = m_pcCfg->getNNPostFilterSEICharacteristicsInputPicOutputFlag(filterIdx);

    sei->m_absentInputPicZeroFlag = m_pcCfg->getNNPostFilterSEICharacteristicsAbsentInputPicZeroFlag(filterIdx);

    sei->m_numInpPicsInOutputTensor = 0;
    if (sei->m_numberInputDecodedPicturesMinus1 > 0)
    {
      for (UInt i = 0; i <= sei->m_numberInputDecodedPicturesMinus1; i++)
      {
        if (sei->m_inputPicOutputFlag[i])
        {
          sei->m_numInpPicsInOutputTensor++;
        }
      }
    }
    else
    {
      sei->m_numInpPicsInOutputTensor = 1;
    }

    if((sei->m_purpose & NNPC_PurposeType::CHROMA_UPSAMPLING) != 0)
    {
      sei->m_outSubCFlag = m_pcCfg->getNNPostFilterSEICharacteristicsOutSubCFlag(filterIdx);
    }
    if((sei->m_purpose & NNPC_PurposeType::COLOURIZATION) != 0)
    {
      sei->m_outColourFormatIdc = m_pcCfg->getNNPostFilterSEICharacteristicsOutColourFormatIdc(filterIdx);
    }

#if 0
    const ChromaFormat chromaFormatIdc = m_pcEncLib->getSPS(0)->getChromaFormatIdc();
#else
    const ChromaFormat chromaFormatIdc = slice->getSPS()->getChromaFormatIdc();
#endif
    UChar subWidthC     = TComSPS::getWinUnitX(chromaFormatIdc);
    UChar subHeightC    = TComSPS::getWinUnitY(chromaFormatIdc);

    UChar outSubWidthC  = subWidthC;
    UChar outSubHeightC = subHeightC;
    if ((sei->m_purpose & NNPC_PurposeType::CHROMA_UPSAMPLING) != 0)
    {
      if (sei->m_outSubCFlag)
      {
        outSubWidthC  = 1;
        outSubHeightC = 1;
      }
      else
      {
        outSubWidthC  = 2;
        outSubHeightC = 1;
      }
    }
    else if ((sei->m_purpose & NNPC_PurposeType::COLOURIZATION) != 0)
    {
      CHECK(sei->m_outColourFormatIdc == ChromaFormat::CHROMA_400, "The value of nnpfc_out_colour_format_idc shall not be equal to 0");
      outSubWidthC  = TComSPS::getWinUnitX(sei->m_outColourFormatIdc);
      outSubHeightC = TComSPS::getWinUnitY(sei->m_outColourFormatIdc);
    }

    if((sei->m_purpose & NNPC_PurposeType::RESOLUTION_UPSAMPLING) != 0)
    {
      sei->m_picWidthNumeratorMinus1 = m_pcCfg->getNNPostFilterSEICharacteristicsPicWidthNumeratorMinus1(filterIdx);
      sei->m_picWidthDenominatorMinus1 = m_pcCfg->getNNPostFilterSEICharacteristicsPicWidthDenominatorMinus1(filterIdx);
      sei->m_picHeightNumeratorMinus1 = m_pcCfg->getNNPostFilterSEICharacteristicsPicHeightNumeratorMinus1(filterIdx);
      sei->m_picHeightDenominatorMinus1 = m_pcCfg->getNNPostFilterSEICharacteristicsPicHeightDenominatorMinus1(filterIdx);
      CHECK(sei->m_picWidthNumeratorMinus1 > 65535, "nnpfc_pic_width_num_minus1 shall be in the range of 0 to 65535");
      CHECK(sei->m_picWidthDenominatorMinus1 > 65535, "nnpfc_pic_width_denom_minus1 shall be in the range of 0 to 65535");
      CHECK(sei->m_picHeightNumeratorMinus1 > 65535, "nnpfc_pic_height_num_minus1 shall be in the range of 0 to 65535");
      CHECK(sei->m_picHeightDenominatorMinus1 > 65535, "nnpfc_pic_height_denom_minus1 shall be in the range of 0 to 65535");
#if 0
      int confWinLeftOffset = m_pcEncLib->getPPS(0)->getConformanceWindow().getWindowLeftOffset();
      int confWinTopOffset = m_pcEncLib->getPPS(0)->getConformanceWindow().getWindowTopOffset();
      int confWinRightOffset = m_pcEncLib->getPPS(0)->getConformanceWindow().getWindowRightOffset();
      int confWinBottomOffset = m_pcEncLib->getPPS(0)->getConformanceWindow().getWindowBottomOffset();
      int ppsPicWidthInLumaSample  = m_pcEncLib->getPPS(0)->getPicWidthInLumaSamples();
      int ppsPicHeightInLumaSample = m_pcEncLib->getPPS(0)->getPicHeightInLumaSamples();
#else
      Int confWinLeftOffset = slice->getSPS()->getConformanceWindow().getWindowLeftOffset();
      Int confWinTopOffset = slice->getSPS()->getConformanceWindow().getWindowTopOffset();
      Int confWinRightOffset = slice->getSPS()->getConformanceWindow().getWindowRightOffset();
      Int confWinBottomOffset = slice->getSPS()->getConformanceWindow().getWindowBottomOffset();
      Int ppsPicWidthInLumaSample  = slice->getSPS()->getPicWidthInLumaSamples();
      Int ppsPicHeightInLumaSample = slice->getSPS()->getPicHeightInLumaSamples();
#endif

      
      Int croppedWidth = ppsPicWidthInLumaSample - subWidthC * (confWinRightOffset + confWinLeftOffset);
      Int croppedHeight = ppsPicHeightInLumaSample - subHeightC * (confWinBottomOffset + confWinTopOffset);
      Int outputPicWidth = (Int)ceil(((Double)croppedWidth * (sei->m_picWidthNumeratorMinus1 + 1)) / (sei->m_picWidthDenominatorMinus1 + 1));
      Int outputPicHeight = (Int)ceil(((Double)croppedHeight * (sei->m_picHeightNumeratorMinus1 + 1)) / (sei->m_picHeightDenominatorMinus1 + 1));

      CHECK(!(outputPicWidth >= croppedWidth && outputPicWidth <= croppedWidth * 16), "output picture width in luma samples shall be in the range of croppedWidth to croppedWidth * 16");
      CHECK(!(outputPicHeight >= croppedHeight && outputPicHeight <= croppedHeight * 16), "output picture height in luma samples shall be in the range of croppedHeight to croppedHeight * 16");

      CHECK((outputPicWidth  % outSubWidthC)  != 0, "The value of nnpfcOutputPicWidth % outSubWidthC shall be equal to 0");
      CHECK((outputPicHeight % outSubHeightC) != 0, "The value of nnpfcOutputPicHeight % outSubHeightC shall be equal to 0");
    }
    if((sei->m_purpose & NNPC_PurposeType::FRAME_RATE_UPSAMPLING) != 0)
    {
      sei->m_numberInterpolatedPictures = m_pcCfg->getNNPostFilterSEICharacteristicsNumberInterpolatedPictures(filterIdx);
      CHECK(sei->m_numberInputDecodedPicturesMinus1 <= 0, "If nnpfc_purpose is FRAME_RATE_UPSAMPLING, m_numberInputDecodedPicturesMinus1 shall be greater than 0");
    }
    if((sei->m_purpose & NNPC_PurposeType::TEMPORAL_EXTRAPOLATION) != 0)
    {
      sei->m_numberExtrapolatedPicturesMinus1 = m_pcCfg->getNNPostFilterSEICharacteristicsNumberExtrapolatedPicturesMinus1(filterIdx);
    }
    if((sei->m_purpose & NNPC_PurposeType::SPATIAL_EXTRAPOLATION) != 0)
    {
      sei->m_spatialExtrapolationLeftOffset = m_pcCfg->getNNPostFilterSEICharacteristicsSpatialExtrapolationLeftOffset(filterIdx);
      sei->m_spatialExtrapolationRightOffset = m_pcCfg->getNNPostFilterSEICharacteristicsSpatialExtrapolationRightOffset(filterIdx);
      sei->m_spatialExtrapolationTopOffset = m_pcCfg->getNNPostFilterSEICharacteristicsSpatialExtrapolationTopOffset(filterIdx);
      sei->m_spatialExtrapolationBottomOffset = m_pcCfg->getNNPostFilterSEICharacteristicsSpatialExtrapolationBottomOffset(filterIdx);
    }

    sei->m_componentLastFlag = m_pcCfg->getNNPostFilterSEICharacteristicsComponentLastFlag(filterIdx);
    sei->m_inpFormatIdc = m_pcCfg->getNNPostFilterSEICharacteristicsInpFormatIdc(filterIdx);
    CHECK(sei->m_inpFormatIdc > 255, "The value of nnpfc_inp_format_idc shall be in the range of 0 to 255");
    if (sei->m_inpFormatIdc == 1)
    {
      sei->m_inpTensorBitDepthLumaMinus8 = m_pcCfg->getNNPostFilterSEICharacteristicsInpTensorBitDepthLumaMinus8(filterIdx);
      sei->m_inpTensorBitDepthChromaMinus8 = m_pcCfg->getNNPostFilterSEICharacteristicsInpTensorBitDepthChromaMinus8(filterIdx);
    }

    sei->m_inpOrderIdc = m_pcCfg->getNNPostFilterSEICharacteristicsInpOrderIdc(filterIdx);
    CHECK((sei->m_purpose & NNPC_PurposeType::CHROMA_UPSAMPLING) != 0 && sei->m_inpOrderIdc == 0, "When nnpfc_purpose & 0x02 is not equal to 0, nnpfc_inp_order_idc shall not be equal to 0");
    sei->m_auxInpIdc             = m_pcCfg->getNNPostFilterSEICharacteristicsAuxInpIdc(filterIdx);

    if ((sei->m_auxInpIdc & 2) > 0)
    {
      sei->m_inbandPromptFlag = m_pcCfg->getNNPostFilterSEICharacteristicsInbandPromptFlag(filterIdx);
      if (sei->m_inbandPromptFlag)
      {
        sei->m_prompt = m_pcCfg->getNNPostFilterSEICharacteristicsPrompt(filterIdx);
      }
    }

    if ((sei->m_auxInpIdc & 4) > 0)
    {
      sei->m_inbandSeedFlag = m_pcCfg->getNNPostFilterSEICharacteristicsInbandSeedFlag(filterIdx);
      if (sei->m_inbandSeedFlag)
      {
        sei->m_seed = m_pcCfg->getNNPostFilterSEICharacteristicsSeed(filterIdx);
      }
    }

    sei->m_outFormatIdc = m_pcCfg->getNNPostFilterSEICharacteristicsOutFormatIdc(filterIdx);
    CHECK(sei->m_outFormatIdc > 255, "The value of nnpfc_out_format_idc shall be in the range of 0 to 255");
    if (sei->m_outFormatIdc == 1)
    {
      sei->m_outTensorBitDepthLumaMinus8 = m_pcCfg->getNNPostFilterSEICharacteristicsOutTensorBitDepthLumaMinus8(filterIdx);
      sei->m_outTensorBitDepthChromaMinus8 = m_pcCfg->getNNPostFilterSEICharacteristicsOutTensorBitDepthChromaMinus8(filterIdx);
    }
    sei->m_sepColDescriptionFlag = m_pcCfg->getNNPostFilterSEICharacteristicsSepColDescriptionFlag(filterIdx);
    if (sei->m_sepColDescriptionFlag)
    {
      sei->m_colPrimaries         = m_pcCfg->getNNPostFilterSEICharacteristicsColPrimaries(filterIdx);
      sei->m_transCharacteristics = m_pcCfg->getNNPostFilterSEICharacteristicsTransCharacteristics(filterIdx);
      if (sei->m_outFormatIdc == 1)
      {
        sei->m_matrixCoeffs         = m_pcCfg->getNNPostFilterSEICharacteristicsMatrixCoeffs(filterIdx);
        CHECK(sei->m_matrixCoeffs == 0 && !(sei->m_outTensorBitDepthChromaMinus8 == sei->m_outTensorBitDepthLumaMinus8 && sei->m_outOrderIdc == 2 && outSubHeightC == 1 && outSubWidthC == 1),
          "nnpfc_matrix_coeffs shall not be equal to 0 unless the following conditions are true: nnpfc_out_tensor_chroma_bitdepth_minus8 is equal to nnpfc_out_tensor_luma_bitdepth_minus8, nnpfc_out_order_idc is equal to 2, outSubHeightC is equal to 1, and outSubWidthC is equal to 1");
        CHECK(sei->m_matrixCoeffs == 8 && !((sei->m_outTensorBitDepthChromaMinus8 == sei->m_outTensorBitDepthLumaMinus8) || (sei->m_outTensorBitDepthChromaMinus8 == (sei->m_outTensorBitDepthLumaMinus8 + 1) && sei->m_outOrderIdc == 2 && outSubHeightC == 1 && outSubWidthC == 1)),
          "nnpfc_matrix_coeffs shall not be equal to 8 unless one of the following conditions is true: nnpfc_out_tensor_chroma_bitdepth_minus8 is equal to nnpfc_out_tensor_luma_bitdepth_minus8 or "
          "nnpfc_out_tensor_chroma_bitdepth_minus8 is equal to nnpfc_out_tensor_luma_bitdepth_minus8 + 1, nnpfc_out_order_idc is equal to 2, outSubHeightC is equal to 1, and outSubWidthC is equal to 1");
      }
    }
    if (sei->m_sepColDescriptionFlag && (sei->m_outFormatIdc == 1))
    {
      sei->m_fullRangeFlag = m_pcCfg->getNNPostFilterSEICharacteristicsFullRangeFlag(filterIdx);
    }
    sei->m_outOrderIdc = m_pcCfg->getNNPostFilterSEICharacteristicsOutOrderIdc(filterIdx);
    CHECK((sei->m_purpose & NNPC_PurposeType::CHROMA_UPSAMPLING) != 0 && (sei->m_outOrderIdc == 0 || sei->m_outOrderIdc == 3), "When nnpfc_purpose & 0x02 is not equal to 0, nnpfc_out_order_idc shall not be equal to 0 or 3");
    CHECK((sei->m_purpose & NNPC_PurposeType::COLOURIZATION) != 0 && sei->m_outOrderIdc == 0, "When nnpfc_purpose & 0x20 is not equal to 0, nnpfc_out_order_idc shall not be equal to 0");
    if(sei->m_outOrderIdc != 0)
    {
      sei->m_chromaLocInfoPresentFlag = m_pcCfg->getNNPostFilterSEICharacteristicsChromaLocInfoPresentFlag(filterIdx);
    }
    else
    {
      sei->m_chromaLocInfoPresentFlag = 0;
    }
      if(sei->m_chromaLocInfoPresentFlag)
      {
        sei->m_chromaSampleLocTypeFrame = m_pcCfg->getNNPostFilterSEICharacteristicsChromaSampleLocTypeFrame(filterIdx);;
      }
    sei->m_constantPatchSizeFlag = m_pcCfg->getNNPostFilterSEICharacteristicsConstantPatchSizeFlag(filterIdx);
    sei->m_patchWidthMinus1 = m_pcCfg->getNNPostFilterSEICharacteristicsPatchWidthMinus1(filterIdx);
    sei->m_patchHeightMinus1 = m_pcCfg->getNNPostFilterSEICharacteristicsPatchHeightMinus1(filterIdx);
    if (sei->m_constantPatchSizeFlag == 0)
    {
      sei->m_extendedPatchWidthCdDeltaMinus1 = m_pcCfg->getNNPostFilterSEICharacteristicsExtendedPatchWidthCdDeltaMinus1(filterIdx);
      sei->m_extendedPatchHeightCdDeltaMinus1 = m_pcCfg->getNNPostFilterSEICharacteristicsExtendedPatchHeightCdDeltaMinus1(filterIdx);
    }
    sei->m_overlap = m_pcCfg->getNNPostFilterSEICharacteristicsOverlap(filterIdx);
    sei->m_paddingType = m_pcCfg->getNNPostFilterSEICharacteristicsPaddingType(filterIdx);
    CHECK((sei->m_paddingType >= 5) && (sei->m_paddingType <= 15), "Reserved nnpfc_padding_type value, shall not be present in bitstreams conforming to this version of VTM");
    CHECK(sei->m_paddingType > 15, "Invalid nnpfc_padding_type value");
    sei->m_lumaPadding = m_pcCfg->getNNPostFilterSEICharacteristicsLumaPadding(filterIdx);
    sei->m_cbPadding = m_pcCfg->getNNPostFilterSEICharacteristicsCbPadding(filterIdx);
    sei->m_crPadding = m_pcCfg->getNNPostFilterSEICharacteristicsCrPadding(filterIdx);

    sei->m_complexityInfoPresentFlag = m_pcCfg->getNNPostFilterSEICharacteristicsComplexityInfoPresentFlag(filterIdx);
    if (sei->m_complexityInfoPresentFlag)
    {
        sei->m_parameterTypeIdc = m_pcCfg->getNNPostFilterSEICharacteristicsParameterTypeIdc(filterIdx);
        sei->m_log2ParameterBitLengthMinus3 = m_pcCfg->getNNPostFilterSEICharacteristicsLog2ParameterBitLengthMinus3(filterIdx);
        sei->m_numParametersIdc = m_pcCfg->getNNPostFilterSEICharacteristicsNumParametersIdc(filterIdx);
        sei->m_numKmacOperationsIdc = m_pcCfg->getNNPostFilterSEICharacteristicsNumKmacOperationsIdc(filterIdx);
        sei->m_totalKilobyteSize = m_pcCfg->getNNPostFilterSEICharacteristicsTotalKilobyteSize(filterIdx);
    }
    if (sei->m_purpose == 0)
    {
      sei->m_applicationPurposeTagUriPresentFlag = m_pcCfg->getNNPostFilterSEICharacteristicsApplicationPurposeTagUriPresentFlag(filterIdx);
      if (sei->m_applicationPurposeTagUriPresentFlag)
      {
        sei->m_applicationPurposeTagUri = m_pcCfg->getNNPostFilterSEICharacteristicsApplicationPurposeTagUri(filterIdx);
      }
    }
    if ((sei->m_purpose & NNPC_PurposeType::SPATIAL_EXTRAPOLATION) != 0 || (sei->m_purpose & NNPC_PurposeType::RESOLUTION_UPSAMPLING) != 0)
    {
      sei->m_scanTypeIdc = m_pcCfg->getNNPostFilterSEICharacteristicsScanTypeIdc(filterIdx);
    }
    sei->m_forHumanViewingIdc = m_pcCfg->getNNPostFilterSEICharacteristicsForHumanViewingIdc(filterIdx);
    sei->m_forMachineAnalysisIdc = m_pcCfg->getNNPostFilterSEICharacteristicsForMachineAnalysisIdc(filterIdx);
  }
  if (sei->m_modeIdc == POST_FILTER_MODE::ISO_IEC_15938_17)
  {
    const std::string payloadFilename = m_pcCfg->getNNPostFilterSEICharacteristicsPayloadFilename(filterIdx);
    std::ifstream     bitstreamFile(payloadFilename.c_str(), std::ifstream::in | std::ifstream::binary);
    if (!bitstreamFile)
    {
      EXIT( "Failed to open bitstream file " << payloadFilename.c_str() << " for reading" ) ;
    }

    bitstreamFile.seekg(0, std::ifstream::end);
    sei->m_payloadLength = bitstreamFile.tellg();
    bitstreamFile.seekg(0, std::ifstream::beg);

    sei->m_payloadByte = new TChar[sei->m_payloadLength];
    bitstreamFile.read(sei->m_payloadByte, sei->m_payloadLength);
    bitstreamFile.close();
  }
}
#endif

#if NNPFA_SEI_MESSAGE
Void SEIEncoder::initSEINeuralNetworkPostFilterActivation(SEINeuralNetworkPostFilterActivation *sei)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(sei != nullptr), "Unspecified error");
  sei->m_targetId = m_pcCfg->getNnPostFilterSEIActivationTargetId();
  sei->m_cancelFlag  = m_pcCfg->getNnPostFilterSEIActivationCancelFlag();
  if(!sei->m_cancelFlag)
  {
    sei->m_persistenceFlag = m_pcCfg->getNnPostFilterSEIActivationPersistenceFlag();
    sei->m_targetBaseFlag = m_pcCfg->getNnPostFilterSEIActivationTargetBaseFlag();
    sei->m_noPrevCLVSFlag = m_pcCfg->getNnPostFilterSEIActivationNoPrevCLVSFlag();
    sei->m_noFollCLVSFlag = m_pcCfg->getNnPostFilterSEIActivationNoFollCLVSFlag();
    sei->m_outputFlag = m_pcCfg->getNnPostFilterSEIActivationOutputFlag();
    sei->m_promptUpdateFlag = m_pcCfg->getNnPostFilterSEIActivationPromptUpdateFlag();
    if (sei->m_promptUpdateFlag)
    {
      sei->m_prompt = m_pcCfg->getNnPostFilterSEIActivationPrompt();
    }
    sei->m_seedUpdateFlag = m_pcCfg->getNnPostFilterSEIActivationSeedUpdateFlag();
    if (sei->m_seedUpdateFlag)
    {
      sei->m_seed = m_pcCfg->getNnPostFilterSEIActivationSeed();
    }
  }
}
#endif

#if JVET_AE0101_PHASE_INDICATION_SEI_MESSAGE
void SEIEncoder::initSEIPhaseIndication(SEIPhaseIndication* seiPhaseIndication)
{
  assert (m_isInitialized);
  assert (seiPhaseIndication != NULL);

  seiPhaseIndication->m_horPhaseNum = m_pcCfg->getHorPhaseNumFullResolution();
  seiPhaseIndication->m_horPhaseDenMinus1 = m_pcCfg->getHorPhaseDenMinus1FullResolution();
  seiPhaseIndication->m_verPhaseNum = m_pcCfg->getVerPhaseNumFullResolution();
  seiPhaseIndication->m_verPhaseDenMinus1 = m_pcCfg->getVerPhaseDenMinus1FullResolution();
}
#endif

#if JVET_AK0107_MODALITY_INFORMATION
Void SEIEncoder::initSEIModalityInfo(SEIModalityInfo *seiMI)
{
  assert(m_isInitialized);
  assert(seiMI != NULL);
  //  Set SEI message parameters read from command line options
  seiMI->m_miCancelFlag = m_pcCfg->getMiCancelFlag(); 
  if (!seiMI->m_miCancelFlag)
  {
    seiMI->m_miPersistenceFlag            = m_pcCfg->getMiPersistenceFlag();
    seiMI->m_miModalityType               = m_pcCfg->getMiModalityType();
    seiMI->m_miSpectrumRangePresentFlag   = m_pcCfg->getMiSpectrumRangePresentFlag();
    if (seiMI->m_miSpectrumRangePresentFlag)
    {
      seiMI->m_miMinWavelengthMantissa         = m_pcCfg->getMiMinWavelengthMantissa();
      seiMI->m_miMinWavelengthExponentPlus15   = m_pcCfg->getMiMinWavelengthExponentPlus15();
      seiMI->m_miMaxWavelengthMantissa         = m_pcCfg->getMiMaxWavelengthMantissa();
      seiMI->m_miMaxWavelengthExponentPlus15   = m_pcCfg->getMiMaxWavelengthExponentPlus15();
    }
  }
}
#endif

//! initialize scalable nesting SEI message.
//! Note: The SEI message structures input into this function will become part of the scalable nesting SEI and will be 
//!       automatically freed, when the nesting SEI is disposed.
Void SEIEncoder::initSEIScalableNesting(SEIScalableNesting *scalableNestingSEI, SEIMessages &nestedSEIs)
{
  assert (m_isInitialized);
  assert (scalableNestingSEI != NULL);

  scalableNestingSEI->m_bitStreamSubsetFlag           = 1;      // If the nested SEI messages are picture buffering SEI messages, picture timing SEI messages or sub-picture timing SEI messages, bitstream_subset_flag shall be equal to 1
  scalableNestingSEI->m_nestingOpFlag                 = 0;
  scalableNestingSEI->m_nestingNumOpsMinus1           = 0;      //nesting_num_ops_minus1
  scalableNestingSEI->m_allLayersFlag                 = 0;
  scalableNestingSEI->m_nestingNoOpMaxTemporalIdPlus1 = 6 + 1;  //nesting_no_op_max_temporal_id_plus1
  scalableNestingSEI->m_nestingNumLayersMinus1        = 1 - 1;  //nesting_num_layers_minus1
  scalableNestingSEI->m_nestingLayerId[0]             = 0;

  scalableNestingSEI->m_nestedSEIs.clear();
  for (SEIMessages::iterator it=nestedSEIs.begin(); it!=nestedSEIs.end(); it++)
  {
    scalableNestingSEI->m_nestedSEIs.push_back((*it));
  }
}

Void SEIEncoder::initSEIRecoveryPoint(SEIRecoveryPoint *recoveryPointSEI, TComSlice *slice)
{
  assert (m_isInitialized);
  assert (recoveryPointSEI != NULL);
  assert (slice != NULL);

  recoveryPointSEI->m_recoveryPocCnt    = 0;
  recoveryPointSEI->m_exactMatchingFlag = ( slice->getPOC() == 0 ) ? (true) : (false);
  recoveryPointSEI->m_brokenLinkFlag    = false;
}

//! calculate hashes for entire reconstructed picture
Void SEIEncoder::initDecodedPictureHashSEI(SEIDecodedPictureHash *decodedPictureHashSEI, TComPic *pcPic, std::string &rHashString, const BitDepths &bitDepths)
{
  assert (m_isInitialized);
  assert (decodedPictureHashSEI!=NULL);
  assert (pcPic!=NULL);

  decodedPictureHashSEI->method = m_pcCfg->getDecodedPictureHashSEIType();
  switch (m_pcCfg->getDecodedPictureHashSEIType())
  {
    case HASHTYPE_MD5:
      {
        UInt numChar=calcMD5(*pcPic->getPicYuvRec(), decodedPictureHashSEI->m_pictureHash, bitDepths);
        rHashString = hashToString(decodedPictureHashSEI->m_pictureHash, numChar);
      }
      break;
    case HASHTYPE_CRC:
      {
        UInt numChar=calcCRC(*pcPic->getPicYuvRec(), decodedPictureHashSEI->m_pictureHash, bitDepths);
        rHashString = hashToString(decodedPictureHashSEI->m_pictureHash, numChar);
      }
      break;
    case HASHTYPE_CHECKSUM:
    default:
      {
        UInt numChar=calcChecksum(*pcPic->getPicYuvRec(), decodedPictureHashSEI->m_pictureHash, bitDepths);
        rHashString = hashToString(decodedPictureHashSEI->m_pictureHash, numChar);
      }
      break;
  }
}

Void SEIEncoder::initTemporalLevel0IndexSEI(SEITemporalLevel0Index *temporalLevel0IndexSEI, TComSlice *slice)
{
  assert (m_isInitialized);
  assert (temporalLevel0IndexSEI!=NULL);
  assert (slice!=NULL);

  if (slice->getRapPicFlag())
  {
    m_tl0Idx = 0;
    m_rapIdx = (m_rapIdx + 1) & 0xFF;
  }
  else
  {
    m_tl0Idx = (m_tl0Idx + (slice->getTLayer() ? 0 : 1)) & 0xFF;
  }
  temporalLevel0IndexSEI->tl0Idx = m_tl0Idx;
  temporalLevel0IndexSEI->rapIdx = m_rapIdx;
}

Void SEIEncoder::initSEITempMotionConstrainedTileSets (SEITempMotionConstrainedTileSets *sei, const TComPPS *pps)
{
  assert (m_isInitialized);
  assert (sei!=NULL);
  assert (pps!=NULL);

  if(pps->getTilesEnabledFlag())
  {
#if MCTS_ENC_CHECK
    if (m_pcCfg->getTMCTSSEITileConstraint())
    {
      sei->m_mc_all_tiles_exact_sample_value_match_flag = true;
      sei->m_each_tile_one_tile_set_flag = true;
      sei->m_limited_tile_set_display_flag = false;
      sei->m_max_mcs_tier_level_idc_present_flag = false;
      sei->setNumberOfTileSets(0);
    }
    else
    {
#endif
      sei->m_mc_all_tiles_exact_sample_value_match_flag = false;
      sei->m_each_tile_one_tile_set_flag                = false;
      sei->m_limited_tile_set_display_flag              = false;
      sei->setNumberOfTileSets((pps->getNumTileColumnsMinus1() + 1) * (pps->getNumTileRowsMinus1() + 1));

      for(Int i=0; i < sei->getNumberOfTileSets(); i++)
      {
        sei->tileSetData(i).m_mcts_id = i;  //depends the application;
        sei->tileSetData(i).setNumberOfTileRects(1);

        for(Int j=0; j<sei->tileSetData(i).getNumberOfTileRects(); j++)
        {
          sei->tileSetData(i).topLeftTileIndex(j)     = i+j;
          sei->tileSetData(i).bottomRightTileIndex(j) = i+j;
        }

        sei->tileSetData(i).m_exact_sample_value_match_flag    = false;
        sei->tileSetData(i).m_mcts_tier_level_idc_present_flag = false;
      }
#if MCTS_ENC_CHECK
    }
#endif
  }
  else
  {
    assert(!"Tile is not enabled");
  }
}

#if MCTS_EXTRACTION && MCTS_ENC_CHECK
Void SEIEncoder::initSEIMCTSExtractionInfo(SEIMCTSExtractionInfoSet *sei, const TComVPS *vps, const TComSPS *sps, const TComPPS *pps)
{
  assert(m_isInitialized);
  assert(sei != NULL);
  assert(vps != NULL);
  assert(sps != NULL);
  assert(pps != NULL);

  TComVPS nestedVPS = *vps;
  TComSPS nestedSPS = *sps;
  TComPPS nestedPPS = *pps;

  if (pps->getTilesEnabledFlag())
  {
    if (m_pcCfg->getTMCTSSEITileConstraint())
    {
      UInt numTiles = (pps->getNumTileColumnsMinus1() + 1) * (pps->getNumTileRowsMinus1() + 1);
      UInt bottomTileRowHeightSampleOffset = ( sps->getPicHeightInLumaSamples() % sps->getMaxCUHeight() ) ? sps->getMaxCUHeight() - (sps->getPicHeightInLumaSamples() % sps->getMaxCUHeight()) : 0;
      UInt rightmostTileColumnWidthSampleOffset = ( sps->getPicWidthInLumaSamples() % sps->getMaxCUWidth() ) ? sps->getMaxCUWidth() - (sps->getPicWidthInLumaSamples() % sps->getMaxCUWidth()) : 0;
      TComPicSym *picSym = (*(m_pcEncTop->getListPic()->begin()))->getPicSym();

      // find MCTS that may share similar extraction info
      for ( UInt mctsIdx = 0; mctsIdx < numTiles; mctsIdx++) 
      {
        bool suitableEISfound = false;
        bool isRightmostTileColumn = (mctsIdx + 1) % (picSym->getNumTileColumnsMinus1() + 1) == 0 ;
        bool isBottomTileRow =  mctsIdx / (picSym->getNumTileColumnsMinus1() + 1) == picSym->getNumTileRowsMinus1();
        UInt curMctsHeight = picSym->getTComTile(mctsIdx)->getTileHeightInCtus() * sps->getMaxCUHeight() - (isBottomTileRow ? bottomTileRowHeightSampleOffset : 0);
        UInt curMctsWidth = picSym->getTComTile(mctsIdx)->getTileWidthInCtus() * sps->getMaxCUWidth() - (isRightmostTileColumn ? rightmostTileColumnWidthSampleOffset : 0);

        for (std::vector< SEIMCTSExtractionInfoSet::MCTSExtractionInfo>::iterator EISiter = sei->m_MCTSExtractionInfoSets.begin(); EISiter != sei->m_MCTSExtractionInfoSets.end() && !suitableEISfound; EISiter++)
        {
          if ( ( EISiter->mctsHeight == curMctsHeight) && ( EISiter->mctsWidth == curMctsWidth) )
          {
            EISiter->m_idxOfMctsInSet.push_back(std::vector<UInt>(1, mctsIdx ));
            suitableEISfound = true;
          }
        }
        if (!suitableEISfound)
        {
          SEIMCTSExtractionInfoSet::MCTSExtractionInfo newEIS;
          newEIS.mctsHeight = curMctsHeight;
          newEIS.mctsWidth = curMctsWidth;
          newEIS.m_idxOfMctsInSet.push_back(std::vector<UInt>(1, mctsIdx));
          sei->m_MCTSExtractionInfoSets.push_back(newEIS);
        }
      }

      // create parameter sets for each extraction info
      for (std::vector<SEIMCTSExtractionInfoSet::MCTSExtractionInfo>::iterator EISiter = sei->m_MCTSExtractionInfoSets.begin(); EISiter != sei->m_MCTSExtractionInfoSets.end(); EISiter++) 
      {
        EISiter->m_sliceReorderingEnabledFlag = false;

        TComOutputBitstream vps_rbsp;
        TComOutputBitstream sps_rbsp;
        TComOutputBitstream pps_rbsp;        
        
        nestedSPS.setPicHeightInLumaSamples(EISiter->mctsHeight);
        nestedSPS.setPicWidthInLumaSamples(EISiter->mctsWidth);
        nestedPPS.setTilesEnabledFlag(false);

        m_pcEncGOP->generateVPS_RBSP(&vps_rbsp, &nestedVPS);
        m_pcEncGOP->generateSPS_RBSP(&sps_rbsp, &nestedSPS);
        m_pcEncGOP->generatePPS_RBSP(&pps_rbsp, &nestedPPS);

        EISiter->m_vpsRbspData.resize(1);
        for (int j = 0; j < EISiter->m_vpsRbspData.size(); j++)
        {
          EISiter->m_vpsRbspDataLength.push_back(vps_rbsp.getByteStreamLength());
          EISiter->m_vpsRbspData[j] = vps_rbsp.getFIFO();
        }
        
        EISiter->m_spsRbspData.resize(1);
        for (int j = 0; j < EISiter->m_spsRbspData.size(); j++)
        {
          EISiter->m_spsRbspDataLength.push_back(sps_rbsp.getByteStreamLength());
          EISiter->m_spsRbspData[j] = sps_rbsp.getFIFO();
        }
        EISiter->m_ppsRbspData.resize(1);
        EISiter->m_ppsNuhTemporalIdPlus1.resize(1);
        for (int j = 0; j < EISiter->m_ppsRbspData.size(); j++)
        {
          EISiter->m_ppsRbspDataLength.push_back(pps_rbsp.getByteStreamLength());
          EISiter->m_ppsRbspData[j] = pps_rbsp.getFIFO();
          EISiter->m_ppsNuhTemporalIdPlus1[j] = 1;
        }
      }
    }
    else
    {
      assert(!"MCTS not activated");
    }
  }
  else
  {
    assert(!"Tiles is not enabled");
  }
}

#endif

Void SEIEncoder::initSEIKneeFunctionInfo(SEIKneeFunctionInfo *seiKneeFunctionInfo)
{
  assert (m_isInitialized);
  assert (seiKneeFunctionInfo!=NULL);

  const TEncCfg::TEncSEIKneeFunctionInformation &knee=m_pcCfg->getKneeFunctionInformationSEI();
  seiKneeFunctionInfo->m_kneeId = knee.m_kneeFunctionId;
  seiKneeFunctionInfo->m_kneeCancelFlag = knee.m_kneeFunctionCancelFlag;
  if ( !seiKneeFunctionInfo->m_kneeCancelFlag )
  {
    seiKneeFunctionInfo->m_kneePersistenceFlag = knee.m_kneeFunctionPersistenceFlag;
    seiKneeFunctionInfo->m_kneeInputDrange = knee.m_inputDRange;
    seiKneeFunctionInfo->m_kneeInputDispLuminance = knee.m_inputDispLuminance;
    seiKneeFunctionInfo->m_kneeOutputDrange = knee.m_outputDRange;
    seiKneeFunctionInfo->m_kneeOutputDispLuminance = knee.m_outputDispLuminance;

    assert(knee.m_kneeSEIKneePointPairs.size()>0);
    seiKneeFunctionInfo->m_kneeNumKneePointsMinus1 = (Int) knee.m_kneeSEIKneePointPairs.size()-1;
    seiKneeFunctionInfo->m_kneeInputKneePoint.resize(seiKneeFunctionInfo->m_kneeNumKneePointsMinus1+1);
    seiKneeFunctionInfo->m_kneeOutputKneePoint.resize(seiKneeFunctionInfo->m_kneeNumKneePointsMinus1+1);
    for(Int i=0; i<=seiKneeFunctionInfo->m_kneeNumKneePointsMinus1; i++)
    {
      seiKneeFunctionInfo->m_kneeInputKneePoint[i]  = knee.m_kneeSEIKneePointPairs[i].inputKneePoint;
      seiKneeFunctionInfo->m_kneeOutputKneePoint[i] = knee.m_kneeSEIKneePointPairs[i].outputKneePoint;
    }
  }
}
Void SEIEncoder::initSEIContentColourVolume(SEIContentColourVolume *seiContentColourVolume)
{
  assert(m_isInitialized);
  assert(seiContentColourVolume != NULL);
  seiContentColourVolume->m_ccvCancelFlag = m_pcCfg->getCcvSEICancelFlag();
  seiContentColourVolume->m_ccvPersistenceFlag = m_pcCfg->getCcvSEIPersistenceFlag();
  
  seiContentColourVolume->m_ccvPrimariesPresentFlag = m_pcCfg->getCcvSEIPrimariesPresentFlag();
  seiContentColourVolume->m_ccvMinLuminanceValuePresentFlag = m_pcCfg->getCcvSEIMinLuminanceValuePresentFlag();
  seiContentColourVolume->m_ccvMaxLuminanceValuePresentFlag = m_pcCfg->getCcvSEIMaxLuminanceValuePresentFlag();
  seiContentColourVolume->m_ccvAvgLuminanceValuePresentFlag = m_pcCfg->getCcvSEIAvgLuminanceValuePresentFlag();

  // Currently we are using a floor operation for setting up the "integer" values for this SEI.
  // This applies to both primaries and luminance limits.
  if (seiContentColourVolume->m_ccvPrimariesPresentFlag == true) 
  {
    for (Int i = 0; i < MAX_NUM_COMPONENT; i++) 
    {
      seiContentColourVolume->m_ccvPrimariesX[i] = (Int) (50000.0 * m_pcCfg->getCcvSEIPrimariesX(i));
      seiContentColourVolume->m_ccvPrimariesY[i] = (Int) (50000.0 * m_pcCfg->getCcvSEIPrimariesY(i));
    }
  }
  
  if (seiContentColourVolume->m_ccvMinLuminanceValuePresentFlag == true)
  {
    seiContentColourVolume->m_ccvMinLuminanceValue = (Int) (10000000 * m_pcCfg->getCcvSEIMinLuminanceValue());
  }
  if (seiContentColourVolume->m_ccvMaxLuminanceValuePresentFlag == true)
  {
    seiContentColourVolume->m_ccvMaxLuminanceValue = (Int) (10000000 * m_pcCfg->getCcvSEIMaxLuminanceValue());
  }
  if (seiContentColourVolume->m_ccvAvgLuminanceValuePresentFlag == true)
  {
    seiContentColourVolume->m_ccvAvgLuminanceValue = (Int) (10000000 * m_pcCfg->getCcvSEIAvgLuminanceValue());
  }
}
#if JVET_AL0061_ENCODER_OPTIMIZATION_INFORMATION_SEI
void SEIEncoder::initSEIEncoderOptimizationInfo(SEIEncoderOptimizationInfo *sei)
{
  sei->m_cancelFlag = m_pcCfg->getEOISEICancelFlag();
  if (!sei->m_cancelFlag)
  {
    sei->m_persistenceFlag = m_pcCfg->getEOISEIPersistenceFlag();
    sei->m_forHumanViewingIdc = m_pcCfg->getEOISEIForHumanViewingIdc();
    sei->m_forMachineAnalysisIdc = m_pcCfg->getEOISEIForMachineAnalysisIdc();
    sei->m_type = m_pcCfg->getEOISEIType();
    if ((sei->m_type & EOI_OptimizationType::OBJECT_BASED_OPTIMIZATION) != 0)
    {
      sei->m_objectBasedIdc = m_pcCfg->getEOISEIObjectBasedIdc();
      if (sei->m_objectBasedIdc & EOI_OBJECT_BASED::COARSER_QUANTIZATION)
      {
        sei->m_quantThresholdDelta = m_pcCfg->getEOISEIQuantThresholdDelta();
        if (sei->m_quantThresholdDelta > 0)
        {
          sei->m_picQuantObjectFlag = m_pcCfg->getEOISEIPicQuantObjectFlag();
        }
      }
    }
    if ((sei->m_type & EOI_OptimizationType::TEMPORAL_RESAMPLING) != 0)
    {
      sei->m_temporalResamplingTypeFlag = m_pcCfg->getEOISEITemporalResamplingTypeFlag();
      sei->m_numIntPics = m_pcCfg->getEOISEINumIntPics();
      if (sei->m_temporalResamplingTypeFlag && sei->m_numIntPics > 0)
      {
        sei->m_srcPicFlag = m_pcCfg->getEOISEISrcPicFlag();
      }
    }
    if ((sei->m_type & EOI_OptimizationType::SPATIAL_RESAMPLING) != 0)
    {
      sei->m_origPicDimensionsFlag = m_pcCfg->getEOISEIOrigPicDimensionsFlag();
      if (sei->m_origPicDimensionsFlag)
      {
        sei->m_origPicWidth = m_pcCfg->getEOISEIOrigPicWidth();
        sei->m_origPicHeight = m_pcCfg->getEOISEIOrigPicHeight();
      }
      else
      {
        sei->m_spatialResamplingTypeFlag = m_pcCfg->getEOISEISpatialResamplingTypeFlag();
      }
    }
    if ((sei->m_type & EOI_OptimizationType::PRIVACY_PROTECTION_OPTIMIZATION) != 0)
    {
      sei->m_privacyProtectionTypeIdc = m_pcCfg->getEOISEIPrivacyProtectionTypeIdc();
      sei->m_privacyProtectedInfoType = m_pcCfg->getEOISEIPrivacyProtectedInfoType();
    }
  }
}
#endif
#if SHUTTER_INTERVAL_SEI_MESSAGE
Void SEIEncoder::initSEIShutterIntervalInfo(SEIShutterIntervalInfo *seiShutterIntervalInfo)
{
  assert(m_isInitialized);
  assert(seiShutterIntervalInfo != NULL);
  seiShutterIntervalInfo->m_siiTimeScale = m_pcCfg->getSiiSEITimeScale();
  seiShutterIntervalInfo->m_siiFixedSIwithinCLVS = m_pcCfg->getSiiSEIFixedSIwithinCLVS();
  if (seiShutterIntervalInfo->m_siiFixedSIwithinCLVS == true)
  {
    seiShutterIntervalInfo->m_siiNumUnitsInShutterInterval = m_pcCfg->getSiiSEINumUnitsInShutterInterval();
  }
  else
  {
    seiShutterIntervalInfo->m_siiMaxSubLayersMinus1 = m_pcCfg->getSiiSEIMaxSubLayersMinus1();
    seiShutterIntervalInfo->m_siiSubLayerNumUnitsInSI.resize(seiShutterIntervalInfo->m_siiMaxSubLayersMinus1+1);
    for (Int i = 0; i <= seiShutterIntervalInfo->m_siiMaxSubLayersMinus1; i++)
    {
      seiShutterIntervalInfo->m_siiSubLayerNumUnitsInSI[i] = m_pcCfg->getSiiSEISubLayerNumUnitsInSI(i);
    }
  }
}
#endif

#if SEI_ENCODER_CONTROL
Void SEIEncoder::initSEIFilmGrainCharacteristics(SEIFilmGrainCharacteristics *seiFilmGrain)
{
  assert(m_isInitialized);
  assert(seiFilmGrain != NULL);
  //  Set SEI message parameters read from command line options
  seiFilmGrain->m_filmGrainCharacteristicsCancelFlag      = m_pcCfg->getFilmGrainCharactersticsSEICancelFlag();
  seiFilmGrain->m_filmGrainCharacteristicsPersistenceFlag = m_pcCfg->getFilmGrainCharactersticsSEIPersistenceFlag();
  seiFilmGrain->m_filmGrainModelId                        = m_pcCfg->getFilmGrainCharactersticsSEIModelID();
  seiFilmGrain->m_separateColourDescriptionPresentFlag    = m_pcCfg->getFilmGrainCharactersticsSEISepColourDescPresent();
  seiFilmGrain->m_blendingModeId                          = m_pcCfg->getFilmGrainCharactersticsSEIBlendingModeID();
  seiFilmGrain->m_log2ScaleFactor                         = m_pcCfg->getFilmGrainCharactersticsSEILog2ScaleFactor();
#if JVET_AL0339_SPATIAL_RESOLUTION_FOR_FGC_SEI
  seiFilmGrain->m_fgPicWidthInLumaSamples                 = m_pcCfg->getFilmGrainCharactersticsSEIPicWidthInLumaSamples();
  seiFilmGrain->m_fgPicHeightInLumaSamples                = m_pcCfg->getFilmGrainCharactersticsSEIPicHeightInLumaSamples();
  if (seiFilmGrain->m_fgPicWidthInLumaSamples > 0 || seiFilmGrain->m_fgPicHeightInLumaSamples > 0)
  {
    seiFilmGrain->m_fgSpatialResolutionPresentFlag = 1;
  }
#endif
  for (int i = 0; i < MAX_NUM_COMPONENT; i++)
  {
    seiFilmGrain->m_compModel[i].bPresentFlag = m_pcCfg->getFGCSEICompModelPresent(i);
#if JVET_X0048_X0103_FILM_GRAIN
    if (seiFilmGrain->m_compModel[i].bPresentFlag)
    {
      seiFilmGrain->m_compModel[i].numModelValues = 1 + m_pcCfg->getFGCSEINumModelValuesMinus1(i);
      seiFilmGrain->m_compModel[i].numIntensityIntervals = 1 + m_pcCfg->getFGCSEINumIntensityIntervalMinus1(i);
      seiFilmGrain->m_compModel[i].intensityValues.resize(seiFilmGrain->m_compModel[i].numIntensityIntervals);
      for (UInt j = 0; j < seiFilmGrain->m_compModel[i].numIntensityIntervals; j++)
      {
        seiFilmGrain->m_compModel[i].intensityValues[j].intensityIntervalLowerBound = m_pcCfg->getFGCSEIIntensityIntervalLowerBound(i, j);
        seiFilmGrain->m_compModel[i].intensityValues[j].intensityIntervalUpperBound = m_pcCfg->getFGCSEIIntensityIntervalUpperBound(i, j);
        seiFilmGrain->m_compModel[i].intensityValues[j].compModelValue.resize(seiFilmGrain->m_compModel[i].numModelValues);
        for (UInt k = 0; k < seiFilmGrain->m_compModel[i].numModelValues; k++)
        {
          seiFilmGrain->m_compModel[i].intensityValues[j].compModelValue[k] = m_pcCfg->getFGCSEICompModelValue(i, j, k);
        }
      }
    }
#endif
  }
}

Void SEIEncoder::initSEIContentLightLevel(SEIContentLightLevelInfo *seiCLL)
{
  assert(m_isInitialized);
  assert(seiCLL != NULL);
  //  Set SEI message parameters read from command line options
  seiCLL->m_maxContentLightLevel = m_pcCfg->getCLLSEIMaxContentLightLevel();
  seiCLL->m_maxPicAverageLightLevel = m_pcCfg->getCLLSEIMaxPicAvgLightLevel();
}

Void SEIEncoder::initSEIAmbientViewingEnvironment(SEIAmbientViewingEnvironment *seiAmbViewEnvironment)
{
  assert(m_isInitialized);
  assert(seiAmbViewEnvironment != NULL);
  //  Set SEI message parameters read from command line options
  seiAmbViewEnvironment->m_ambientIlluminance = m_pcCfg->getAmbientViewingEnvironmentSEIIlluminance();
  seiAmbViewEnvironment->m_ambientLightX = m_pcCfg->getAmbientViewingEnvironmentSEIAmbientLightX();
  seiAmbViewEnvironment->m_ambientLightY = m_pcCfg->getAmbientViewingEnvironmentSEIAmbientLightY();
}
#endif

Void SEIEncoder::initSEIErp(SEIEquirectangularProjection* seiEquirectangularProjection)
{
  assert (m_isInitialized);
  assert (seiEquirectangularProjection!=NULL);

  seiEquirectangularProjection->m_erpCancelFlag = m_pcCfg->getErpSEICancelFlag();
  if (!seiEquirectangularProjection->m_erpCancelFlag)
  {
    seiEquirectangularProjection->m_erpPersistenceFlag   = m_pcCfg->getErpSEIPersistenceFlag();
    seiEquirectangularProjection->m_erpGuardBandFlag     = m_pcCfg->getErpSEIGuardBandFlag();
    if (seiEquirectangularProjection->m_erpGuardBandFlag == 1)
    {
      seiEquirectangularProjection->m_erpGuardBandType       = m_pcCfg->getErpSEIGuardBandType();
      seiEquirectangularProjection->m_erpLeftGuardBandWidth  = m_pcCfg->getErpSEILeftGuardBandWidth();
      seiEquirectangularProjection->m_erpRightGuardBandWidth = m_pcCfg->getErpSEIRightGuardBandWidth();
    }
  }
}

Void SEIEncoder::initSEISphereRotation(SEISphereRotation* seiSphereRotation)
{
  assert (m_isInitialized);
  assert (seiSphereRotation!=NULL);

  seiSphereRotation->m_sphereRotationCancelFlag = m_pcCfg->getSphereRotationSEICancelFlag();
  if ( !seiSphereRotation->m_sphereRotationCancelFlag )
  {
    seiSphereRotation->m_sphereRotationPersistenceFlag = m_pcCfg->getSphereRotationSEIPersistenceFlag();
    seiSphereRotation->m_sphereRotationYaw = m_pcCfg->getSphereRotationSEIYaw();
    seiSphereRotation->m_sphereRotationPitch = m_pcCfg->getSphereRotationSEIPitch();
    seiSphereRotation->m_sphereRotationRoll = m_pcCfg->getSphereRotationSEIRoll();
  }
}

Void SEIEncoder::initSEIOmniViewport(SEIOmniViewport* seiOmniViewport)
{
  assert (m_isInitialized);
  assert (seiOmniViewport!=NULL);

  seiOmniViewport->m_omniViewportId = m_pcCfg->getOmniViewportSEIId();
  seiOmniViewport->m_omniViewportCancelFlag = m_pcCfg->getOmniViewportSEICancelFlag();
  if ( !seiOmniViewport->m_omniViewportCancelFlag )
  {
    seiOmniViewport->m_omniViewportPersistenceFlag = m_pcCfg->getOmniViewportSEIPersistenceFlag();
    seiOmniViewport->m_omniViewportCntMinus1 = m_pcCfg->getOmniViewportSEICntMinus1();

    seiOmniViewport->m_omniViewportRegions.resize(seiOmniViewport->m_omniViewportCntMinus1+1);
    for (UInt i = 0; i <= seiOmniViewport->m_omniViewportCntMinus1; i++)
    {
      SEIOmniViewport::OmniViewport &viewport = seiOmniViewport->m_omniViewportRegions[i];
      viewport.azimuthCentre   = m_pcCfg->getOmniViewportSEIAzimuthCentre(i);
      viewport.elevationCentre = m_pcCfg->getOmniViewportSEIElevationCentre(i);
      viewport.tiltCentre      = m_pcCfg->getOmniViewportSEITiltCentre(i);
      viewport.horRange        = m_pcCfg->getOmniViewportSEIHorRange(i);
      viewport.verRange        = m_pcCfg->getOmniViewportSEIVerRange(i);
    }
  }
}

Void SEIEncoder::initSEICubemapProjection(SEICubemapProjection *seiCubemapProjection)
{
  assert(m_isInitialized);
  assert(seiCubemapProjection != NULL);
  seiCubemapProjection->m_cmpCancelFlag = m_pcCfg->getCmpSEICmpCancelFlag();
  seiCubemapProjection->m_cmpPersistenceFlag = m_pcCfg->getCmpSEICmpPersistenceFlag();
}

Void SEIEncoder::initSEIRegionWisePacking(SEIRegionWisePacking *seiRegionWisePacking)
{
  assert (m_isInitialized);
  assert (seiRegionWisePacking!=NULL);
  seiRegionWisePacking->m_rwpCancelFlag                          = m_pcCfg->getRwpSEIRwpCancelFlag();
  seiRegionWisePacking->m_rwpPersistenceFlag                     = m_pcCfg->getRwpSEIRwpPersistenceFlag();
  seiRegionWisePacking->m_constituentPictureMatchingFlag         = m_pcCfg->getRwpSEIConstituentPictureMatchingFlag();
  seiRegionWisePacking->m_numPackedRegions                       = m_pcCfg->getRwpSEINumPackedRegions();
  seiRegionWisePacking->m_projPictureWidth                       = m_pcCfg->getRwpSEIProjPictureWidth();
  seiRegionWisePacking->m_projPictureHeight                      = m_pcCfg->getRwpSEIProjPictureHeight();
  seiRegionWisePacking->m_packedPictureWidth                     = m_pcCfg->getRwpSEIPackedPictureWidth();
  seiRegionWisePacking->m_packedPictureHeight                    = m_pcCfg->getRwpSEIPackedPictureHeight();
  seiRegionWisePacking->m_rwpTransformType.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpGuardBandFlag.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_projRegionWidth.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_projRegionHeight.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpProjRegionTop.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_projRegionLeft.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_packedRegionWidth.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_packedRegionHeight.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_packedRegionTop.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_packedRegionLeft.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpLeftGuardBandWidth.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpRightGuardBandWidth.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpTopGuardBandHeight.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpBottomGuardBandHeight.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpGuardBandNotUsedForPredFlag.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpGuardBandType.resize(4*seiRegionWisePacking->m_numPackedRegions); 
  for( Int i=0; i < seiRegionWisePacking->m_numPackedRegions; i++ )
  {
    seiRegionWisePacking->m_rwpTransformType[i]                  = m_pcCfg->getRwpSEIRwpTransformType(i);
    seiRegionWisePacking->m_rwpGuardBandFlag[i]                  = m_pcCfg->getRwpSEIRwpGuardBandFlag(i);
    seiRegionWisePacking->m_projRegionWidth[i]                   = m_pcCfg->getRwpSEIProjRegionWidth(i);
    seiRegionWisePacking->m_projRegionHeight[i]                  = m_pcCfg->getRwpSEIProjRegionHeight(i);
    seiRegionWisePacking->m_rwpProjRegionTop[i]                  = m_pcCfg->getRwpSEIRwpSEIProjRegionTop(i);
    seiRegionWisePacking->m_projRegionLeft[i]                    = m_pcCfg->getRwpSEIProjRegionLeft(i);
    seiRegionWisePacking->m_packedRegionWidth[i]                 = m_pcCfg->getRwpSEIPackedRegionWidth(i);
    seiRegionWisePacking->m_packedRegionHeight[i]                = m_pcCfg->getRwpSEIPackedRegionHeight(i);
    seiRegionWisePacking->m_packedRegionTop[i]                   = m_pcCfg->getRwpSEIPackedRegionTop(i);
    seiRegionWisePacking->m_packedRegionLeft[i]                  = m_pcCfg->getRwpSEIPackedRegionLeft(i);
    if( seiRegionWisePacking->m_rwpGuardBandFlag[i] )
    {
      seiRegionWisePacking->m_rwpLeftGuardBandWidth[i]           =  m_pcCfg->getRwpSEIRwpLeftGuardBandWidth(i);
      seiRegionWisePacking->m_rwpRightGuardBandWidth[i]          =  m_pcCfg->getRwpSEIRwpRightGuardBandWidth(i);
      seiRegionWisePacking->m_rwpTopGuardBandHeight[i]           =  m_pcCfg->getRwpSEIRwpTopGuardBandHeight(i);
      seiRegionWisePacking->m_rwpBottomGuardBandHeight[i]        =  m_pcCfg->getRwpSEIRwpBottomGuardBandHeight(i);
      seiRegionWisePacking->m_rwpGuardBandNotUsedForPredFlag[i]  =  m_pcCfg->getRwpSEIRwpGuardBandNotUsedForPredFlag(i);
      for( Int j=0; j < 4; j++ )
      {
        seiRegionWisePacking->m_rwpGuardBandType[i*4 + j]         =  m_pcCfg->getRwpSEIRwpGuardBandType(i*4 + j);
      }
    }
  }
}

Void SEIEncoder::initSEIFisheyeVideoInfo(SEIFisheyeVideoInfo *seiFisheyeVideoInfo)
{
  assert(m_isInitialized);
  assert(seiFisheyeVideoInfo != NULL);
  seiFisheyeVideoInfo->values = m_pcCfg->getFviSEIData();
}

template <typename T>
static Void readTokenValue(T            &returnedValue, /// value returned
                           Bool         &failed,        /// used and updated
                           std::istream &is,            /// stream to read token from
                           const TChar  *pToken,        /// token string
                           Bool (*parseFn) (std::istream &, T &) /// parsing function
                           )
{
  returnedValue=T();
  if (failed)
  {
    return;
  }

  Int c;
  // Ignore any whitespace
  while ((c=is.get())!=EOF && isspace(c));
  // test for comment mark
  while (c=='#')
  {
    // Ignore to the end of the line
    while ((c=is.get())!=EOF && (c!=10 && c!=13));
    // Ignore any white space at the start of the next line
    while ((c=is.get())!=EOF && isspace(c));
  }
  // test first character of token
  failed=(c!=pToken[0]);
  // test remaining characters of token
  Int pos;
  for(pos=1;!failed && pToken[pos]!=0 && is.get()==pToken[pos]; pos++);
  failed|=(pToken[pos]!=0);
  // Ignore any whitespace before the ':'
  while (!failed && (c=is.get())!=EOF && isspace(c));
  failed|=(c!=':');
  // Now read the value associated with the token:
  if (!failed)
  {
    failed=parseFn(is, returnedValue);
  }
  if (failed)
  {
    std::cerr << "Unable to read token '" << pToken << "'\n";
  }
}

template <typename T>
static Bool readTokenValueParsing(std::istream &is, T & returnedValue)
{
  is >> returnedValue;
  Bool failed=!is.good();
  if (!failed)
  {
    Int c=is.get();
    failed=(c!=EOF && !isspace(c));
  }
  return failed;
}

template <>
Bool readTokenValueParsing(std::istream &is, std::string & returnedValue)
{
  Int c;
  // ignore any initial white space
  do
  {
    c=is.get();
  } while (isspace(c) && c!=10 && c!=13 && c!=EOF);

  Int currentQuoteSymbol=0;
  Int trailingSpaces=0;

  while (c!=EOF && c!=0 && c!=10 && c!=13)
  {
    if (currentQuoteSymbol==0 && (c=='"' || c=='\''))
    {
      currentQuoteSymbol=c;
    }
    else if (c==currentQuoteSymbol)
    {
      currentQuoteSymbol=0;
    }
    else if (isspace(c) && !currentQuoteSymbol)
    {
      trailingSpaces++;
    }
    else
    {
      if (c=='\\')
      {
        c=is.get();
        if (c==EOF || c==10 || c==13 || c==0)
        {
          return true; // not properly escaped '\'
        }
      }
      if (trailingSpaces)
      {
        returnedValue+=string(trailingSpaces, ' ');
        trailingSpaces=0;
      }
      returnedValue+=c;
    }
    c=is.get();
  }

  return currentQuoteSymbol!=0; // error if we didn't find a closing quote.
}

template <typename T>
static Void readTokenValue(T            &returnedValue, /// value returned
                           Bool         &failed,        /// used and updated
                           std::istream &is,            /// stream to read token from
                           const TChar  *pToken)        /// token string
{
  readTokenValue(returnedValue, failed, is, pToken, readTokenValueParsing);
}

template <typename T>
static Void readTokenValueAndValidate(T            &returnedValue, /// value returned
                                      Bool         &failed,        /// used and updated
                                      std::istream &is,            /// stream to read token from
                                      const TChar  *pToken,        /// token string
                                      const T      &minInclusive,  /// minimum value allowed, inclusive
                                      const T      &maxInclusive)  /// maximum value allowed, inclusive
{
  readTokenValue(returnedValue, failed, is, pToken);
  if (!failed)
  {
    if (returnedValue<minInclusive || returnedValue>maxInclusive)
    {
      failed=true;
      std::cerr << "Value for token " << pToken << " must be in the range " << minInclusive << " to " << maxInclusive << " (inclusive); value read: " << returnedValue << std::endl;
    }
  }
}

// Bool version does not have maximum and minimum values.
static Void readTokenValueAndValidate(Bool         &returnedValue, /// value returned
                                      Bool         &failed,        /// used and updated
                                      std::istream &is,            /// stream to read token from
                                      const TChar  *pToken)        /// token string
{
  readTokenValue(returnedValue, failed, is, pToken);
}



template <typename T>
static Void readTokenValue(std::vector<T> &returnedValue, /// value returned
                           Bool         &failed,        /// used and updated
                           std::istream &is,            /// stream to read token from
                           const TChar  *pToken,         /// token string
                           const UInt   &numValues)     /// Num of values to be read in array
{
  returnedValue=std::vector<T>();
  if (failed)
  {
    return;
  }

  Int c;
  // Ignore any whitespace
  while ((c=is.get())!=EOF && isspace(c));
  // test for comment mark
  while (c=='#')
  {
    // Ignore to the end of the line
    while ((c=is.get())!=EOF && (c!=10 && c!=13));
    // Ignore any white space at the start of the next line
    while ((c=is.get())!=EOF && isspace(c));
  }
  // test first character of token
  failed=(c!=pToken[0]);
  // test remaining characters of token
  Int pos;
  for(pos=1;!failed && pToken[pos]!=0 && is.get()==pToken[pos]; pos++);
  failed|=(pToken[pos]!=0);
  // Ignore any whitespace before the ':'
  while (!failed && (c=is.get())!=EOF && isspace(c));
  failed|=(c!=':');
  // Now read the value associated with the token:
  for(UInt i = 0; i < numValues; i++)
  {
    if (!failed)
    {
      is >> returnedValue[i];
      failed=!is.good();
      if (!failed)
      {
        c=is.get();
        failed=(c!=EOF && !isspace(c));
      }
    }
    if (failed)
    {
      std::cerr << "Unable to read token '" << pToken << "[" << i << "]'\n";
    }
  }
}
// Version to read an array 
template <typename T>
static Void readTokenValueAndValidate(std::vector<T> &returnedValue, /// value returned
                                      Bool         &failed,        /// used and updated
                                      std::istream &is,            /// stream to read token from
                                      const TChar  *pToken,        /// token string
                                      const T      &minInclusive,  /// minimum value allowed, inclusive
                                      const T      &maxInclusive,  /// maximum value allowed, inclusive
                                      const UInt   &numValues)
{
  readTokenValue<Int>(returnedValue, failed, is, pToken, numValues);
  if (!failed)
  {
    for(UInt i = 0; i < numValues; i++)
    {
      if (returnedValue[i]<minInclusive || returnedValue[i]>maxInclusive)
      {
        failed=true;
        std::cerr << "Value for token " << pToken << "[" << i  << "] must be in the range " << minInclusive << " to " << maxInclusive << " (inclusive); value read: " << returnedValue[i] << std::endl;
      }
    }
  }
}

Bool SEIEncoder::initSEIColourRemappingInfo(SEIColourRemappingInfo* seiColourRemappingInfo, Int currPOC) // returns true on success, false on failure.
{
  assert (m_isInitialized);
  assert (seiColourRemappingInfo!=NULL);

  // reading external Colour Remapping Information SEI message parameters from file
  if( !m_pcCfg->getColourRemapInfoSEIFileRoot().empty())
  {
    Bool failed=false;

    // building the CRI file name with poc num in prefix "_poc.txt"
    std::string colourRemapSEIFileWithPoc(m_pcCfg->getColourRemapInfoSEIFileRoot());
    {
      std::stringstream suffix;
      suffix << "_" << currPOC << ".txt";
      colourRemapSEIFileWithPoc+=suffix.str();
    }

    std::ifstream fic(colourRemapSEIFileWithPoc.c_str());
    if (!fic.good() || !fic.is_open())
    {
      std::cerr <<  "No Colour Remapping Information SEI parameters file " << colourRemapSEIFileWithPoc << " for POC " << currPOC << std::endl;
      return false;
    }

    // TODO: identify and remove duplication with decoder parsing through abstraction.
    readColourRemapSEI(fic, seiColourRemappingInfo, failed );

    if( failed )
    {
      std::cerr << "Error while reading Colour Remapping Information SEI parameters file '" << colourRemapSEIFileWithPoc << "'" << std::endl;
      exit(EXIT_FAILURE);
    }
  }
  return true;
}

Void SEIEncoder::readRNSEIWindow(std::istream &fic, RNSEIWindowVec::iterator regionIter, Bool &failed )
{
  Int regionId;
  Int offLeft, offRight, offTop, offBottom;
  fic >> regionId >> offLeft >> offRight >> offTop >> offBottom;
  (*regionIter).setRegionId(regionId);
  (*regionIter).setWindow(offLeft, offRight, offTop, offBottom);
}
Void SEIEncoder::readColourRemapSEI(std::istream &fic, SEIColourRemappingInfo *seiColourRemappingInfo, Bool &failed )
{
  readTokenValueAndValidate<UInt>(seiColourRemappingInfo->m_colourRemapId,         failed, fic, "colour_remap_id",        UInt(0), UInt(0x7fffffff) );
  readTokenValueAndValidate(seiColourRemappingInfo->m_colourRemapCancelFlag, failed, fic, "colour_remap_cancel_flag" );
  if( !seiColourRemappingInfo->m_colourRemapCancelFlag )
  {
    readTokenValueAndValidate(seiColourRemappingInfo->m_colourRemapPersistenceFlag,            failed, fic, "colour_remap_persistence_flag" );
    readTokenValueAndValidate(seiColourRemappingInfo->m_colourRemapVideoSignalInfoPresentFlag, failed, fic, "colour_remap_video_signal_info_present_flag");
    if( seiColourRemappingInfo->m_colourRemapVideoSignalInfoPresentFlag )
    {
      readTokenValueAndValidate(seiColourRemappingInfo->m_colourRemapFullRangeFlag,      failed, fic, "colour_remap_full_range_flag" );
      readTokenValueAndValidate(seiColourRemappingInfo->m_colourRemapPrimaries,          failed, fic, "colour_remap_primaries",           Int(0), Int(255) );
      readTokenValueAndValidate(seiColourRemappingInfo->m_colourRemapTransferFunction,   failed, fic, "colour_remap_transfer_function",   Int(0), Int(255) );
      readTokenValueAndValidate(seiColourRemappingInfo->m_colourRemapMatrixCoefficients, failed, fic, "colour_remap_matrix_coefficients", Int(0), Int(255) );
    }
    readTokenValueAndValidate(seiColourRemappingInfo->m_colourRemapInputBitDepth, failed, fic, "colour_remap_input_bit_depth",            Int(8), Int(16) );
    readTokenValueAndValidate(seiColourRemappingInfo->m_colourRemapBitDepth,      failed, fic, "colour_remap_bit_depth",                  Int(8), Int(16) );

    const Int maximumInputValue    = (1 << (((seiColourRemappingInfo->m_colourRemapInputBitDepth + 7) >> 3) << 3)) - 1;
    const Int maximumRemappedValue = (1 << (((seiColourRemappingInfo->m_colourRemapBitDepth      + 7) >> 3) << 3)) - 1;

    for( Int c=0 ; c<3 ; c++ )
    {
      readTokenValueAndValidate(seiColourRemappingInfo->m_preLutNumValMinus1[c],         failed, fic, "pre_lut_num_val_minus1[c]",        Int(0), Int(32) );
      if( seiColourRemappingInfo->m_preLutNumValMinus1[c]>0 )
      {
        seiColourRemappingInfo->m_preLut[c].resize(seiColourRemappingInfo->m_preLutNumValMinus1[c]+1);
        for( Int i=0 ; i<=seiColourRemappingInfo->m_preLutNumValMinus1[c] ; i++ )
        {
          readTokenValueAndValidate(seiColourRemappingInfo->m_preLut[c][i].codedValue,   failed, fic, "pre_lut_coded_value[c][i]",  Int(0), maximumInputValue    );
          readTokenValueAndValidate(seiColourRemappingInfo->m_preLut[c][i].targetValue,  failed, fic, "pre_lut_target_value[c][i]", Int(0), maximumRemappedValue );
        }
      }
    }
    readTokenValueAndValidate(seiColourRemappingInfo->m_colourRemapMatrixPresentFlag, failed, fic, "colour_remap_matrix_present_flag" );
    if( seiColourRemappingInfo->m_colourRemapMatrixPresentFlag )
    {
      readTokenValueAndValidate(seiColourRemappingInfo->m_log2MatrixDenom, failed, fic, "log2_matrix_denom", Int(0), Int(15) );
      for( Int c=0 ; c<3 ; c++ )
      {
        for( Int i=0 ; i<3 ; i++ )
        {
          readTokenValueAndValidate(seiColourRemappingInfo->m_colourRemapCoeffs[c][i], failed, fic, "colour_remap_coeffs[c][i]", -32768, 32767 );
        }
      }
    }
    for( Int c=0 ; c<3 ; c++ )
    {
      readTokenValueAndValidate(seiColourRemappingInfo->m_postLutNumValMinus1[c], failed, fic, "post_lut_num_val_minus1[c]", Int(0), Int(32) );
      if( seiColourRemappingInfo->m_postLutNumValMinus1[c]>0 )
      {
        seiColourRemappingInfo->m_postLut[c].resize(seiColourRemappingInfo->m_postLutNumValMinus1[c]+1);
        for( Int i=0 ; i<=seiColourRemappingInfo->m_postLutNumValMinus1[c] ; i++ )
        {
          readTokenValueAndValidate(seiColourRemappingInfo->m_postLut[c][i].codedValue,  failed, fic, "post_lut_coded_value[c][i]",  Int(0), maximumRemappedValue );
          readTokenValueAndValidate(seiColourRemappingInfo->m_postLut[c][i].targetValue, failed, fic, "post_lut_target_value[c][i]", Int(0), maximumRemappedValue );
        }
      }
    }
  }
}
Void SEIEncoder::readToneMappingInfoSEI(std::istream &fic, SEIToneMappingInfo *seiToneMappingInfo , Bool &failed )
{
  readTokenValueAndValidate(seiToneMappingInfo->m_toneMapId,                          failed, fic,       "SEIToneMapId",        Int(0), Int(255) );
  readTokenValueAndValidate(seiToneMappingInfo->m_toneMapCancelFlag,                  failed, fic,       "SEIToneMapCancelFlag");
  readTokenValueAndValidate(seiToneMappingInfo->m_toneMapPersistenceFlag,             failed, fic,       "SEIToneMapPersistenceFlag");
  readTokenValueAndValidate(seiToneMappingInfo->m_codedDataBitDepth,                  failed, fic,       "SEIToneMapCodedDataBitDepth",        Int(8), Int(14) );
  readTokenValueAndValidate(seiToneMappingInfo->m_targetBitDepth,                     failed, fic,       "SEIToneMapTargetBitDepth",        Int(8), Int(14) );
  readTokenValueAndValidate(seiToneMappingInfo->m_modelId,                            failed, fic,       "SEIToneMapModelId",        Int(0), Int(4) );
  readTokenValueAndValidate(seiToneMappingInfo->m_minValue,                           failed, fic,       "SEIToneMapMinValue",       Int(0), Int((1<<14)-1) );
  readTokenValueAndValidate(seiToneMappingInfo->m_maxValue,                           failed, fic,       "SEIToneMapMaxValue",       Int(0), Int((1<<14)-1) );
  readTokenValueAndValidate(seiToneMappingInfo->m_sigmoidMidpoint,                    failed, fic,       "SEIToneMapSigmoidMidpoint",       Int(0), Int((1<<14)-1) );
  readTokenValueAndValidate(seiToneMappingInfo->m_sigmoidWidth,                       failed, fic,       "SEIToneMapSigmoidWidth",       Int(0), Int((1<<14)-1) );
  readTokenValueAndValidate<Int>(seiToneMappingInfo->m_startOfCodedInterval,               failed, fic,       "SEIToneMapStartOfCodedInterval",       Int(0), Int((1<<16)-1), UInt(1<<seiToneMappingInfo->m_targetBitDepth) );
  readTokenValueAndValidate(seiToneMappingInfo->m_numPivots,                          failed, fic,       "SEIToneMapNumPivots",       Int(0), Int((1<<16)-1) );
  readTokenValueAndValidate(seiToneMappingInfo->m_codedPivotValue,                    failed, fic,       "SEIToneMapCodedPivotValue",       Int(0), Int((1<<16)-1), seiToneMappingInfo->m_numPivots );
  readTokenValueAndValidate(seiToneMappingInfo->m_targetPivotValue,                   failed, fic,       "SEIToneMapTargetPivotValue",       Int(0), Int((1<<16)-1), seiToneMappingInfo->m_numPivots );
  readTokenValueAndValidate(seiToneMappingInfo->m_cameraIsoSpeedIdc,                  failed, fic,       "SEIToneMapCameraIsoSpeedIdc",       Int(0), Int(255) );
  readTokenValueAndValidate(seiToneMappingInfo->m_cameraIsoSpeedValue,                failed, fic,       "SEIToneMapCameraIsoSpeedValue",       Int(0), Int((~0)-1) );
  readTokenValueAndValidate(seiToneMappingInfo->m_exposureIndexIdc,                   failed, fic,       "SEIToneMapExposureIndexIdc",       Int(0), Int(255) );
  readTokenValueAndValidate(seiToneMappingInfo->m_exposureIndexValue,                 failed, fic,       "SEIToneMapExposureIndexValue",       Int(0), Int((~0)-1) );
  readTokenValueAndValidate(seiToneMappingInfo->m_exposureCompensationValueSignFlag,  failed, fic,       "SEIToneMapExposureCompensationValueSignFlag");
  readTokenValueAndValidate(seiToneMappingInfo->m_exposureCompensationValueNumerator, failed, fic,       "SEIToneMapExposureCompensationValueNumerator",       Int(0), Int((1<<16)-1) );
  readTokenValueAndValidate(seiToneMappingInfo->m_exposureCompensationValueDenomIdc,  failed, fic,       "SEIToneMapExposureCompensationValueDenomIdc",       Int(0), Int((1<<16)-1) );
  readTokenValueAndValidate(seiToneMappingInfo->m_refScreenLuminanceWhite,            failed, fic,       "SEIToneMapRefScreenLuminanceWhite",       Int(0), Int(10000) );
  readTokenValueAndValidate(seiToneMappingInfo->m_extendedRangeWhiteLevel,            failed, fic,       "SEIToneMapExtendedRangeWhiteLevel",       Int(100), Int(10000) );
  readTokenValueAndValidate(seiToneMappingInfo->m_nominalBlackLevelLumaCodeValue,     failed, fic,       "SEIToneMapNominalBlackLevelLumaCodeValue",       Int(0), Int((1<<16)-1) );
  readTokenValueAndValidate(seiToneMappingInfo->m_nominalWhiteLevelLumaCodeValue,     failed, fic,       "SEIToneMapNominalWhiteLevelLumaCodeValue",       Int(0), Int((1<<16)-1) );
  readTokenValueAndValidate(seiToneMappingInfo->m_extendedWhiteLevelLumaCodeValue,    failed, fic,       "SEIToneMapExtendedWhiteLevelLumaCodeValue",       Int(0), Int((1<<16)-1) );
}
Void SEIEncoder::readChromaResamplingFilterHintSEI(std::istream &fic, SEIChromaResamplingFilterHint *seiChromaResamplingFilterHint, Bool &failed )
{
  Int horFilType, verFilType;
  readTokenValueAndValidate(horFilType,         failed, fic,       "SEIChromaResamplingHorizontalFilterType",        Int(0), Int(2) );
  readTokenValueAndValidate(verFilType,         failed, fic,       "SEIChromaResamplingVerticalFilterType",          Int(0), Int(2) );
  if(horFilType == 1 || verFilType == 1)
  {
    cout << "Encoder support needed for scanning additional syntax elements of chroma resampling filter hint SEI message.\n";
    cout << "Addn. code needed in readChromaResamplingFilterHintSEI() and initSEIChromaResamplingFilterHint().\n";
    exit(1);
  }
  initSEIChromaResamplingFilterHint(seiChromaResamplingFilterHint, horFilType, verFilType);
}
Void SEIEncoder::readKneeFunctionInfoSEI(std::istream &fic, SEIKneeFunctionInfo *seiKneeFunctionInfo, Bool &failed )
{
  readTokenValueAndValidate(seiKneeFunctionInfo->m_kneeId,                  failed, fic,       "SEIKneeFunctionId",                 Int(0), Int(255));
  readTokenValueAndValidate(seiKneeFunctionInfo->m_kneeCancelFlag,          failed, fic,       "SEIKneeFunctionCancelFlag"      );
  readTokenValueAndValidate(seiKneeFunctionInfo->m_kneePersistenceFlag,     failed, fic,       "SEIKneeFunctionPersistenceFlag" );
  readTokenValueAndValidate(seiKneeFunctionInfo->m_kneeInputDrange,         failed, fic,       "SEIKneeFunctionInputDrange",        Int(0), Int(1000) );
  readTokenValueAndValidate(seiKneeFunctionInfo->m_kneeInputDispLuminance,  failed, fic,       "SEIKneeFunctionInputDispLuminance", Int(0), Int(10000) );
  readTokenValueAndValidate(seiKneeFunctionInfo->m_kneeOutputDrange,        failed, fic,       "SEIKneeFunctionOutputDrange",       Int(0), Int(1000) );
  readTokenValueAndValidate(seiKneeFunctionInfo->m_kneeOutputDispLuminance, failed, fic,       "SEIKneeFunctionOutputDispLuminance",Int(0), Int(10000) );
  readTokenValueAndValidate(seiKneeFunctionInfo->m_kneeNumKneePointsMinus1, failed, fic,       "SEIKneeFunctionNumKneePointsMinus1",Int(0), Int(998) );
  readTokenValueAndValidate(seiKneeFunctionInfo->m_kneeInputKneePoint,      failed, fic,       "SEIKneeFunctionInputKneePointValue",Int(0), Int(1000), seiKneeFunctionInfo->m_kneeNumKneePointsMinus1+1 );
  readTokenValueAndValidate(seiKneeFunctionInfo->m_kneeOutputKneePoint,     failed, fic,       "SEIKneeFunctionInputKneePointValue",Int(0), Int(1000), seiKneeFunctionInfo->m_kneeNumKneePointsMinus1+1 );

}
Void SEIEncoder::readContentColourVolumeSEI(std::istream &fic, SEIContentColourVolume *seiContentColourVolume, Bool &failed )
{
  Double dCode, primaries[2][MAX_NUM_COMPONENT];
  readTokenValueAndValidate(seiContentColourVolume->m_ccvCancelFlag,              failed, fic,       "SEICCVCancelFlag"          );
  readTokenValueAndValidate(seiContentColourVolume->m_ccvPersistenceFlag,         failed, fic,       "SEICCVPersistenceFlag"     );
  readTokenValueAndValidate(seiContentColourVolume->m_ccvPrimariesPresentFlag,    failed, fic,  "SEICCVPrimariesPresent"    );
  readTokenValueAndValidate(primaries[0][0],    failed, fic,  "m_ccvSEIPrimariesX0",    Double(-100), Double(100));
  readTokenValueAndValidate(primaries[1][1],    failed, fic,  "m_ccvSEIPrimariesY0",    Double(-100), Double(100));
  readTokenValueAndValidate(primaries[0][2],    failed, fic,  "m_ccvSEIPrimariesX1",    Double(-100), Double(100));
  readTokenValueAndValidate(primaries[1][0],    failed, fic,  "m_ccvSEIPrimariesY1",    Double(-100), Double(100));
  readTokenValueAndValidate(primaries[0][1],    failed, fic,  "m_ccvSEIPrimariesX2",    Double(-100), Double(100));
  readTokenValueAndValidate(primaries[1][2],    failed, fic,  "m_ccvSEIPrimariesY2",    Double(-100), Double(100));
  if(seiContentColourVolume->m_ccvPrimariesPresentFlag)
  {
    for (Int i = 0; i < MAX_NUM_COMPONENT; i++) 
    {
      seiContentColourVolume->m_ccvPrimariesX[i] = (Int) (50000.0 * primaries[0][i]);
      seiContentColourVolume->m_ccvPrimariesY[i] = (Int) (50000.0 * primaries[1][i]);
    }
  }
  readTokenValueAndValidate(seiContentColourVolume->m_ccvMinLuminanceValuePresentFlag,    failed, fic,  "SEICCVMinLuminanceValuePresent"    );
  readTokenValueAndValidate(dCode,    failed, fic,  "SEICCVMinLuminanceValue",    Double(0), Double(429.4967295));
  if(seiContentColourVolume->m_ccvMinLuminanceValuePresentFlag)
  {
    seiContentColourVolume->m_ccvMinLuminanceValue = (Int) (10000000 * dCode);
  }
  readTokenValueAndValidate(seiContentColourVolume->m_ccvMaxLuminanceValuePresentFlag,    failed, fic,  "SEICCVMaxLuminanceValuePresent"    );
  readTokenValueAndValidate(dCode,    failed, fic,  "SEICCVMaxLuminanceValue",    Double(0), Double(429.4967295));
  if(seiContentColourVolume->m_ccvMaxLuminanceValuePresentFlag)
  {
    seiContentColourVolume->m_ccvMaxLuminanceValue = (Int) (10000000 * dCode);
  }
  readTokenValueAndValidate(seiContentColourVolume->m_ccvAvgLuminanceValuePresentFlag,    failed, fic,  "SEICCVAvgLuminanceValuePresent"    );
  readTokenValueAndValidate(dCode,    failed, fic,  "SEICCVAvgLuminanceValue",    Double(0), Double(429.4967295));
  if(seiContentColourVolume->m_ccvAvgLuminanceValuePresentFlag)
  {
    seiContentColourVolume->m_ccvAvgLuminanceValue = (Int) (10000000 * dCode);
  }
}
Bool SEIEncoder::initSEIRegionalNesting(SEIRegionalNesting* seiRegionalNesting, Int currPOC) // returns true on success, false on failure.
{
  assert (m_isInitialized);
  assert (seiRegionalNesting!=NULL);
  Int numRegions; 
  RNSEIWindowVec regions;

  // reading external regional nesting SEI message parameters from file
  if( !m_pcCfg->getRegionalNestingSEIFileRoot().empty())
  {
    Bool failed=false;

    // building the regional nesting file name with poc num in prefix "_poc.txt"
    std::string regionalNestingSEIFileWithPoc(m_pcCfg->getRegionalNestingSEIFileRoot());
    {
      std::stringstream suffix;
      suffix << "_" << currPOC << ".txt";
      regionalNestingSEIFileWithPoc+=suffix.str();
    }

    std::ifstream fic(regionalNestingSEIFileWithPoc.c_str());
    if (!fic.good() || !fic.is_open())
    {
      std::cerr <<  "No Regional Nesting Information SEI parameters file " << regionalNestingSEIFileWithPoc << " for POC " << currPOC << std::endl;
      return false;
    }
    
    Int payloadType, numSEIs;
    readTokenValueAndValidate( numSEIs, failed, fic, "num_seis", 0, 255 );
    // Loop through each region - ID - SEI
    for(Int i = 0; i < numSEIs; i++)
    {
      RegionalSEI *regSEI = NULL;
      // Parse num region
      readTokenValueAndValidate( numRegions, failed, fic, "num_regions", 0, 255 );
      regions.resize(numRegions);
      RNSEIWindowVec::iterator regionIter;
      for(regionIter = regions.begin(); regionIter != regions.end(); regionIter++)
      {
        readRNSEIWindow(fic, regionIter, failed);
      }
      // Parse payloadType
      readTokenValueAndValidate( payloadType, failed, fic, "payloadType", 0, 255 );
      if(!RegionalSEI::checkRegionalNestedSEIPayloadType(SEI::PayloadType(payloadType)))
      {
        cout << "PayloadType " << payloadType << " not supported in regional nesting SEI message. Exiting.\n";
        exit(1);
      }

      // Parse SEI
      switch(SEI::PayloadType(payloadType))
      {
      case SEI::FILM_GRAIN_CHARACTERISTICS: // TBD - no encoder support
      case SEI::POST_FILTER_HINT:           // TBD - no encoder support
      case SEI::USER_DATA_REGISTERED_ITU_T_T35: // TBD: Support for user-data SEI to be added
      case SEI::USER_DATA_UNREGISTERED:     // TBD: Support for user-data SEI to be added
        {
          cout << "Encoder support for " << SEI::getSEIMessageString(SEI::PayloadType(payloadType)) << 
                " nested in regional nesting SEI message needs to be added.\n";
          exit(1);
        }
      case SEI::TONE_MAPPING_INFO:  
        {
        SEIToneMappingInfo *seiToneMappingInfo = new SEIToneMappingInfo;
        readToneMappingInfoSEI(fic, seiToneMappingInfo, failed);
        regSEI = new RegionalSEI(seiToneMappingInfo, regions);
        break;
        }
      case SEI::CHROMA_RESAMPLING_FILTER_HINT:
        {
        SEIChromaResamplingFilterHint *seiChromaResamplingFilterHint  = new SEIChromaResamplingFilterHint;
        readChromaResamplingFilterHintSEI(fic, seiChromaResamplingFilterHint, failed);
        regSEI = new RegionalSEI(seiChromaResamplingFilterHint, regions);
        break;
        }
      case SEI::KNEE_FUNCTION_INFO:
        {
        SEIKneeFunctionInfo *seiKneeFunctionInfo = new SEIKneeFunctionInfo;
        readKneeFunctionInfoSEI(fic, seiKneeFunctionInfo, failed);
        regSEI = new RegionalSEI(seiKneeFunctionInfo, regions);
        break;
        }
      case SEI::COLOUR_REMAPPING_INFO:
        {
        SEIColourRemappingInfo *seiColourRemappingInfo = new SEIColourRemappingInfo;
        readColourRemapSEI(fic, seiColourRemappingInfo, failed);
        regSEI = new RegionalSEI(seiColourRemappingInfo, regions);
        break;
        }
      case SEI::CONTENT_COLOUR_VOLUME:
        {
        SEIContentColourVolume *seiContentColourVolume = new SEIContentColourVolume;
        readContentColourVolumeSEI(fic, seiContentColourVolume, failed);
        regSEI = new RegionalSEI(seiContentColourVolume, regions);
        break;
        }
      default:
        cout << "Unable to read the payloadType " << payloadType << " in " << regionalNestingSEIFileWithPoc << std::endl;
        exit(1);
      }
      if( failed )
      {
        std::cerr << "Error while reading regional nesting SEI parameters file '" << regionalNestingSEIFileWithPoc << "'" << std::endl;
        exit(EXIT_FAILURE);
      }
      // Add to SEI
      if(regSEI)
      {
        seiRegionalNesting->addRegionalSEI(regSEI);
        delete regSEI;
      }
      else
      {
        // Error - SEI should've been read.
      }
    }
  }
  return true;
}


Void SEIEncoder::readAnnotatedRegionSEI(std::istream &fic, SEIAnnotatedRegions *seiAnnoRegion, Bool &failed)
{
  readTokenValueAndValidate(seiAnnoRegion->m_hdr.m_cancelFlag, failed, fic, "SEIArCancelFlag");
  if (!seiAnnoRegion->m_hdr.m_cancelFlag)
  {
    readTokenValueAndValidate(seiAnnoRegion->m_hdr.m_notOptimizedForViewingFlag, failed, fic, "SEIArNotOptForViewingFlag");
    readTokenValueAndValidate(seiAnnoRegion->m_hdr.m_trueMotionFlag, failed, fic, "SEIArTrueMotionFlag");
    readTokenValueAndValidate(seiAnnoRegion->m_hdr.m_occludedObjectFlag, failed, fic, "SEIArOccludedObjsFlag");
    readTokenValueAndValidate(seiAnnoRegion->m_hdr.m_partialObjectFlagPresentFlag, failed, fic, "SEIArPartialObjsFlagPresentFlag");
    readTokenValueAndValidate(seiAnnoRegion->m_hdr.m_objectLabelPresentFlag, failed, fic, "SEIArObjLabelPresentFlag");
    readTokenValueAndValidate(seiAnnoRegion->m_hdr.m_objectConfidenceInfoPresentFlag, failed, fic, "SEIArObjConfInfoPresentFlag");
    if (seiAnnoRegion->m_hdr.m_objectConfidenceInfoPresentFlag)
    {
      readTokenValueAndValidate<UInt>(seiAnnoRegion->m_hdr.m_objectConfidenceLength, failed, fic, "SEIArObjDetConfLength", UInt(0), UInt(255));
    }
    if (seiAnnoRegion->m_hdr.m_objectLabelPresentFlag)
    {
      readTokenValueAndValidate(seiAnnoRegion->m_hdr.m_objectLabelLanguagePresentFlag, failed, fic, "SEIArObjLabelLangPresentFlag");
      if (seiAnnoRegion->m_hdr.m_objectLabelLanguagePresentFlag)
      {
        readTokenValue(seiAnnoRegion->m_hdr.m_annotatedRegionsObjectLabelLang, failed, fic, "SEIArLabelLanguage");
      }
      UInt numLabelUpdates=0;
      readTokenValueAndValidate<UInt>(numLabelUpdates, failed, fic, "SEIArNumLabelUpdates", UInt(0), UInt(255));
      seiAnnoRegion->m_annotatedLabels.resize(numLabelUpdates);
      for (auto it=seiAnnoRegion->m_annotatedLabels.begin(); it!=seiAnnoRegion->m_annotatedLabels.end(); it++)
      {
        SEIAnnotatedRegions::AnnotatedRegionLabel &ar=it->second;
        readTokenValueAndValidate(it->first, failed, fic, "SEIArLabelIdc[c]", UInt(0), UInt(255));
        bool cancelFlag;
        readTokenValueAndValidate(cancelFlag, failed, fic, "SEIArLabelCancelFlag[c]");
        ar.labelValid=!cancelFlag;
        if (ar.labelValid)
        {
          readTokenValue(ar.label, failed, fic, "SEIArLabel[c]");
        }
      }
    }

    UInt numObjectUpdates=0;
    readTokenValueAndValidate<UInt>(numObjectUpdates, failed, fic, "SEIArNumObjUpdates", UInt(0), UInt(255));
    seiAnnoRegion->m_annotatedRegions.resize(numObjectUpdates);
    for (auto it=seiAnnoRegion->m_annotatedRegions.begin(); it!=seiAnnoRegion->m_annotatedRegions.end(); it++)
    {
      SEIAnnotatedRegions::AnnotatedRegionObject &ar = it->second;
      readTokenValueAndValidate(it->first, failed, fic, "SEIArObjIdx[c]", UInt(0), UInt(255));
      readTokenValueAndValidate(ar.objectCancelFlag, failed, fic, "SEIArObjCancelFlag[c]");
      ar.objectLabelValid=false;
      ar.boundingBoxValid=false;
      if (!ar.objectCancelFlag)
      {
        if (seiAnnoRegion->m_hdr.m_objectLabelPresentFlag)
        {
          readTokenValueAndValidate(ar.objectLabelValid, failed, fic, "SEIArObjLabelUpdateFlag[c]");
          if (ar.objectLabelValid)
          {
            readTokenValueAndValidate<UInt>(ar.objLabelIdx, failed, fic, "SEIArObjectLabelIdc[c]", UInt(0), UInt(255));
          }
        }

        readTokenValueAndValidate(ar.boundingBoxValid, failed, fic, "SEIArBoundBoxUpdateFlag[c]");
        if (ar.boundingBoxValid)
        {
          readTokenValueAndValidate(ar.boundingBoxCancelFlag, failed, fic, "SEIArBoundBoxCancelFlag[c]");
          if (!ar.boundingBoxCancelFlag)
          {
            readTokenValueAndValidate<UInt>(ar.boundingBoxTop, failed, fic, "SEIArObjTop[c]", UInt(0), UInt(0x7fffffff));
            readTokenValueAndValidate<UInt>(ar.boundingBoxLeft, failed, fic, "SEIArObjLeft[c]", UInt(0), UInt(0x7fffffff));
            readTokenValueAndValidate<UInt>(ar.boundingBoxWidth, failed, fic, "SEIArObjWidth[c]", UInt(0), UInt(0x7fffffff));
            readTokenValueAndValidate<UInt>(ar.boundingBoxHeight, failed, fic, "SEIArObjHeight[c]", UInt(0), UInt(0x7fffffff));
            if (seiAnnoRegion->m_hdr.m_partialObjectFlagPresentFlag)
            {
              readTokenValueAndValidate(ar.partialObjectFlag, failed, fic, "SEIArObjPartUpdateFlag[c]");
            }
            if (seiAnnoRegion->m_hdr.m_objectConfidenceInfoPresentFlag)
            {
              readTokenValueAndValidate<UInt>(ar.objectConfidence, failed, fic, "SEIArObjDetConf[c]", UInt(0), UInt(1<<seiAnnoRegion->m_hdr.m_objectConfidenceLength)-1);
            }
          }
          if (seiAnnoRegion->m_hdr.m_objectConfidenceInfoPresentFlag)
          {
            readTokenValueAndValidate<UInt>(ar.objectConfidence, failed, fic, "SEIArObjDetConf[c]", UInt(0), UInt(1<<seiAnnoRegion->m_hdr.m_objectConfidenceLength)-1);
          }
        }
#if JVET_T0050_ANNOTATED_REGIONS_SEI
        //Compare with existing attributes to decide whether it's a static object
        //First check whether it's an existing object (or) new object
        auto destIt = m_pcCfg->m_arObjects.find(it->first);
        //New object
        if (destIt == m_pcCfg->m_arObjects.end())
        {
          //New object arrived, needs to be appended to the map of tracked objects
          m_pcCfg->m_arObjects[it->first] = ar;
        }
        //Existing object
        else
        {
          // Size remains the same
          if(m_pcCfg->m_arObjects[it->first].boundingBoxWidth == ar.boundingBoxWidth &&
             m_pcCfg->m_arObjects[it->first].boundingBoxHeight == ar.boundingBoxHeight)
          {
            if(m_pcCfg->m_arObjects[it->first].boundingBoxTop == ar.boundingBoxTop &&
               m_pcCfg->m_arObjects[it->first].boundingBoxLeft == ar.boundingBoxLeft)
            {
              ar.boundingBoxValid = 0;
            }
          }
        }
#endif
      }
#if JVET_T0050_ANNOTATED_REGIONS_SEI
      else
      {
        //Object has been marked for deletion
        auto destIt = m_pcCfg->m_arObjects.find(it->first);
        if (destIt != m_pcCfg->m_arObjects.end())
        {
          m_pcCfg->m_arObjects.erase(destIt);
        }
      }
#endif
    }
  }
#if JVET_T0050_ANNOTATED_REGIONS_SEI
  else
  {
    seiAnnoRegion->m_annotatedRegions.clear();
    seiAnnoRegion->m_annotatedLabels.clear();
    m_pcCfg->m_arObjects.clear();
  }
#endif
}

Bool SEIEncoder::initSEIAnnotatedRegions(SEIAnnotatedRegions* SEIAnnoReg, Int currPOC)
{
  assert(m_isInitialized);
  assert(SEIAnnoReg != NULL);

  // reading external Colour Remapping Information SEI message parameters from file
  if (!m_pcCfg->getAnnotatedRegionSEIFileRoot().empty())
  {
    Bool failed = false;
    // building the annotated regions file name with poc num in prefix "_poc.txt"
    std::string AnnoRegionSEIFileWithPoc(m_pcCfg->getAnnotatedRegionSEIFileRoot());
    {
      std::stringstream suffix;
      suffix << "_" << currPOC << ".txt";
      AnnoRegionSEIFileWithPoc += suffix.str();
    }
    std::ifstream fic(AnnoRegionSEIFileWithPoc.c_str());
    if (!fic.good() || !fic.is_open())
    {
      std::cerr << "No Annotated Regions SEI parameters file " << AnnoRegionSEIFileWithPoc << " for POC " << currPOC << std::endl;
      return false;
    }
    //Read annotated region SEI parameters from the cfg file
    readAnnotatedRegionSEI(fic, SEIAnnoReg, failed);
    if (failed)
    {
      std::cerr << "Error while reading Annotated Regions SEI parameters file '" << AnnoRegionSEIFileWithPoc << "'" << std::endl;
      exit(EXIT_FAILURE);
    }
  }
  return true;
}
Void SEIEncoder::initSEIChromaResamplingFilterHint(SEIChromaResamplingFilterHint *seiChromaResamplingFilterHint, Int iHorFilterIndex, Int iVerFilterIndex)
{
  assert (m_isInitialized);
  assert (seiChromaResamplingFilterHint!=NULL);

  seiChromaResamplingFilterHint->m_verChromaFilterIdc = iVerFilterIndex;
  seiChromaResamplingFilterHint->m_horChromaFilterIdc = iHorFilterIndex;
  seiChromaResamplingFilterHint->m_verFilteringFieldProcessingFlag = 1;
  seiChromaResamplingFilterHint->m_targetFormatIdc = 3;
  seiChromaResamplingFilterHint->m_perfectReconstructionFlag = false;

  // this creates some example filter values, if explicit filter definition is selected
  if (seiChromaResamplingFilterHint->m_verChromaFilterIdc == 1)
  {
    const Int numVerticalFilters = 3;
    const Int verTapLengthMinus1[] = {5,3,3};

    seiChromaResamplingFilterHint->m_verFilterCoeff.resize(numVerticalFilters);
    for(Int i = 0; i < numVerticalFilters; i ++)
    {
      seiChromaResamplingFilterHint->m_verFilterCoeff[i].resize(verTapLengthMinus1[i]+1);
    }
    // Note: C++11 -> seiChromaResamplingFilterHint->m_verFilterCoeff[0] = {-3,13,31,23,3,-3};
    seiChromaResamplingFilterHint->m_verFilterCoeff[0][0] = -3;
    seiChromaResamplingFilterHint->m_verFilterCoeff[0][1] = 13;
    seiChromaResamplingFilterHint->m_verFilterCoeff[0][2] = 31;
    seiChromaResamplingFilterHint->m_verFilterCoeff[0][3] = 23;
    seiChromaResamplingFilterHint->m_verFilterCoeff[0][4] = 3;
    seiChromaResamplingFilterHint->m_verFilterCoeff[0][5] = -3;

    seiChromaResamplingFilterHint->m_verFilterCoeff[1][0] = -1;
    seiChromaResamplingFilterHint->m_verFilterCoeff[1][1] = 25;
    seiChromaResamplingFilterHint->m_verFilterCoeff[1][2] = 247;
    seiChromaResamplingFilterHint->m_verFilterCoeff[1][3] = -15;

    seiChromaResamplingFilterHint->m_verFilterCoeff[2][0] = -20;
    seiChromaResamplingFilterHint->m_verFilterCoeff[2][1] = 186;
    seiChromaResamplingFilterHint->m_verFilterCoeff[2][2] = 100;
    seiChromaResamplingFilterHint->m_verFilterCoeff[2][3] = -10;
  }
  else
  {
    seiChromaResamplingFilterHint->m_verFilterCoeff.resize(0);
  }

  if (seiChromaResamplingFilterHint->m_horChromaFilterIdc == 1)
  {
    Int const numHorizontalFilters = 1;
    const Int horTapLengthMinus1[] = {3};

    seiChromaResamplingFilterHint->m_horFilterCoeff.resize(numHorizontalFilters);
    for(Int i = 0; i < numHorizontalFilters; i ++)
    {
      seiChromaResamplingFilterHint->m_horFilterCoeff[i].resize(horTapLengthMinus1[i]+1);
    }
    seiChromaResamplingFilterHint->m_horFilterCoeff[0][0] = 1;
    seiChromaResamplingFilterHint->m_horFilterCoeff[0][1] = 6;
    seiChromaResamplingFilterHint->m_horFilterCoeff[0][2] = 1;
  }
  else
  {
    seiChromaResamplingFilterHint->m_horFilterCoeff.resize(0);
  }
}

Void SEIEncoder::initSEITimeCode(SEITimeCode *seiTimeCode)
{
  assert (m_isInitialized);
  assert (seiTimeCode!=NULL);
  //  Set data as per command line options
  seiTimeCode->numClockTs = m_pcCfg->getNumberOfTimesets();
  for(Int i = 0; i < seiTimeCode->numClockTs; i++)
  {
    seiTimeCode->timeSetArray[i] = m_pcCfg->getTimeSet(i);
  }
}

Void SEIEncoder::initSEIAlternativeTransferCharacteristics(SEIAlternativeTransferCharacteristics *seiAltTransCharacteristics)
{
  assert (m_isInitialized);
  assert (seiAltTransCharacteristics!=NULL);
  //  Set SEI message parameters read from command line options
  seiAltTransCharacteristics->m_preferredTransferCharacteristics = m_pcCfg->getSEIPreferredTransferCharacteristics();
}

Void SEIEncoder::initSEIGreenMetadataInfo(SEIGreenMetadataInfo *seiGreenMetadataInfo, UInt u)
{
    assert (m_isInitialized);
    assert (seiGreenMetadataInfo!=NULL);

    seiGreenMetadataInfo->m_greenMetadataType = m_pcCfg->getSEIGreenMetadataType();
    seiGreenMetadataInfo->m_xsdMetricType = m_pcCfg->getSEIXSDMetricType();
    seiGreenMetadataInfo->m_xsdMetricValue = u;
}

#if JCTVC_AD0021_SEI_MANIFEST
Void SEIEncoder::initSEISEIManifest(SEIManifest* seiSeiManifest, const SEIMessages& seiMessages)
{
  assert(m_isInitialized);
  assert(seiSeiManifest != NULL);
  seiSeiManifest->m_manifestNumSeiMsgTypes = 0;
  for (auto& it : seiMessages)
  {
    seiSeiManifest->m_manifestNumSeiMsgTypes += 1;
    auto tempPayloadType = it->payloadType();
    seiSeiManifest->m_manifestSeiPayloadType.push_back(tempPayloadType);
    auto description = seiSeiManifest->getSEIMessageDescription(tempPayloadType);
    seiSeiManifest->m_manifestSeiDescription.push_back(description);
  }
}
#endif

#if JCTVC_AD0021_SEI_PREFIX_INDICATION
Void SEIEncoder::initSEISEIPrefixIndication(SEIPrefixIndication* seiSeiPrefixIndications, const SEI* sei)
{
  assert(m_isInitialized);
  assert(seiSeiPrefixIndications != NULL);
  seiSeiPrefixIndications->m_prefixSeiPayloadType = sei->payloadType();
  seiSeiPrefixIndications->m_numSeiPrefixIndicationsMinus1 = seiSeiPrefixIndications->getNumsOfSeiPrefixIndications(sei) - 1;
  seiSeiPrefixIndications->m_payload = sei;
}

#endif 
#if JVET_AL0062_AI_USAGE_RESTRICTIONS_SEI
void SEIEncoder::initSEIAIUsageRestrictions(SEIAIUsageRestrictions *sei)
{
  
  sei->m_cancelFlag = m_pcCfg->getAURSEICancelFlag();
  if (!sei->m_cancelFlag)
  {
    sei->m_persistenceFlag = m_pcCfg->getAURSEIPersistenceFlag();
    sei->m_numRestrictionsMinus1 = m_pcCfg->getAURSEINumRestrictionsMinus1();
    sei->m_restrictions.resize(sei->m_numRestrictionsMinus1 + 1);
    sei->m_contextPresentFlag.resize(sei->m_numRestrictionsMinus1 + 1);
    sei->m_context.resize(sei->m_numRestrictionsMinus1 + 1);
    for (uint32_t i = 0; i <= sei->m_numRestrictionsMinus1; i++)
    {
      sei->m_restrictions[i] = m_pcCfg->getAURSEIRestrictions(i);
      sei->m_contextPresentFlag[i] = m_pcCfg->getAURSEIContextPresentFlag(i);
      if (sei->m_contextPresentFlag[i])
        sei->m_context[i] = m_pcCfg->getAURSEIContext(i);
    }
  }
}
#endif


#if JVET_AK0194_DSC_SEI
void SEIEncoder::initSEIDigitallySignedContentInitialization(SEIDigitallySignedContentInitialization *sei, bool startFlag)
{
  sei->dsciId = m_pcCfg->getDigitallySignedContentSEICfg().dscId;
  sei->dsciNumVerificationSubstreams = m_pcCfg->getDigitallySignedContentSEICfg().numVerificationSubstreams;
  sei->dsciRefSubstreamFlag = m_pcCfg->getDigitallySignedContentSEICfg().refSubstreamFlag;
  sei->dsciHashMethodType = m_pcCfg->getDigitallySignedContentSEICfg().hashMethod;
  sei->dsciKeySourceUri = m_pcCfg->getDigitallySignedContentSEICfg().publicKeyUri;
  sei->dsciVSSImplicitAssociationModeFlag = m_pcCfg->getDigitallySignedContentSEICfg().implicitAssociationModeFlag;
  sei->dsciUseKeyRegisterIdxFlag = m_pcCfg->getDigitallySignedContentSEICfg().keyIdEnabled;
  sei->dsciKeyRegisterIdx = m_pcCfg->getDigitallySignedContentSEICfg().keyId;
  sei->dsciSignedContentStartFlag = startFlag;
  sei->dsciSEISigningFlag = m_pcCfg->getDigitallySignedContentSEICfg().signAURSEI ||
                            m_pcCfg->getDigitallySignedContentSEICfg().signGFVSEI ||
                            m_pcCfg->getDigitallySignedContentSEICfg().signGFVESEI ||
                            m_pcCfg->getDigitallySignedContentSEICfg().signNNPFCSEI ||
                            m_pcCfg->getDigitallySignedContentSEICfg().signNNPFASEI;
}
void SEIEncoder::initSEIDigitallySignedContentSelection(SEIDigitallySignedContentSelection *sei, int substream)
{
  sei->dscsId = m_pcCfg->getDigitallySignedContentSEICfg().dscId;
  sei->dscsVerificationSubstreamId = substream;
}
void SEIEncoder::initSEIDigitallySignedContentVerification(SEIDigitallySignedContentVerification *sei, int32_t substream, const std::vector<uint8_t> &signature, bool endFlag)
{
  sei->dscvId = m_pcCfg->getDigitallySignedContentSEICfg().dscId;
  sei->dscvVerificationSubstreamId = substream;
  sei->dscvSignatureLengthInOctets = (int32_t) signature.size();
  sei->dscvSignature = signature;
  sei->dscvSignedContentEndFlag = endFlag;
}
#endif
#if JVET_AJ0207_GFV
Void SEIEncoder::initSEIGenerativeFaceVideo(SEIGenerativeFaceVideo *sei, int currframeindex)
{
  assert(m_isInitialized);
  assert(sei != nullptr);
  sei->m_number = m_pcCfg->getGenerativeFaceVideoSEINumber();
  sei->m_basePicFlag = m_pcCfg->getGenerativeFaceVideoSEIBasePicFlag();
  sei->m_nnPresentFlag = m_pcCfg->getGenerativeFaceVideoSEINNPresentFlag();
  sei->m_nnModeIdc = m_pcCfg->getGenerativeFaceVideoSEINNModeIdc();
  sei->m_nnTagURI = m_pcCfg->getGenerativeFaceVideoSEINNTagURI();
  sei->m_nnURI = m_pcCfg->getGenerativeFaceVideoSEINNURI();
  sei->m_chromaKeyInfoPresentFlag = m_pcCfg->getGenerativeFaceVideoSEIChromaKeyInfoPresentFlag();
  sei->m_chromaKeyValuePresentFlag.resize(3);
  sei->m_chromaKeyValue.resize(3);
  sei->m_chromaKeyThrPresentFlag.resize(2);
  sei->m_chromaKeyThrValue.resize(2);
  if (sei->m_chromaKeyInfoPresentFlag)
  {
    for (uint32_t chromac = 0; chromac < 3; chromac++)
    {
      sei->m_chromaKeyValuePresentFlag[chromac] = m_pcCfg->getGenerativeFaceVideoSEIChromaKeyValuePresentFlag(chromac);
      if (sei->m_chromaKeyValuePresentFlag[chromac])
      {
        sei->m_chromaKeyValue[chromac] = m_pcCfg->getGenerativeFaceVideoSEIChromaKeyValue(chromac);
      }
    }
    for (uint32_t chromai = 0; chromai < 2; chromai++)
    {
      sei->m_chromaKeyThrPresentFlag[chromai] = m_pcCfg->getGenerativeFaceVideoSEIChromaKeyThrPresentFlag(chromai);
      if (sei->m_chromaKeyThrPresentFlag[chromai])
      {
        sei->m_chromaKeyThrValue[chromai] = m_pcCfg->getGenerativeFaceVideoSEIChromaKeyThrValue(chromai);
      }
    }
  }
  sei->m_payloadFilename = m_pcCfg->getGenerativeFaceVideoSEIPayloadFilename();
  sei->m_currentid = currframeindex;
  sei->m_id = m_pcCfg->getGenerativeFaceVideoSEIId(sei->m_currentid);
  sei->m_cnt = m_pcCfg->getGenerativeFaceVideoSEICnt(sei->m_currentid);
  sei->m_drivePicFusionFlag = m_pcCfg->getGenerativeFaceVideoSEIDrivePicFusionFlag(sei->m_currentid);
  sei->m_lowConfidenceFaceParameterFlag = m_pcCfg->getGenerativeFaceVideoSEILowConfidenceFaceParameterFlag(sei->m_currentid);
  sei->m_coordinatePresentFlag = m_pcCfg->getGenerativeFaceVideoSEICoordinatePresentFlag(sei->m_currentid);
  sei->m_coordinateQuantizationFactor = m_pcCfg->getGenerativeFaceVideoSEICoordinateQuantizationFactor(sei->m_currentid);
  sei->m_coordinatePredFlag = m_pcCfg->getGenerativeFaceVideoSEICoordinatePredFlag(sei->m_currentid);
  sei->m_3DCoordinateFlag = m_pcCfg->getGenerativeFaceVideoSEI3DCoordinateFlag(sei->m_currentid);
  sei->m_coordinatePointNum = m_pcCfg->getGenerativeFaceVideoSEICoordinatePointNum(sei->m_currentid);
  // Coordinate Parameters
  if (sei->m_coordinatePresentFlag == 1)
  {
    for (uint32_t coordinateId = 0; coordinateId < sei->m_coordinatePointNum; coordinateId++)
    {
      sei->m_coordinateX.push_back(m_pcCfg->getGenerativeFaceVideoSEICoordinateXTesonr(sei->m_currentid, coordinateId));
      sei->m_coordinateY.push_back(m_pcCfg->getGenerativeFaceVideoSEICoordinateYTesonr(sei->m_currentid, coordinateId));
    }
    if (sei->m_3DCoordinateFlag == 1)
    {
      sei->m_coordinateZMaxValue = m_pcCfg->getGenerativeFaceVideoSEIZCoordinateMaxValue(sei->m_currentid, 0);
      for (uint32_t coordinateId = 0; coordinateId < sei->m_coordinatePointNum; coordinateId++)
      {
        sei->m_coordinateZ.push_back(m_pcCfg->getGenerativeFaceVideoSEICoordinateZTesonr(sei->m_currentid, coordinateId));
      }
    }
  }
  // Matrix Parameters
  sei->m_matrixPresentFlag = m_pcCfg->getGenerativeFaceVideoSEIMatrixPresentFlag(sei->m_currentid);
  sei->m_matrixElementPrecisionFactor = m_pcCfg->getGenerativeFaceVideoSEIMatrixElementPrecisionFactor(sei->m_currentid);
  sei->m_numMatrixType = m_pcCfg->getGenerativeFaceVideoSEINumMatrixType(sei->m_currentid);
  sei->m_matrixPredFlag = m_pcCfg->getGenerativeFaceVideoSEIMatrixPredFlag(sei->m_currentid);
  if (sei->m_matrixPresentFlag == 1)
  {
    uint32_t matrixWidth = 0;
    uint32_t matrixHeight = 0;
    uint32_t numMatrices = 0;
    for (uint32_t matrixId = 0; matrixId < sei->m_numMatrixType; matrixId++)
    {
      sei->m_matrixElement.push_back(std::vector<std::vector<std::vector<double>>>());
      sei->m_matrixTypeIdx.push_back(m_pcCfg->getGenerativeFaceVideoSEIMatrixTypeIdx(sei->m_currentid, matrixId));
      sei->m_matrix3DSpaceFlag.push_back(m_pcCfg->getGenerativeFaceVideoSEIMatrix3DSpaceFlag(sei->m_currentid, matrixId));
      sei->m_numMatrices.push_back(m_pcCfg->getGenerativeFaceVideoSEINumMatrices(sei->m_currentid, matrixId));
      sei->m_matrixWidth.push_back(m_pcCfg->getGenerativeFaceVideoSEIMatrixWidth(sei->m_currentid, matrixId));
      sei->m_matrixHeight.push_back(m_pcCfg->getGenerativeFaceVideoSEIMatrixHeight(sei->m_currentid, matrixId));
      sei->m_numMatricestonumKpsFlag.push_back(m_pcCfg->getGenerativeFaceVideoSEINumMatricestoNumKpsFlag(sei->m_currentid, matrixId));
      sei->m_numMatricesInfo.push_back(m_pcCfg->getGenerativeFaceVideoSEINumMatricesInfo(sei->m_currentid, matrixId));
      if (sei->m_matrixTypeIdx[matrixId] == 0 || sei->m_matrixTypeIdx[matrixId] == 1)
      {
        matrixHeight = sei->m_3DCoordinateFlag + 2;
        matrixWidth = sei->m_3DCoordinateFlag + 2;
      }
      else if (sei->m_matrixTypeIdx[matrixId] == 4)
      {
        matrixWidth = (sei->m_coordinatePresentFlag ? sei->m_3DCoordinateFlag : sei->m_matrix3DSpaceFlag[matrixId]) + 2;
        matrixHeight = (sei->m_coordinatePresentFlag ? sei->m_3DCoordinateFlag : sei->m_matrix3DSpaceFlag[matrixId]) + 2;
      }
      else if (sei->m_matrixTypeIdx[matrixId] == 5 || sei->m_matrixTypeIdx[matrixId] == 6)
      {
        matrixWidth = 1;
        matrixHeight = (sei->m_coordinatePresentFlag ? sei->m_3DCoordinateFlag : sei->m_matrix3DSpaceFlag[matrixId]) + 2;
      }
      else
      {
        matrixHeight = sei->m_matrixHeight[matrixId];
        matrixWidth = sei->m_matrixWidth[matrixId];
      }
      if (sei->m_matrixTypeIdx[matrixId] == 0 || sei->m_matrixTypeIdx[matrixId] == 1)
      {
        if (sei->m_coordinatePresentFlag)
        {
          numMatrices = sei->m_numMatricestonumKpsFlag[matrixId] ? sei->m_coordinatePointNum : (sei->m_numMatricesInfo[matrixId] < (sei->m_coordinatePointNum - 1) ? (sei->m_numMatricesInfo[matrixId] + 1) : (sei->m_numMatricesInfo[matrixId] + 2));
        }
        else
        {
          numMatrices = sei->m_numMatricesInfo[matrixId] + 1;
        }
      }
      else if (sei->m_matrixTypeIdx[matrixId] >= 2 && sei->m_matrixTypeIdx[matrixId] < 7)
      {
        numMatrices = 1;
      }
      else
      {
        numMatrices = sei->m_numMatrices[matrixId];
      }
      sei->m_numMatricesstore.push_back(numMatrices);
      sei->m_matrixWidthstore.push_back(matrixWidth);
      sei->m_matrixHeightstore.push_back(matrixHeight);
      for (uint32_t j = 0; j < numMatrices; j++)
      {
        sei->m_matrixElement[matrixId].push_back(std::vector< std::vector<double> >());
        for (uint32_t k = 0; k < matrixHeight; k++)
        {
          sei->m_matrixElement[matrixId][j].push_back(std::vector<double>());
          for (uint32_t l = 0; l < matrixWidth; l++)
          {
            sei->m_matrixElement[matrixId][j][k].push_back(m_pcCfg->getGenerativeFaceVideoSEIMatrixElement(sei->m_currentid, matrixId, j, k, l));
          }
        }
      }
    }
  }
  if (sei->m_nnPresentFlag)
  {
    if (sei->m_nnModeIdc == 0)
    {
      std::ifstream     bitstreamFile(sei->m_payloadFilename.c_str(), std::ifstream::in | std::ifstream::binary);
      if (!bitstreamFile)
      {
        std::cerr << "Failed to open bitstream file " << sei->m_payloadFilename.c_str() << " for reading";
        exit(1);
      }
      bitstreamFile.seekg(0, std::ifstream::end);
      sei->m_payloadLength = bitstreamFile.tellg();
      bitstreamFile.seekg(0, std::ifstream::beg);
      sei->m_payloadByte = new char[sei->m_payloadLength];
      bitstreamFile.read(sei->m_payloadByte, sei->m_payloadLength);
      bitstreamFile.close();
    }
  }
}
#if JVET_AK0239_GEFV
Void SEIEncoder::initSEIGenerativeFaceVideoEnhancement(SEIGenerativeFaceVideoEnhancement *sei, int currframeindex)
{
  assert(m_isInitialized);
  assert(sei != nullptr);
  sei->m_number = m_pcCfg->getGenerativeFaceVideoEnhancementSEINumber();
  sei->m_basePicFlag = m_pcCfg->getGenerativeFaceVideoEnhancementSEIBasePicFlag();
  sei->m_nnPresentFlag = m_pcCfg->getGenerativeFaceVideoEnhancementSEINNPresentFlag();
  sei->m_nnModeIdc = m_pcCfg->getGenerativeFaceVideoEnhancementSEINNModeIdc();
  sei->m_nnTagURI = m_pcCfg->getGenerativeFaceVideoEnhancementSEINNTagURI();
  sei->m_nnURI = m_pcCfg->getGenerativeFaceVideoEnhancementSEINNURI();
  sei->m_payloadFilename = m_pcCfg->getGenerativeFaceVideoEnhancementSEIPayloadFilename();
  sei->m_currentid = currframeindex;
  sei->m_id = m_pcCfg->getGenerativeFaceVideoEnhancementSEIId(sei->m_currentid);
  sei->m_gfvcnt = m_pcCfg->getGenerativeFaceVideoEnhancementSEIGFVCnt(sei->m_currentid);
  sei->m_gfvid = m_pcCfg->getGenerativeFaceVideoEnhancementSEIGFVId(sei->m_currentid);
  sei->m_pupilPresentIdx = m_pcCfg->getGenerativeFaceVideoEnhancementSEIPupilPresentIdx(sei->m_currentid);
  sei->m_pupilCoordinatePrecisionFactor = m_pcCfg->getGenerativeFaceVideoEnhancementSEIPupilCoordinatePrecisionFactor(sei->m_currentid);
  sei->m_pupilLeftEyeCoordinateX = m_pcCfg->getGenerativeFaceVideoEnhancementSEIPupilLeftEyeCoordinateX(sei->m_currentid);
  sei->m_pupilLeftEyeCoordinateY = m_pcCfg->getGenerativeFaceVideoEnhancementSEIPupilLeftEyeCoordinateY(sei->m_currentid);
  sei->m_pupilRightEyeCoordinateX = m_pcCfg->getGenerativeFaceVideoEnhancementSEIPupilRightEyeCoordinateX(sei->m_currentid);
  sei->m_pupilRightEyeCoordinateY = m_pcCfg->getGenerativeFaceVideoEnhancementSEIPupilRightEyeCoordinateY(sei->m_currentid);

  sei->m_matrixElementPrecisionFactor = m_pcCfg->getGenerativeFaceVideoEnhancementSEIMatrixElementPrecisionFactor(sei->m_currentid);
  sei->m_numMatrices = m_pcCfg->getGenerativeFaceVideoEnhancementSEINumMatrices(sei->m_currentid);
  sei->m_matrixPresentFlag = m_pcCfg->getGenerativeFaceVideoEnhancementSEIMatrixPresentFlag(sei->m_currentid);
  sei->m_matrixPredFlag = m_pcCfg->getGenerativeFaceVideoEnhancementSEIMatrixPredFlag(sei->m_currentid);
  if (sei->m_matrixPresentFlag == 1)
  {
    for (uint32_t j = 0; j < sei->m_numMatrices; j++)
    {
      sei->m_matrixElement.push_back(std::vector< std::vector<double>>());

      sei->m_matrixWidth.push_back(m_pcCfg->getGenerativeFaceVideoEnhancementSEIMatrixWidth(sei->m_currentid, j));
      sei->m_matrixHeight.push_back(m_pcCfg->getGenerativeFaceVideoEnhancementSEIMatrixHeight(sei->m_currentid, j));
      for (uint32_t k = 0; k < sei->m_matrixWidth[j]; k++)
      {
        sei->m_matrixElement[j].push_back(std::vector<double>());
        for (uint32_t l = 0; l < sei->m_matrixHeight[j]; l++)
        {
          sei->m_matrixElement[j][k].push_back(m_pcCfg->getGenerativeFaceVideoEnhancementSEIMatrixElement(sei->m_currentid, j, k, l));
        }
      }
    }
  }
  if (sei->m_nnPresentFlag)
  {
    if (sei->m_nnModeIdc == 0)
    {
      std::ifstream     bitstreamFile(sei->m_payloadFilename.c_str(), std::ifstream::in | std::ifstream::binary);
      if (!bitstreamFile)
      {
        std::cerr << "Failed to open bitstream file " << sei->m_payloadFilename.c_str() << " for reading";
        exit(1);
      }
      bitstreamFile.seekg(0, std::ifstream::end);
      sei->m_payloadLength = bitstreamFile.tellg();
      bitstreamFile.seekg(0, std::ifstream::beg);
      sei->m_payloadByte = new char[sei->m_payloadLength];
      bitstreamFile.read(sei->m_payloadByte, sei->m_payloadLength);
      bitstreamFile.close();
    }
  }
}
#endif
#if JVET_AK0140_PACKED_REGIONS_INFORMATION_SEI
void SEIEncoder::initSEIPackedRegionsInfo(SEIPackedRegionsInfo* sei)
{
  sei->m_cancelFlag = m_pcCfg->getPriSEICancelFlag();
  sei->m_persistenceFlag = m_pcCfg->getPriSEIPersistenceFlag();
  sei->m_numRegionsMinus1 = m_pcCfg->getPriSEINumRegionsMinus1();
  sei->m_layerId = 0;  // Always 0 in HM encoder
  sei->m_multilayerFlag = m_pcCfg->getPriSEIMultilayerFlag();
  sei->m_useMaxDimensionsFlag = m_pcCfg->getPriSEIUseMaxDimensionsFlag();
  sei->m_log2UnitSize = m_pcCfg->getPriSEILog2UnitSize();
  sei->m_regionSizeLenMinus1 = m_pcCfg->getPriSEIRegionSizeLenMinus1();
  sei->m_regionIdPresentFlag = m_pcCfg->getPriSEIRegionIdPresentFlag();
  sei->m_targetPicParamsPresentFlag = m_pcCfg->getPriSEITargetPicParamsPresentFlag();
  sei->m_targetPicWidthMinus1 = m_pcCfg->getPriSEITargetPicWidthMinus1();
  sei->m_targetPicHeightMinus1 = m_pcCfg->getPriSEITargetPicHeightMinus1();
  sei->m_numResamplingRatiosMinus1 = m_pcCfg->getPriSEINumResamplingRatiosMinus1();

  sei->m_resamplingWidthNumMinus1.resize(sei->m_numResamplingRatiosMinus1 + 1);
  sei->m_resamplingWidthDenomMinus1.resize(sei->m_numResamplingRatiosMinus1 + 1);
  sei->m_fixedAspectRatioFlag.resize(sei->m_numResamplingRatiosMinus1 + 1);
  sei->m_resamplingHeightNumMinus1.resize(sei->m_numResamplingRatiosMinus1 + 1);
  sei->m_resamplingHeightDenomMinus1.resize(sei->m_numResamplingRatiosMinus1 + 1);
  for (uint32_t i = 0; i <= sei->m_numResamplingRatiosMinus1; i++)
  {
    sei->m_resamplingWidthNumMinus1[i] = m_pcCfg->getPriSEIResamplingWidthNumMinus1(i);
    sei->m_resamplingWidthDenomMinus1[i] = m_pcCfg->getPriSEIResamplingWidthDenomMinus1(i);
    sei->m_fixedAspectRatioFlag[i] = m_pcCfg->getPriSEIFixedAspectRatioFlag(i);
    sei->m_resamplingHeightNumMinus1[i] = m_pcCfg->getPriSEIResamplingHeightNumMinus1(i);
    sei->m_resamplingHeightDenomMinus1[i] = m_pcCfg->getPriSEIResamplingHeightDenomMinus1(i);
  }

  sei->m_regionId.resize(sei->m_numRegionsMinus1 + 1);
  sei->m_regionLayerId.resize(sei->m_numRegionsMinus1 + 1);
  sei->m_regionIsALayerFlag.resize(sei->m_numRegionsMinus1 + 1);
  sei->m_regionTopLeftInUnitsX.resize(sei->m_numRegionsMinus1 + 1);
  sei->m_regionTopLeftInUnitsY.resize(sei->m_numRegionsMinus1 + 1);
  sei->m_regionWidthInUnitsMinus1.resize(sei->m_numRegionsMinus1 + 1);
  sei->m_regionHeightInUnitsMinus1.resize(sei->m_numRegionsMinus1 + 1);
  sei->m_resamplingRatioIdx.resize(sei->m_numRegionsMinus1 + 1);
  sei->m_targetRegionTopLeftInUnitsX.resize(sei->m_numRegionsMinus1 + 1);
  sei->m_targetRegionTopLeftInUnitsY.resize(sei->m_numRegionsMinus1 + 1);
  for (uint32_t i = 0; i <= sei->m_numRegionsMinus1; i++)
  {
    sei->m_regionId[i] = m_pcCfg->getPriSEIRegionId(i);
    if (sei->m_multilayerFlag)
    {
      sei->m_regionLayerId[i] = m_pcCfg->getPriSEIRegionLayerId(i);
      sei->m_regionIsALayerFlag[i] = m_pcCfg->getPriSEIRegionIsALayerFlag(i);
    }
    else
    {
      sei->m_regionLayerId[i] = 0;
      sei->m_regionIsALayerFlag[i] = 0;
    }
    if (!sei->m_regionIsALayerFlag[i])
    {
      sei->m_regionTopLeftInUnitsX[i] = m_pcCfg->getPriSEIRegionTopLeftInUnitsX(i);
      sei->m_regionTopLeftInUnitsY[i] = m_pcCfg->getPriSEIRegionTopLeftInUnitsY(i);
      sei->m_regionWidthInUnitsMinus1[i] = m_pcCfg->getPriSEIRegionWidthInUnitsMinus1(i);
      sei->m_regionHeightInUnitsMinus1[i] = m_pcCfg->getPriSEIRegionHeightInUnitsMinus1(i);
    }
    else
    {
      sei->m_regionTopLeftInUnitsX[i] = 0;
      sei->m_regionTopLeftInUnitsY[i] = 0;
      sei->m_regionWidthInUnitsMinus1[i] = 0;
      sei->m_regionHeightInUnitsMinus1[i] = 0;
    }
    sei->m_resamplingRatioIdx[i] = m_pcCfg->getPriSEIResamplingRatioIdx(i);
    sei->m_targetRegionTopLeftInUnitsX[i] = m_pcCfg->getPriSEITargetRegionTopLeftInUnitsX(i);
    sei->m_targetRegionTopLeftInUnitsY[i] = m_pcCfg->getPriSEITargetRegionTopLeftInUnitsY(i);
  }
}
#endif

#if JVET_AK2006_SPTI_SEI_MESSAGE
void SEIEncoder::initSEISourcePictureTimingInfo(SEISourcePictureTimingInfo *SEISourcePictureTimingInfo) 
{

  assert(m_isInitialized);
  assert(SEISourcePictureTimingInfo != NULL);

  SEISourcePictureTimingInfo->m_sptiSEIEnabled = m_pcCfg->getSptiSEIEnabled();
  SEISourcePictureTimingInfo->m_sptiSourceTimingEqualsOutputTimingFlag = m_pcCfg->getmSptiSEISourceTimingEqualsOutputTimingFlag();
  SEISourcePictureTimingInfo->m_sptiSourceType = m_pcCfg->getmSptiSEISourceType();
  SEISourcePictureTimingInfo->m_sptiTimeScale = m_pcCfg->getmSptiSEITimeScale();
  SEISourcePictureTimingInfo->m_sptiNumUnitsInElementalInterval = m_pcCfg->getmSptiSEINumUnitsInElementalInterval();
  SEISourcePictureTimingInfo->m_sptiDirectionFlag = m_pcCfg->getmSptiSEIDirectionFlag();
  SEISourcePictureTimingInfo->m_sptiMaxSublayersMinus1 = m_pcCfg->getMaxTempLayer() - 1;
  SEISourcePictureTimingInfo->m_sptiCancelFlag = 0;
  SEISourcePictureTimingInfo->m_sptiPersistenceFlag = 1;
  SEISourcePictureTimingInfo->m_sptiSourceTypePresentFlag = (SEISourcePictureTimingInfo->m_sptiSourceType == 0 ? 0 : 1);

  int sptiMinTemporalSublayer = (SEISourcePictureTimingInfo->m_sptiPersistenceFlag ? 0 : SEISourcePictureTimingInfo->m_sptiMaxSublayersMinus1);

  for (int i = sptiMinTemporalSublayer; i <= SEISourcePictureTimingInfo->m_sptiMaxSublayersMinus1; i++) 
  {
    SEISourcePictureTimingInfo->m_sptiSublayerIntervalScaleFactor[i] = 1 << (SEISourcePictureTimingInfo->m_sptiMaxSublayersMinus1 - i);
    SEISourcePictureTimingInfo->m_sptiSublayerSynthesizedPictureFlag[i] = false;
  }
#endif
}

#endif


 
//! \}
