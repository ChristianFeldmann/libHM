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

/** \file     SEI.cpp
    \brief    helper functions for SEI handling
*/

#include "CommonDef.h"
#include "SEI.h"
#include <iostream>

const std::vector<SEI::PayloadType> SEI::prefix_sei_messages({
  SEI::BUFFERING_PERIOD,
  SEI::PICTURE_TIMING,
  SEI::PAN_SCAN_RECT,
  SEI::FILLER_PAYLOAD,
  SEI::USER_DATA_REGISTERED_ITU_T_T35,
  SEI::USER_DATA_UNREGISTERED,
  SEI::RECOVERY_POINT,
  SEI::SCENE_INFO,
  SEI::PICTURE_SNAPSHOT,
  SEI::PROGRESSIVE_REFINEMENT_SEGMENT_START,
  SEI::PROGRESSIVE_REFINEMENT_SEGMENT_END,
  SEI::FILM_GRAIN_CHARACTERISTICS,
  SEI::POST_FILTER_HINT,
  SEI::TONE_MAPPING_INFO,
  SEI::FRAME_PACKING,
  SEI::DISPLAY_ORIENTATION,
  SEI::GREEN_METADATA,
  SEI::SOP_DESCRIPTION,
  SEI::ACTIVE_PARAMETER_SETS,
  SEI::DECODING_UNIT_INFO,
  SEI::TEMPORAL_LEVEL0_INDEX,
  SEI::SCALABLE_NESTING,
  SEI::REGION_REFRESH_INFO,
  SEI::NO_DISPLAY,
  SEI::TIME_CODE,
  SEI::MASTERING_DISPLAY_COLOUR_VOLUME,
  SEI::SEGM_RECT_FRAME_PACKING,
  SEI::TEMP_MOTION_CONSTRAINED_TILE_SETS,
  SEI::CHROMA_RESAMPLING_FILTER_HINT,
  SEI::KNEE_FUNCTION_INFO,
  SEI::COLOUR_REMAPPING_INFO,
  SEI::DEINTERLACE_FIELD_IDENTIFICATION,
  SEI::CONTENT_LIGHT_LEVEL_INFO,
  SEI::DEPENDENT_RAP_INDICATION,
  SEI::CODED_REGION_COMPLETION,
  SEI::ALTERNATIVE_TRANSFER_CHARACTERISTICS,
  SEI::AMBIENT_VIEWING_ENVIRONMENT,
  SEI::CONTENT_COLOUR_VOLUME,
  SEI::EQUIRECTANGULAR_PROJECTION,
  SEI::SPHERE_ROTATION,
  SEI::OMNI_VIEWPORT,
  SEI::CUBEMAP_PROJECTION,
  SEI::REGION_WISE_PACKING,
  SEI::FISHEYE_VIDEO_INFO,
  SEI::REGIONAL_NESTING,
#if SHUTTER_INTERVAL_SEI_MESSAGE
  SEI::SHUTTER_INTERVAL_INFO,
#endif
#if NNPFC_SEI_MESSAGE
  SEI::NEURAL_NETWORK_POST_FILTER_CHARACTERISTICS,
#endif
#if NNPFA_SEI_MESSAGE
  SEI::NEURAL_NETWORK_POST_FILTER_ACTIVATION,
#endif
#if JVET_AE0101_PHASE_INDICATION_SEI_MESSAGE
  SEI::PHASE_INDICATION,
#endif
#if JVET_AK0107_MODALITY_INFORMATION
  SEI::MODALITY_INFORMATION,
#endif
#if JVET_AK0194_DSC_SEI_DECODER_SYNTAX
  SEI::DIGITALLY_SIGNED_CONTENT_INITIALIZATION,
  SEI::DIGITALLY_SIGNED_CONTENT_SELECTION,
#endif
#if JVET_AL0062_AI_USAGE_RESTRICTIONS_SEI
  SEI::AI_USAGE_RESTRICTIONS,
#endif
#if JVET_AL0061_ENCODER_OPTIMIZATION_INFORMATION_SEI
  SEI::ENCODER_OPTIMIZATION_INFO,
#endif
});

const std::vector<SEI::PayloadType> SEI::suffix_sei_messages({
  SEI::FILLER_PAYLOAD,
  SEI::USER_DATA_REGISTERED_ITU_T_T35,
  SEI::USER_DATA_UNREGISTERED,
  SEI::PROGRESSIVE_REFINEMENT_SEGMENT_END,
  SEI::POST_FILTER_HINT,
  SEI::DECODED_PICTURE_HASH,
  SEI::CODED_REGION_COMPLETION,
#if JVET_AL0061_ENCODER_OPTIMIZATION_INFORMATION_SEI
  SEI::ENCODER_OPTIMIZATION_INFO,
#endif
#if JVET_AK0194_DSC_SEI_DECODER_SYNTAX
  SEI::DIGITALLY_SIGNED_CONTENT_VERIFICATION,
#endif
  #if JVET_AJ0207_GFV
  SEI::GENERATIVE_FACE_VIDEO,
#endif
#if JVET_AK0239_GEFV
  SEI::GENERATIVE_ENHANCEMENT_FACE_VIDEO
#endif
});

const std::vector<SEI::PayloadType> SEI::regional_nesting_sei_messages({
  SEI::USER_DATA_REGISTERED_ITU_T_T35,
  SEI::USER_DATA_UNREGISTERED,
  SEI::FILM_GRAIN_CHARACTERISTICS,
  SEI::POST_FILTER_HINT,
  SEI::TONE_MAPPING_INFO,
  SEI::CHROMA_RESAMPLING_FILTER_HINT,
  SEI::KNEE_FUNCTION_INFO,
  SEI::COLOUR_REMAPPING_INFO,
  SEI::CONTENT_COLOUR_VOLUME,
});

SEIMessages getSeisByType(SEIMessages &seiList, SEI::PayloadType seiType)
{
  SEIMessages result;

  for (SEIMessages::iterator it=seiList.begin(); it!=seiList.end(); it++)
  {
    if ((*it)->payloadType() == seiType)
    {
      result.push_back(*it);
    }
  }
  return result;
}

SEIMessages extractSeisByType(SEIMessages &seiList, SEI::PayloadType seiType)
{
  SEIMessages result;

  SEIMessages::iterator it=seiList.begin();
  while ( it!=seiList.end() )
  {
    if ((*it)->payloadType() == seiType)
    {
      result.push_back(*it);
      it = seiList.erase(it);
    }
    else
    {
      it++;
    }
  }
  return result;
}


Void deleteSEIs (SEIMessages &seiList)
{
  for (SEIMessages::iterator it=seiList.begin(); it!=seiList.end(); it++)
  {
    delete (*it);
  }
  seiList.clear();
}

void SEIBufferingPeriod::copyTo (SEIBufferingPeriod& target)
{
  target.m_bpSeqParameterSetId = m_bpSeqParameterSetId;
  target.m_rapCpbParamsPresentFlag = m_rapCpbParamsPresentFlag;
  target.m_cpbDelayOffset = m_cpbDelayOffset;
  target.m_dpbDelayOffset = m_dpbDelayOffset;
  target.m_concatenationFlag = m_concatenationFlag;
  target.m_auCpbRemovalDelayDelta = m_auCpbRemovalDelayDelta;
  ::memcpy(target.m_initialCpbRemovalDelay, m_initialCpbRemovalDelay, sizeof(m_initialCpbRemovalDelay));
  ::memcpy(target.m_initialCpbRemovalDelayOffset, m_initialCpbRemovalDelayOffset, sizeof(m_initialCpbRemovalDelayOffset));
  ::memcpy(target.m_initialAltCpbRemovalDelay, m_initialAltCpbRemovalDelay, sizeof(m_initialAltCpbRemovalDelay));
  ::memcpy(target.m_initialAltCpbRemovalDelayOffset, m_initialAltCpbRemovalDelayOffset, sizeof(m_initialAltCpbRemovalDelayOffset));
}

void SEIPictureTiming::copyTo (SEIPictureTiming& target)
{
  target.m_picStruct = m_picStruct;
  target.m_sourceScanType = m_sourceScanType;
  target.m_duplicateFlag = m_duplicateFlag;

  target.m_auCpbRemovalDelay = m_auCpbRemovalDelay;
  target.m_picDpbOutputDelay = m_picDpbOutputDelay;
  target.m_picDpbOutputDuDelay = m_picDpbOutputDuDelay;
  target.m_numDecodingUnitsMinus1 = m_numDecodingUnitsMinus1;
  target.m_duCommonCpbRemovalDelayFlag = m_duCommonCpbRemovalDelayFlag;
  target.m_duCommonCpbRemovalDelayMinus1 = m_duCommonCpbRemovalDelayMinus1;

  target.m_numNalusInDuMinus1 = m_numNalusInDuMinus1;
  target.m_duCpbRemovalDelayMinus1 = m_duCpbRemovalDelayMinus1;
}

std::ostream& operator<<(std::ostream  &os, RNSEIWindow const &region)
{
  os << region.getRegionId() << " " << region.getWindowLeftOffset() <<
      region.getWindowRightOffset() << " " << region.getWindowTopOffset() << " "  <<
      region.getWindowBottomOffset() << "\n";
  return os;
}

SEIRegionalNesting::~SEIRegionalNesting()
{
  // Delete SEI messages
  for(Int i = 0; i < m_rnSeiMessages.size(); i++)
  {
    delete m_rnSeiMessages[i].m_seiMessage;
  }
}

Void SEIRegionalNesting::addRegionalSEI(RegionalSEI *regSEI)
{
  // Check if no conflict with region IDs of regions
  const RNSEIWindowVec newRegions = regSEI->getRegions();
  RNSEIWindowVec regionsToAdd;
  std::vector<UInt> listOfIndices;

  // Loop through regions to add
  for(RNSEIWindowVec::const_iterator iterNew = newRegions.begin();
            iterNew != newRegions.end(); iterNew++)
  {
    Bool addNewRegion = m_regions.empty() ? true : false;
    Bool foundRegion = false;
    // Loop through regions already present
    for(RNSEIWindowVec::const_iterator iterRef = m_regions.begin();
            !foundRegion && iterRef != m_regions.end(); iterRef++)
    {
      if( (*iterNew) == (*iterRef) )  // Check if same region present
      {
        listOfIndices.push_back((UInt)(iterRef - m_regions.begin()));  // Add index
        foundRegion = true;
      }
      else if( iterRef->checkSameID(*iterNew) )  // Check if there is a region ID class
      {
        // Two regions are different yet have same ID value; violates constraint
        std::cout << "Two different regions have the same ID; please check.\n";
        std::cout << "Region:" << (*iterNew) << "\n";
        std::cout << "Region:" << (*iterRef) << "\n";
        exit(1);
      }
    }        
    addNewRegion = !foundRegion;
    
    if(addNewRegion)
    {
      // Index is current size of the m_regions;
      listOfIndices.push_back((UInt)m_regions.size());
      m_regions.push_back((*iterNew));
    }
  }
  if(listOfIndices.empty())
  {
    std::cout << "Unable to add regions to the regional nesting SEI.\n";
    exit(1);
  }
  SEIListOfIndices seiWithListOfIndices(listOfIndices, regSEI->dissociateSEIObject());
  addRegionalSEI(seiWithListOfIndices);
}

#if JCTVC_AD0021_SEI_MANIFEST
SEIManifest::SEIManifestDescription SEIManifest::getSEIMessageDescription(const PayloadType payloadType)
{
  std::vector<PayloadType> necessary = { FRAME_PACKING, EQUIRECTANGULAR_PROJECTION  /*,GENERALIZED_CUBEMAP_PROJECTION*/
    ,SPHERE_ROTATION, REGION_WISE_PACKING };

  std::vector<PayloadType> undetermined = { USER_DATA_REGISTERED_ITU_T_T35, USER_DATA_UNREGISTERED };

  for (auto pt : necessary)
  {
    if (payloadType == pt)
    {
      return NECESSARY_SEI_MESSAGE;
    }
  }
  for (auto pt : undetermined)
  {
    if (payloadType == pt)
    {
      return UNDETERMINED_SEI_MESSAGE;
    }
  }
  return UNNECESSARY_SEI_MESSAGE;
}
#endif

#if JCTVC_AD0021_SEI_PREFIX_INDICATION
uint8_t SEIPrefixIndication::getNumsOfSeiPrefixIndications(const SEI* sei)
{
  PayloadType payloadType = sei->payloadType();

  //Unable to determine how many indicators are needed, it will be determined in xWriteSEIPrefixIndication() return 3
  std::vector<PayloadType> indicationN = { REGION_WISE_PACKING };
  // Need two indicators to finish writing the SEI prefix indication message(return 2)
  std::vector<PayloadType> indication2 = { SPHERE_ROTATION };

  for (auto pt : indicationN)
  {
    if (payloadType == pt)
    {
      return 3;
    }
  }
  for (auto pt : indication2)
  {
    if (payloadType == pt)
    {
      return 2;
    }
  }
  return 1;
}
#endif

#if NNPFC_SEI_MESSAGE
SEINeuralNetworkPostFilterCharacteristics::SEINeuralNetworkPostFilterCharacteristics(
  const SEINeuralNetworkPostFilterCharacteristics& sei)
{
  m_id = sei.m_id;
  m_modeIdc = sei.m_modeIdc;
  m_propertyPresentFlag = sei.m_propertyPresentFlag;
  m_purpose = sei.m_purpose;
  m_outSubCFlag = sei.m_outSubCFlag;
  m_outSubWidthC = sei.m_outSubWidthC;
  m_outSubHeightC = sei.m_outSubHeightC;
  m_outColourFormatIdc = sei.m_outColourFormatIdc;
  m_picWidthNumeratorMinus1 = sei.m_picWidthNumeratorMinus1;
  m_picWidthDenominatorMinus1 = sei.m_picWidthDenominatorMinus1;
  m_picHeightNumeratorMinus1 = sei.m_picHeightNumeratorMinus1;
  m_picHeightDenominatorMinus1 = sei.m_picHeightDenominatorMinus1;
  m_picWidthInLumaSamples = sei.m_picWidthInLumaSamples;
  m_picHeightInLumaSamples = sei.m_picHeightInLumaSamples;
  m_inpTensorBitDepthLumaMinus8 = sei.m_inpTensorBitDepthLumaMinus8;
  m_inpTensorBitDepthChromaMinus8 = sei.m_inpTensorBitDepthChromaMinus8;
  m_outTensorBitDepthLumaMinus8 = sei.m_outTensorBitDepthLumaMinus8;
  m_outTensorBitDepthChromaMinus8 = sei.m_outTensorBitDepthChromaMinus8;
  m_componentLastFlag = sei.m_componentLastFlag;
  m_inpFormatIdc = sei.m_inpFormatIdc;
  m_auxInpIdc = sei.m_auxInpIdc;
  m_sepColDescriptionFlag = sei.m_sepColDescriptionFlag;
  m_fullRangeFlag = sei.m_fullRangeFlag;
  m_colPrimaries = sei.m_colPrimaries;
  m_transCharacteristics = sei.m_transCharacteristics;
  m_matrixCoeffs = sei.m_matrixCoeffs;
  m_inpOrderIdc = sei.m_inpOrderIdc;
  m_outFormatIdc = sei.m_outFormatIdc;
  m_outOrderIdc = sei.m_outOrderIdc;
  m_constantPatchSizeFlag = sei.m_constantPatchSizeFlag;
  m_patchWidthMinus1 = sei.m_patchWidthMinus1;
  m_patchHeightMinus1 = sei.m_patchHeightMinus1;
  m_extendedPatchWidthCdDeltaMinus1 = sei.m_extendedPatchWidthCdDeltaMinus1;
  m_extendedPatchHeightCdDeltaMinus1 = sei.m_extendedPatchHeightCdDeltaMinus1;
  m_overlap = sei.m_overlap;
  m_paddingType = sei.m_paddingType;
  m_lumaPadding = sei.m_lumaPadding;
  m_cbPadding = sei.m_cbPadding;
  m_crPadding = sei.m_crPadding;
  m_payloadLength = sei.m_payloadLength;
  m_payloadByte = sei.m_payloadByte ? new TChar(*sei.m_payloadByte) : nullptr;
  m_complexityInfoPresentFlag = sei.m_complexityInfoPresentFlag;
  m_applicationPurposeTagUriPresentFlag = sei.m_applicationPurposeTagUriPresentFlag;
  m_applicationPurposeTagUri = sei.m_applicationPurposeTagUri;
  m_scanTypeIdc = sei.m_scanTypeIdc;
  m_forHumanViewingIdc = sei.m_forHumanViewingIdc;
  m_forMachineAnalysisIdc = sei.m_forMachineAnalysisIdc;
  m_uriTag = sei.m_uriTag;
  m_uri = sei.m_uri;
  m_parameterTypeIdc = sei.m_parameterTypeIdc;
  m_log2ParameterBitLengthMinus3 = sei.m_log2ParameterBitLengthMinus3;
  m_numParametersIdc = sei.m_numParametersIdc;
  m_numKmacOperationsIdc = sei.m_numKmacOperationsIdc;
  m_totalKilobyteSize = sei.m_totalKilobyteSize;
  m_numberInputDecodedPicturesMinus1 = sei.m_numberInputDecodedPicturesMinus1;
  m_numberInterpolatedPictures = sei.m_numberInterpolatedPictures;
  m_numberExtrapolatedPicturesMinus1 = sei.m_numberExtrapolatedPicturesMinus1;
  m_spatialExtrapolationLeftOffset = sei.m_spatialExtrapolationLeftOffset;
  m_spatialExtrapolationRightOffset = sei.m_spatialExtrapolationRightOffset;
  m_spatialExtrapolationTopOffset = sei.m_spatialExtrapolationTopOffset;
  m_spatialExtrapolationBottomOffset = sei.m_spatialExtrapolationBottomOffset;
  m_inbandPromptFlag = sei.m_inbandPromptFlag;
  m_prompt =  sei.m_prompt;
  m_inputPicOutputFlag = sei.m_inputPicOutputFlag;
  m_inbandSeedFlag = sei.m_inbandSeedFlag;
  m_seed = sei.m_seed;
}

Bool SEINeuralNetworkPostFilterCharacteristics::operator == (const SEINeuralNetworkPostFilterCharacteristics& sei)
{
  Bool result = 
  m_id == sei.m_id &&
  m_modeIdc == sei.m_modeIdc &&
  m_propertyPresentFlag == sei.m_propertyPresentFlag &&
  m_purpose == sei.m_purpose &&
  m_outSubCFlag == sei.m_outSubCFlag &&
  m_outSubWidthC == sei.m_outSubWidthC &&
  m_outSubHeightC == sei.m_outSubHeightC &&
  m_outColourFormatIdc == sei.m_outColourFormatIdc &&
  m_picWidthNumeratorMinus1 == sei.m_picWidthNumeratorMinus1 &&
  m_picWidthDenominatorMinus1 == sei.m_picWidthDenominatorMinus1 &&
  m_picHeightNumeratorMinus1 == sei.m_picHeightNumeratorMinus1 &&
  m_picHeightDenominatorMinus1 == sei.m_picHeightDenominatorMinus1 &&
  m_picWidthInLumaSamples == sei.m_picWidthInLumaSamples &&
  m_picHeightInLumaSamples == sei.m_picHeightInLumaSamples &&
  m_inpTensorBitDepthLumaMinus8 == sei.m_inpTensorBitDepthLumaMinus8 &&
  m_inpTensorBitDepthChromaMinus8 == sei.m_inpTensorBitDepthChromaMinus8 &&
  m_outTensorBitDepthLumaMinus8 == sei.m_outTensorBitDepthLumaMinus8 &&
  m_outTensorBitDepthChromaMinus8 == sei.m_outTensorBitDepthChromaMinus8 &&
  m_componentLastFlag == sei.m_componentLastFlag &&
  m_inpFormatIdc == sei.m_inpFormatIdc &&
  m_auxInpIdc == sei.m_auxInpIdc &&
  m_sepColDescriptionFlag == sei.m_sepColDescriptionFlag &&
  m_fullRangeFlag == sei.m_fullRangeFlag &&
  m_colPrimaries == sei.m_colPrimaries &&
  m_transCharacteristics == sei.m_transCharacteristics &&
  m_matrixCoeffs == sei.m_matrixCoeffs &&
  m_inpOrderIdc == sei.m_inpOrderIdc &&
  m_outFormatIdc == sei.m_outFormatIdc &&
  m_outOrderIdc == sei.m_outOrderIdc &&
  m_constantPatchSizeFlag == sei.m_constantPatchSizeFlag &&
  m_patchWidthMinus1 == sei.m_patchWidthMinus1 &&
  m_patchHeightMinus1 == sei.m_patchHeightMinus1 &&
  m_extendedPatchWidthCdDeltaMinus1 == sei.m_extendedPatchWidthCdDeltaMinus1 &&
  m_extendedPatchHeightCdDeltaMinus1 == sei.m_extendedPatchHeightCdDeltaMinus1 &&
  m_overlap == sei.m_overlap &&
  m_paddingType == sei.m_paddingType &&
  m_lumaPadding == sei.m_lumaPadding &&
  m_cbPadding == sei.m_cbPadding &&
  m_crPadding == sei.m_crPadding &&
  m_complexityInfoPresentFlag == sei.m_complexityInfoPresentFlag &&
  m_applicationPurposeTagUriPresentFlag == sei.m_applicationPurposeTagUriPresentFlag &&
  m_applicationPurposeTagUri == sei.m_applicationPurposeTagUri &&
  m_scanTypeIdc == sei.m_scanTypeIdc &&
  m_forHumanViewingIdc == sei.m_forHumanViewingIdc &&
  m_forMachineAnalysisIdc == sei.m_forMachineAnalysisIdc &&
  m_uriTag == sei.m_uriTag &&
  m_uri == sei.m_uri &&
  m_parameterTypeIdc == sei.m_parameterTypeIdc &&
  m_log2ParameterBitLengthMinus3 == sei.m_log2ParameterBitLengthMinus3 &&
  m_numParametersIdc == sei.m_numParametersIdc &&
  m_numKmacOperationsIdc == sei.m_numKmacOperationsIdc &&
  m_totalKilobyteSize == sei.m_totalKilobyteSize &&
  m_numberInputDecodedPicturesMinus1 == sei.m_numberInputDecodedPicturesMinus1 &&
  m_numberInterpolatedPictures == sei.m_numberInterpolatedPictures &&
  m_numberExtrapolatedPicturesMinus1 == sei.m_numberExtrapolatedPicturesMinus1 &&
  m_spatialExtrapolationLeftOffset == sei.m_spatialExtrapolationLeftOffset &&
  m_spatialExtrapolationRightOffset == sei.m_spatialExtrapolationRightOffset &&
  m_spatialExtrapolationTopOffset == sei.m_spatialExtrapolationTopOffset &&
  m_spatialExtrapolationBottomOffset == sei.m_spatialExtrapolationBottomOffset &&
  m_inbandPromptFlag == sei.m_inbandPromptFlag  &&
  m_prompt ==  sei.m_prompt  &&
  m_inputPicOutputFlag == sei.m_inputPicOutputFlag &&
  m_payloadLength == sei.m_payloadLength &&
  m_inbandSeedFlag == sei.m_inbandSeedFlag &&
  m_seed == sei.m_seed;

  if (m_payloadByte && sei.m_payloadByte && m_payloadLength == sei.m_payloadLength)
  {
    result &= !std::strncmp(m_payloadByte, sei.m_payloadByte, m_payloadLength);
  }
  else if ((m_payloadByte && !sei.m_payloadByte) || (!m_payloadByte && sei.m_payloadByte))
  {
    result = false;
  }

  return result;
}
#endif

#if NNPFA_SEI_MESSAGE
SEINeuralNetworkPostFilterActivation::SEINeuralNetworkPostFilterActivation(
  const SEINeuralNetworkPostFilterActivation& sei)
{
  m_targetId = sei.m_targetId;
  m_cancelFlag = sei.m_cancelFlag;
  m_persistenceFlag = sei.m_persistenceFlag;
  m_targetBaseFlag = sei.m_targetBaseFlag;
  m_noPrevCLVSFlag = sei.m_noPrevCLVSFlag;
  m_noFollCLVSFlag = sei.m_noFollCLVSFlag;
  m_outputFlag = sei.m_outputFlag;
  m_promptUpdateFlag = sei.m_promptUpdateFlag;
  m_prompt = sei.m_prompt;
  m_seedUpdateFlag = sei.m_seedUpdateFlag;
  m_seed = sei.m_seed;
}
#endif

#if JVET_AJ0207_GFV
SEIGenerativeFaceVideo::SEIGenerativeFaceVideo(const SEIGenerativeFaceVideo& sei)
{
    m_number = sei.m_number;
    m_currentid = sei.m_currentid;
    m_id = sei.m_id;
    m_cnt = sei.m_cnt;
    m_basePicFlag = sei.m_basePicFlag;
    m_nnPresentFlag = sei.m_nnPresentFlag;
    m_nnModeIdc = sei.m_nnModeIdc;
    m_nnTagURI = sei.m_nnTagURI;
    m_nnURI = sei.m_nnURI;
    m_chromaKeyInfoPresentFlag = sei.m_chromaKeyInfoPresentFlag;
    m_chromaKeyValuePresentFlag = sei.m_chromaKeyValuePresentFlag;
    m_chromaKeyValue = sei.m_chromaKeyValue;
    m_chromaKeyThrPresentFlag = sei.m_chromaKeyThrPresentFlag;
    m_chromaKeyThrValue = sei.m_chromaKeyThrValue;
    m_drivePicFusionFlag = sei.m_drivePicFusionFlag;
    m_lowConfidenceFaceParameterFlag = sei.m_lowConfidenceFaceParameterFlag;
    m_coordinatePresentFlag = sei.m_coordinatePresentFlag;
    m_coordinateQuantizationFactor = sei.m_coordinateQuantizationFactor;
    m_coordinatePredFlag = sei.m_coordinatePredFlag;
    m_3DCoordinateFlag = sei.m_3DCoordinateFlag;
    m_coordinatePointNum = sei.m_coordinatePointNum;
    m_coordinateX = sei.m_coordinateX;
    m_coordinateY = sei.m_coordinateY;
    m_coordinateZMaxValue = sei.m_coordinateZMaxValue;
    m_coordinateZ = sei.m_coordinateZ;
    m_matrixPresentFlag = sei.m_matrixPresentFlag;
    m_matrixElementPrecisionFactor = sei.m_matrixElementPrecisionFactor;
    m_matrixPredFlag = sei.m_matrixPredFlag;
    m_numMatricestonumKpsFlag = sei.m_numMatricestonumKpsFlag;
    m_numMatricesInfo = sei.m_numMatricesInfo;
    m_numMatrixType = sei.m_numMatrixType;
    m_matrixTypeIdx = sei.m_matrixTypeIdx;
    m_matrix3DSpaceFlag = sei.m_matrix3DSpaceFlag;
    m_numMatrices = sei.m_numMatrices;
    m_matrixWidth = sei.m_matrixWidth;
    m_matrixHeight = sei.m_matrixHeight;
    m_matrixElement = sei.m_matrixElement;
    m_payloadFilename = sei.m_payloadFilename;
    m_payloadLength = sei.m_payloadLength;
    m_payloadByte = sei.m_payloadByte ? new char(*sei.m_payloadByte) : nullptr;
    m_numMatricesstore = sei.m_numMatricesstore;
    m_matrixWidthstore = sei.m_matrixWidthstore;
    m_matrixHeightstore = sei.m_matrixHeightstore;
}
#endif
#if JVET_AK0239_GEFV
SEIGenerativeFaceVideoEnhancement::SEIGenerativeFaceVideoEnhancement(const SEIGenerativeFaceVideoEnhancement& sei)
{
    m_number = sei.m_number;
    m_currentid = sei.m_currentid;
    m_id = sei.m_id;
    m_gfvcnt = sei.m_gfvcnt;
    m_gfvid = sei.m_gfvid;
    m_basePicFlag = sei.m_basePicFlag;
    m_nnPresentFlag = sei.m_nnPresentFlag;
    m_nnModeIdc = sei.m_nnModeIdc;
    m_nnTagURI = sei.m_nnTagURI;
    m_nnURI = sei.m_nnURI;
    m_matrixElementPrecisionFactor = sei.m_matrixElementPrecisionFactor;
    m_matrixPresentFlag = sei.m_matrixPresentFlag;
    m_matrixPredFlag = sei.m_matrixPredFlag;
    m_numMatrices = sei.m_numMatrices;
    m_matrixWidth = sei.m_matrixWidth;
    m_matrixHeight = sei.m_matrixHeight;
    m_matrixElement = sei.m_matrixElement;
    m_payloadFilename = sei.m_payloadFilename;
    m_payloadLength = sei.m_payloadLength;
    m_payloadByte = sei.m_payloadByte ? new char(*sei.m_payloadByte) : nullptr;
    m_pupilPresentIdx = sei.m_pupilPresentIdx;
    m_pupilCoordinatePrecisionFactor = sei.m_pupilCoordinatePrecisionFactor;
    m_pupilLeftEyeCoordinateX = sei.m_pupilLeftEyeCoordinateX;
    m_pupilLeftEyeCoordinateY = sei.m_pupilLeftEyeCoordinateY;
    m_pupilRightEyeCoordinateX = sei.m_pupilRightEyeCoordinateX;
    m_pupilRightEyeCoordinateY = sei.m_pupilRightEyeCoordinateY;
}
#endif

#if JVET_AK2006_SPTI_SEI_MESSAGE
SEISourcePictureTimingInfo::SEISourcePictureTimingInfo(const SEISourcePictureTimingInfo &sei)
{
  m_sptiSEIEnabled = sei.m_sptiSEIEnabled;
  m_sptiSourceTimingEqualsOutputTimingFlag = sei.m_sptiSourceTimingEqualsOutputTimingFlag;
  m_sptiSourceType = sei.m_sptiSourceType;
  m_sptiTimeScale = sei.m_sptiTimeScale;
  m_sptiNumUnitsInElementalInterval = sei.m_sptiNumUnitsInElementalInterval;
  m_sptiDirectionFlag = sei.m_sptiDirectionFlag;
#endif
}

// Static member
const TChar *SEI::getSEIMessageString(SEI::PayloadType payloadType)
{
  switch (payloadType)
  {
    case SEI::BUFFERING_PERIOD:                     return "Buffering period";
    case SEI::PICTURE_TIMING:                       return "Picture timing";
    case SEI::PAN_SCAN_RECT:                        return "Pan-scan rectangle";
    case SEI::FILLER_PAYLOAD:                       return "Filler payload";
    case SEI::USER_DATA_REGISTERED_ITU_T_T35:       return "User data registered";
    case SEI::USER_DATA_UNREGISTERED:               return "User data unregistered";
    case SEI::RECOVERY_POINT:                       return "Recovery point";
    case SEI::SCENE_INFO:                           return "Scene information";
    case SEI::PICTURE_SNAPSHOT:                     return "Picture snapshot";
    case SEI::PROGRESSIVE_REFINEMENT_SEGMENT_START: return "Progressive refinement segment start";
    case SEI::PROGRESSIVE_REFINEMENT_SEGMENT_END:   return "Progressive refinement segment end";
    case SEI::FILM_GRAIN_CHARACTERISTICS:           return "Film grain characteristics";
    case SEI::POST_FILTER_HINT:                     return "Post filter hint";
    case SEI::TONE_MAPPING_INFO:                    return "Tone mapping information";
    case SEI::KNEE_FUNCTION_INFO:                   return "Knee function information";
    case SEI::FRAME_PACKING:                        return "Frame packing arrangement";
    case SEI::DISPLAY_ORIENTATION:                  return "Display orientation";
    case SEI::GREEN_METADATA:                       return "Green metadata information";
    case SEI::SOP_DESCRIPTION:                      return "Structure of pictures information";
    case SEI::ACTIVE_PARAMETER_SETS:                return "Active parameter sets";
    case SEI::DECODING_UNIT_INFO:                   return "Decoding unit information";
    case SEI::TEMPORAL_LEVEL0_INDEX:                return "Temporal sub-layer zero index";
    case SEI::DECODED_PICTURE_HASH:                 return "Decoded picture hash";
    case SEI::SCALABLE_NESTING:                     return "Scalable nesting";
    case SEI::REGION_REFRESH_INFO:                  return "Region refresh information";
    case SEI::NO_DISPLAY:                           return "No display";
    case SEI::TIME_CODE:                            return "Time code";
    case SEI::MASTERING_DISPLAY_COLOUR_VOLUME:      return "Mastering display colour volume";
    case SEI::SEGM_RECT_FRAME_PACKING:              return "Segmented rectangular frame packing arrangement";
    case SEI::TEMP_MOTION_CONSTRAINED_TILE_SETS:    return "Temporal motion constrained tile sets";
    case SEI::CHROMA_RESAMPLING_FILTER_HINT:        return "Chroma sampling filter hint";
    case SEI::COLOUR_REMAPPING_INFO:                return "Colour remapping info";
    case SEI::DEINTERLACE_FIELD_IDENTIFICATION:     return "Deinterlace field identification";
    case SEI::CONTENT_LIGHT_LEVEL_INFO:             return "Content light level info";
    case SEI::DEPENDENT_RAP_INDICATION:             return "Dependent RAP indication";
    case SEI::CODED_REGION_COMPLETION:              return "Coded region completion";
    case SEI::ALTERNATIVE_TRANSFER_CHARACTERISTICS: return "Alternative transfer characteristics";
    case SEI::AMBIENT_VIEWING_ENVIRONMENT:          return "Ambient viewing environment";
    case SEI::CONTENT_COLOUR_VOLUME:                return "Content Colour Volume";
    case SEI::EQUIRECTANGULAR_PROJECTION:           return "Equirectangular projection";
    case SEI::SPHERE_ROTATION:                      return "Sphere rotation";
    case SEI::OMNI_VIEWPORT:                        return "Omni viewport";
    case SEI::CUBEMAP_PROJECTION:                  return "Cubemap projection";
    case SEI::REGION_WISE_PACKING:                  return "Region wise packing information";
    case SEI::FISHEYE_VIDEO_INFO:                   return "Fisheye video information";
    case SEI::REGIONAL_NESTING:                     return "Regional nesting";
#if MCTS_EXTRACTION
    case SEI::MCTS_EXTRACTION_INFO_SET:             return "MCTS extraction information";
#endif
#if JCTVC_AD0021_SEI_MANIFEST
    case SEI::SEI_MANIFEST:                         return "SEI manifest";
#endif
#if JCTVC_AD0021_SEI_PREFIX_INDICATION
    case SEI::SEI_PREFIX_INDICATION:                return "SEI prefix indication";
#endif
    case SEI::ANNOTATED_REGIONS:                    return "Annotated Region";
#if SHUTTER_INTERVAL_SEI_MESSAGE
    case SEI::SHUTTER_INTERVAL_INFO:                return "Shutter interval information";
#endif
#if NNPFC_SEI_MESSAGE
    case SEI::NEURAL_NETWORK_POST_FILTER_CHARACTERISTICS: return "Neural network post-filter characteristics";
#endif
#if NNPFA_SEI_MESSAGE
    case SEI::NEURAL_NETWORK_POST_FILTER_ACTIVATION:return "Neural network post-filter activation";
#endif
#if JVET_AE0101_PHASE_INDICATION_SEI_MESSAGE
    case SEI::PHASE_INDICATION:                     return "Phase Indication";
#endif
#if JVET_AL0062_AI_USAGE_RESTRICTIONS_SEI
    case SEI::AI_USAGE_RESTRICTIONS:                return "AI usage restrictions";
#endif   
#if JVET_AL0061_ENCODER_OPTIMIZATION_INFORMATION_SEI
    case SEI::ENCODER_OPTIMIZATION_INFO:            return "Encoder optimization information";
#endif
#if JVET_AK0107_MODALITY_INFORMATION
    case SEI::MODALITY_INFORMATION:                 return "Modality information";
#endif
#if JVET_AJ0207_GFV
    case SEI::GENERATIVE_FACE_VIDEO:                return "Generative face video";
#endif
#if JVET_AK0239_GEFV
    case SEI::GENERATIVE_ENHANCEMENT_FACE_VIDEO:    return "Generative enhancement face video";
#endif
#if JVET_AK2006_SPTI_SEI_MESSAGE
    case SEI::SOURCE_PICTURE_TIMING_INFO:           return "Source picture timing info";
#endif
    default:                                        return "Unknown";
  }
}


