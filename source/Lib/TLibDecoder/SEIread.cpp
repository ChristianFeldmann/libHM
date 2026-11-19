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

/**
 \file     SEIread.cpp
 \brief    reading functionality for SEI messages
 */

#include "TLibCommon/CommonDef.h"
#include "TLibCommon/TComBitStream.h"
#include "TLibCommon/SEI.h"
#include "TLibCommon/TComSlice.h"
#include "SyntaxElementParser.h"
#include "SEIread.h"
#include "TLibCommon/TComPicYuv.h"
#include <iomanip>
#if JVET_AK0239_GEFV || JVET_AJ0207_GFV
#include <fstream>
#endif
#if JVET_AK2006_SPTI_SEI_MESSAGE
#include <climits>
#endif



//! \ingroup TLibDecoder
//! \{


#if ENC_DEC_TRACE
Void  SEIReader::xTraceSEIHeader()
{
  fprintf( g_hTrace, "=========== SEI message ===========\n");
}

Void  SEIReader::xTraceSEIMessageType(SEI::PayloadType payloadType)
{
  fprintf( g_hTrace, "=========== %s SEI message ===========\n", SEI::getSEIMessageString(payloadType));
}
#endif

Void SEIReader::sei_read_scode(std::ostream *pOS, UInt uiLength, Int& ruiCode, const TChar *pSymbolName)
{
  READ_SCODE(uiLength, ruiCode, pSymbolName);
  if (pOS)
  {
    (*pOS) << "  " << std::setw(55) << pSymbolName << ": " << ruiCode << "\n";
  }
}

Void SEIReader::sei_read_code(std::ostream *pOS, UInt uiLength, UInt& ruiCode, const TChar *pSymbolName)
{
  READ_CODE(uiLength, ruiCode, pSymbolName);
  if (pOS)
  {
    (*pOS) << "  " << std::setw(55) << pSymbolName << ": " << ruiCode << "\n";
  }
}

Void SEIReader::sei_read_uvlc(std::ostream *pOS, UInt& ruiCode, const TChar *pSymbolName)
{
  READ_UVLC(ruiCode, pSymbolName);
  if (pOS)
  {
    (*pOS) << "  " << std::setw(55) << pSymbolName << ": " << ruiCode << "\n";
  }
}

Void SEIReader::sei_read_svlc(std::ostream *pOS, Int& ruiCode, const TChar *pSymbolName)
{
  READ_SVLC(ruiCode, pSymbolName);
  if (pOS)
  {
    (*pOS) << "  " << std::setw(55) << pSymbolName << ": " << ruiCode << "\n";
  }
}

Void SEIReader::sei_read_flag(std::ostream *pOS, UInt& ruiCode, const TChar *pSymbolName)
{
  READ_FLAG(ruiCode, pSymbolName);
  if (pOS)
  {
    (*pOS) << "  " << std::setw(55) << pSymbolName << ": " << (ruiCode?1:0) << "\n";
  }
}

void SEIReader::sei_read_string(std::ostream* os, std::string& code, const TChar* symbolName)
{
  READ_STRING(code, symbolName);
  if (os)
  {
    (*os) << "  " << std::setw(55) << symbolName << ": " << code << "\n";
  }
}

bool SEIReader::xPayloadExtensionPresent()
{
  if (getBitstream()->getNumBitsLeft() == 0)
  {
    return false;
  }
  else if (getBitstream()->getNumBitsLeft() > 8)
  {
    return true;
  }

  uint32_t remBits = getBitstream()->peekBits(getBitstream()->getNumBitsLeft());
  return remBits != (1 << (getBitstream()->getNumBitsLeft() - 1));
}

static inline Void output_sei_message_header(SEI &sei, std::ostream *pDecodedMessageOutputStream, UInt payloadSize)
{
  if (pDecodedMessageOutputStream)
  {
    std::string seiMessageHdr(SEI::getSEIMessageString(sei.payloadType())); seiMessageHdr+=" SEI message";
    (*pDecodedMessageOutputStream) << std::setfill('-') << std::setw((int)seiMessageHdr.size()) << "-" << std::setfill(' ') << "\n" << seiMessageHdr << " (" << payloadSize << " bytes)"<< "\n";
  }
}

#undef READ_SCODE
#undef READ_CODE
#undef READ_SVLC
#undef READ_UVLC
#undef READ_FLAG


/**
 * unmarshal a single SEI message from bitstream bs
 */
SEIMessages::iterator  SEIReader::parseSEImessage(TComInputBitstream* bs, SEIMessages& seis, const NalUnitType nalUnitType, const TComSPS *sps, std::ostream *pDecodedMessageOutputStream)
{
  setBitstream(bs);

  SEIMessages   seiListInCurNalu;
  SEIMessages::iterator newSEI = seis.end();
  bool seiMessageRead = false;

  assert(!m_pcBitstream->getNumBitsUntilByteAligned());
  do
  {
    if(nalUnitType == NAL_UNIT_PREFIX_SEI)
    {
      seiMessageRead = xReadSEImessage(seis, nalUnitType, sps, pDecodedMessageOutputStream, SEI::prefix_sei_messages, std::string("prefix SEI"));
    }
    else if (nalUnitType == NAL_UNIT_SUFFIX_SEI)
    {
      seiMessageRead = xReadSEImessage(seis, nalUnitType, sps, pDecodedMessageOutputStream, SEI::suffix_sei_messages, std::string("suffix SEI"));
    }
    else
    {
      std::cerr << "Unsupported SEI NAL unit type '" << nalUnitType << "'" << std::endl;
      exit(EXIT_FAILURE);
    }

    if (seiMessageRead)
    {
      seiListInCurNalu.push_back(seis.back());
      if (newSEI == seis.end())
      {
        newSEI = --seis.end();
      }
    }

    /* SEI messages are an integer number of bytes, something has failed
    * in the parsing if bitstream not byte-aligned */
    assert(!m_pcBitstream->getNumBitsUntilByteAligned());
  }
  while (m_pcBitstream->getNumBitsLeft() > 8);

  xReadRbspTrailingBits();
  
  return newSEI;
}
Void SEIReader::xReadSEIPayloadData(Int const payloadType, Int const payloadSize, SEI *&sei, const NalUnitType nalUnitType, const TComSPS *sps, 
  std::ostream *pDecodedMessageOutputStream, std::string const &typeName)
{
  switch(payloadType)
  {
    case SEI::BUFFERING_PERIOD:
      if (!sps)
      {
        printf ("Warning: Found Buffering period SEI message, but no active SPS is available. Ignoring.");
      }
      else
      {
        sei = new SEIBufferingPeriod;
        xParseSEIBufferingPeriod((SEIBufferingPeriod&) *sei, payloadSize, sps, pDecodedMessageOutputStream);
      }
      break;
    case SEI::PICTURE_TIMING:
      if (!sps)
      {
        printf ("Warning: Found Picture timing SEI message, but no active SPS is available. Ignoring.");
      }
      else
      {
        sei = new SEIPictureTiming;
        xParseSEIPictureTiming((SEIPictureTiming&)*sei, payloadSize, sps, pDecodedMessageOutputStream);
      }
      break;
    case SEI::PAN_SCAN_RECT:
      sei = new SEIPanScanRect;
      xParseSEIPanScanRect((SEIPanScanRect&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::FILLER_PAYLOAD:
      sei = new SEIFillerPayload;
      xParseSEIFillerPayload((SEIFillerPayload&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::USER_DATA_REGISTERED_ITU_T_T35:
      sei = new SEIUserDataRegistered;
      xParseSEIUserDataRegistered((SEIUserDataRegistered&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::USER_DATA_UNREGISTERED:
      sei = new SEIUserDataUnregistered;
      xParseSEIUserDataUnregistered((SEIUserDataUnregistered&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::RECOVERY_POINT:
      sei = new SEIRecoveryPoint;
      xParseSEIRecoveryPoint((SEIRecoveryPoint&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::SCENE_INFO:
      sei = new SEISceneInfo;
      xParseSEISceneInfo((SEISceneInfo&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::PICTURE_SNAPSHOT:
      sei = new SEIPictureSnapshot;
      xParseSEIPictureSnapshot((SEIPictureSnapshot&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::PROGRESSIVE_REFINEMENT_SEGMENT_START:
      sei = new SEIProgressiveRefinementSegmentStart;
      xParseSEIProgressiveRefinementSegmentStart((SEIProgressiveRefinementSegmentStart&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::PROGRESSIVE_REFINEMENT_SEGMENT_END:
      sei = new SEIProgressiveRefinementSegmentEnd;
      xParseSEIProgressiveRefinementSegmentEnd((SEIProgressiveRefinementSegmentEnd&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::FILM_GRAIN_CHARACTERISTICS:
      sei = new SEIFilmGrainCharacteristics;
      xParseSEIFilmGrainCharacteristics((SEIFilmGrainCharacteristics&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::POST_FILTER_HINT:
      if (!sps)
      {
        printf ("Warning: post filter hint SEI message, but no active SPS is available. Ignoring.");
      }
      else
      {
        sei = new SEIPostFilterHint;
        xParseSEIPostFilterHint((SEIPostFilterHint&) *sei, payloadSize, sps, pDecodedMessageOutputStream);
      }
      break;
    case SEI::TONE_MAPPING_INFO:
      sei = new SEIToneMappingInfo;
      xParseSEIToneMappingInfo((SEIToneMappingInfo&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::FRAME_PACKING:
      sei = new SEIFramePacking;
      xParseSEIFramePacking((SEIFramePacking&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::DISPLAY_ORIENTATION:
      sei = new SEIDisplayOrientation;
      xParseSEIDisplayOrientation((SEIDisplayOrientation&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::GREEN_METADATA:
      sei = new SEIGreenMetadataInfo;
      xParseSEIGreenMetadataInfo((SEIGreenMetadataInfo&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::SOP_DESCRIPTION:
      sei = new SEISOPDescription;
      xParseSEISOPDescription((SEISOPDescription&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::DECODED_PICTURE_HASH:
      sei = new SEIDecodedPictureHash;
      xParseSEIDecodedPictureHash((SEIDecodedPictureHash&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::ACTIVE_PARAMETER_SETS:
      sei = new SEIActiveParameterSets;
      xParseSEIActiveParameterSets((SEIActiveParameterSets&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::DECODING_UNIT_INFO:
      if (!sps)
      {
        printf ("Warning: Found Decoding unit SEI message, but no active SPS is available. Ignoring.");
      }
      else
      {
        sei = new SEIDecodingUnitInfo;
        xParseSEIDecodingUnitInfo((SEIDecodingUnitInfo&) *sei, payloadSize, sps, pDecodedMessageOutputStream);
      }
      break;
    case SEI::TEMPORAL_LEVEL0_INDEX:
      sei = new SEITemporalLevel0Index;
      xParseSEITemporalLevel0Index((SEITemporalLevel0Index&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::SCALABLE_NESTING:
      sei = new SEIScalableNesting;
      xParseSEIScalableNesting((SEIScalableNesting&) *sei, nalUnitType, payloadSize, sps, pDecodedMessageOutputStream);
      break;
    case SEI::REGION_REFRESH_INFO:
      sei = new SEIRegionRefreshInfo;
      xParseSEIRegionRefreshInfo((SEIRegionRefreshInfo&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::NO_DISPLAY:
      sei = new SEINoDisplay;
      xParseSEINoDisplay((SEINoDisplay&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::TIME_CODE:
      sei = new SEITimeCode;
      xParseSEITimeCode((SEITimeCode&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::MASTERING_DISPLAY_COLOUR_VOLUME:
      sei = new SEIMasteringDisplayColourVolume;
      xParseSEIMasteringDisplayColourVolume((SEIMasteringDisplayColourVolume&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::SEGM_RECT_FRAME_PACKING:
      sei = new SEISegmentedRectFramePacking;
      xParseSEISegmentedRectFramePacking((SEISegmentedRectFramePacking&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::TEMP_MOTION_CONSTRAINED_TILE_SETS:
      sei = new SEITempMotionConstrainedTileSets;
      xParseSEITempMotionConstraintsTileSets((SEITempMotionConstrainedTileSets&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
#if MCTS_EXTRACTION
    case SEI::MCTS_EXTRACTION_INFO_SET:
      sei = new SEIMCTSExtractionInfoSet;
      xParseSEIMCTSExtractionInfoSet((SEIMCTSExtractionInfoSet&)*sei, payloadSize, pDecodedMessageOutputStream);
      break;
#endif
    case SEI::CHROMA_RESAMPLING_FILTER_HINT:
      sei = new SEIChromaResamplingFilterHint;
      xParseSEIChromaResamplingFilterHint((SEIChromaResamplingFilterHint&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::KNEE_FUNCTION_INFO:
      sei = new SEIKneeFunctionInfo;
      xParseSEIKneeFunctionInfo((SEIKneeFunctionInfo&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::COLOUR_REMAPPING_INFO:
      sei = new SEIColourRemappingInfo;
      xParseSEIColourRemappingInfo((SEIColourRemappingInfo&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::DEINTERLACE_FIELD_IDENTIFICATION:
      sei = new SEIDeinterlaceFieldIdentification;
      xParseSEIDeinterlaceFieldIdentification((SEIDeinterlaceFieldIdentification&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::CONTENT_LIGHT_LEVEL_INFO:
      sei = new SEIContentLightLevelInfo;
      xParseSEIContentLightLevelInfo((SEIContentLightLevelInfo&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::DEPENDENT_RAP_INDICATION:
      sei = new SEIDependentRAPIndication;
      xParseSEIDependentRAPIndication((SEIDependentRAPIndication&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::CODED_REGION_COMPLETION:
      sei = new SEICodedRegionCompletion;
      xParseSEICodedRegionCompletion((SEICodedRegionCompletion&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::ALTERNATIVE_TRANSFER_CHARACTERISTICS:
      sei = new SEIAlternativeTransferCharacteristics;
      xParseSEIAlternativeTransferCharacteristics((SEIAlternativeTransferCharacteristics&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::AMBIENT_VIEWING_ENVIRONMENT:
      sei = new SEIAmbientViewingEnvironment;
      xParseSEIAmbientViewingEnvironment((SEIAmbientViewingEnvironment&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::CONTENT_COLOUR_VOLUME:
      sei = new SEIContentColourVolume;
      xParseSEIContentColourVolume((SEIContentColourVolume&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::EQUIRECTANGULAR_PROJECTION:
      sei = new SEIEquirectangularProjection;
      xParseSEIEquirectangularProjection((SEIEquirectangularProjection&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::SPHERE_ROTATION:
      sei = new SEISphereRotation;
      xParseSEISphereRotation((SEISphereRotation&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::OMNI_VIEWPORT:
      sei = new SEIOmniViewport;
      xParseSEIOmniViewport((SEIOmniViewport&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::CUBEMAP_PROJECTION:
      sei = new SEICubemapProjection;
      xParseSEICubemapProjection((SEICubemapProjection&)*sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::REGION_WISE_PACKING:
      sei = new SEIRegionWisePacking;
      xParseSEIRegionWisePacking((SEIRegionWisePacking&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;

    case SEI::ANNOTATED_REGIONS:
      sei = new SEIAnnotatedRegions;
      xParseSEIAnnotatedRegions((SEIAnnotatedRegions&)*sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::FISHEYE_VIDEO_INFO:
      sei = new SEIFisheyeVideoInfo;
      xParseSEIFisheyeVideoInfo((SEIFisheyeVideoInfo&)*sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::REGIONAL_NESTING:
      sei = new SEIRegionalNesting;
      xParseSEIRegionalNesting((SEIRegionalNesting&) *sei, payloadSize, sps, pDecodedMessageOutputStream);
      break;
#if SHUTTER_INTERVAL_SEI_MESSAGE
    case SEI::SHUTTER_INTERVAL_INFO:
      sei = new SEIShutterIntervalInfo;
      xParseSEIShutterInterval((SEIShutterIntervalInfo&)*sei, payloadSize, pDecodedMessageOutputStream);
      break;
#endif
#if JVET_AL0062_AI_USAGE_RESTRICTIONS_SEI
    case SEI::PayloadType::AI_USAGE_RESTRICTIONS:
      sei = new SEIAIUsageRestrictions;
      xParseSEIAIUsageRestrictions((SEIAIUsageRestrictions &)*sei, payloadSize, pDecodedMessageOutputStream);
      break;
#endif
#if JCTVC_AD0021_SEI_MANIFEST
    case SEI::SEI_MANIFEST:
      sei = new SEIManifest;
      xParseSEISEIManifest((SEIManifest&)*sei, payloadSize, pDecodedMessageOutputStream);
      break;
#endif
#if JCTVC_AD0021_SEI_PREFIX_INDICATION
    case SEI::SEI_PREFIX_INDICATION:
      sei = new SEIPrefixIndication;
      xParseSEISEIPrefixIndication((SEIPrefixIndication&)*sei, payloadSize, pDecodedMessageOutputStream);
      break;
#endif
#if NNPFC_SEI_MESSAGE
  case SEI::NEURAL_NETWORK_POST_FILTER_CHARACTERISTICS:
    sei = new SEINeuralNetworkPostFilterCharacteristics;
    xParseSEINNPostFilterCharacteristics((SEINeuralNetworkPostFilterCharacteristics &) *sei, payloadSize, sps, pDecodedMessageOutputStream);

    if (xCheckNnpfcSeiMsg( ((SEINeuralNetworkPostFilterCharacteristics*)sei)->m_id, ((SEINeuralNetworkPostFilterCharacteristics*)sei)->m_baseFlag, nnpfcValues) )
    {
      nnpfcValues.push_back(((SEINeuralNetworkPostFilterCharacteristics*)sei)->m_id);
    }
    break;
#endif
#if NNPFA_SEI_MESSAGE
    case SEI::NEURAL_NETWORK_POST_FILTER_ACTIVATION:
      sei = new SEINeuralNetworkPostFilterActivation;
      xParseSEINNPostFilterActivation((SEINeuralNetworkPostFilterActivation &) *sei, payloadSize, pDecodedMessageOutputStream);
      nnpfcProcessed = false;
      CHECK(nnpfcValues.size() == 0, "At leaset one NNPFC SEI message should precede NNPFA");
      for(Int i=0; i<nnpfcValues.size(); ++i)
      {
        if(((SEINeuralNetworkPostFilterActivation*)sei)->m_targetId == nnpfcValues[i])
        {
          //In the case that the NNPFA activates a non-base filter, only consider it process when we have NNPFC that updates the base filter present
          if(((SEINeuralNetworkPostFilterActivation*)sei)->m_targetBaseFlag ||
             (!((SEINeuralNetworkPostFilterActivation*)sei)->m_targetBaseFlag && xCheckNnpfcUpdatePresentSeiMsg( ((SEINeuralNetworkPostFilterActivation*)sei)->m_targetId, nnpfcValues)) )
          {
            nnpfcProcessed = true;
          }
        }
      }
      CHECK(!nnpfcProcessed, "No NNPFC, no NNPFA");
      nnpfcProcessed = false;
      break;
#endif
#if JVET_AE0101_PHASE_INDICATION_SEI_MESSAGE
    case SEI::PayloadType::PHASE_INDICATION:
      sei = new SEIPhaseIndication;
      xParseSEIPhaseIndication((SEIPhaseIndication &) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
#endif

#if JVET_AL0061_ENCODER_OPTIMIZATION_INFORMATION_SEI
    case SEI::PayloadType::ENCODER_OPTIMIZATION_INFO:
      sei = new SEIEncoderOptimizationInfo;
      xParseSEIEncoderOptimizationInfo((SEIEncoderOptimizationInfo &)*sei, payloadSize, pDecodedMessageOutputStream);
      break;
#endif
#if JVET_AK0107_MODALITY_INFORMATION
    case SEI::MODALITY_INFORMATION:
      sei = new SEIModalityInfo; 
      xParseSEIModalityInfo((SEIModalityInfo &) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
#endif
#if JVET_AK0194_DSC_SEI_DECODER_SYNTAX
    case SEI::PayloadType::DIGITALLY_SIGNED_CONTENT_INITIALIZATION:
      sei = new SEIDigitallySignedContentInitialization;
      xParseSEIDigitallySignedContentInitialization((SEIDigitallySignedContentInitialization &) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::PayloadType::DIGITALLY_SIGNED_CONTENT_SELECTION:
      sei = new SEIDigitallySignedContentSelection;
      xParseSEIDigitallySignedContentSelection((SEIDigitallySignedContentSelection &) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::PayloadType::DIGITALLY_SIGNED_CONTENT_VERIFICATION:
      sei = new SEIDigitallySignedContentVerification;
      xParseSEIDigitallySignedContentVerification((SEIDigitallySignedContentVerification &) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
#endif
#if JVET_AK0140_PACKED_REGIONS_INFORMATION_SEI
    case SEI::PayloadType::PACKED_REGIONS_INFO:
      sei = new SEIPackedRegionsInfo;
      xParsePackedRegionsInfo((SEIPackedRegionsInfo &) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
#endif
#if JVET_AJ0207_GFV 
    case SEI::PayloadType::GENERATIVE_FACE_VIDEO:
      sei = new SEIGenerativeFaceVideo;
      xParseSEIGenerativeFaceVideo((SEIGenerativeFaceVideo &)*sei, payloadSize, pDecodedMessageOutputStream);
      break;
#endif
#if JVET_AK0239_GEFV 
    case SEI::PayloadType::GENERATIVE_ENHANCEMENT_FACE_VIDEO:
      sei = new SEIGenerativeFaceVideoEnhancement;
      xParseSEIGenerativeFaceVideoEnhancement((SEIGenerativeFaceVideoEnhancement &)*sei, payloadSize, pDecodedMessageOutputStream);
      break;
#endif
#if JVET_AK2006_SPTI_SEI_MESSAGE
    case SEI::PayloadType::SOURCE_PICTURE_TIMING_INFO:
      sei = new SEISourcePictureTimingInfo(sps->getMaxTLayers() - 1);
      xParseSEISourcePictureTimingInfo((SEISourcePictureTimingInfo &)*sei, payloadSize, pDecodedMessageOutputStream);
#endif
    default:
      for (UInt i = 0; i < payloadSize; i++)
      {
        UInt seiByte;
        std::string msg = std::string("unknown ")+typeName+std::string(" payload byte");
        sei_read_code (NULL, 8, seiByte, msg.c_str());
      }
      printf ("Unknown prefix SEI message (payloadType = %d) was found!\n", payloadType);
      if (pDecodedMessageOutputStream)
      {
        (*pDecodedMessageOutputStream) << "Unknown "<< typeName << " message (payloadType = " << payloadType << ") was found!\n";
      }
      break;
    }
}

#if NNPFC_SEI_MESSAGE
Bool SEIReader::xCheckNnpfcSeiMsg(UInt seiId, Bool baseFlag, const std::vector<Int> nnpfcValueList)
{
  if (baseFlag)
  {
    //Check if this is a new filter or a repetition of an existing base flag
    for (auto val : nnpfcValueList)
    {
      if (val == seiId)
      {
        //The filter is a repetition.
        return false;
      }
    }
  }
  else
  {
    Bool filterHasPresent = false;
    for(auto val : nnpfcValueList)
    {
      if (val == seiId)
      {
        filterHasPresent = true;
        break;
      }
    }
    CHECK(!filterHasPresent, "Cannot have update filter without base filter already present!");
  }
  return true;
}

Bool SEIReader::xCheckNnpfcUpdatePresentSeiMsg(UInt seiId, const std::vector<Int> nnpfcValueList)
{
  Int count = 0;
  for (auto val : nnpfcValueList)
  {
    if (val == seiId)
    {
      count++;
      if (count == 2)
      {
        return true;
      }
    }
  }
  return false;
}
#endif

Bool SEIReader::xReadSEImessage(SEIMessages& seis, const NalUnitType nalUnitType, const TComSPS *sps, std::ostream *pDecodedMessageOutputStream, const vector<SEI::PayloadType>& allowedSeiTypes, std::string const &typeName)
{
#if ENC_DEC_TRACE
  xTraceSEIHeader();
#endif
  Int payloadType = 0;
  UInt val = 0;

  do
  {
    sei_read_code(NULL, 8, val, "payload_type");
    payloadType += val;
  } while (val==0xFF);

  UInt payloadSize = 0;
  do
  {
    sei_read_code(NULL, 8, val, "payload_size");
    payloadSize += val;
  } while (val==0xFF);

#if ENC_DEC_TRACE
  xTraceSEIMessageType((SEI::PayloadType)payloadType);
#endif

  if(payloadSize == 0)
  {
    return false;
  }

  /* extract the payload for this single SEI message.
   * This allows greater safety in erroneous parsing of an SEI message
   * from affecting subsequent messages.
   * After parsing the payload, bs needs to be restored as the primary
   * bitstream.
   */
  TComInputBitstream *bs = getBitstream();
  setBitstream(bs->extractSubstream(payloadSize * 8));

  SEI *sei = NULL;

  if (std::find(allowedSeiTypes.begin(), allowedSeiTypes.begin(), payloadType) !=  allowedSeiTypes.end())
  {
    xReadSEIPayloadData(payloadType, payloadSize, sei, nalUnitType, sps, pDecodedMessageOutputStream, typeName);
  } 
  else
  {
    for (UInt i = 0; i < payloadSize; i++)
    {
      UInt seiByte;
      sei_read_code (NULL, 8, seiByte, "unknown SEI payload byte");
    }
    printf ("Unknown SEI message (payloadType = %d) was found!\n", payloadType);
    if (pDecodedMessageOutputStream)
    {
      (*pDecodedMessageOutputStream) << "Unknown SEI message (payloadType = " << payloadType << ") was found!\n";
    }
  }

  if (sei != NULL)
  {
    seis.push_back(sei);
  }

  /* By definition the underlying bitstream terminates in a byte-aligned manner.
   * 1. Extract all bar the last MIN(bitsremaining,nine) bits as reserved_payload_extension_data
   * 2. Examine the final 8 bits to determine the payload_bit_equal_to_one marker
   * 3. Extract the remainingreserved_payload_extension_data bits.
   *
   * If there are fewer than 9 bits available, extract them.
   */
  Int payloadBitsRemaining = getBitstream()->getNumBitsLeft();
  if (payloadBitsRemaining) /* more_data_in_payload() */
  {
    for (; payloadBitsRemaining > 9; payloadBitsRemaining--)
    {
      UInt reservedPayloadExtensionData;
      sei_read_code ( pDecodedMessageOutputStream, 1, reservedPayloadExtensionData, "reserved_payload_extension_data");
    }

    /* 2 */
    Int finalBits = getBitstream()->peekBits(payloadBitsRemaining);
    Int finalPayloadBits = 0;
    for (Int mask = 0xff; finalBits & (mask >> finalPayloadBits); finalPayloadBits++)
    {
      continue;
    }

    /* 3 */
    for (; payloadBitsRemaining > 9 - finalPayloadBits; payloadBitsRemaining--)
    {
      UInt reservedPayloadExtensionData;
      sei_read_flag ( 0, reservedPayloadExtensionData, "reserved_payload_extension_data");
    }

    UInt dummy;
    sei_read_flag( 0, dummy, "payload_bit_equal_to_one"); payloadBitsRemaining--;
    while (payloadBitsRemaining)
    {
      sei_read_flag( 0, dummy, "payload_bit_equal_to_zero"); payloadBitsRemaining--;
    }
  }

  /* restore primary bitstream for sei_message */
  delete getBitstream();
  setBitstream(bs);
  
  return sei != nullptr;
}


Void SEIReader::xParseSEIBufferingPeriod(SEIBufferingPeriod& sei, UInt payloadSize, const TComSPS *sps, std::ostream *pDecodedMessageOutputStream)
{
  Int i, nalOrVcl;
  UInt code;

  const TComVUI *pVUI = sps->getVuiParameters();
  const TComHRD *pHRD = pVUI->getHrdParameters();

  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_uvlc( pDecodedMessageOutputStream, code, "bp_seq_parameter_set_id" );                         sei.m_bpSeqParameterSetId     = code;
  if( !pHRD->getSubPicCpbParamsPresentFlag() )
  {
    sei_read_flag( pDecodedMessageOutputStream, code, "irap_cpb_params_present_flag" );                   sei.m_rapCpbParamsPresentFlag = code;
  }
  if( sei.m_rapCpbParamsPresentFlag )
  {
    sei_read_code( pDecodedMessageOutputStream, pHRD->getCpbRemovalDelayLengthMinus1() + 1, code, "cpb_delay_offset" );      sei.m_cpbDelayOffset = code;
    sei_read_code( pDecodedMessageOutputStream, pHRD->getDpbOutputDelayLengthMinus1()  + 1, code, "dpb_delay_offset" );      sei.m_dpbDelayOffset = code;
  }

  //read splicing flag and cpb_removal_delay_delta
  sei_read_flag( pDecodedMessageOutputStream, code, "concatenation_flag");
  sei.m_concatenationFlag = code;
  sei_read_code( pDecodedMessageOutputStream, ( pHRD->getCpbRemovalDelayLengthMinus1() + 1 ), code, "au_cpb_removal_delay_delta_minus1" );
  sei.m_auCpbRemovalDelayDelta = code + 1;

  for( nalOrVcl = 0; nalOrVcl < 2; nalOrVcl ++ )
  {
    if( ( ( nalOrVcl == 0 ) && ( pHRD->getNalHrdParametersPresentFlag() ) ) ||
        ( ( nalOrVcl == 1 ) && ( pHRD->getVclHrdParametersPresentFlag() ) ) )
    {
      for( i = 0; i < ( pHRD->getCpbCntMinus1( 0 ) + 1 ); i ++ )
      {
        sei_read_code( pDecodedMessageOutputStream, ( pHRD->getInitialCpbRemovalDelayLengthMinus1() + 1 ) , code, nalOrVcl?"vcl_initial_cpb_removal_delay":"nal_initial_cpb_removal_delay" );
        sei.m_initialCpbRemovalDelay[i][nalOrVcl] = code;
        sei_read_code( pDecodedMessageOutputStream, ( pHRD->getInitialCpbRemovalDelayLengthMinus1() + 1 ) , code, nalOrVcl?"vcl_initial_cpb_removal_offset":"nal_initial_cpb_removal_offset" );
        sei.m_initialCpbRemovalDelayOffset[i][nalOrVcl] = code;
        if( pHRD->getSubPicCpbParamsPresentFlag() || sei.m_rapCpbParamsPresentFlag )
        {
          sei_read_code( pDecodedMessageOutputStream, ( pHRD->getInitialCpbRemovalDelayLengthMinus1() + 1 ) , code, nalOrVcl?"vcl_initial_alt_cpb_removal_delay":"nal_initial_alt_cpb_removal_delay" );
          sei.m_initialAltCpbRemovalDelay[i][nalOrVcl] = code;
          sei_read_code( pDecodedMessageOutputStream, ( pHRD->getInitialCpbRemovalDelayLengthMinus1() + 1 ) , code, nalOrVcl?"vcl_initial_alt_cpb_removal_offset":"nal_initial_alt_cpb_removal_offset" );
          sei.m_initialAltCpbRemovalDelayOffset[i][nalOrVcl] = code;
        }
      }
    }
  }
}


Void SEIReader::xParseSEIPictureTiming(SEIPictureTiming& sei, UInt payloadSize, const TComSPS *sps, std::ostream *pDecodedMessageOutputStream)
{
  Int i;
  UInt code;

  const TComVUI *vui = sps->getVuiParameters();
  const TComHRD *hrd = vui->getHrdParameters();
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  if( vui->getFrameFieldInfoPresentFlag() )
  {
    sei_read_code( pDecodedMessageOutputStream, 4, code, "pic_struct" );             sei.m_picStruct            = code;
    sei_read_code( pDecodedMessageOutputStream, 2, code, "source_scan_type" );       sei.m_sourceScanType       = code;
    sei_read_flag( pDecodedMessageOutputStream,    code, "duplicate_flag" );         sei.m_duplicateFlag        = (code == 1);
  }

  if( hrd->getCpbDpbDelaysPresentFlag())
  {
    sei_read_code( pDecodedMessageOutputStream, ( hrd->getCpbRemovalDelayLengthMinus1() + 1 ), code, "au_cpb_removal_delay_minus1" );
    sei.m_auCpbRemovalDelay = code + 1;
    sei_read_code( pDecodedMessageOutputStream, ( hrd->getDpbOutputDelayLengthMinus1() + 1 ), code, "pic_dpb_output_delay" );
    sei.m_picDpbOutputDelay = code;

    if(hrd->getSubPicCpbParamsPresentFlag())
    {
      sei_read_code( pDecodedMessageOutputStream, hrd->getDpbOutputDelayDuLengthMinus1()+1, code, "pic_dpb_output_du_delay" );
      sei.m_picDpbOutputDuDelay = code;
    }

    if( hrd->getSubPicCpbParamsPresentFlag() && hrd->getSubPicCpbParamsInPicTimingSEIFlag() )
    {
      sei_read_uvlc( pDecodedMessageOutputStream, code, "num_decoding_units_minus1");
      sei.m_numDecodingUnitsMinus1 = code;
      sei_read_flag( pDecodedMessageOutputStream, code, "du_common_cpb_removal_delay_flag" );
      sei.m_duCommonCpbRemovalDelayFlag = code;
      if( sei.m_duCommonCpbRemovalDelayFlag )
      {
        sei_read_code( pDecodedMessageOutputStream, ( hrd->getDuCpbRemovalDelayLengthMinus1() + 1 ), code, "du_common_cpb_removal_delay_increment_minus1" );
        sei.m_duCommonCpbRemovalDelayMinus1 = code;
      }
      sei.m_numNalusInDuMinus1.resize(sei.m_numDecodingUnitsMinus1 + 1 );
      sei.m_duCpbRemovalDelayMinus1.resize( sei.m_numDecodingUnitsMinus1 + 1 );

      for( i = 0; i <= sei.m_numDecodingUnitsMinus1; i ++ )
      {
        sei_read_uvlc( pDecodedMessageOutputStream, code, "num_nalus_in_du_minus1[i]");
        sei.m_numNalusInDuMinus1[ i ] = code;
        if( ( !sei.m_duCommonCpbRemovalDelayFlag ) && ( i < sei.m_numDecodingUnitsMinus1 ) )
        {
          sei_read_code( pDecodedMessageOutputStream, ( hrd->getDuCpbRemovalDelayLengthMinus1() + 1 ), code, "du_cpb_removal_delay_minus1[i]" );
          sei.m_duCpbRemovalDelayMinus1[ i ] = code;
        }
      }
    }
  }
}


Void SEIReader::xParseSEIPanScanRect(SEIPanScanRect& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  UInt code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  sei_read_uvlc( pDecodedMessageOutputStream, code, "pan_scan_rect_id" );          sei.m_panScanRectId = code;
  sei_read_flag( pDecodedMessageOutputStream, code, "pan_scan_rect_cancel_flag" ); sei.m_panScanRectCancelFlag = code!=0;
  if (!sei.m_panScanRectCancelFlag)
  {
    UInt numRegions;
    sei_read_uvlc( pDecodedMessageOutputStream, numRegions, "pan_scan_cnt_minus1" ); numRegions++;
    sei.m_panScanRectRegions.resize(numRegions);
    for(UInt region=0; region<numRegions; region++)
    {
      SEIPanScanRect::PanScanRect &rect=sei.m_panScanRectRegions[region];
      Int  i;
      sei_read_svlc( pDecodedMessageOutputStream, i, "pan_scan_rect_left_offset[i]" );   rect.leftOffset   = i;
      sei_read_svlc( pDecodedMessageOutputStream, i, "pan_scan_rect_right_offset[i]" );  rect.rightOffset  = i;
      sei_read_svlc( pDecodedMessageOutputStream, i, "pan_scan_rect_top_offset[i]" );    rect.topOffset    = i;
      sei_read_svlc( pDecodedMessageOutputStream, i, "pan_scan_rect_bottom_offset[i]" ); rect.bottomOffset = i;
    }
    sei_read_flag( pDecodedMessageOutputStream, code, "pan_scan_rect_persistence_flag" ); sei.m_panScanRectPersistenceFlag = code!=0;
  }
  else
  {
    sei.m_panScanRectRegions.clear();
    sei.m_panScanRectPersistenceFlag=false;
  }
}


Void SEIReader::xParseSEIFillerPayload(SEIFillerPayload& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  sei.m_numFillerFFBytes = payloadSize;
  Bool allBytesWereFF=true;
  for(UInt k=0; k<payloadSize; k++)
  {
    UInt code;
    sei_read_code( NULL, 8, code, "ff_byte" );
    if (code!=0xff) allBytesWereFF=false;
  }
  if (pDecodedMessageOutputStream && !allBytesWereFF)
  {
    (*pDecodedMessageOutputStream) << "  not all filler payload bytes were 0xff\n";
  }
}


Void SEIReader::xParseSEIUserDataRegistered(SEIUserDataRegistered& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  UInt code;
  assert(payloadSize>0);
  sei_read_code( pDecodedMessageOutputStream, 8, code, "itu_t_t35_country_code" ); payloadSize--;
  if (code == 255)
  {
    assert(payloadSize>0);
    sei_read_code( pDecodedMessageOutputStream, 8, code, "itu_t_t35_country_code_extension_byte" ); payloadSize--;
    code+=255;
  }
  sei.m_ituCountryCode = code;
  sei.m_userData.resize(payloadSize);
  for (UInt i = 0; i < sei.m_userData.size(); i++)
  {
    sei_read_code( NULL, 8, code, "itu_t_t35_payload_byte" );
    sei.m_userData[i] = code;
  }
  if (pDecodedMessageOutputStream)
  {
    (*pDecodedMessageOutputStream) << "  itu_t_t35 payload size: " << sei.m_userData.size() << "\n";
  }
}


Void SEIReader::xParseSEIUserDataUnregistered(SEIUserDataUnregistered &sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  assert(payloadSize >= ISO_IEC_11578_LEN);
  UInt val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  for (UInt i = 0; i < ISO_IEC_11578_LEN; i++)
  {
    sei_read_code( pDecodedMessageOutputStream, 8, val, "uuid_iso_iec_11578");
    sei.m_uuid_iso_iec_11578[i] = val;
  }

  sei.m_userData.resize(payloadSize - ISO_IEC_11578_LEN);
  for (UInt i = 0; i < sei.m_userData.size(); i++)
  {
    sei_read_code( NULL, 8, val, "user_data_payload_byte" );
    sei.m_userData[i] = val;
  }
  if (pDecodedMessageOutputStream)
  {
    (*pDecodedMessageOutputStream) << "  User data payload size: " << sei.m_userData.size() << "\n";
  }
}


Void SEIReader::xParseSEIRecoveryPoint(SEIRecoveryPoint& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  Int  iCode;
  UInt uiCode;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_svlc( pDecodedMessageOutputStream, iCode,  "recovery_poc_cnt" );      sei.m_recoveryPocCnt     = iCode;
  sei_read_flag( pDecodedMessageOutputStream, uiCode, "exact_matching_flag" );   sei.m_exactMatchingFlag  = uiCode;
  sei_read_flag( pDecodedMessageOutputStream, uiCode, "broken_link_flag" );      sei.m_brokenLinkFlag     = uiCode;
}


Void SEIReader::xParseSEISceneInfo(SEISceneInfo& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  UInt code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_flag( pDecodedMessageOutputStream, code, "scene_info_present_flag" ); sei.m_bSceneInfoPresentFlag = code!=0;
  if (sei.m_bSceneInfoPresentFlag)
  {
    sei_read_flag( pDecodedMessageOutputStream, code, "prev_scene_id_valid_flag" ); sei.m_bPrevSceneIdValidFlag = code!=0;
    sei_read_uvlc( pDecodedMessageOutputStream, code, "scene_id" );                 sei.m_sceneId = code;
    sei_read_uvlc( pDecodedMessageOutputStream, code, "scene_transition_type" );    sei.m_sceneTransitionType = code;
    if (sei.m_sceneTransitionType > 3)
    {
      sei_read_uvlc( pDecodedMessageOutputStream, code, "second_scene_id" );        sei.m_secondSceneId = code;
    }
    else
    {
      sei.m_secondSceneId = 0; // set to known value.
    }
  }
}


Void SEIReader::xParseSEIPictureSnapshot(SEIPictureSnapshot& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  UInt code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_uvlc( pDecodedMessageOutputStream, code, "snapshot_id" ); sei.m_snapshotId = code;
}


Void SEIReader::xParseSEIProgressiveRefinementSegmentStart(SEIProgressiveRefinementSegmentStart& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  UInt code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_uvlc( pDecodedMessageOutputStream, code, "progressive_refinement_id" ); sei.m_progressiveRefinementId = code;
  sei_read_uvlc( pDecodedMessageOutputStream, code, "pic_order_cnt_delta" );       sei.m_picOrderCntDelta = code;
}


Void SEIReader::xParseSEIProgressiveRefinementSegmentEnd(SEIProgressiveRefinementSegmentEnd& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  UInt code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_uvlc( pDecodedMessageOutputStream, code, "progressive_refinement_id" ); sei.m_progressiveRefinementId = code;
}


Void SEIReader::xParseSEIFilmGrainCharacteristics(SEIFilmGrainCharacteristics& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  UInt code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_flag( pDecodedMessageOutputStream, code, "film_grain_characteristics_cancel_flag" );     sei.m_filmGrainCharacteristicsCancelFlag = code!=0;
  if (!sei.m_filmGrainCharacteristicsCancelFlag)
  {
    sei_read_code( pDecodedMessageOutputStream, 2, code, "film_grain_model_id" );                   sei.m_filmGrainModelId = code;
    sei_read_flag( pDecodedMessageOutputStream,    code, "separate_colour_description_present_flag" ); sei.m_separateColourDescriptionPresentFlag = code!=0;
    if (sei.m_separateColourDescriptionPresentFlag)
    {
      sei_read_code( pDecodedMessageOutputStream, 3, code, "film_grain_bit_depth_luma_minus8" );    sei.m_filmGrainBitDepthLumaMinus8      = code;
      sei_read_code( pDecodedMessageOutputStream, 3, code, "film_grain_bit_depth_chroma_minus8" );  sei.m_filmGrainBitDepthChromaMinus8    = code;
      sei_read_flag( pDecodedMessageOutputStream,    code, "film_grain_full_range_flag" );          sei.m_filmGrainFullRangeFlag           = code!=0;
      sei_read_code( pDecodedMessageOutputStream, 8, code, "film_grain_colour_primaries" );         sei.m_filmGrainColourPrimaries         = code;
      sei_read_code( pDecodedMessageOutputStream, 8, code, "film_grain_transfer_characteristics" ); sei.m_filmGrainTransferCharacteristics = code;
      sei_read_code( pDecodedMessageOutputStream, 8, code, "film_grain_matrix_coeffs" );            sei.m_filmGrainMatrixCoeffs            = code;
    }
    sei_read_code( pDecodedMessageOutputStream, 2, code, "blending_mode_id" );                      sei.m_blendingModeId                   = code;
    sei_read_code( pDecodedMessageOutputStream, 4, code, "log2_scale_factor" );                     sei.m_log2ScaleFactor                  = code;
    for(Int c=0; c<3; c++)
    {
      sei_read_flag( pDecodedMessageOutputStream,    code, "comp_model_present_flag[c]" );          sei.m_compModel[c].bPresentFlag        = code!=0;
    }
    for(Int c=0; c<3; c++)
    {
      SEIFilmGrainCharacteristics::CompModel &cm=sei.m_compModel[c];
      if (cm.bPresentFlag)
      {
#if JVET_X0048_X0103_FILM_GRAIN
        sei_read_code( pDecodedMessageOutputStream, 8, code, "num_intensity_intervals_minus1[c]"); cm.numIntensityIntervals = code + 1;
#else
        UInt numIntensityIntervals;
        sei_read_code( pDecodedMessageOutputStream, 8, code, "num_intensity_intervals_minus1[c]" ); numIntensityIntervals = code+1;
#endif
        sei_read_code( pDecodedMessageOutputStream, 3, code, "num_model_values_minus1[c]" );        cm.numModelValues     = code+1;
#if JVET_X0048_X0103_FILM_GRAIN
        cm.intensityValues.resize(cm.numIntensityIntervals);
        for (UInt interval = 0; interval < cm.numIntensityIntervals; interval++)
#else
        cm.intensityValues.resize(numIntensityIntervals);
        for(UInt interval=0; interval<numIntensityIntervals; interval++)
#endif
        {
          SEIFilmGrainCharacteristics::CompModelIntensityValues &cmiv=cm.intensityValues[interval];
          sei_read_code( pDecodedMessageOutputStream, 8, code, "intensity_interval_lower_bound[c][i]" ); cmiv.intensityIntervalLowerBound=code;
          sei_read_code( pDecodedMessageOutputStream, 8, code, "intensity_interval_upper_bound[c][i]" ); cmiv.intensityIntervalUpperBound=code;
          cmiv.compModelValue.resize(cm.numModelValues);
          for(UInt j=0; j<cm.numModelValues; j++)
          {
            sei_read_svlc( pDecodedMessageOutputStream, cmiv.compModelValue[j], "comp_model_value[c][i]" );
          }
        }
      }
    } // for c
    sei_read_flag( pDecodedMessageOutputStream, code, "film_grain_characteristics_persistence_flag" ); sei.m_filmGrainCharacteristicsPersistenceFlag = code!=0;
#if JVET_AL0339_SPATIAL_RESOLUTION_FOR_FGC_SEI
    if (xPayloadExtensionPresent())
    {
      sei_read_flag( pDecodedMessageOutputStream, code, "film_grain_spatial_resolution_present_flag" ); sei.m_fgSpatialResolutionPresentFlag = code!=0;
      if (sei.m_fgSpatialResolutionPresentFlag)
      {
        sei_read_uvlc(pDecodedMessageOutputStream, code, "fg_pic_width_in_luma_samples"); sei.m_fgPicWidthInLumaSamples = code;
        sei_read_uvlc(pDecodedMessageOutputStream, code, "fg_pic_height_in_luma_samples"); sei.m_fgPicHeightInLumaSamples = code;
      }
    }
#endif
  } // cancel flag
}


Void SEIReader::xParseSEIPostFilterHint(SEIPostFilterHint& sei, UInt payloadSize, const TComSPS *sps, std::ostream *pDecodedMessageOutputStream)
{
  UInt code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_uvlc( pDecodedMessageOutputStream,    code, "filter_hint_size_y" ); sei.m_filterHintSizeY = code;
  sei_read_uvlc( pDecodedMessageOutputStream,    code, "filter_hint_size_x" ); sei.m_filterHintSizeX = code;
  sei_read_code( pDecodedMessageOutputStream, 2, code, "filter_hint_type"   ); sei.m_filterHintType  = code;

  sei.m_bIsMonochrome = (sps->getChromaFormatIdc() == CHROMA_400);
  const UInt numChromaChannels = sei.m_bIsMonochrome ? 1:3;

  sei.m_filterHintValues.resize(numChromaChannels * sei.m_filterHintSizeX * sei.m_filterHintSizeY);
  for(std::size_t i=0; i<sei.m_filterHintValues.size(); i++)
  {
    Int v;
    sei_read_svlc( pDecodedMessageOutputStream, v, "filter_hint_value[][][]" ); sei.m_filterHintValues[i] = code;
  }
}


Void SEIReader::xParseSEIToneMappingInfo(SEIToneMappingInfo& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  Int i;
  UInt val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  sei_read_uvlc( pDecodedMessageOutputStream, val, "tone_map_id" );                         sei.m_toneMapId = val;
  sei_read_flag( pDecodedMessageOutputStream, val, "tone_map_cancel_flag" );                sei.m_toneMapCancelFlag = val;

  if ( !sei.m_toneMapCancelFlag )
  {
    sei_read_flag( pDecodedMessageOutputStream, val, "tone_map_persistence_flag" );         sei.m_toneMapPersistenceFlag = val;
    sei_read_code( pDecodedMessageOutputStream, 8, val, "coded_data_bit_depth" );           sei.m_codedDataBitDepth = val;
    sei_read_code( pDecodedMessageOutputStream, 8, val, "target_bit_depth" );               sei.m_targetBitDepth = val;
    sei_read_uvlc( pDecodedMessageOutputStream, val, "tone_map_model_id" );                 sei.m_modelId = val;
    switch(sei.m_modelId)
    {
    case 0:
      {
        sei_read_code( pDecodedMessageOutputStream, 32, val, "min_value" );                 sei.m_minValue = val;
        sei_read_code( pDecodedMessageOutputStream, 32, val, "max_value" );                 sei.m_maxValue = val;
        break;
      }
    case 1:
      {
        sei_read_code( pDecodedMessageOutputStream, 32, val, "sigmoid_midpoint" );          sei.m_sigmoidMidpoint = val;
        sei_read_code( pDecodedMessageOutputStream, 32, val, "sigmoid_width" );             sei.m_sigmoidWidth = val;
        break;
      }
    case 2:
      {
        UInt num = 1u << sei.m_targetBitDepth;
        sei.m_startOfCodedInterval.resize(num+1);
        for(i = 0; i < num; i++)
        {
          sei_read_code( pDecodedMessageOutputStream, ((( sei.m_codedDataBitDepth + 7 ) >> 3 ) << 3), val, "start_of_coded_interval[i]" );
          sei.m_startOfCodedInterval[i] = val;
        }
        sei.m_startOfCodedInterval[num] = 1u << sei.m_codedDataBitDepth;
        break;
      }
    case 3:
      {
        sei_read_code( pDecodedMessageOutputStream, 16, val,  "num_pivots" );                       sei.m_numPivots = val;
        sei.m_codedPivotValue.resize(sei.m_numPivots);
        sei.m_targetPivotValue.resize(sei.m_numPivots);
        for(i = 0; i < sei.m_numPivots; i++ )
        {
          sei_read_code( pDecodedMessageOutputStream, ((( sei.m_codedDataBitDepth + 7 ) >> 3 ) << 3), val, "coded_pivot_value[i]" );
          sei.m_codedPivotValue[i] = val;
          sei_read_code( pDecodedMessageOutputStream, ((( sei.m_targetBitDepth + 7 ) >> 3 ) << 3),    val, "target_pivot_value[i]" );
          sei.m_targetPivotValue[i] = val;
        }
        break;
      }
    case 4:
      {
        sei_read_code( pDecodedMessageOutputStream, 8, val, "camera_iso_speed_idc" );                     sei.m_cameraIsoSpeedIdc = val;
        if( sei.m_cameraIsoSpeedIdc == 255) //Extended_ISO
        {
          sei_read_code( pDecodedMessageOutputStream, 32,   val,   "camera_iso_speed_value" );            sei.m_cameraIsoSpeedValue = val;
        }
        sei_read_code( pDecodedMessageOutputStream, 8, val, "exposure_index_idc" );                       sei.m_exposureIndexIdc = val;
        if( sei.m_exposureIndexIdc == 255) //Extended_ISO
        {
          sei_read_code( pDecodedMessageOutputStream, 32,   val,   "exposure_index_value" );              sei.m_exposureIndexValue = val;
        }
        sei_read_flag( pDecodedMessageOutputStream, val, "exposure_compensation_value_sign_flag" );       sei.m_exposureCompensationValueSignFlag = val;
        sei_read_code( pDecodedMessageOutputStream, 16, val, "exposure_compensation_value_numerator" );   sei.m_exposureCompensationValueNumerator = val;
        sei_read_code( pDecodedMessageOutputStream, 16, val, "exposure_compensation_value_denom_idc" );   sei.m_exposureCompensationValueDenomIdc = val;
        sei_read_code( pDecodedMessageOutputStream, 32, val, "ref_screen_luminance_white" );              sei.m_refScreenLuminanceWhite = val;
        sei_read_code( pDecodedMessageOutputStream, 32, val, "extended_range_white_level" );              sei.m_extendedRangeWhiteLevel = val;
        sei_read_code( pDecodedMessageOutputStream, 16, val, "nominal_black_level_code_value" );          sei.m_nominalBlackLevelLumaCodeValue = val;
        sei_read_code( pDecodedMessageOutputStream, 16, val, "nominal_white_level_code_value" );          sei.m_nominalWhiteLevelLumaCodeValue= val;
        sei_read_code( pDecodedMessageOutputStream, 16, val, "extended_white_level_code_value" );         sei.m_extendedWhiteLevelLumaCodeValue = val;
        break;
      }
    default:
      {
        assert(!"Undefined SEIToneMapModelId");
        break;
      }
    }//switch model id
  }// if(!sei.m_toneMapCancelFlag)
}


Void SEIReader::xParseSEIFramePacking(SEIFramePacking& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  UInt val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_uvlc( pDecodedMessageOutputStream, val, "frame_packing_arrangement_id" );                 sei.m_arrangementId = val;
  sei_read_flag( pDecodedMessageOutputStream, val, "frame_packing_arrangement_cancel_flag" );        sei.m_arrangementCancelFlag = val;

  if ( !sei.m_arrangementCancelFlag )
  {
    sei_read_code( pDecodedMessageOutputStream, 7, val, "frame_packing_arrangement_type" );          sei.m_arrangementType = val;
    assert((sei.m_arrangementType > 2) && (sei.m_arrangementType < 6) );

    sei_read_flag( pDecodedMessageOutputStream, val, "quincunx_sampling_flag" );                     sei.m_quincunxSamplingFlag = val;

    sei_read_code( pDecodedMessageOutputStream, 6, val, "content_interpretation_type" );             sei.m_contentInterpretationType = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "spatial_flipping_flag" );                      sei.m_spatialFlippingFlag = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "frame0_flipped_flag" );                        sei.m_frame0FlippedFlag = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "field_views_flag" );                           sei.m_fieldViewsFlag = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "current_frame_is_frame0_flag" );               sei.m_currentFrameIsFrame0Flag = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "frame0_self_contained_flag" );                 sei.m_frame0SelfContainedFlag = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "frame1_self_contained_flag" );                 sei.m_frame1SelfContainedFlag = val;

    if ( sei.m_quincunxSamplingFlag == 0 && sei.m_arrangementType != 5)
    {
      sei_read_code( pDecodedMessageOutputStream, 4, val, "frame0_grid_position_x" );                sei.m_frame0GridPositionX = val;
      sei_read_code( pDecodedMessageOutputStream, 4, val, "frame0_grid_position_y" );                sei.m_frame0GridPositionY = val;
      sei_read_code( pDecodedMessageOutputStream, 4, val, "frame1_grid_position_x" );                sei.m_frame1GridPositionX = val;
      sei_read_code( pDecodedMessageOutputStream, 4, val, "frame1_grid_position_y" );                sei.m_frame1GridPositionY = val;
    }

    sei_read_code( pDecodedMessageOutputStream, 8, val, "frame_packing_arrangement_reserved_byte" );   sei.m_arrangementReservedByte = val;
    sei_read_flag( pDecodedMessageOutputStream, val,  "frame_packing_arrangement_persistence_flag" );  sei.m_arrangementPersistenceFlag = (val != 0);
  }
  sei_read_flag( pDecodedMessageOutputStream, val, "upsampled_aspect_ratio_flag" );                  sei.m_upsampledAspectRatio = val;
}


Void SEIReader::xParseSEIDisplayOrientation(SEIDisplayOrientation& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  UInt val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  sei_read_flag( pDecodedMessageOutputStream, val,       "display_orientation_cancel_flag" );       sei.cancelFlag            = val;
  if( !sei.cancelFlag )
  {
    sei_read_flag( pDecodedMessageOutputStream, val,     "hor_flip" );                              sei.horFlip               = val;
    sei_read_flag( pDecodedMessageOutputStream, val,     "ver_flip" );                              sei.verFlip               = val;
    sei_read_code( pDecodedMessageOutputStream, 16, val, "anticlockwise_rotation" );                sei.anticlockwiseRotation = val;
    sei_read_flag( pDecodedMessageOutputStream, val,     "display_orientation_persistence_flag" );  sei.persistenceFlag       = val;
  }
}


Void SEIReader::xParseSEIGreenMetadataInfo(SEIGreenMetadataInfo& sei, UInt payloadSize, ostream* pDecodedMessageOutputStream)
{
  UInt code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_code(pDecodedMessageOutputStream, 8, code, "green_metadata_type");
  sei.m_greenMetadataType = code;

  sei_read_code(pDecodedMessageOutputStream, 8, code, "xsd_metric_type");
  sei.m_xsdMetricType = code;

  sei_read_code(pDecodedMessageOutputStream, 16, code, "xsd_metric_value");
  sei.m_xsdMetricValue = code;
}


Void SEIReader::xParseSEISOPDescription(SEISOPDescription &sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  Int iCode;
  UInt uiCode;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_uvlc( pDecodedMessageOutputStream, uiCode,           "sop_seq_parameter_set_id"            ); sei.m_sopSeqParameterSetId = uiCode;
  sei_read_uvlc( pDecodedMessageOutputStream, uiCode,           "num_pics_in_sop_minus1"              ); sei.m_numPicsInSopMinus1 = uiCode;
  for (UInt i = 0; i <= sei.m_numPicsInSopMinus1; i++)
  {
    sei_read_code( pDecodedMessageOutputStream, 6, uiCode,                     "sop_vcl_nut[i]" );  sei.m_sopDescVclNaluType[i] = uiCode;
    sei_read_code( pDecodedMessageOutputStream, 3, sei.m_sopDescTemporalId[i], "sop_temporal_id[i]"   );  sei.m_sopDescTemporalId[i] = uiCode;
    if (sei.m_sopDescVclNaluType[i] != NAL_UNIT_CODED_SLICE_IDR_W_RADL && sei.m_sopDescVclNaluType[i] != NAL_UNIT_CODED_SLICE_IDR_N_LP)
    {
      sei_read_uvlc( pDecodedMessageOutputStream, sei.m_sopDescStRpsIdx[i],    "sop_short_term_rps_idx[i]"    ); sei.m_sopDescStRpsIdx[i] = uiCode;
    }
    if (i > 0)
    {
      sei_read_svlc( pDecodedMessageOutputStream, iCode,                       "sop_poc_delta[i]"     ); sei.m_sopDescPocDelta[i] = iCode;
    }
  }
}


Void SEIReader::xParseSEIActiveParameterSets(SEIActiveParameterSets& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  UInt val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_code( pDecodedMessageOutputStream, 4, val, "active_video_parameter_set_id");   sei.activeVPSId = val;
  sei_read_flag( pDecodedMessageOutputStream,    val, "self_contained_cvs_flag");         sei.m_selfContainedCvsFlag     = (val != 0);
  sei_read_flag( pDecodedMessageOutputStream,    val, "no_parameter_set_update_flag");    sei.m_noParameterSetUpdateFlag = (val != 0);
  sei_read_uvlc( pDecodedMessageOutputStream,    val, "num_sps_ids_minus1");              sei.numSpsIdsMinus1 = val;

  sei.activeSeqParameterSetId.resize(sei.numSpsIdsMinus1 + 1);
  for (Int i=0; i < (sei.numSpsIdsMinus1 + 1); i++)
  {
    sei_read_uvlc( pDecodedMessageOutputStream, val, "active_seq_parameter_set_id[i]");    sei.activeSeqParameterSetId[i] = val;
  }
}


Void SEIReader::xParseSEIDecodingUnitInfo(SEIDecodingUnitInfo& sei, UInt payloadSize, const TComSPS *sps, std::ostream *pDecodedMessageOutputStream)
{
  UInt val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  sei_read_uvlc( pDecodedMessageOutputStream, val, "decoding_unit_idx");
  sei.m_decodingUnitIdx = val;

  const TComVUI *vui = sps->getVuiParameters();
  if(vui->getHrdParameters()->getSubPicCpbParamsInPicTimingSEIFlag())
  {
    sei_read_code( pDecodedMessageOutputStream, ( vui->getHrdParameters()->getDuCpbRemovalDelayLengthMinus1() + 1 ), val, "du_spt_cpb_removal_delay_increment");
    sei.m_duSptCpbRemovalDelay = val;
  }
  else
  {
    sei.m_duSptCpbRemovalDelay = 0;
  }
  sei_read_flag( pDecodedMessageOutputStream, val, "dpb_output_du_delay_present_flag"); sei.m_dpbOutputDuDelayPresentFlag = (val != 0);
  if(sei.m_dpbOutputDuDelayPresentFlag)
  {
    sei_read_code( pDecodedMessageOutputStream, vui->getHrdParameters()->getDpbOutputDelayDuLengthMinus1() + 1, val, "pic_spt_dpb_output_du_delay");
    sei.m_picSptDpbOutputDuDelay = val;
  }
}


Void SEIReader::xParseSEITemporalLevel0Index(SEITemporalLevel0Index& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  UInt val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  sei_read_code( pDecodedMessageOutputStream, 8, val, "temporal_sub_layer_zero_idx" );  sei.tl0Idx = val;
  sei_read_code( pDecodedMessageOutputStream, 8, val, "irap_pic_id" );  sei.rapIdx = val;
}


Void SEIReader::xParseSEIDecodedPictureHash(SEIDecodedPictureHash& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  UInt bytesRead = 0;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  UInt val;
  sei_read_code( pDecodedMessageOutputStream, 8, val, "hash_type");
  sei.method = static_cast<HashType>(val); bytesRead++;

  const TChar *traceString="\0";
  switch (sei.method)
  {
    case HASHTYPE_MD5: traceString="picture_md5"; break;
    case HASHTYPE_CRC: traceString="picture_crc"; break;
    case HASHTYPE_CHECKSUM: traceString="picture_checksum"; break;
    default: assert(false); break;
  }

  if (pDecodedMessageOutputStream)
  {
    (*pDecodedMessageOutputStream) << "  " << std::setw(55) << traceString << ": " << std::hex << std::setfill('0');
  }

  sei.m_pictureHash.hash.clear();
  for(;bytesRead < payloadSize; bytesRead++)
  {
    sei_read_code( NULL, 8, val, traceString);
    sei.m_pictureHash.hash.push_back((UChar)val);
    if (pDecodedMessageOutputStream)
    {
      (*pDecodedMessageOutputStream) << std::setw(2) << val;
    }
  }

  if (pDecodedMessageOutputStream)
  {
    (*pDecodedMessageOutputStream) << std::dec << std::setfill(' ') << "\n";
  }
}


Void SEIReader::xParseSEIScalableNesting(SEIScalableNesting& sei, const NalUnitType nalUnitType, UInt payloadSize, const TComSPS *sps, std::ostream *pDecodedMessageOutputStream)
{
  UInt uiCode;
  SEIMessages seis;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_flag( pDecodedMessageOutputStream, uiCode,            "bitstream_subset_flag"         ); sei.m_bitStreamSubsetFlag = uiCode;
  sei_read_flag( pDecodedMessageOutputStream, uiCode,            "nesting_op_flag"               ); sei.m_nestingOpFlag = uiCode;
  if (sei.m_nestingOpFlag)
  {
    sei_read_flag( pDecodedMessageOutputStream, uiCode,            "default_op_flag"               ); sei.m_defaultOpFlag = uiCode;
    sei_read_uvlc( pDecodedMessageOutputStream, uiCode,            "nesting_num_ops_minus1"        ); sei.m_nestingNumOpsMinus1 = uiCode;
    for (UInt i = sei.m_defaultOpFlag; i <= sei.m_nestingNumOpsMinus1; i++)
    {
      sei_read_code( pDecodedMessageOutputStream, 3,        uiCode,  "nesting_max_temporal_id_plus1[i]"   ); sei.m_nestingMaxTemporalIdPlus1[i] = uiCode;
      sei_read_uvlc( pDecodedMessageOutputStream, uiCode,            "nesting_op_idx[i]"                  ); sei.m_nestingOpIdx[i] = uiCode;
    }
  }
  else
  {
    sei_read_flag( pDecodedMessageOutputStream, uiCode,            "all_layers_flag"               ); sei.m_allLayersFlag       = uiCode;
    if (!sei.m_allLayersFlag)
    {
      sei_read_code( pDecodedMessageOutputStream, 3,        uiCode,  "nesting_no_op_max_temporal_id_plus1"  ); sei.m_nestingNoOpMaxTemporalIdPlus1 = uiCode;
      sei_read_uvlc( pDecodedMessageOutputStream, uiCode,            "nesting_num_layers_minus1"            ); sei.m_nestingNumLayersMinus1        = uiCode;
      for (UInt i = 0; i <= sei.m_nestingNumLayersMinus1; i++)
      {
        sei_read_code( pDecodedMessageOutputStream, 6,           uiCode,     "nesting_layer_id[i]"      ); sei.m_nestingLayerId[i]   = uiCode;
      }
    }
  }

  // byte alignment
  while ( m_pcBitstream->getNumBitsRead() % 8 != 0 )
  {
    UInt code;
    sei_read_flag( pDecodedMessageOutputStream, code, "nesting_zero_bit" );
  }

  // read nested SEI messages
  do
  {
    if(nalUnitType == NAL_UNIT_PREFIX_SEI)
    {
      xReadSEImessage(sei.m_nestedSEIs, nalUnitType, sps, pDecodedMessageOutputStream, SEI::prefix_sei_messages, std::string("scalable nested SEI"));
    }
    else
    {
      xReadSEImessage(sei.m_nestedSEIs, nalUnitType, sps, pDecodedMessageOutputStream, SEI::suffix_sei_messages, std::string("scalable nested SEI"));
    }
  } while (m_pcBitstream->getNumBitsLeft() > 8);

  if (pDecodedMessageOutputStream)
  {
    (*pDecodedMessageOutputStream) << "End of scalable nesting SEI message\n";
  }
}


Void SEIReader::xParseSEIRegionRefreshInfo(SEIRegionRefreshInfo& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  UInt val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  sei_read_flag( pDecodedMessageOutputStream, val, "refreshed_region_flag" ); sei.m_gdrForegroundFlag = val ? 1 : 0;
}


Void SEIReader::xParseSEINoDisplay(SEINoDisplay& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  sei.m_noDisplay = true;
}


Void SEIReader::xParseSEITimeCode(SEITimeCode& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  UInt code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  sei_read_code( pDecodedMessageOutputStream, 2, code, "num_clock_ts"); sei.numClockTs = code;
  for(Int i = 0; i < sei.numClockTs; i++)
  {
    TComSEITimeSet currentTimeSet;
    sei_read_flag( pDecodedMessageOutputStream, code, "clock_time_stamp_flag[i]"); currentTimeSet.clockTimeStampFlag = code;
    if(currentTimeSet.clockTimeStampFlag)
    {
      sei_read_flag( pDecodedMessageOutputStream, code, "nuit_field_based_flag"); currentTimeSet.numUnitFieldBasedFlag = code;
      sei_read_code( pDecodedMessageOutputStream, 5, code, "counting_type"); currentTimeSet.countingType = code;
      sei_read_flag( pDecodedMessageOutputStream, code, "full_timestamp_flag"); currentTimeSet.fullTimeStampFlag = code;
      sei_read_flag( pDecodedMessageOutputStream, code, "discontinuity_flag"); currentTimeSet.discontinuityFlag = code;
      sei_read_flag( pDecodedMessageOutputStream, code, "cnt_dropped_flag"); currentTimeSet.cntDroppedFlag = code;
      sei_read_code( pDecodedMessageOutputStream, 9, code, "n_frames"); currentTimeSet.numberOfFrames = code;
      if(currentTimeSet.fullTimeStampFlag)
      {
        sei_read_code( pDecodedMessageOutputStream, 6, code, "seconds_value"); currentTimeSet.secondsValue = code;
        sei_read_code( pDecodedMessageOutputStream, 6, code, "minutes_value"); currentTimeSet.minutesValue = code;
        sei_read_code( pDecodedMessageOutputStream, 5, code, "hours_value"); currentTimeSet.hoursValue = code;
      }
      else
      {
        sei_read_flag( pDecodedMessageOutputStream, code, "seconds_flag"); currentTimeSet.secondsFlag = code;
        if(currentTimeSet.secondsFlag)
        {
          sei_read_code( pDecodedMessageOutputStream, 6, code, "seconds_value"); currentTimeSet.secondsValue = code;
          sei_read_flag( pDecodedMessageOutputStream, code, "minutes_flag"); currentTimeSet.minutesFlag = code;
          if(currentTimeSet.minutesFlag)
          {
            sei_read_code( pDecodedMessageOutputStream, 6, code, "minutes_value"); currentTimeSet.minutesValue = code;
            sei_read_flag( pDecodedMessageOutputStream, code, "hours_flag"); currentTimeSet.hoursFlag = code;
            if(currentTimeSet.hoursFlag)
            {
              sei_read_code( pDecodedMessageOutputStream, 5, code, "hours_value"); currentTimeSet.hoursValue = code;
            }
          }
        }
      }
      sei_read_code( pDecodedMessageOutputStream, 5, code, "time_offset_length"); currentTimeSet.timeOffsetLength = code;
      if(currentTimeSet.timeOffsetLength > 0)
      {
        sei_read_code( pDecodedMessageOutputStream, currentTimeSet.timeOffsetLength, code, "time_offset_value");
        if((code & (1 << (currentTimeSet.timeOffsetLength-1))) == 0)
        {
          currentTimeSet.timeOffsetValue = code;
        }
        else
        {
          code &= (1<< (currentTimeSet.timeOffsetLength-1)) - 1;
          currentTimeSet.timeOffsetValue = ~code + 1;
        }
      }
    }
    sei.timeSetArray[i] = currentTimeSet;
  }
}


Void SEIReader::xParseSEIMasteringDisplayColourVolume(SEIMasteringDisplayColourVolume& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  UInt code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_code( pDecodedMessageOutputStream, 16, code, "display_primaries_x[0]" ); sei.values.primaries[0][0] = code;
  sei_read_code( pDecodedMessageOutputStream, 16, code, "display_primaries_y[0]" ); sei.values.primaries[0][1] = code;

  sei_read_code( pDecodedMessageOutputStream, 16, code, "display_primaries_x[1]" ); sei.values.primaries[1][0] = code;
  sei_read_code( pDecodedMessageOutputStream, 16, code, "display_primaries_y[1]" ); sei.values.primaries[1][1] = code;

  sei_read_code( pDecodedMessageOutputStream, 16, code, "display_primaries_x[2]" ); sei.values.primaries[2][0] = code;
  sei_read_code( pDecodedMessageOutputStream, 16, code, "display_primaries_y[2]" ); sei.values.primaries[2][1] = code;


  sei_read_code( pDecodedMessageOutputStream, 16, code, "white_point_x" ); sei.values.whitePoint[0] = code;
  sei_read_code( pDecodedMessageOutputStream, 16, code, "white_point_y" ); sei.values.whitePoint[1] = code;

  sei_read_code( pDecodedMessageOutputStream, 32, code, "max_display_mastering_luminance" ); sei.values.maxLuminance = code;
  sei_read_code( pDecodedMessageOutputStream, 32, code, "min_display_mastering_luminance" ); sei.values.minLuminance = code;
}


Void SEIReader::xParseSEISegmentedRectFramePacking(SEISegmentedRectFramePacking& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  UInt val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  sei_read_flag( pDecodedMessageOutputStream, val,       "segmented_rect_frame_packing_arrangement_cancel_flag" );       sei.m_arrangementCancelFlag            = val;
  if( !sei.m_arrangementCancelFlag )
  {
    sei_read_code( pDecodedMessageOutputStream, 2, val, "segmented_rect_content_interpretation_type" );                sei.m_contentInterpretationType = val;
    sei_read_flag( pDecodedMessageOutputStream, val,     "segmented_rect_frame_packing_arrangement_persistence" );                              sei.m_arrangementPersistenceFlag               = val;
  }
}


Void SEIReader::xParseSEITempMotionConstraintsTileSets(SEITempMotionConstrainedTileSets& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  UInt code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  sei_read_flag( pDecodedMessageOutputStream, code, "mc_all_tiles_exact_sample_value_match_flag");  sei.m_mc_all_tiles_exact_sample_value_match_flag = (code != 0);
  sei_read_flag( pDecodedMessageOutputStream, code, "each_tile_one_tile_set_flag");                 sei.m_each_tile_one_tile_set_flag                = (code != 0);

  if(!sei.m_each_tile_one_tile_set_flag)
  {
    sei_read_flag( pDecodedMessageOutputStream, code, "limited_tile_set_display_flag");  sei.m_limited_tile_set_display_flag = (code != 0);
    sei_read_uvlc( pDecodedMessageOutputStream, code, "num_sets_in_message_minus1");     sei.setNumberOfTileSets(code + 1);

    if(sei.getNumberOfTileSets() != 0)
    {
      for(Int i = 0; i < sei.getNumberOfTileSets(); i++)
      {
        sei_read_uvlc( pDecodedMessageOutputStream, code, "mcts_id");  sei.tileSetData(i).m_mcts_id = code;

        if(sei.m_limited_tile_set_display_flag)
        {
          sei_read_flag( pDecodedMessageOutputStream, code, "display_tile_set_flag");  sei.tileSetData(i).m_display_tile_set_flag = (code != 1);
        }

        sei_read_uvlc( pDecodedMessageOutputStream, code, "num_tile_rects_in_set_minus1");  sei.tileSetData(i).setNumberOfTileRects(code + 1);

        for(Int j=0; j<sei.tileSetData(i).getNumberOfTileRects(); j++)
        {
          sei_read_uvlc( pDecodedMessageOutputStream, code, "top_left_tile_index");      sei.tileSetData(i).topLeftTileIndex(j)     = code;
          sei_read_uvlc( pDecodedMessageOutputStream, code, "bottom_right_tile_index");  sei.tileSetData(i).bottomRightTileIndex(j) = code;
        }

        if(!sei.m_mc_all_tiles_exact_sample_value_match_flag)
        {
          sei_read_flag( pDecodedMessageOutputStream, code, "exact_sample_value_match_flag");   sei.tileSetData(i).m_exact_sample_value_match_flag    = (code != 0);
        }
        sei_read_flag( pDecodedMessageOutputStream, code, "mcts_tier_level_idc_present_flag");  sei.tileSetData(i).m_mcts_tier_level_idc_present_flag = (code != 0);

        if(sei.tileSetData(i).m_mcts_tier_level_idc_present_flag)
        {
          sei_read_flag( pDecodedMessageOutputStream, code,    "mcts_tier_flag"); sei.tileSetData(i).m_mcts_tier_flag = (code != 0);
          sei_read_code( pDecodedMessageOutputStream, 8, code, "mcts_level_idc"); sei.tileSetData(i).m_mcts_level_idc =  code;
        }
      }
    }
  }
  else
  {
    sei_read_flag( pDecodedMessageOutputStream, code, "max_mcs_tier_level_idc_present_flag");  sei.m_max_mcs_tier_level_idc_present_flag = code;
    if(sei.m_max_mcs_tier_level_idc_present_flag)
    {
      sei_read_flag( pDecodedMessageOutputStream, code, "max_mcts_tier_flag");  sei.m_max_mcts_tier_flag = code;
      sei_read_code( pDecodedMessageOutputStream, 8, code, "max_mcts_level_idc"); sei.m_max_mcts_level_idc = code;
    }
  }
}

#if MCTS_EXTRACTION
Void SEIReader::xParseSEIMCTSExtractionInfoSet(SEIMCTSExtractionInfoSet& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  UInt code;
  UInt numInfoSetsMinus1;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_uvlc(pDecodedMessageOutputStream, code, "num_sets_in_message_minus1"); numInfoSetsMinus1 = code;
  for (Int i = 0; i <= numInfoSetsMinus1; i++)
  {
    SEIMCTSExtractionInfoSet::MCTSExtractionInfo EIS;

    sei_read_uvlc(pDecodedMessageOutputStream, code, "num_mcts_sets_minus1[ i ]");
    EIS.m_idxOfMctsInSet.resize(code + 1);
    for (Int j = 0; j < EIS.m_idxOfMctsInSet.size(); j++)
    {
      sei_read_uvlc(pDecodedMessageOutputStream, code, "num_mcts_in_set_minus1[ i ][ j ]");
      EIS.m_idxOfMctsInSet[j].resize(code + 1);
      for (Int k = 0; k < EIS.m_idxOfMctsInSet[j].size(); k++)
      {
        sei_read_uvlc(pDecodedMessageOutputStream, code, "idx_of_mcts_in_set[ i ][ j ][ k ]"); EIS.m_idxOfMctsInSet[j][k] = code;
      }
    }
    sei_read_flag(pDecodedMessageOutputStream, code, "slice_reordering_enabled_flag[ i ]");  EIS.m_sliceReorderingEnabledFlag = (code != 0);
    if (EIS.m_sliceReorderingEnabledFlag) 
    {
      sei_read_uvlc(pDecodedMessageOutputStream, code, "num_slice_segments_minus1[ i ]");
      EIS.m_outputSliceSegmentAddress.resize(code + 1);
      for (Int j = 0; j < EIS.m_outputSliceSegmentAddress.size(); j++)
      {
        sei_read_uvlc(pDecodedMessageOutputStream, code, "output_slice_segment_address[ i ][ j ]"); EIS.m_outputSliceSegmentAddress[j] = code;
      }
    }

    sei_read_uvlc(pDecodedMessageOutputStream, code, "num_vps_in_info_set_minus1[i]");
    EIS.m_vpsRbspData.resize(code + 1);
    EIS.m_vpsRbspDataLength.resize(code + 1);
    for (Int j = 0; j < EIS.m_vpsRbspDataLength.size(); j++)
    {
      sei_read_uvlc(pDecodedMessageOutputStream, code, "vps_rbsp_data_length[i][j]"); EIS.m_vpsRbspDataLength[j] = code;
    }
    sei_read_uvlc(pDecodedMessageOutputStream, code, "num_sps_in_info_set_minus1[i]");
    EIS.m_spsRbspData.resize(code + 1);
    EIS.m_spsRbspDataLength.resize(code + 1);
    for (Int j = 0; j < EIS.m_spsRbspDataLength.size(); j++)
    {
      sei_read_uvlc(pDecodedMessageOutputStream, code, "sps_rbsp_data_length[i][j]"); EIS.m_spsRbspDataLength[j] = code;
    }
    sei_read_uvlc(pDecodedMessageOutputStream, code, "num_pps_in_info_set_minus1[i]");
    EIS.m_ppsRbspData.resize(code + 1);
    EIS.m_ppsNuhTemporalIdPlus1.resize(code + 1);
    EIS.m_ppsRbspDataLength.resize(code + 1);
    for (Int j = 0; j < EIS.m_ppsRbspDataLength.size(); j++)
    {
      sei_read_uvlc(pDecodedMessageOutputStream, code, "pps_nuh_temporal_id_plus1[i][j]"); EIS.m_ppsNuhTemporalIdPlus1[j] = code;
      sei_read_uvlc(pDecodedMessageOutputStream, code, "sps_rbsp_data_length[i][j]"); EIS.m_ppsRbspDataLength[j] = code;
    }

    // byte alignment
    while (m_pcBitstream->getNumBitsRead() % 8 != 0)
    {
      sei_read_flag(pDecodedMessageOutputStream, code, "mcts_alignment_bit_equal_to_zero");
    }

    for (Int j = 0; j < EIS.m_vpsRbspData.size(); j++)
    {
      EIS.m_vpsRbspData[j].resize(EIS.m_vpsRbspDataLength[j]);
      for (Int k = 0; k < EIS.m_vpsRbspDataLength[j]; k++) 
      {
        sei_read_code(pDecodedMessageOutputStream, 8, code, "vps_rbsp_data_byte[ i ][ j ][ k ]"); EIS.m_vpsRbspData[j][k] = code;
      }
    }
    for (Int j = 0; j < EIS.m_spsRbspData.size(); j++)
    {
      EIS.m_spsRbspData[j].resize(EIS.m_spsRbspDataLength[j]);
      for (Int k = 0; k < EIS.m_spsRbspDataLength[j]; k++) 
      {
        sei_read_code(pDecodedMessageOutputStream, 8, code, "sps_rbsp_data_byte[ i ][ j ][ k ]"); EIS.m_spsRbspData[j][k] = code;
      }
    }
    for (Int j = 0; j < EIS.m_ppsRbspData.size(); j++)
    {
      EIS.m_ppsRbspData[j].resize(EIS.m_ppsRbspDataLength[j]);
      for (Int k = 0; k < EIS.m_ppsRbspDataLength[j]; k++) 
      {
        sei_read_code(pDecodedMessageOutputStream, 8, code, "pps_rbsp_data_byte[ i ][ j ][ k ]"); EIS.m_ppsRbspData[j][k] = code;
      }
    }
    sei.m_MCTSExtractionInfoSets.push_back(EIS);
  }
}

#endif

Void SEIReader::xParseSEIChromaResamplingFilterHint(SEIChromaResamplingFilterHint& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  UInt uiCode;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_code( pDecodedMessageOutputStream, 8, uiCode, "ver_chroma_filter_idc"); sei.m_verChromaFilterIdc = uiCode;
  sei_read_code( pDecodedMessageOutputStream, 8, uiCode, "hor_chroma_filter_idc"); sei.m_horChromaFilterIdc = uiCode;
  sei_read_flag( pDecodedMessageOutputStream, uiCode, "ver_filtering_field_processing_flag"); sei.m_verFilteringFieldProcessingFlag = uiCode;
  if(sei.m_verChromaFilterIdc == 1 || sei.m_horChromaFilterIdc == 1)
  {
    sei_read_uvlc( pDecodedMessageOutputStream, uiCode, "target_format_idc"); sei.m_targetFormatIdc = uiCode;
    if(sei.m_verChromaFilterIdc == 1)
    {
      UInt numVerticalFilters;
      sei_read_uvlc( pDecodedMessageOutputStream, numVerticalFilters, "num_vertical_filters"); sei.m_verFilterCoeff.resize(numVerticalFilters);
      if(numVerticalFilters > 0)
      {
        for(Int i = 0; i < numVerticalFilters; i++)
        {
          UInt verTapLengthMinus1;
          sei_read_uvlc( pDecodedMessageOutputStream, verTapLengthMinus1, "ver_tap_length_minus_1"); sei.m_verFilterCoeff[i].resize(verTapLengthMinus1+1);
          for(Int j = 0; j < (verTapLengthMinus1 + 1); j++)
          {
            sei_read_svlc( pDecodedMessageOutputStream, sei.m_verFilterCoeff[i][j], "ver_filter_coeff");
          }
        }
      }
    }
    if(sei.m_horChromaFilterIdc == 1)
    {
      UInt numHorizontalFilters;
      sei_read_uvlc( pDecodedMessageOutputStream, numHorizontalFilters, "num_horizontal_filters"); sei.m_horFilterCoeff.resize(numHorizontalFilters);
      if(numHorizontalFilters  > 0)
      {
        for(Int i = 0; i < numHorizontalFilters; i++)
        {
          UInt horTapLengthMinus1;
          sei_read_uvlc( pDecodedMessageOutputStream, horTapLengthMinus1, "hor_tap_length_minus_1"); sei.m_horFilterCoeff[i].resize(horTapLengthMinus1+1);
          for(Int j = 0; j < (horTapLengthMinus1 + 1); j++)
          {
            sei_read_svlc( pDecodedMessageOutputStream, sei.m_horFilterCoeff[i][j], "hor_filter_coeff");
          }
        }
      }
    }
  }
}


Void SEIReader::xParseSEIKneeFunctionInfo(SEIKneeFunctionInfo& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  Int i;
  UInt val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_uvlc( pDecodedMessageOutputStream, val, "knee_function_id" );                   sei.m_kneeId = val;
  sei_read_flag( pDecodedMessageOutputStream, val, "knee_function_cancel_flag" );          sei.m_kneeCancelFlag = val;
  if ( !sei.m_kneeCancelFlag )
  {
    sei_read_flag( pDecodedMessageOutputStream, val, "knee_function_persistence_flag" );   sei.m_kneePersistenceFlag = val;
    sei_read_code( pDecodedMessageOutputStream, 32, val, "input_d_range" );                sei.m_kneeInputDrange = val;
    sei_read_code( pDecodedMessageOutputStream, 32, val, "input_disp_luminance" );         sei.m_kneeInputDispLuminance = val;
    sei_read_code( pDecodedMessageOutputStream, 32, val, "output_d_range" );               sei.m_kneeOutputDrange = val;
    sei_read_code( pDecodedMessageOutputStream, 32, val, "output_disp_luminance" );        sei.m_kneeOutputDispLuminance = val;
    sei_read_uvlc( pDecodedMessageOutputStream, val, "num_knee_points_minus1" );           sei.m_kneeNumKneePointsMinus1 = val;
    assert( sei.m_kneeNumKneePointsMinus1 > 0 );
    sei.m_kneeInputKneePoint.resize(sei.m_kneeNumKneePointsMinus1+1);
    sei.m_kneeOutputKneePoint.resize(sei.m_kneeNumKneePointsMinus1+1);
    for(i = 0; i <= sei.m_kneeNumKneePointsMinus1; i++ )
    {
      sei_read_code( pDecodedMessageOutputStream, 10, val, "input_knee_point" );           sei.m_kneeInputKneePoint[i] = val;
      sei_read_code( pDecodedMessageOutputStream, 10, val, "output_knee_point" );          sei.m_kneeOutputKneePoint[i] = val;
    }
  }
}

Void SEIReader::xParseSEIContentColourVolume(SEIContentColourVolume& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  Int i;
  UInt val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_flag( pDecodedMessageOutputStream, val, "ccv_cancel_flag" );          sei.m_ccvCancelFlag = val;
  if ( !sei.m_ccvCancelFlag )
  {
    Int iVal;
    sei_read_flag( pDecodedMessageOutputStream, val, "ccv_persistence_flag" );   sei.m_ccvPersistenceFlag = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "ccv_primaries_present_flag" );   sei.m_ccvPrimariesPresentFlag = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "ccv_min_luminance_value_present_flag" );   sei.m_ccvMinLuminanceValuePresentFlag = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "ccv_max_luminance_value_present_flag" );   sei.m_ccvMaxLuminanceValuePresentFlag = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "ccv_avg_luminance_value_present_flag" );   sei.m_ccvAvgLuminanceValuePresentFlag = val;
    
    if (sei.m_ccvPrimariesPresentFlag) 
    {
      for (i = 0; i < MAX_NUM_COMPONENT; i++) 
      {
        sei_read_scode( pDecodedMessageOutputStream, 32, iVal, "ccv_primaries_x[i]" );          sei.m_ccvPrimariesX[i] = iVal;
        sei_read_scode( pDecodedMessageOutputStream, 32, iVal, "ccv_primaries_y[i]" );          sei.m_ccvPrimariesY[i] = iVal;
      }
    }
    if (sei.m_ccvMinLuminanceValuePresentFlag) 
    {
      sei_read_code( pDecodedMessageOutputStream, 32, val,     "ccv_min_luminance_value" );   sei.m_ccvMinLuminanceValue = val;
    }
    if (sei.m_ccvMaxLuminanceValuePresentFlag) 
    {
      sei_read_code( pDecodedMessageOutputStream, 32, val,     "ccv_max_luminance_value" );   sei.m_ccvMaxLuminanceValue = val;
    }
    if (sei.m_ccvAvgLuminanceValuePresentFlag) 
    {
      sei_read_code( pDecodedMessageOutputStream, 32, val,     "ccv_avg_luminance_value" );   sei.m_ccvAvgLuminanceValue = val;
    }
  }
}
#if JVET_AL0061_ENCODER_OPTIMIZATION_INFORMATION_SEI
void SEIReader::xParseSEIEncoderOptimizationInfo(SEIEncoderOptimizationInfo& sei, uint32_t payloadSize, std::ostream* pDecodedMessageOutputStream)
{
  uint32_t val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  sei_read_flag(pDecodedMessageOutputStream, val, "eoi_cancel_flag");
  sei.m_cancelFlag = val;
  if (!sei.m_cancelFlag)
  {
    sei_read_flag(pDecodedMessageOutputStream, val, "eoi_persistence_flag");
    sei.m_persistenceFlag = val;
    sei_read_code(pDecodedMessageOutputStream, 2, val, "eoi_for_human_viewing_idc");
    sei.m_forHumanViewingIdc = val;
    sei_read_code(pDecodedMessageOutputStream, 2, val, "eoi_for_machine_analysis_idc");
    sei.m_forMachineAnalysisIdc = val;
    sei_read_code(pDecodedMessageOutputStream, 2, val, "eoi_reserved_zero_2bits");
    
    sei_read_code(pDecodedMessageOutputStream, 16, val, "eoi_type");
    sei.m_type = val;
    if ((sei.m_type & EOI_OptimizationType::OBJECT_BASED_OPTIMIZATION) != 0)
    {
      sei_read_code(pDecodedMessageOutputStream, 16, val, "eoi_object_based_idc");
      sei.m_objectBasedIdc = val;
      if (sei.m_objectBasedIdc & EOI_OBJECT_BASED::COARSER_QUANTIZATION)
      {
        sei_read_uvlc(pDecodedMessageOutputStream, val, "eoi_quant_threshold_delta");
        sei.m_quantThresholdDelta = val;
        if (sei.m_quantThresholdDelta > 0)
        {
          sei_read_flag(pDecodedMessageOutputStream, val, "eoi_pic_quant_object_flag");
          sei.m_picQuantObjectFlag = val;
        }
      }
    }
    if ((sei.m_type & EOI_OptimizationType::TEMPORAL_RESAMPLING) != 0)
    {
      sei_read_flag(pDecodedMessageOutputStream, val, "eoi_temporal_resampling_type_flag");
      sei.m_temporalResamplingTypeFlag = val;
      sei_read_uvlc(pDecodedMessageOutputStream, val, "eoi_num_int_pics");
      sei.m_numIntPics = val;
      if (sei.m_temporalResamplingTypeFlag && sei.m_numIntPics > 0)
      {
        sei_read_flag(pDecodedMessageOutputStream, val, "eoi_src_pic_flag");
        sei.m_srcPicFlag = val;
      }
    }

    if ((sei.m_type & EOI_OptimizationType::SPATIAL_RESAMPLING) != 0)
    {
      sei_read_flag(pDecodedMessageOutputStream, val, "eoi_orig_pic_dimensions_flag");
      sei.m_origPicDimensionsFlag = val;
      if (sei.m_origPicDimensionsFlag)
      {
        sei_read_code(pDecodedMessageOutputStream, 16, val, "eoi_orig_pic_width");
        sei.m_origPicWidth = val;
        sei_read_code(pDecodedMessageOutputStream, 16, val, "eoi_orig_pic_height");
        sei.m_origPicHeight = val;
      }
      else
      {
        sei_read_flag(pDecodedMessageOutputStream, val, "eoi_spatial_resampling_type_flag");
        sei.m_spatialResamplingTypeFlag = val;
      }
    }

    if ((sei.m_type & EOI_OptimizationType::PRIVACY_PROTECTION_OPTIMIZATION) != 0)
    {
      sei_read_code(pDecodedMessageOutputStream, 16, val, "eoi_privacy_protection_type_idc");
      sei.m_privacyProtectionTypeIdc = val;
      sei_read_code(pDecodedMessageOutputStream, 8, val, "eoi_privacy_protected_info_type");
      sei.m_privacyProtectedInfoType = val;
    }
  }
}

#endif
#if SHUTTER_INTERVAL_SEI_MESSAGE
Void SEIReader::xParseSEIShutterInterval(SEIShutterIntervalInfo& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  Int i;
  UInt val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  sei_read_code(pDecodedMessageOutputStream, 32, val, "sii_time_scale");                      sei.m_siiTimeScale = val;
  sei_read_flag(pDecodedMessageOutputStream, val, "fixed_shutter_interval_within_clvs_flag"); sei.m_siiFixedSIwithinCLVS = val;
  if (sei.m_siiFixedSIwithinCLVS)
  {
    sei_read_code(pDecodedMessageOutputStream, 32, val, "sii_num_units_in_shutter_interval");   sei.m_siiNumUnitsInShutterInterval = val;
  }
  else
  {
    sei_read_code(pDecodedMessageOutputStream, 3,  val, "sii_max_sub_layers_minus1 ");          sei.m_siiMaxSubLayersMinus1 = val;
    sei.m_siiSubLayerNumUnitsInSI.resize(sei.m_siiMaxSubLayersMinus1 + 1);
    for (i = 0; i <= sei.m_siiMaxSubLayersMinus1; i++)
    {
      sei_read_code(pDecodedMessageOutputStream, 32, val, "sub_layer_num_units_in_shutter_interval[ i ]");
      sei.m_siiSubLayerNumUnitsInSI[i] = val;
    }
  }
}
#endif

#if NNPFC_SEI_MESSAGE
Void SEIReader::xParseSEINNPostFilterCharacteristics(SEINeuralNetworkPostFilterCharacteristics& sei, UInt payloadSize, const TComSPS* sps, std::ostream *pDecodedMessageOutputStream)
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  UInt val;

  sei_read_code(pDecodedMessageOutputStream, 16, val, "nnpfc_purpose");
  sei.m_purpose = val;
  CHECK(sei.m_purpose >= 128 && sei.m_purpose <= 65535, "Reserved nnpfc_purpose value");

  sei_read_uvlc( pDecodedMessageOutputStream, val, "nnpfc_id" );
  sei.m_id = val;
  CHECK((sei.m_id >= 256 && sei.m_id <= 511) || (sei.m_id >= (1<<31) && sei.m_id <= MAX_NNPFC_ID), "Reserved nnpfc_id value, shall ignore the SEI message");

  sei_read_flag(pDecodedMessageOutputStream, val, "nnpfc_base_flag");
  sei.m_baseFlag = val;

  sei_read_uvlc( pDecodedMessageOutputStream, val, "nnpfc_mode_idc" );
  sei.m_modeIdc = val;

  if (sei.m_modeIdc == POST_FILTER_MODE::URI)
  {
    std::string val2;
    while (m_pcBitstream->getNumBitsUntilByteAligned() != 0)
    {
      sei_read_flag(pDecodedMessageOutputStream, val, "nnpfc_alignment_zero_bit");
      CHECK(val != 0, "nnpfc_alignment_zero_bit not equal to zero");
    }

    sei_read_string(pDecodedMessageOutputStream, val2, "nnpfc_uri_tag");
    sei.m_uriTag = val2;

    val2 = "";
    sei_read_string(pDecodedMessageOutputStream, val2, "nnpfc_uri");
    sei.m_uri = val2;
  }

  sei_read_flag(pDecodedMessageOutputStream, val, "nnpfc_property_present_flag");
  sei.m_propertyPresentFlag = val;

  if (sei.m_propertyPresentFlag)
  {
    ChromaFormat chromaFormatIdc = sps->getChromaFormatIdc();
    UChar        subWidthC;
    UChar        subHeightC;
    if (chromaFormatIdc == ChromaFormat::CHROMA_420)
    {
      subWidthC  = 2;
      subHeightC = 2;
    }
    else if (chromaFormatIdc == ChromaFormat::CHROMA_422)
    {
      subWidthC  = 2;
      subHeightC = 1;
    }
    else
    {
      subWidthC  = 1;
      subHeightC = 1;
    }

    sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_number_of_input_pictures_minus1");
    sei.m_numberInputDecodedPicturesMinus1 = val;

    sei.m_inputPicOutputFlag.clear();
    sei.m_numInpPicsInOutputTensor = 0;
    if (sei.m_numberInputDecodedPicturesMinus1 > 0)
    {
      Bool atLeastOne = false;
      for (Int i = 0; i <= sei.m_numberInputDecodedPicturesMinus1; i++)
      {
        sei_read_flag(pDecodedMessageOutputStream, val, "nnpfc_input_pic_filtering_flag");
        sei.m_inputPicOutputFlag.push_back((Bool)val);
        if (sei.m_inputPicOutputFlag[i])
        {
          atLeastOne = true;
          sei.m_numInpPicsInOutputTensor++;
        }
      }
      if ((sei.m_purpose & NNPC_PurposeType::FRAME_RATE_UPSAMPLING) == 0)
      {
        CHECK(!atLeastOne, "When picRateUpsamplingFlag is equal to 0 and nnpfc_num_input_pics_minus1 is greater than 0, at least one value of nnpfc_input_pic_filtering_flag[i] shall be greater than 0");
      }
      sei_read_flag(pDecodedMessageOutputStream, val, "nnpfc_absent_input_pic_zero_flag");
      sei.m_absentInputPicZeroFlag = val;
    }
    else
    {
      sei.m_inputPicOutputFlag.push_back(true);
      sei.m_numInpPicsInOutputTensor = 1;
    }

    if((sei.m_purpose & NNPC_PurposeType::CHROMA_UPSAMPLING) != 0)
    {
      sei_read_flag(pDecodedMessageOutputStream, val, "nnpfc_out_sub_c_flag");
      sei.m_outSubCFlag = val;

      CHECK(((subWidthC == 2) && (subHeightC == 1) && (sei.m_outSubCFlag == 0)),
            "If SubWidthC is equal to 2 and SubHeightC is equal to 1, nnpfc_out_sub_c_flag shall not be equal to 0");

      if (sei.m_outSubCFlag)
      {
        sei.m_outSubWidthC = 1;
        sei.m_outSubHeightC = 1;
      }
      else
      {
        sei.m_outSubWidthC = 2;
        sei.m_outSubHeightC = 1;
      }
    }

    CHECK(((subWidthC == 1) && (subHeightC == 1)) && ((sei.m_purpose & NNPC_PurposeType::CHROMA_UPSAMPLING) != 0),
          "If SubWidthC is equal to 1 and SubHeightC is equal to 1, nnpfc_purpose & 0x02 shall be equal to 0");

    if((sei.m_purpose & NNPC_PurposeType::COLOURIZATION) != 0)
    {
      CHECK(((sei.m_purpose & NNPC_PurposeType::CHROMA_UPSAMPLING) != 0), "When chromaUpsamplingFlag is not equal to 0, colourizationFlag shall be equal to 0");

      sei_read_code(pDecodedMessageOutputStream, 2, val, "nnpfc_out_colour_format_idc");
      sei.m_outColourFormatIdc = ChromaFormat(val);
      CHECK(sei.m_outColourFormatIdc == ChromaFormat::CHROMA_400,
            "The value of nnpfc_out_colour_format_idc shall not be equal to 0");

      sei.m_outSubWidthC  = TComSPS::getWinUnitX(sei.m_outColourFormatIdc);
      sei.m_outSubHeightC = TComSPS::getWinUnitY(sei.m_outColourFormatIdc);
    }

    if (((sei.m_purpose & NNPC_PurposeType::CHROMA_UPSAMPLING) == 0) && ((sei.m_purpose & NNPC_PurposeType::COLOURIZATION) == 0))
    {
      sei.m_outSubWidthC  = subWidthC;
      sei.m_outSubHeightC = subHeightC;
    }

    if((sei.m_purpose & NNPC_PurposeType::RESOLUTION_UPSAMPLING) != 0)
    {
      sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_pic_width_num_minus1");
      sei.m_picWidthNumeratorMinus1 = val;
      sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_pic_width_denominator_minus1");
      sei.m_picWidthDenominatorMinus1 = val;

      sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_pic_height_num_minus1");
      sei.m_picHeightNumeratorMinus1 = val;
      sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_pic_height_denominator_minus1");
      sei.m_picHeightDenominatorMinus1 = val;
      CHECK(sei.m_picWidthNumeratorMinus1 > 65535, "nnpfc_pic_width_num_minus1 shall be in the range of 0 to 65535");
      CHECK(sei.m_picWidthDenominatorMinus1 > 65535, "nnpfc_pic_width_denom_minus1 shall be in the range of 0 to 65535");
      CHECK(sei.m_picHeightNumeratorMinus1 > 65535, "nnpfc_pic_height_num_minus1 shall be in the range of 0 to 65535");
      CHECK(sei.m_picHeightDenominatorMinus1 > 65535, "nnpfc_pic_height_denom_minus1 shall be in the range of 0 to 65535");
      Int scaledHeightRatio = 16 * (sei.m_picHeightNumeratorMinus1 + 1) / (sei.m_picHeightDenominatorMinus1 + 1);
      Int scaledWidthRatio = 16 * (sei.m_picWidthNumeratorMinus1 + 1) / (sei.m_picWidthDenominatorMinus1 + 1);

      CHECK((scaledHeightRatio < 1) && (scaledHeightRatio > 256), "The value range of heightRatio shall be in the range of 1/16 to 16, inclusive");
      CHECK((scaledWidthRatio < 1) && (scaledWidthRatio > 256), "The value range of widthRatio shall be in the range of 1/16 to 16, inclusive");
    }

    if((sei.m_purpose & NNPC_PurposeType::FRAME_RATE_UPSAMPLING) != 0)
    {
      CHECK(sei.m_numberInputDecodedPicturesMinus1 <= 0, "If nnpfc_purpose is FRAME_RATE_UPSAMPLING, nnpfc_num_input_pics_minus1 shall be greater than 0");
      sei.m_numberInterpolatedPictures.resize(sei.m_numberInputDecodedPicturesMinus1);
      Bool allZeroFlag = false;
      for (Int i = 0; i < sei.m_numberInterpolatedPictures.size(); i++)
      {
        sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_interpolated_pictures");
        sei.m_numberInterpolatedPictures[i] = val;
        if(sei.m_numberInterpolatedPictures[i] > 0)
        {
          allZeroFlag = true;
        }
      }
      CHECK(!allZeroFlag, "At least one value of nnpfc_interpolated_pics[i] shall be greater than 0");
    }

    if ((sei.m_purpose & NNPC_PurposeType::TEMPORAL_EXTRAPOLATION) != 0)
    {
      sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_extrapolated_pics_minus1");
      sei.m_numberExtrapolatedPicturesMinus1 = val;
    }

    if((sei.m_purpose & NNPC_PurposeType::SPATIAL_EXTRAPOLATION) != 0)
    {
      Int value;
      sei_read_svlc(pDecodedMessageOutputStream, value, "nnpfc_spatial_extrapolation_left_offset");
      sei.m_spatialExtrapolationLeftOffset = value;
      sei_read_svlc(pDecodedMessageOutputStream, value, "nnpfc_spatial_extrapolation_right_offset");
      sei.m_spatialExtrapolationRightOffset = value;
      sei_read_svlc(pDecodedMessageOutputStream, value, "nnpfc_spatial_extrapolation_top_offset");
      sei.m_spatialExtrapolationTopOffset = value;
      sei_read_svlc(pDecodedMessageOutputStream, value, "nnpfc_spatial_extrapolation_bottom_offset");
      sei.m_spatialExtrapolationBottomOffset = value;
    }

    sei_read_flag(pDecodedMessageOutputStream, val, "nnpfc_component_last_flag");
    sei.m_componentLastFlag = val;

    sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_inp_format_idc");
    sei.m_inpFormatIdc = val;
    CHECK(sei.m_inpFormatIdc > 255, "The value of nnpfc_inp_format_idc shall be in the range of 0 to 255");

    sei_read_uvlc(pDecodedMessageOutputStream,val,"nnpfc_auxiliary_inp_idc");
    sei.m_auxInpIdc = val;
    CHECK(val > 7, "The value of nnpfc_auxiliary_inp_idc shall be in the range of 0 to 7");
    if ((sei.m_auxInpIdc & 2) > 0)
    {
      sei_read_flag(pDecodedMessageOutputStream, val, "nnpfc_inband_prompt_flag");
      sei.m_inbandPromptFlag = val;
      if (sei.m_inbandPromptFlag)
      {
        std::string valp;
        while (m_pcBitstream->getNumBitsUntilByteAligned() != 0)
        {
          sei_read_flag(pDecodedMessageOutputStream, val, "nnpfc_alignment_zero_bit_c");
          CHECK(val != 0, "nnpfc_alignment_zero_bit_c not equal to zero");
        }
        sei_read_string(pDecodedMessageOutputStream, valp, "nnpfc_prompt");
        sei.m_prompt = valp;
      }
    }
    if ((sei.m_auxInpIdc & 4) > 0)
    {
      sei_read_flag(pDecodedMessageOutputStream, val, "nnpfc_inband_seed_flag");
      sei.m_inbandSeedFlag = val;
      if (sei.m_inbandSeedFlag)
      {
        sei_read_code(pDecodedMessageOutputStream, 16, val, "nnpfc_seed");
        sei.m_seed = val;
      }
    }
    sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_inp_order_idc");
    sei.m_inpOrderIdc = val;
    CHECK(val > 3, "The value of nnpfc_inp_order_idc shall be in the range of 0 to 3");
    CHECK(((sei.m_purpose & NNPC_PurposeType::CHROMA_UPSAMPLING) != 0)  && sei.m_inpOrderIdc == 0, "When nnpfc_purpose & 0x02 is not equal to 0, nnpfc_inp_order_idc shall not be equal to 0.");

    CHECK((chromaFormatIdc == ChromaFormat::CHROMA_400) && (sei.m_inpOrderIdc != 0), "When ChromaFormatIdc is equal to 0, nnpfc_inp_order_idc shall be equal to 0");
    CHECK((chromaFormatIdc != ChromaFormat::CHROMA_420) && (sei.m_inpOrderIdc == 3), "When ChromaFormatIdc is not equal to 1, nnpfc_inp_order_idc shall not be equal to 3");
    CHECK(((sei.m_purpose & NNPC_PurposeType::CHROMA_UPSAMPLING) != 0) && (sei.m_inpOrderIdc == 0), "When chromaUpsamplingFlag is equal to 1, nnpfc_inp_order_idc shall not be equal to 0");

    if (sei.m_inpFormatIdc == 1)
    {
      if (sei.m_inpOrderIdc != 1)
      {
        sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_inp_tensor_luma_bitdepth_minus8");
        sei.m_inpTensorBitDepthLumaMinus8 = val;
        CHECK(val > 24, "The value of nnpfc_inp_tensor_luma_bitdepth_minus8 shall be in the range of 0 to 24");
      }
      if (sei.m_inpOrderIdc != 0)
      {
        sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_inp_tensor_chroma_bitdepth_minus8");
        sei.m_inpTensorBitDepthChromaMinus8 = val;
        CHECK(val > 24, "The value of nnpfc_inp_tensor_chroma_bitdepth_minus8 shall be in the range of 0 to 24");
      }
    }

    sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_out_format_idc");
    sei.m_outFormatIdc = val;
    CHECK(sei.m_outFormatIdc > 255, "The value of nnpfc_out_format_idc shall be in the range of 0 to 255");

    sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_out_order_idc");
    sei.m_outOrderIdc = val;
    CHECK(((sei.m_purpose & NNPC_PurposeType::CHROMA_UPSAMPLING) != 0)  && (sei.m_outOrderIdc == 0 || sei.m_outOrderIdc == 3), "When nnpfc_purpose & 0x02 is not equal to 0, nnpfc_out_order_idc shall not be equal to 0 or 3.");
    CHECK(((sei.m_purpose & NNPC_PurposeType::COLOURIZATION) != 0)  && sei.m_outOrderIdc == 0, "When nnpfc_purpose & 0x20 is not equal to 0, nnpfc_out_order_idc shall not be equal to 0.");

    if (sei.m_outFormatIdc == 1)
    {
      if (sei.m_outOrderIdc != 1)
      {
        sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_out_tensor_luma_bitdepth_minus8");
        sei.m_outTensorBitDepthLumaMinus8 = val;
        CHECK(val > 24, "The value of nnpfc_out_tensor_luma_bitdepth_minus8 shall be in the range of 0 to 24");
      }
      if (sei.m_outOrderIdc != 0)
      {
        sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_out_tensor_chroma_bitdepth_minus8");
        sei.m_outTensorBitDepthChromaMinus8 = val; 
        CHECK(val > 24, "The value of nnpfc_out_tensor_chroma_bitdepth_minus8 shall be in the range of 0 to 24");
      }
    }

    if((sei.m_outFormatIdc == 1) && (sei.m_inpFormatIdc == 1) && (sei.m_outOrderIdc > 1) && (sei.m_inpOrderIdc > 1))
    {
      CHECK((sei.m_outTensorBitDepthLumaMinus8 > sei.m_inpTensorBitDepthLumaMinus8) && (sei.m_outTensorBitDepthChromaMinus8 < sei.m_inpTensorBitDepthChromaMinus8), "When outTensorBitDepthLuma is greater than inpTensorBitDepthLuma, outTensorBitDepthChroma shall not be less than inpTensorBitDepthChroma");
      CHECK((sei.m_outTensorBitDepthLumaMinus8 < sei.m_inpTensorBitDepthLumaMinus8) && (sei.m_outTensorBitDepthChromaMinus8 > sei.m_inpTensorBitDepthChromaMinus8), "When outTensorBitDepthChroma is greater than inpTensorBitDepthChroma, outTensorBitDepthLuma shall not be less than inpTensorBitDepthLuma");
    }

    sei_read_flag(pDecodedMessageOutputStream,val,"nnpfc_sep_col_desc_flag");
    sei.m_sepColDescriptionFlag = val;

    if (sei.m_sepColDescriptionFlag)
    {
      sei_read_code(pDecodedMessageOutputStream, 8, val,"nnpfc_col_primaries");
      sei.m_colPrimaries = val;
      sei_read_code(pDecodedMessageOutputStream, 8, val,"nnpfc_trans_characteristics");
      sei.m_transCharacteristics = val;
      if (sei.m_outFormatIdc == 1)
      {
        sei_read_code(pDecodedMessageOutputStream, 8, val, "nnpfc_matrix_coeffs");
        sei.m_matrixCoeffs = val;
        CHECK(sei.m_matrixCoeffs == 0 && !(sei.m_outTensorBitDepthChromaMinus8 == sei.m_outTensorBitDepthLumaMinus8 && sei.m_outOrderIdc == 2 && sei.m_outSubHeightC == 1 && sei.m_outSubWidthC == 1),
          "nnpfc_matrix_coeffs shall not be equal to 0 unless the following conditions are true: nnpfc_out_tensor_chroma_bitdepth_minus8 is equal to nnpfc_out_tensor_luma_bitdepth_minus8, nnpfc_out_order_idc is equal to 2, outSubHeightC is equal to 1, and outSubWidthC is equal to 1");
        CHECK(sei.m_matrixCoeffs == 8 && !((sei.m_outTensorBitDepthChromaMinus8 == sei.m_outTensorBitDepthLumaMinus8) || (sei.m_outTensorBitDepthChromaMinus8 == (sei.m_outTensorBitDepthLumaMinus8 + 1) && sei.m_outOrderIdc == 2 && sei.m_outSubHeightC == 1 && sei.m_outSubWidthC == 1)),
          "nnpfc_matrix_coeffs shall not be equal to 8 unless one of the following conditions is true: nnpfc_out_tensor_chroma_bitdepth_minus8 is equal to nnpfc_out_tensor_luma_bitdepth_minus8 or "
          "nnpfc_out_tensor_chroma_bitdepth_minus8 is equal to nnpfc_out_tensor_luma_bitdepth_minus8 + 1, nnpfc_out_order_idc is equal to 2, outSubHeightC is equal to 1, and outSubWidthC is equal to 1");
      }
    }

    if (sei.m_sepColDescriptionFlag & (sei.m_outFormatIdc == 1))
    {
      sei_read_flag(pDecodedMessageOutputStream, val, "nnpfc_full_range_flag");
      sei.m_fullRangeFlag = val;
    }

    if (sei.m_outOrderIdc != 0)
    {
      sei_read_flag(pDecodedMessageOutputStream,val,"nnpfc_chroma_loc_info_present_flag");
      sei.m_chromaLocInfoPresentFlag = val;
    }
    else
    {
      sei.m_chromaLocInfoPresentFlag = 0;
    }

    CHECK((sei.m_outColourFormatIdc != ChromaFormat::CHROMA_420) && sei.m_chromaLocInfoPresentFlag, "When nnpfc_out_colour_format_idc is not equal to 1, the value of nnpfc_chroma_loc_info_present_flag shall be equal to 0");
    CHECK((sei.m_purpose & NNPC_PurposeType::COLOURIZATION) && sei.m_chromaLocInfoPresentFlag,"When colourizationFlag is equal to 0, the value of nnpfc_chroma_loc_info_present_flag shall be equal to 0");
    
    if(sei.m_chromaLocInfoPresentFlag)
    {
      sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_chroma_sample_loc_type_frame");
      sei.m_chromaSampleLocTypeFrame = static_cast<Chroma420LocType>(val);
      CHECK(sei.m_chromaSampleLocTypeFrame > Chroma420LocType::UNSPECIFIED, "The value of nnpfc_chroma_sample_loc_type_frame shall be in the range of 0 to 6, inclusive");
    }

    if((sei.m_purpose & NNPC_PurposeType::SPATIAL_EXTRAPOLATION) == 0)
    {
      sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_overlap");
      sei.m_overlap = val;

      sei_read_flag(pDecodedMessageOutputStream, val, "nnpfc_constant_patch_size_flag");
      sei.m_constantPatchSizeFlag = val;
    }
    else
    {
      sei.m_overlap = 0;
      sei.m_constantPatchSizeFlag = 1;
    }

    if (sei.m_constantPatchSizeFlag)
    {
      sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_patch_width_minus1");
      sei.m_patchWidthMinus1 = val;

      sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_patch_height_minus1");
      sei.m_patchHeightMinus1 = val;
    }
    else
    {
      sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_extended_patch_width_cd_delta_minus1");
      sei.m_extendedPatchWidthCdDeltaMinus1 = val;

      sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_extended_patch_height_cd_delta_minus1");
      sei.m_extendedPatchHeightCdDeltaMinus1 = val;
    }

    sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_padding_type");
    sei.m_paddingType = val;
  if((sei.m_paddingType >= 5) && (sei.m_paddingType <= 15))
  {
    std::cout<<"Reserved nnpfc_padding_type value, shall ignore the SEI message"<<std::endl;
    return;
  }
  CHECK(sei.m_paddingType > 15, "Values of nnpfc_padding_type greater than 15 shall not be present in bitstreams");

    if (sei.m_paddingType == NNPC_PaddingType::FIXED_PADDING)
    {
      if (sei.m_inpOrderIdc != 1)
      {
        sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_luma_padding_val");
        sei.m_lumaPadding = val;
        CHECK(sei.m_lumaPadding > ((1 << sps->getBitDepth(ChannelType::CHANNEL_TYPE_LUMA)) - 1), "The value of nnpfc_luma_padding_val shall be in the range of 0 to ( 1  <<  BitDepthY ) - 1");
      }
      if (sei.m_inpOrderIdc != 0)
      {
        sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_cb_padding_val");
        sei.m_cbPadding = val;
        CHECK(sei.m_cbPadding > ((1 << sps->getBitDepth(ChannelType::CHANNEL_TYPE_CHROMA)) - 1), "The value of nnpfc_cb_padding_val shall be in the range of 0 to ( 1  <<  BitDepthC ) - 1");

        sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_cr_padding_val");
        sei.m_crPadding = val;
        CHECK(sei.m_crPadding > ((1 << sps->getBitDepth(ChannelType::CHANNEL_TYPE_CHROMA)) - 1), "The value of nnpfc_cr_padding_val shall be in the range of 0 to ( 1  <<  BitDepthC ) - 1");
      }
    }

    sei_read_flag(pDecodedMessageOutputStream, val, "nnpfc_complexity_info_present_flag");
    sei.m_complexityInfoPresentFlag = val;

    if (sei.m_complexityInfoPresentFlag)
    {
        sei_read_code(pDecodedMessageOutputStream, 2, val, "nnpfc_parameter_type_idc");
        sei.m_parameterTypeIdc = val;
        if (sei.m_parameterTypeIdc != 2)
        {
          sei_read_code(pDecodedMessageOutputStream, 2, val, "nnpfc_log2_parameter_bit_length_minus3");
          sei.m_log2ParameterBitLengthMinus3 = val;
        }

        sei_read_code(pDecodedMessageOutputStream, 6, val, "nnpfc_num_parameters_idc");
        sei.m_numParametersIdc = val;

        sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_num_kmac_operations_idc");
        sei.m_numKmacOperationsIdc = val;

        sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_total_kilobyte_size");
        sei.m_totalKilobyteSize = val;
    }

    sei_read_uvlc(pDecodedMessageOutputStream, val, "nnpfc_metadata_extension_num_bits");
    UInt metadataExtensionNumBits = val;
    UInt numberExtensionBitsUsed = 0;
    if ( metadataExtensionNumBits > 0 ) 
    { 
      CHECK (val > 4096, "Values of nnpfc_metadata_extension_num_bits greater than 4096 shall not be present in bitstreams");
      if ( sei.m_purpose == 0 )
      { 
        sei_read_flag(pDecodedMessageOutputStream, val, "nnpfc_application_purpose_tag_uri_present_flag");
        sei.m_applicationPurposeTagUriPresentFlag = val;
        numberExtensionBitsUsed++;
        if ( sei.m_applicationPurposeTagUriPresentFlag )
        { 
          std::string val2;
          while (m_pcBitstream->getNumBitsUntilByteAligned() != 0)
          {
            sei_read_flag(pDecodedMessageOutputStream, val, "nnpfc_metadata_alignment_zero_bit");
            CHECK(val != 0, "nnpfc_metadata_alignment_zero_bit not equal to zero");
          }
          sei_read_string(pDecodedMessageOutputStream, val2, "nnpfc_application_purpose_tag_uri");
          sei.m_applicationPurposeTagUri = val2;
          numberExtensionBitsUsed += (static_cast<UInt>(sei.m_applicationPurposeTagUri.length() + 1) * 8);
        }
      }
      if ((sei.m_purpose & NNPC_PurposeType::SPATIAL_EXTRAPOLATION) != 0 || (sei.m_purpose & NNPC_PurposeType::RESOLUTION_UPSAMPLING) != 0)
      {
        sei_read_code(pDecodedMessageOutputStream, 2, val, "nnpfc_scan_type_idc");
      }
      sei_read_code(pDecodedMessageOutputStream, 2, val, "nnpfc_for_human_viewing_idc");
      sei.m_forHumanViewingIdc = val;
      numberExtensionBitsUsed += 2;
      sei_read_code(pDecodedMessageOutputStream, 2, val, "nnpfc_for_machine_analysis_idc");
      sei.m_forMachineAnalysisIdc = val;
      numberExtensionBitsUsed += 2;
      UInt numberExtensionBitRemaining = metadataExtensionNumBits - numberExtensionBitsUsed;
      for (uint32_t i = 0; i < numberExtensionBitRemaining; i++)
      {
        UInt val2;
        sei_read_code(pDecodedMessageOutputStream, 1, val2, "nnpfc_reserved_metadata_extension");
      } 
    }
  }

  if (sei.m_modeIdc == POST_FILTER_MODE::ISO_IEC_15938_17)
  {
    while (m_pcBitstream->getNumBitsUntilByteAligned() != 0)
    {
      sei_read_flag( pDecodedMessageOutputStream,   val,    "nnpfc_alignment_zero_bit");
      CHECK (val != 0, "nnpfc_alignment_zero_bit not equal to zero");
    }

    Int payloadBytesRemaining = getBitstream()->getNumBitsLeft() / 8;
    sei.m_payloadLength = payloadBytesRemaining;
    sei.m_payloadByte = new TChar[sei.m_payloadLength];
    Int code;

    std::string filename = "payloadByte" + std::to_string(sei.m_id) + ".nnr";

    std::ofstream outFile(filename.c_str(), std::ofstream::binary);

    for (Int i = 0; i < payloadBytesRemaining; i++)
    {
      sei_read_scode ( pDecodedMessageOutputStream, 8, code, "nnpfc_payload_byte[i]");
      sei.m_payloadByte[i] = (TChar)code;
      outFile.write((TChar*)&code, 1);
    }
    outFile.close();
  }
}
#endif

#if NNPFA_SEI_MESSAGE
Void SEIReader::xParseSEINNPostFilterActivation(SEINeuralNetworkPostFilterActivation &sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  UInt val;

  sei_read_uvlc( pDecodedMessageOutputStream, val, "nnpfa_target_id" );
  sei.m_targetId =val;
  CHECK(sei.m_targetId > MAX_NNPFA_ID, "The value of nnpfa_target_id shall be in the range of 0 to 2^32 - 2");
  sei_read_flag( pDecodedMessageOutputStream, val, "nnpfa_cancel_flag" );
  sei.m_cancelFlag = val;

  if(!sei.m_cancelFlag)
  {
    sei_read_flag( pDecodedMessageOutputStream, val, "nnpfa_persistence_flag" );
    sei.m_persistenceFlag = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "nnpfa_target_base_flag" );
    sei.m_targetBaseFlag = val;

    sei_read_flag( pDecodedMessageOutputStream, val, "nnpfa_no_prev_clvs_flag" );
    sei.m_noPrevCLVSFlag = val;
    if (sei.m_persistenceFlag)
    {
      sei_read_flag( pDecodedMessageOutputStream, val, "nnpfa_no_foll_clvs_flag" );
      sei.m_noFollCLVSFlag = val;
    }

    sei_read_uvlc( pDecodedMessageOutputStream, val, "nnpfa_num_output_entries" );
    UInt numOutputEntries = val;
    sei.m_outputFlag.resize(numOutputEntries);
    for (UInt i = 0; i < numOutputEntries; i++)
    {
      sei_read_flag( pDecodedMessageOutputStream, val, "nnpfa_output_flag" );
      sei.m_outputFlag[i] = val;
    }
    Int payloadBytesRemaining = getBitstream()->getNumBitsLeft() / 8;
    if (payloadBytesRemaining)
    {
      sei_read_flag( pDecodedMessageOutputStream, val, "nnpfa_prompt_update_flag" );
      sei.m_promptUpdateFlag = val;
      if (sei.m_promptUpdateFlag)
      {
        std::string valp;
        while (m_pcBitstream->getNumBitsUntilByteAligned() != 0)
        {
          sei_read_flag(pDecodedMessageOutputStream, val, "nnpfa_alignment_zero_bit");
          CHECK(val != 0, "nnpfa_alignment_zero_bit not equal to zero");
        }
        sei_read_string(pDecodedMessageOutputStream, valp, "nnpfa_prompt");
        sei.m_prompt = valp;
      }
      sei_read_flag( pDecodedMessageOutputStream, val, "nnpfa_seed_update_flag" );
      sei.m_seedUpdateFlag = val;
      if (sei.m_seedUpdateFlag)
      {
        sei_read_code(pDecodedMessageOutputStream, 16, val, "nnpfa_seed");
        sei.m_seed = val;
      }
    }
  }
}
#endif

#if JVET_AE0101_PHASE_INDICATION_SEI_MESSAGE
void SEIReader::xParseSEIPhaseIndication(SEIPhaseIndication& sei, uint32_t payloadSize, std::ostream* pDecodedMessageOutputStream)
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  uint32_t val;

  sei_read_code(pDecodedMessageOutputStream, 8, val, "hor_phase_num");
  sei.m_horPhaseNum = val;
  sei_read_code(pDecodedMessageOutputStream, 8, val, "hor_phase_den_minus1");
  sei.m_horPhaseDenMinus1 = val;
  sei_read_code(pDecodedMessageOutputStream, 8, val, "ver_phase_num");
  sei.m_verPhaseNum = val;
  sei_read_code(pDecodedMessageOutputStream, 8, val, "ver_phase_den_minus1");
  sei.m_verPhaseDenMinus1 = val;

  assert(sei.m_horPhaseNum <= sei.m_horPhaseDenMinus1 + 1);
  assert(sei.m_verPhaseNum <= sei.m_verPhaseDenMinus1 + 1);
}
#endif

Void SEIReader::xParseSEIEquirectangularProjection(SEIEquirectangularProjection& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  UInt val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_flag( pDecodedMessageOutputStream, val,       "erp_cancel_flag" );              sei.m_erpCancelFlag = val;
  if( !sei.m_erpCancelFlag )
  {
    sei_read_flag( pDecodedMessageOutputStream, val,      "erp_persistence_flag"    );     sei.m_erpPersistenceFlag   = val;
    sei_read_flag( pDecodedMessageOutputStream, val,      "erp_guard_band_flag"     );     sei.m_erpGuardBandFlag     = val;
    sei_read_code( pDecodedMessageOutputStream, 2, val,   "erp_reserved_zero_2bits" );
    if ( sei.m_erpGuardBandFlag == 1)
    {
      sei_read_code( pDecodedMessageOutputStream, 3, val,     "erp_guard_band_type"       );   sei.m_erpGuardBandType  = val;
      sei_read_code( pDecodedMessageOutputStream, 8, val,     "erp_left_guard_band_width" );   sei.m_erpLeftGuardBandWidth = val;
      sei_read_code( pDecodedMessageOutputStream, 8, val,     "erp_right_guard_band_width");   sei.m_erpRightGuardBandWidth = val;
    }
  }
}

Void SEIReader::xParseSEISphereRotation(SEISphereRotation& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  UInt val;
  Int  sval;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  sei_read_flag( pDecodedMessageOutputStream, val,       "sphere_rotation_cancel_flag" );              sei.m_sphereRotationCancelFlag = val;
  if( !sei.m_sphereRotationCancelFlag )
  {
    sei_read_flag ( pDecodedMessageOutputStream,      val,   "sphere_rotation_persistence_flag"    );     sei.m_sphereRotationPersistenceFlag = val;
    sei_read_code ( pDecodedMessageOutputStream, 6,   val,   "sphere_rotation_reserved_zero_6bits" );
    sei_read_scode( pDecodedMessageOutputStream, 32, sval,   "sphere_rotation_yaw"                 );     sei.m_sphereRotationYaw = sval;
    sei_read_scode( pDecodedMessageOutputStream, 32, sval,   "sphere_rotation_pitch"               );     sei.m_sphereRotationPitch = sval;
    sei_read_scode( pDecodedMessageOutputStream, 32, sval,   "sphere_rotation_roll"                );     sei.m_sphereRotationRoll = sval;
  }
}

Void SEIReader::xParseSEIOmniViewport(SEIOmniViewport& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  UInt code;
  Int  scode;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  sei_read_code( pDecodedMessageOutputStream, 10, code, "omni_viewport_id"          ); sei.m_omniViewportId         = code;
  sei_read_flag( pDecodedMessageOutputStream,     code, "omni_viewport_cancel_flag" ); sei.m_omniViewportCancelFlag = code;
  if (!sei.m_omniViewportCancelFlag)
  {
    UInt numRegions;
    sei_read_flag( pDecodedMessageOutputStream,    code,       "omni_viewport_persistence_flag" ); sei.m_omniViewportPersistenceFlag = code;    
    sei_read_code( pDecodedMessageOutputStream, 4, numRegions, "omni_viewport_cnt_minus1"       ); numRegions++;
    sei.m_omniViewportRegions.resize(numRegions);
    for(UInt region=0; region<numRegions; region++)
    {
      SEIOmniViewport::OmniViewport &viewport = sei.m_omniViewportRegions[region];
      sei_read_scode( pDecodedMessageOutputStream, 32, scode, "omni_viewport_azimuth_centre"   );   viewport.azimuthCentre = scode;
      sei_read_scode( pDecodedMessageOutputStream, 32, scode, "omni_viewport_elevation_centre" );   viewport.elevationCentre = scode;
      sei_read_scode( pDecodedMessageOutputStream, 32, scode, "omni_viewport_tilt_centre"      );   viewport.tiltCentre = code;
      sei_read_code( pDecodedMessageOutputStream,  32, code, "omni_viewport_hor_range"         );   viewport.horRange        = code;
      sei_read_code( pDecodedMessageOutputStream,  32, code, "omni_viewport_ver_range"         );   viewport.verRange        = code;
    }    
  }
  else
  {
    sei.m_omniViewportRegions.clear();
    sei.m_omniViewportPersistenceFlag=false;
  }
}

Void SEIReader::xParseSEIAnnotatedRegions(SEIAnnotatedRegions& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  UInt val;

  sei_read_flag(pDecodedMessageOutputStream, val, "ar_cancel_flag");                                   sei.m_hdr.m_cancelFlag = val;
  if (!sei.m_hdr.m_cancelFlag)
  {
    sei_read_flag(pDecodedMessageOutputStream, val, "ar_not_optimized_for_viewing_flag");              sei.m_hdr.m_notOptimizedForViewingFlag = val;
    sei_read_flag(pDecodedMessageOutputStream, val, "ar_true_motion_flag");                            sei.m_hdr.m_trueMotionFlag = val;
    sei_read_flag(pDecodedMessageOutputStream, val, "ar_occluded_object_flag");                        sei.m_hdr.m_occludedObjectFlag = val; // must be constant
    sei_read_flag(pDecodedMessageOutputStream, val, "ar_partial_object_flag_present_flag");            sei.m_hdr.m_partialObjectFlagPresentFlag = val; // must be constant
    sei_read_flag(pDecodedMessageOutputStream, val, "ar_object_label_present_flag");                   sei.m_hdr.m_objectLabelPresentFlag = val;
    sei_read_flag(pDecodedMessageOutputStream, val, "ar_object_confidence_info_present_flag");         sei.m_hdr.m_objectConfidenceInfoPresentFlag = val; // must be constant
    if (sei.m_hdr.m_objectConfidenceInfoPresentFlag)
    {
      sei_read_code(pDecodedMessageOutputStream, 4, val, "ar_object_confidence_length_minus_1"); sei.m_hdr.m_objectConfidenceLength = (val + 1); // must be constant
    }
    if (sei.m_hdr.m_objectLabelPresentFlag)
    {
      sei_read_flag(pDecodedMessageOutputStream, val, "ar_object_label_language_present_flag");      sei.m_hdr.m_objectLabelLanguagePresentFlag = val;
      if (sei.m_hdr.m_objectLabelLanguagePresentFlag)
      {
        // byte alignment
        while (m_pcBitstream->getNumBitsRead() % 8 != 0)
        {
          UInt code;
          sei_read_flag(pDecodedMessageOutputStream, code, "ar_bit_equal_to_zero");
        }
        sei.m_hdr.m_annotatedRegionsObjectLabelLang.clear();
        do
        {
          sei_read_code(pDecodedMessageOutputStream, 8, val, "ar_label_language");
          if (val)
          {
            assert(sei.m_hdr.m_annotatedRegionsObjectLabelLang.size()<256);
            sei.m_hdr.m_annotatedRegionsObjectLabelLang.push_back((char)val);
          }
        } while (val != '\0');
      }

      UInt numLabelUpdates;
      sei_read_uvlc(pDecodedMessageOutputStream, numLabelUpdates, "ar_num_label_updates");
      assert(numLabelUpdates<256);
      sei.m_annotatedLabels.clear();
      sei.m_annotatedLabels.resize(numLabelUpdates);
      for (auto it=sei.m_annotatedLabels.begin(); it!=sei.m_annotatedLabels.end(); it++)
      {
        SEIAnnotatedRegions::AnnotatedRegionLabel &ar = it->second;
        sei_read_uvlc(pDecodedMessageOutputStream, val, "ar_label_idx[]");             it->first = val;
        assert(val<256);
        sei_read_flag(pDecodedMessageOutputStream, val, "ar_label_cancel_flag");       ar.labelValid = !val;
        if (ar.labelValid)
        {
          ar.label.clear();
          // byte alignment
          while (m_pcBitstream->getNumBitsRead() % 8 != 0)
          {
            UInt code;
            sei_read_flag(pDecodedMessageOutputStream, code, "ar_bit_equal_to_zero");
          }
          do
          {
            sei_read_code(pDecodedMessageOutputStream, 8, val, "ar_label[]");
            if (val)
            {
              assert(ar.label.size()<256);
              ar.label.push_back((char)val);
            }
          } while (val != '\0');
        }
      }
    }

    UInt numObjUpdates;
    sei_read_uvlc(pDecodedMessageOutputStream, numObjUpdates, "ar_num_object_updates");
    assert(numObjUpdates<256);
    sei.m_annotatedRegions.clear();
    sei.m_annotatedRegions.resize(numObjUpdates);
    for (auto it=sei.m_annotatedRegions.begin(); it!=sei.m_annotatedRegions.end(); it++)
    {
      sei_read_uvlc(pDecodedMessageOutputStream, val, "ar_object_idx"); it->first=val;
      assert(val<256);
      SEIAnnotatedRegions::AnnotatedRegionObject &ar = it->second;
      sei_read_flag(pDecodedMessageOutputStream, val, "ar_object_cancel_flag");                           ar.objectCancelFlag = val;
      ar.objectLabelValid=false;
      ar.boundingBoxValid=false;

      if (!ar.objectCancelFlag)
      {
        if (sei.m_hdr.m_objectLabelPresentFlag)
        {
          sei_read_flag(pDecodedMessageOutputStream, val, "ar_object_label_update_flag");             ar.objectLabelValid = val;
          if (ar.objectLabelValid)
          {
            sei_read_uvlc(pDecodedMessageOutputStream, val, "ar_object_label_idx");                      ar.objLabelIdx = val;
            assert(val<256);
          }
        }
        sei_read_flag(pDecodedMessageOutputStream, val, "ar_bounding_box_update_flag");              ar.boundingBoxValid = val;
        if (ar.boundingBoxValid)
        {
          sei_read_flag(pDecodedMessageOutputStream, val, "ar_bounding_box_cancel_flag");             ar.boundingBoxCancelFlag = val;
          if (!ar.boundingBoxCancelFlag)
          {
            sei_read_code(pDecodedMessageOutputStream, 16, val, "ar_bounding_box_top");                      ar.boundingBoxTop = val;
            sei_read_code(pDecodedMessageOutputStream, 16, val, "ar_bounding_box_left");                     ar.boundingBoxLeft = val;
            sei_read_code(pDecodedMessageOutputStream, 16, val, "ar_bounding_box_width");                    ar.boundingBoxWidth = val;
            sei_read_code(pDecodedMessageOutputStream, 16, val, "ar_bounding_box_height");                   ar.boundingBoxHeight = val;
            if (sei.m_hdr.m_partialObjectFlagPresentFlag)
            {
              sei_read_flag(pDecodedMessageOutputStream, val, "ar_partial_object_flag");                ar.partialObjectFlag = val;
            }
            if (sei.m_hdr.m_objectConfidenceInfoPresentFlag)
            {
              sei_read_code(pDecodedMessageOutputStream, sei.m_hdr.m_objectConfidenceLength, val, "ar_object_confidence"); ar.objectConfidence = val;
            }
          }
        }
      }
    }
  }
}

Void SEIReader::xParseSEICubemapProjection(SEICubemapProjection& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  UInt val;

  sei_read_flag(pDecodedMessageOutputStream, val, "cmp_cancel_flag");                      sei.m_cmpCancelFlag = val;
  if (!sei.m_cmpCancelFlag)
  {
    sei_read_flag(pDecodedMessageOutputStream, val, "cmp_persistence_flag");                 sei.m_cmpPersistenceFlag = val;
  }
}



Void SEIReader::xParseSEIRegionWisePacking(SEIRegionWisePacking& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  UInt val;

  sei_read_flag( pDecodedMessageOutputStream,           val,      "rwp_cancel_flag" );                      sei.m_rwpCancelFlag = val;
  if (!sei.m_rwpCancelFlag)
  {
    sei_read_flag( pDecodedMessageOutputStream,           val,    "rwp_persistence_flag" );                 sei.m_rwpPersistenceFlag = val;
    sei_read_flag( pDecodedMessageOutputStream,           val,    "constituent_picture_matching_flag" );    sei.m_constituentPictureMatchingFlag = val;
    sei_read_code( pDecodedMessageOutputStream,       5,  val,    "rwp_reserved_zero_5bits" );
    sei_read_code( pDecodedMessageOutputStream,       8,  val,    "num_packed_regions" );                   sei.m_numPackedRegions = val;
    sei_read_code( pDecodedMessageOutputStream,       32, val,    "proj_picture_width" );                   sei.m_projPictureWidth = val;
    sei_read_code( pDecodedMessageOutputStream,       32, val,    "proj_picture_height" );                  sei.m_projPictureHeight = val;
    sei_read_code( pDecodedMessageOutputStream,       16, val,    "packed_picture_width" );                 sei.m_packedPictureWidth = val;
    sei_read_code( pDecodedMessageOutputStream,       16, val,    "packed_picture_height" );                sei.m_packedPictureHeight = val;
    
    sei.m_rwpTransformType.resize(sei.m_numPackedRegions);
    sei.m_rwpGuardBandFlag.resize(sei.m_numPackedRegions);
    sei.m_projRegionWidth.resize(sei.m_numPackedRegions);
    sei.m_projRegionHeight.resize(sei.m_numPackedRegions);
    sei.m_rwpProjRegionTop.resize(sei.m_numPackedRegions);
    sei.m_projRegionLeft.resize(sei.m_numPackedRegions);
    sei.m_packedRegionWidth.resize(sei.m_numPackedRegions);
    sei.m_packedRegionHeight.resize(sei.m_numPackedRegions);
    sei.m_packedRegionTop.resize(sei.m_numPackedRegions);
    sei.m_packedRegionLeft.resize(sei.m_numPackedRegions);
    sei.m_rwpLeftGuardBandWidth.resize(sei.m_numPackedRegions);
    sei.m_rwpRightGuardBandWidth.resize(sei.m_numPackedRegions);
    sei.m_rwpTopGuardBandHeight.resize(sei.m_numPackedRegions);
    sei.m_rwpBottomGuardBandHeight.resize(sei.m_numPackedRegions);
    sei.m_rwpGuardBandNotUsedForPredFlag.resize(sei.m_numPackedRegions);
    sei.m_rwpGuardBandType.resize(4*sei.m_numPackedRegions);

    for( Int i=0; i < sei.m_numPackedRegions; i++ )
    {
      sei_read_code( pDecodedMessageOutputStream,     4,  val,    "rwp_reserved_zero_4bits" );
      sei_read_code( pDecodedMessageOutputStream,     3,  val,    "rwp_tTransform_type" );                  sei.m_rwpTransformType[i] = val;
      sei_read_flag( pDecodedMessageOutputStream,         val,    "rwp_guard_band_flag" );                  sei.m_rwpGuardBandFlag[i] = val;
      sei_read_code( pDecodedMessageOutputStream,     32, val,    "proj_region_width" );                    sei.m_projRegionWidth[i] = val;
      sei_read_code( pDecodedMessageOutputStream,     32, val,    "proj_region_height" );                   sei.m_projRegionHeight[i] = val;
      sei_read_code( pDecodedMessageOutputStream,     32, val,    "rwp_proj_regionTop" );                   sei.m_rwpProjRegionTop[i] = val;
      sei_read_code( pDecodedMessageOutputStream,     32, val,    "proj_region_left" );                     sei.m_projRegionLeft[i] = val;
      sei_read_code( pDecodedMessageOutputStream,     16, val,    "packed_region_width" );                  sei.m_packedRegionWidth[i] = val;
      sei_read_code( pDecodedMessageOutputStream,     16, val,    "packed_region_height" );                 sei.m_packedRegionHeight[i] = val;
      sei_read_code( pDecodedMessageOutputStream,     16, val,    "packed_region_top" );                    sei.m_packedRegionTop[i] = val;
      sei_read_code( pDecodedMessageOutputStream,     16, val,    "packed_region_left" );                   sei.m_packedRegionLeft[i] = val;
      if( sei.m_rwpGuardBandFlag[i] )
      {
        sei_read_code( pDecodedMessageOutputStream,   8,  val,    "rwp_left_guard_band_width" );            sei.m_rwpLeftGuardBandWidth[i] = val;
        sei_read_code( pDecodedMessageOutputStream,   8,  val,    "rwp_right_guard_band_width" );           sei.m_rwpRightGuardBandWidth[i] = val;
        sei_read_code( pDecodedMessageOutputStream,   8,  val,    "rwp_top_guard_band_height" );            sei.m_rwpTopGuardBandHeight[i]  = val;
        sei_read_code( pDecodedMessageOutputStream,   8,  val,    "rwp_bottom_guard_band_height" );         sei. m_rwpBottomGuardBandHeight[i]  = val;
        sei_read_flag( pDecodedMessageOutputStream,       val,    "rwp_guard_band_not_used_forPred_flag" ); sei.m_rwpGuardBandNotUsedForPredFlag[i] = val;
        for( Int j=0; j < 4; j++ )
        {
          sei_read_code( pDecodedMessageOutputStream, 3,  val,     "rwp_guard_band_type" ); sei.m_rwpGuardBandType[i*4 + j] = val;
        }
        sei_read_code( pDecodedMessageOutputStream,   3,  val,    "rwp_guard_band_reserved_zero_3bits" );
      }
    }
  }
}

Void SEIReader::xParseSEIFisheyeVideoInfo(SEIFisheyeVideoInfo& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  UInt val;

  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  sei.values = TComSEIFisheyeVideoInfo();
  
  TComSEIFisheyeVideoInfo &info=sei.values;

  sei_read_flag(pDecodedMessageOutputStream, val, "fisheye_cancel_flag"); info.m_fisheyeCancelFlag = val;
  if (!info.m_fisheyeCancelFlag)
  {
    Int sval;

    sei_read_flag(pDecodedMessageOutputStream, val, "fisheye_persistence_flag");        info.m_fisheyePersistenceFlag = val;
    sei_read_code(pDecodedMessageOutputStream, 3, val, "fisheye_view_dimension_idc");   info.m_fisheyeViewDimensionIdc = val;
    sei_read_code(pDecodedMessageOutputStream, 3, val, "fisheye_reserved_zero_3bits");
    sei_read_code(pDecodedMessageOutputStream, 8, val, "fisheye_num_active_area_minus1");
    info.m_fisheyeActiveAreas.resize(val+1);

    for (std::size_t i = 0; i < info.m_fisheyeActiveAreas.size(); i++)
    {
      TComSEIFisheyeVideoInfo::ActiveAreaInfo &area=info.m_fisheyeActiveAreas[i];
      sei_read_code(pDecodedMessageOutputStream, 32, val, "fisheye_circular_region_centre_x[i]");  area.m_fisheyeCircularRegionCentreX = val;
      sei_read_code(pDecodedMessageOutputStream, 32, val, "fisheye_circular_region_centre_y[i]");  area.m_fisheyeCircularRegionCentreY = val;
      sei_read_code(pDecodedMessageOutputStream, 32, val, "fisheye_rect_region_top[i]");           area.m_fisheyeRectRegionTop = val;
      sei_read_code(pDecodedMessageOutputStream, 32, val, "fisheye_rect_region_left[i]");          area.m_fisheyeRectRegionLeft = val;
      sei_read_code(pDecodedMessageOutputStream, 32, val, "fisheye_rect_region_width[i]");         area.m_fisheyeRectRegionWidth = val;
      sei_read_code(pDecodedMessageOutputStream, 32, val, "fisheye_rect_region_Height[i]");        area.m_fisheyeRectRegionHeight = val;
      sei_read_code(pDecodedMessageOutputStream, 32, val, "fisheye_circular_region_radius[i]");    area.m_fisheyeCircularRegionRadius = val;
      sei_read_code(pDecodedMessageOutputStream, 32, val, "fisheye_scene_radius[i]");              area.m_fisheyeSceneRadius = val;

      sei_read_scode(pDecodedMessageOutputStream, 32, sval, "fisheye_camera_centre_azimuth[i]");   area.m_fisheyeCameraCentreAzimuth = sval;
      sei_read_scode(pDecodedMessageOutputStream, 32, sval, "fisheye_camera_centre_elevation[i]"); area.m_fisheyeCameraCentreElevation = sval;
      sei_read_scode(pDecodedMessageOutputStream, 32, sval, "fisheye_camera_centre_tilt[i]");      area.m_fisheyeCameraCentreTilt = sval;

      sei_read_code(pDecodedMessageOutputStream, 32, val, "fisheye_camera_centre_offset_x[i]");    area.m_fisheyeCameraCentreOffsetX = val;
      sei_read_code(pDecodedMessageOutputStream, 32, val, "fisheye_camera_centre_offset_y[i]");    area.m_fisheyeCameraCentreOffsetY = val;
      sei_read_code(pDecodedMessageOutputStream, 32, val, "fisheye_camera_centre_offset_z[i]");    area.m_fisheyeCameraCentreOffsetZ = val;
      sei_read_code(pDecodedMessageOutputStream, 32, val, "fisheye_field_of_view[i]");             area.m_fisheyeFieldOfView = val;
      sei_read_code(pDecodedMessageOutputStream, 16, val, "fisheye_num_polynomial_coeffs[i]");
      area.m_fisheyePolynomialCoeff.resize(val);

      for (std::size_t j = 0; j < area.m_fisheyePolynomialCoeff.size(); j++)
      {
        sei_read_scode(pDecodedMessageOutputStream, 32, sval, "fisheye_polynomial_coeff[i][j]");   area.m_fisheyePolynomialCoeff[j] = sval;
      }
    }
  }
}

Void SEIReader::xParseSEIColourRemappingInfo(SEIColourRemappingInfo& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  UInt  uiVal;
  Int   iVal;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_uvlc( pDecodedMessageOutputStream, uiVal, "colour_remap_id" );          sei.m_colourRemapId = uiVal;
  sei_read_flag( pDecodedMessageOutputStream, uiVal, "colour_remap_cancel_flag" ); sei.m_colourRemapCancelFlag = uiVal;
  if( !sei.m_colourRemapCancelFlag ) 
  {
    sei_read_flag( pDecodedMessageOutputStream, uiVal, "colour_remap_persistence_flag" );                sei.m_colourRemapPersistenceFlag = uiVal;
    sei_read_flag( pDecodedMessageOutputStream, uiVal, "colour_remap_video_signal_info_present_flag" );  sei.m_colourRemapVideoSignalInfoPresentFlag = uiVal;
    if ( sei.m_colourRemapVideoSignalInfoPresentFlag )
    {
      sei_read_flag( pDecodedMessageOutputStream, uiVal,    "colour_remap_full_range_flag" );            sei.m_colourRemapFullRangeFlag = uiVal;
      sei_read_code( pDecodedMessageOutputStream, 8, uiVal, "colour_remap_primaries" );                  sei.m_colourRemapPrimaries = uiVal;
      sei_read_code( pDecodedMessageOutputStream, 8, uiVal, "colour_remap_transfer_function" );          sei.m_colourRemapTransferFunction = uiVal;
      sei_read_code( pDecodedMessageOutputStream, 8, uiVal, "colour_remap_matrix_coefficients" );        sei.m_colourRemapMatrixCoefficients = uiVal;
    }
    sei_read_code( pDecodedMessageOutputStream, 8, uiVal, "colour_remap_input_bit_depth" );              sei.m_colourRemapInputBitDepth = uiVal;
    sei_read_code( pDecodedMessageOutputStream, 8, uiVal, "colour_remap_bit_depth" );                    sei.m_colourRemapBitDepth = uiVal;
  
    for( Int c=0 ; c<3 ; c++ )
    {
      sei_read_code( pDecodedMessageOutputStream, 8, uiVal, "pre_lut_num_val_minus1[c]" ); sei.m_preLutNumValMinus1[c] = (uiVal==0) ? 1 : uiVal;
      sei.m_preLut[c].resize(sei.m_preLutNumValMinus1[c]+1);
      if( uiVal> 0 )
      {
        for ( Int i=0 ; i<=sei.m_preLutNumValMinus1[c] ; i++ )
        {
          sei_read_code( pDecodedMessageOutputStream, (( sei.m_colourRemapInputBitDepth   + 7 ) >> 3 ) << 3, uiVal, "pre_lut_coded_value[c][i]" );  sei.m_preLut[c][i].codedValue  = uiVal;
          sei_read_code( pDecodedMessageOutputStream, (( sei.m_colourRemapBitDepth + 7 ) >> 3 ) << 3, uiVal, "pre_lut_target_value[c][i]" ); sei.m_preLut[c][i].targetValue = uiVal;
        }
      }
      else // pre_lut_num_val_minus1[c] == 0
      {
        sei.m_preLut[c][0].codedValue  = 0;
        sei.m_preLut[c][0].targetValue = 0;
        sei.m_preLut[c][1].codedValue  = (1 << sei.m_colourRemapInputBitDepth) - 1 ;
        sei.m_preLut[c][1].targetValue = (1 << sei.m_colourRemapBitDepth) - 1 ;
      }
    }

    sei_read_flag( pDecodedMessageOutputStream, uiVal,      "colour_remap_matrix_present_flag" ); sei.m_colourRemapMatrixPresentFlag = uiVal;
    if( sei.m_colourRemapMatrixPresentFlag )
    {
      sei_read_code( pDecodedMessageOutputStream, 4, uiVal, "log2_matrix_denom" ); sei.m_log2MatrixDenom = uiVal;
      for ( Int c=0 ; c<3 ; c++ )
      {
        for ( Int i=0 ; i<3 ; i++ )
        {
          sei_read_svlc( pDecodedMessageOutputStream, iVal, "colour_remap_coeffs[c][i]" ); sei.m_colourRemapCoeffs[c][i] = iVal;
        }
      }
    }
    else // setting default matrix (I3)
    {
      sei.m_log2MatrixDenom = 10;
      for ( Int c=0 ; c<3 ; c++ )
      {
        for ( Int i=0 ; i<3 ; i++ )
        {
          sei.m_colourRemapCoeffs[c][i] = (c==i) << sei.m_log2MatrixDenom;
        }
      }
    }
    for( Int c=0 ; c<3 ; c++ )
    {
      sei_read_code( pDecodedMessageOutputStream, 8, uiVal, "post_lut_num_val_minus1[c]" ); sei.m_postLutNumValMinus1[c] = (uiVal==0) ? 1 : uiVal;
      sei.m_postLut[c].resize(sei.m_postLutNumValMinus1[c]+1);
      if( uiVal > 0 )
      {
        for ( Int i=0 ; i<=sei.m_postLutNumValMinus1[c] ; i++ )
        {
          sei_read_code( pDecodedMessageOutputStream, (( sei.m_colourRemapBitDepth + 7 ) >> 3 ) << 3, uiVal, "post_lut_coded_value[c][i]" );  sei.m_postLut[c][i].codedValue = uiVal;
          sei_read_code( pDecodedMessageOutputStream, (( sei.m_colourRemapBitDepth + 7 ) >> 3 ) << 3, uiVal, "post_lut_target_value[c][i]" ); sei.m_postLut[c][i].targetValue = uiVal;
        }
      }
      else
      {
        sei.m_postLut[c][0].codedValue  = 0;
        sei.m_postLut[c][0].targetValue = 0;
        sei.m_postLut[c][1].targetValue = (1 << sei.m_colourRemapBitDepth) - 1;
        sei.m_postLut[c][1].codedValue  = (1 << sei.m_colourRemapBitDepth) - 1;
      }
    }
  }
}


Void SEIReader::xParseSEIDeinterlaceFieldIdentification( SEIDeinterlaceFieldIdentification& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream )
{
  UInt code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_flag(pDecodedMessageOutputStream, code, "deinterlaced_picture_source_parity_flag"); sei.m_deinterlacedPictureSourceParityFlag = code!=0;
}


Void SEIReader::xParseSEIContentLightLevelInfo( SEIContentLightLevelInfo& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream )
{
  UInt code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_code(pDecodedMessageOutputStream, 16, code, "max_content_light_level");     sei.m_maxContentLightLevel    = code;
  sei_read_code(pDecodedMessageOutputStream, 16, code, "max_pic_average_light_level"); sei.m_maxPicAverageLightLevel = code;
}


Void SEIReader::xParseSEIDependentRAPIndication( SEIDependentRAPIndication& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream )
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
}


Void SEIReader::xParseSEICodedRegionCompletion( SEICodedRegionCompletion& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream )
{
  UInt code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_uvlc(pDecodedMessageOutputStream, code, "next_segment_address"); sei.m_nextSegmentAddress= code;
  if (code)
  {
    sei_read_flag(pDecodedMessageOutputStream, code, "independent_slice_segment_flag"); sei.m_independentSliceSegmentFlag = code!=0;
  }
  else
  {
    sei.m_independentSliceSegmentFlag=false; // initialise to known value.
  }
}


Void SEIReader::xParseSEIAlternativeTransferCharacteristics(SEIAlternativeTransferCharacteristics& sei, UInt payloadSize, ostream* pDecodedMessageOutputStream)
{
  UInt code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_code(pDecodedMessageOutputStream, 8, code, "preferred_transfer_characteristics"); sei.m_preferredTransferCharacteristics = code;
}


Void SEIReader::xParseSEIAmbientViewingEnvironment( SEIAmbientViewingEnvironment& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream )
{
  UInt code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_code(pDecodedMessageOutputStream, 32, code, "ambient_illuminance"); sei.m_ambientIlluminance= code;
  sei_read_code(pDecodedMessageOutputStream, 16, code, "ambient_light_x");     sei.m_ambientLightX     = (UShort)code;
  sei_read_code(pDecodedMessageOutputStream, 16, code, "ambient_light_y");     sei.m_ambientLightY     = (UShort)code;
}
Void SEIReader::xParseSEIRegionalNesting( SEIRegionalNesting& sei, UInt payloadSize, const TComSPS *sps, std::ostream *pDecodedMessageOutputStream )
{
  UInt uiCode;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  UInt numRegions, numSEIs;
  
  sei_read_code(pDecodedMessageOutputStream, 16, uiCode, "regional_nesting_id");
  sei_read_code(pDecodedMessageOutputStream,  8, uiCode, "regional_nesting_num_rect_regions"); numRegions = uiCode;

  sei.clearRegions();
  for(UInt i = 0; i < numRegions; i++)
  {
    RNSEIWindow region;
    Int lOffset, rOffset, tOffset, bOffset, regionId;
    sei_read_code(pDecodedMessageOutputStream,  8, uiCode, "regional_nesting_rect_region_id");      regionId = uiCode;
    sei_read_code(pDecodedMessageOutputStream, 16, uiCode, "regional_nesting_rect_left_offset");    lOffset  = uiCode;
    sei_read_code(pDecodedMessageOutputStream, 16, uiCode, "regional_nesting_rect_right_offset");   rOffset  = uiCode;
    sei_read_code(pDecodedMessageOutputStream, 16, uiCode, "regional_nesting_rect_top_offset");     tOffset  = uiCode;
    sei_read_code(pDecodedMessageOutputStream, 16, uiCode, "regional_nesting_rect_bottom_offset");  bOffset  = uiCode;
    region.setRegionId(regionId);
    region.setWindow(lOffset, rOffset, tOffset, bOffset);
    
    sei.addRegion(&region);
  }
  sei_read_code(pDecodedMessageOutputStream,  8, uiCode, "num_sei_messages_in_regional_nesting_minus1");      numSEIs = uiCode + 1;
  for(UInt i = 0; i < numSEIs; i++)
  {
    SEIRegionalNesting::SEIListOfIndices seiWithRegionIndices;
    UInt numRegionsForSEI;
    sei_read_code(pDecodedMessageOutputStream,  8, uiCode, "num_regions_for_sei_message[i]"); numRegionsForSEI = uiCode;
    for(UInt j = 0; j < numRegionsForSEI; j++) 
    {
      sei_read_code(pDecodedMessageOutputStream,  8, uiCode, "regional_nesting_sei_region_idx[i][j]");
      seiWithRegionIndices.m_listOfIndices.push_back(uiCode);
    }

    SEIMessages seiReg;
    xReadSEImessage(seiReg, NAL_UNIT_PREFIX_SEI, sps, pDecodedMessageOutputStream, SEI::regional_nesting_sei_messages, std::string(""));
    seiWithRegionIndices.m_seiMessage = seiReg.front();
    sei.addRegionalSEI( seiWithRegionIndices ) ;
  }
}

#if JCTVC_AD0021_SEI_MANIFEST
Void SEIReader::xParseSEISEIManifest(SEIManifest& sei, UInt payloadSize, std::ostream* pDecodedMessageOutputStream)
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  UInt val;
  sei_read_code(pDecodedMessageOutputStream, 16, val, "manifest_num_sei_msg_types");          sei.m_manifestNumSeiMsgTypes = val;

  if (sei.m_manifestNumSeiMsgTypes > 0)
  {
    sei.m_manifestSeiPayloadType.resize(sei.m_manifestNumSeiMsgTypes);
    sei.m_manifestSeiDescription.resize(sei.m_manifestNumSeiMsgTypes);
    for (Int i = 0; i < sei.m_manifestNumSeiMsgTypes; i++)
    {
      sei_read_code(pDecodedMessageOutputStream, 16, val, "manifest_sei_payload_types");           sei.m_manifestSeiPayloadType[i] = val;
      sei_read_code(pDecodedMessageOutputStream, 8, val, "manifest_sei_description");             sei.m_manifestSeiDescription[i] = val;
    }
  }
}
#endif

#if JCTVC_AD0021_SEI_PREFIX_INDICATION
Void SEIReader::xParseSEISEIPrefixIndication(SEIPrefixIndication& sei, UInt payloadSize, std::ostream* pDecodedMessageOutputStream)
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  UInt val;
  UInt bitsRead = 0;
  sei_read_code(pDecodedMessageOutputStream, 16, val, "prefix_sei_payload_type");                sei.m_prefixSeiPayloadType = val;
  sei_read_code(pDecodedMessageOutputStream, 8, val, "num_sei_prefix_indications_minus1");      sei.m_numSeiPrefixIndicationsMinus1 = val;
  if (sei.m_numSeiPrefixIndicationsMinus1 >= 0)
  {
    sei.m_numBitsInPrefixIndicationMinus1.resize(sei.m_numSeiPrefixIndicationsMinus1 + 1);
    sei.m_seiPrefixDataBit.resize(sei.m_numSeiPrefixIndicationsMinus1 + 1);
    for (Int i = 0; i <= sei.m_numSeiPrefixIndicationsMinus1; i++)
    {
      sei_read_code(pDecodedMessageOutputStream, 16, val, "num_bits_in_prefix_indication_minus1");   sei.m_numBitsInPrefixIndicationMinus1[i] = val;
      sei.m_seiPrefixDataBit[i].resize(sei.m_numBitsInPrefixIndicationMinus1[i] + 1);
      for (Int j = 0; j <= sei.m_numBitsInPrefixIndicationMinus1[i]; j++)
      {
        sei_read_code(pDecodedMessageOutputStream, 1, val, "sei_prefix_data_bit");   sei.m_seiPrefixDataBit[i][j] = val;
        bitsRead += 1;
      }
      while (bitsRead % 8 != 0)
      {
        sei_read_code(pDecodedMessageOutputStream, 1, val, "byte_alignment_bit_equal_to_one");
        bitsRead += 1;
      }
    }
  }
}
#endif

#if  JVET_AL0062_AI_USAGE_RESTRICTIONS_SEI
void SEIReader::xParseSEIAIUsageRestrictions(SEIAIUsageRestrictions& sei, uint32_t payloadSize, std::ostream* pDecodedMessageOutputStream)
{
  uint32_t val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  sei_read_flag(pDecodedMessageOutputStream, val, "aur_cancel_flag");
  sei.m_cancelFlag = val;
  if (!sei.m_cancelFlag)
  {
    sei_read_flag(pDecodedMessageOutputStream, val, "aur_persistence_flag");
    sei.m_persistenceFlag = val;
    sei_read_uvlc(pDecodedMessageOutputStream, val, "aur_num_restrictions_minus1");
    sei.m_numRestrictionsMinus1 = val;
    sei.m_restrictions.resize(sei.m_numRestrictionsMinus1 + 1);
    sei.m_contextPresentFlag.resize(sei.m_numRestrictionsMinus1 + 1);
    sei.m_context.resize(sei.m_numRestrictionsMinus1 + 1);
    for (uint32_t i = 0; i <= sei.m_numRestrictionsMinus1; i++)
    {
      sei_read_uvlc(pDecodedMessageOutputStream, val, "aur_restriction");
      sei.m_restrictions[i] = val;
      sei_read_flag(pDecodedMessageOutputStream, val, "aur_context_present_flag");
      sei.m_contextPresentFlag[i] = val;
      if (sei.m_contextPresentFlag[i])
      {
        sei_read_uvlc(pDecodedMessageOutputStream, val, "aur_context");
        sei.m_context[i] = val;
      }
    }
  }
}
#endif 

#if JVET_AK0194_DSC_SEI_DECODER_SYNTAX
void SEIReader::xParseSEIDigitallySignedContentInitialization(SEIDigitallySignedContentInitialization &sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  unsigned int val;
  sei_read_code(pDecodedMessageOutputStream, 8, val, "dsci_id");
  sei.dsciId = val;
  sei_read_code(pDecodedMessageOutputStream, 8, val, "dsci_hash_method_type");
  sei.dsciHashMethodType = val;
  sei_read_uvlc(pDecodedMessageOutputStream, val, "dsci_key_retrieval_mode_idc");
  sei.dsciKeyRetrievalModeIdc = val;
  if (sei.dsciKeyRetrievalModeIdc == 1)
  {
    sei_read_flag(pDecodedMessageOutputStream, val, "dsci_use_key_register_idx_flag");
    sei.dsciUseKeyRegisterIdxFlag = (val!=0);
    if( sei.dsciUseKeyRegisterIdxFlag )
    {
      sei_read_uvlc(pDecodedMessageOutputStream, val, "dsci_key_register_idx");
      sei.dsciKeyRegisterIdx = val;
    }
  }
  sei_read_flag(pDecodedMessageOutputStream, val, "dsci_content_uuid_present_flag");
  sei.dsciContentUuidPresentFlag = (val!=0);
  if (sei.dsciContentUuidPresentFlag)
  {
    for (int i=0; i<16; i++)
    {
      sei_read_code(pDecodedMessageOutputStream, 8, val, "dsci_content_uuid");
      sei.dsciContentUuid[i] = val;
    }
  }
  sei_read_uvlc(pDecodedMessageOutputStream, val, "dsci_num_verification_substreams_minus1");
  sei.dsciNumVerificationSubstreams = val + 1;
  sei.dsciRefSubstreamFlag.resize(sei.dsciNumVerificationSubstreams);
  for (int i = 1; i < sei.dsciNumVerificationSubstreams; i++)
  {
    sei.dsciRefSubstreamFlag[i].resize(i);
    for (int j = 0; j < i; j++)
    {
      sei_read_flag(pDecodedMessageOutputStream, val, "dsci_ref_substream_flag");
      sei.dsciRefSubstreamFlag[i][j] = (val!=0);
    }
  }
  sei_read_flag(pDecodedMessageOutputStream, val, "dsci_vss_implicit_association_mode_flag");
  sei.dsciVSSImplicitAssociationModeFlag = (val!=0);
  sei_read_flag(pDecodedMessageOutputStream, val, "dsci_signed_content_start_flag");
  sei.dsciSignedContentStartFlag = (val!=0);
  sei_read_flag(pDecodedMessageOutputStream, val, "dsci_sei_signing_flag");
  sei.dsciSEISigningFlag = (val!=0);
  while (!isByteAligned())
  {
    sei_read_flag(pDecodedMessageOutputStream, val, "dsci_alignment_zero_bit");
    CHECK(val!=0, "dsci_alignment_zero_bit not equal to zero")
  }
  sei_read_string(pDecodedMessageOutputStream, sei.dsciKeySourceUri, "twci_key_source_uri");
}

void SEIReader::xParseSEIDigitallySignedContentSelection(SEIDigitallySignedContentSelection &sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  unsigned int val;
  sei_read_code(pDecodedMessageOutputStream, 8, val, "dscs_id");
  sei.dscsId = val;
  sei_read_code(pDecodedMessageOutputStream, 8, val, "dscs_verification_substream_id");
  sei.dscsVerificationSubstreamId = val;
}

void SEIReader::xParseSEIDigitallySignedContentVerification(SEIDigitallySignedContentVerification &sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  unsigned int val;
  sei_read_code(pDecodedMessageOutputStream, 8, val, "dscv_id");
  sei.dscvId = val;
  sei_read_code(pDecodedMessageOutputStream, 8, val, "dscv_verification_substream_id");
  sei.dscvVerificationSubstreamId = val;
  sei_read_code(pDecodedMessageOutputStream, 24, val, "dscv_signature_length_in_octets_minus1");
  sei.dscvSignatureLengthInOctets = val + 1;
  sei.dscvSignature.resize(sei.dscvSignatureLengthInOctets);
  for (int i=0; i< sei.dscvSignature.size(); i++)
  {
    sei_read_code(pDecodedMessageOutputStream, 8, val, "dscv_signature");
    sei.dscvSignature[i] = val;
  }
  sei_read_flag(pDecodedMessageOutputStream, val, "dscv_signed_content_end_flag");
  sei.dscvSignedContentEndFlag = (val!=0);
}
#endif

#if JVET_AK0107_MODALITY_INFORMATION
Void SEIReader::xParseSEIModalityInfo(SEIModalityInfo& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  sei_read_flag( pDecodedMessageOutputStream,           code,    "modality_info_cancel_flag" );                      sei.m_miCancelFlag = code;
  if (!sei.m_miCancelFlag)
   {
    sei_read_flag( pDecodedMessageOutputStream,         code,    "modality_info_persistence_flag" );                 sei.m_miPersistenceFlag = code;
    sei_read_code( pDecodedMessageOutputStream,     5,  code,    "modality_type" );                                  sei.m_miModalityType = code;
    sei_read_flag( pDecodedMessageOutputStream,         code,    "spectrum_range_present_flag" );                    sei.m_miSpectrumRangePresentFlag = code;
    if (sei.m_miSpectrumRangePresentFlag)
    {
      sei_read_code( pDecodedMessageOutputStream,  11,  code,    "min_wavelength_mantissa " );                       sei.m_miMinWavelengthMantissa = code;
      sei_read_code( pDecodedMessageOutputStream,  5,   code,    "min_wavelength_exponent_plus15" );                 sei.m_miMinWavelengthExponentPlus15 = code;
      sei_read_code( pDecodedMessageOutputStream,  11,  code,    "max_wavelength_mantissa " );                       sei.m_miMaxWavelengthMantissa = code;
      sei_read_code( pDecodedMessageOutputStream,  5,   code,    "max_wavelength_exponent_plus15" );                 sei.m_miMaxWavelengthExponentPlus15 = code;
    }
    sei_read_uvlc(pDecodedMessageOutputStream,          code,    "modality_type_extension_bits");   // modality_type_extension_bits shall be equal to 0 in the current edition
    if (code > 2048){
      printf ("Warning: Values of modality_type_extension_bits greater than 2048 shall not be present in bitstreams.");
    }
    for (UInt i = 0; i < code; i++)
    {
      UInt code2;
      sei_read_code(pDecodedMessageOutputStream, 1, code2, "reserved_modality_type_extension");   // Decoders shall ignore the presence and value of reserved_modality_type_extension                                                   

    }
  }
}
#endif
#if JVET_AK2006_SPTI_SEI_MESSAGE
void SEIReader::xParseSEISourcePictureTimingInfo(SEISourcePictureTimingInfo &sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream) 
{
  uint32_t val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  sei_read_flag(pDecodedMessageOutputStream, val, "spti_cancel_flag");
  sei.m_sptiCancelFlag = val;
  if (!sei.m_sptiCancelFlag) 
  {
    sei_read_flag(pDecodedMessageOutputStream, val, "spti_persistence_flag");
    sei.m_sptiPersistenceFlag = val;
    sei_read_flag(pDecodedMessageOutputStream, val, "spti_source_timing_equals_output_timing_flag");
    sei.m_sptiSourceTimingEqualsOutputTimingFlag = val;
    if (!sei.m_sptiSourceTimingEqualsOutputTimingFlag) 
    {
      sei_read_flag(pDecodedMessageOutputStream, val, "spti_source_type_present_flag");
      sei.m_sptiSourceTypePresentFlag = val;
      if (sei.m_sptiSourceTypePresentFlag) 
      {
        sei_read_code(pDecodedMessageOutputStream, 16, val, "spti_source_type");
        sei.m_sptiSourceType = val;
        assert(sei.m_sptiSourceType >= 0 && sei.m_sptiSourceType <= 127);
      }
      sei_read_code(pDecodedMessageOutputStream, 32, val, "spti_time_scale");
      sei.m_sptiTimeScale = val;
      assert(sei.m_sptiTimeScale != 0);
      sei_read_code(pDecodedMessageOutputStream, 32, val, "spti_num_units_in_elemental_interval");
      sei.m_sptiNumUnitsInElementalInterval = val;
      assert(sei.m_sptiNumUnitsInElementalInterval != 0);

      sei_read_flag(pDecodedMessageOutputStream, val, "spti_direction_flag");
      sei.m_sptiDirectionFlag = val;
      if (sei.m_sptiPersistenceFlag) 
      {
        sei_read_code(pDecodedMessageOutputStream, 3, val, "spti_max_sublayers_minus_1");
        sei.m_sptiMaxSublayersMinus1 = val;
      }
      int sptiMinTemporalSublayer = (sei.m_sptiPersistenceFlag ? 0 : sei.m_sptiMaxSublayersMinus1);

      for (int i = sptiMinTemporalSublayer; i <= sei.m_sptiMaxSublayersMinus1; i++) 
      {
        sei_read_uvlc(pDecodedMessageOutputStream, val, "spti_sublayer_interval_scale_factor");
        assert(val >= 0 && val <= UINT_MAX - 1);
        sei.m_sptiSublayerIntervalScaleFactor[i] = val;
        sei_read_flag(pDecodedMessageOutputStream, val, "spti_sublayer_synthesized_picture_flag");
        sei.m_sptiSublayerSynthesizedPictureFlag[i] = val;
      }
    }
  }
}
#endif
#if JVET_AJ0207_GFV
void SEIReader::xParseSEIGenerativeFaceVideo(SEIGenerativeFaceVideo & sei, uint32_t payloadSize, std::ostream* pDecodedMessageOutputStream)
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  uint32_t   val;
  bool       valueSignFlag;
  uint32_t   id;
  uint32_t   cnt;
  uint32_t   drivePicFusionFlag;
  uint32_t   lowConfidenceFaceParameterFlag;
  bool       coordinatePresentFlag;
  uint32_t   coordinateQuantizationFactor;
  bool       coordinatePredFlag;
  bool       m3DCoordinateFlag;
  uint32_t   coordinatePointNum;
  bool       matrixPresentFlag;
  uint32_t   matrixElementPrecisionFactor;
  bool       matrixPredFlag;
  uint32_t   numMatrixType;
  sei.m_chromaKeyValuePresentFlag.resize(3);
  sei.m_chromaKeyValue.resize(3);
  sei.m_chromaKeyThrPresentFlag.resize(2);
  sei.m_chromaKeyThrValue.resize(2);
  std::vector<double>      coordinateX;
  std::vector<double>      coordinateY;
  std::vector<double>      coordinateZ;
  uint32_t                 coordinateZMaxValue;
  std::vector<uint32_t>    matrixTypeIdx;
  std::vector<uint32_t>    matrix3DSpaceFlag;
  std::vector<uint32_t>    numMatrices;
  std::vector<uint32_t>    matrixWidth;
  std::vector<uint32_t>    matrixHeight;
  std::vector<std::vector<std::vector<std::vector<double>>>>   matrixElement;
  std::vector<uint32_t>    numMatricestoNumKpsFlag;
  std::vector<uint32_t>    numMatricesInfo;

  sei_read_uvlc(pDecodedMessageOutputStream, val, "gfv_id");
  id = val;
  sei_read_uvlc(pDecodedMessageOutputStream, val, "gfv_cnt");
  cnt = val;
  if (cnt == 0)
  {
    sei_read_flag(pDecodedMessageOutputStream, val, "gfv_base_picture_flag");
    sei.m_basePicFlag = val;
  }
  else
  {
    sei.m_basePicFlag = 0;
  }
  if (sei.m_basePicFlag)
  {
    sei_read_flag(pDecodedMessageOutputStream, val, "gfv_nn_present_flag");
    sei.m_nnPresentFlag = val;
    if (sei.m_nnPresentFlag)
    {
      sei_read_uvlc(pDecodedMessageOutputStream, val, "gfv_mode_idc");
      sei.m_nnModeIdc = val;
      if (sei.m_nnModeIdc == 1)
      {
        std::string val2;
        while (m_pcBitstream->getNumBitsRead() % 8 != 0)
        {
          sei_read_flag(pDecodedMessageOutputStream, val, "gfv_nn_alignment_zero_bit_a");
          assert(val == 0);
        }
        sei_read_string(pDecodedMessageOutputStream, val2, "gfv_uri_tag");
        sei.m_nnTagURI = val2;
        val2 = "";
        sei_read_string(pDecodedMessageOutputStream, val2, "gfv_uri");
        sei.m_nnURI = val2;
      }
    }
    sei_read_flag(pDecodedMessageOutputStream, val, "gfv_chroma_key_info_present_flag");
    sei.m_chromaKeyInfoPresentFlag = val;
    if (sei.m_chromaKeyInfoPresentFlag)
    {
      for (uint32_t chromac = 0; chromac < 3; chromac++)
      {
        sei_read_flag(pDecodedMessageOutputStream, val, "gfv_chroma_key_value_present_flag[c]");
        sei.m_chromaKeyValuePresentFlag[chromac] = val;
        if (sei.m_chromaKeyValuePresentFlag[chromac])
        {
          sei_read_code(pDecodedMessageOutputStream, 8, val, "gfv_chroma_key_value[chromac]");
          sei.m_chromaKeyValue[chromac] = val;
        }
      }
      bool chromaKeyDefaultValueFlag = (!(sei.m_chromaKeyValuePresentFlag[0] || sei.m_chromaKeyValuePresentFlag[1] || sei.m_chromaKeyValuePresentFlag[2]));
      std::vector<uint32_t> gfvChromaKeyValue;
      if (chromaKeyDefaultValueFlag == 1)
      {
        gfvChromaKeyValue.push_back(50);
        gfvChromaKeyValue.push_back(220);
        gfvChromaKeyValue.push_back(100);
      }
      else
      {
        for (uint32_t chromac = 0; chromac < 3; chromac++)
        {
          if (sei.m_chromaKeyValuePresentFlag[chromac])
          {
            gfvChromaKeyValue.push_back(sei.m_chromaKeyValue[chromac]);
          }
        }
      }
      if (pDecodedMessageOutputStream)
      {
        (*pDecodedMessageOutputStream) << "  " << std::setw(55) << "gfv_chroma_key_value" << ": ";
        for (uint32_t chromac = 0; chromac < gfvChromaKeyValue.size(); chromac++)
        {
          (*pDecodedMessageOutputStream) << (gfvChromaKeyValue[chromac]) << " ";
        }
        (*pDecodedMessageOutputStream) << "\n";
      }
      for (uint32_t chromai = 0; chromai < 2; chromai++)
      {
        sei_read_flag(pDecodedMessageOutputStream, val, "gfv_chroma_key_thr_present_flag[i]");
        sei.m_chromaKeyThrPresentFlag[chromai] = val;
        if (sei.m_chromaKeyThrPresentFlag[chromai])
        {
          sei_read_uvlc(pDecodedMessageOutputStream, val, "gfv_chroma_key_thr_value[i]");
          sei.m_chromaKeyThrValue[chromai] = val;
        }
        else
        {
          sei.m_chromaKeyThrValue[chromai] = (chromai == 0) ? 48 : 75;
        }
      }
      if (pDecodedMessageOutputStream)
      {
        (*pDecodedMessageOutputStream) << "  " << std::setw(55) << "gfv_chroma_key_thr_value" << ": ";
        for (uint32_t chromai = 0; chromai < sei.m_chromaKeyThrValue.size(); chromai++)
        {
          (*pDecodedMessageOutputStream) << (sei.m_chromaKeyThrValue[chromai]) << " ";
        }
        (*pDecodedMessageOutputStream) << "\n";
      }
    }
  }
  else
  {
    sei_read_flag(pDecodedMessageOutputStream, drivePicFusionFlag, "gfv_drive_picture_fusion_flag");
  }
  sei_read_flag(pDecodedMessageOutputStream, lowConfidenceFaceParameterFlag, "gfv_low_confidence_face_parameter_flag");
  sei_read_flag(pDecodedMessageOutputStream, val, "gfv_coordinate_present_flag");
  coordinatePresentFlag = val;
  if (coordinatePresentFlag)
  {
    sei_read_flag(pDecodedMessageOutputStream, val, "gfv_kps_pred_flag");
    coordinatePredFlag = val;
    if (sei.m_basePicFlag || !coordinatePredFlag)
    {
      sei_read_uvlc(pDecodedMessageOutputStream, val, "gfv_coordinate_precision_factor_minus1");
      assert(val <= 31 && val >= 0);
      coordinateQuantizationFactor = val + 1;
      sei_read_uvlc(pDecodedMessageOutputStream, val, "gfv_num_kps_minus1");
      coordinatePointNum = val + 1;
      sei_read_flag(pDecodedMessageOutputStream, val, "gfv_coordinate_z_present_flag");
      m3DCoordinateFlag = val;
      if (sei.m_basePicFlag)
      {
        baseCoordinateQuantizationFactor = coordinateQuantizationFactor;
        basdCoordinatePointNum = coordinatePointNum;
        base3DCoordinateFlag = m3DCoordinateFlag;
      }
      if (m3DCoordinateFlag == 1)
      {
        sei_read_uvlc(pDecodedMessageOutputStream, val, "gfv_coordinate_z_max_value_minus1");
        coordinateZMaxValue = val + 1;
        if (sei.m_basePicFlag)
        {
          baseCoordinateZMaxValue = coordinateZMaxValue;
        }
      }
    }
    else
    {
      coordinateQuantizationFactor = baseCoordinateQuantizationFactor;
      coordinatePointNum = basdCoordinatePointNum;
      m3DCoordinateFlag = base3DCoordinateFlag;
      if (m3DCoordinateFlag == 1)
      {
        coordinateZMaxValue = baseCoordinateZMaxValue;
      }
    }
    if (sei.m_basePicFlag)
    {
      baseCoordinateX.resize(0);
      baseCoordinateY.resize(0);
      baseCoordinateZ.resize(0);
      prevCoordinateX.resize(0);
      prevCoordinateY.resize(0);
      prevCoordinateZ.resize(0);

    }
    for (uint32_t i = 0; i < coordinatePointNum; i++)
    {
      if (!coordinatePredFlag)
      {
        sei_read_uvlc(pDecodedMessageOutputStream, val, "gfv_coordinate_x_abs[ i ]");
        int curCoordinateXInt = val;
        double coordinateXTensorAbs = ((double)curCoordinateXInt) / (1 << coordinateQuantizationFactor);
        valueSignFlag = 0;
        if (curCoordinateXInt)
        {
          sei_read_flag(pDecodedMessageOutputStream, val, "gfv_coordinate_x_sign_flag");
          valueSignFlag = val;
        }
        double coordinateXTensor = valueSignFlag ? -coordinateXTensorAbs : coordinateXTensorAbs;
        coordinateX.push_back(coordinateXTensor);
        if (sei.m_basePicFlag)
        {
          baseCoordinateX.push_back(coordinateXTensor);
          prevCoordinateX.push_back(coordinateXTensor);
        }
        else
        {
          prevCoordinateX[i] = coordinateXTensor;
        }
        sei_read_uvlc(pDecodedMessageOutputStream, val, "gfv_coordinate_y_abs[ i ]");
        int curCoordinateYInt = val;
        double coordinateYTensorAbs = ((double)curCoordinateYInt) / (1 << coordinateQuantizationFactor);
        valueSignFlag = 0;
        if (curCoordinateYInt)
        {
          sei_read_flag(pDecodedMessageOutputStream, val, "gfv_coordinate_y_sign_flag");
          valueSignFlag = val;
        }
        double coordinateYTensor = valueSignFlag ? -coordinateYTensorAbs : coordinateYTensorAbs;
        coordinateY.push_back(coordinateYTensor);
        if (sei.m_basePicFlag)
        {
          baseCoordinateY.push_back(coordinateYTensor);
          prevCoordinateY.push_back(coordinateYTensor);
        }
        else
        {
          prevCoordinateY[i] = coordinateYTensor;
        }
        if (m3DCoordinateFlag == 1)
        {
          sei_read_uvlc(pDecodedMessageOutputStream, val, "gfv_coordinate_z_abs[ i ]");
          int curCoordinateZInt = val;
          double coordinateZTensorAbs = ((double)curCoordinateZInt / (1 << coordinateQuantizationFactor));
          valueSignFlag = 0;
          if (curCoordinateZInt)
          {
            sei_read_flag(pDecodedMessageOutputStream, val, "gfv_coordinate_z_sign_flag");
            valueSignFlag = val;
          }
          double coordinateZTensor = valueSignFlag ? -coordinateZTensorAbs : coordinateZTensorAbs;
          coordinateZ.push_back(coordinateZTensor);
          if (sei.m_basePicFlag)
          {
            baseCoordinateZ.push_back(coordinateZTensor);
            prevCoordinateZ.push_back(coordinateZTensor);
          }
          else
          {
            prevCoordinateZ[i] = coordinateZTensor;
          }
        }
      }
      else
      {
        sei_read_uvlc(pDecodedMessageOutputStream, val, "gfv_coordinate_dx_abs[ i ]");
        int curCoordinateXInt = val;
        double coordinateXTensorAbs = ((double)curCoordinateXInt) / (1 << coordinateQuantizationFactor);
        valueSignFlag = 0;
        if (curCoordinateXInt)
        {
          sei_read_flag(pDecodedMessageOutputStream, val, "gfv_coordinate_dx_sign_flag");
          valueSignFlag = val;
        }
        double  coordinateXTensor = (valueSignFlag ? -coordinateXTensorAbs : coordinateXTensorAbs) + (sei.m_basePicFlag ? (i == 0 ? 0 : prevCoordinateX[i - 1]) : (cnt == 0 ? baseCoordinateX[i] : prevCoordinateX[i]));
        coordinateX.push_back(coordinateXTensor);
        if (sei.m_basePicFlag)
        {
          baseCoordinateX.push_back(coordinateXTensor);
          prevCoordinateX.push_back(coordinateXTensor);
        }
        else
        {
          prevCoordinateX[i] = coordinateXTensor;
        }
        sei_read_uvlc(pDecodedMessageOutputStream, val, "gfv_coordinate_dy_abs[ i ]");
        int curCoordinateYInt = val;
        double coordinateYTensorAbs = ((double)curCoordinateYInt) / (1 << coordinateQuantizationFactor);
        valueSignFlag = 0;
        if (curCoordinateYInt)
        {
          sei_read_flag(pDecodedMessageOutputStream, val, "gfv_coordinate_dy_sign_flag");
          valueSignFlag = val;
        }
        double  coordinateYTensor = (valueSignFlag ? -coordinateYTensorAbs : coordinateYTensorAbs) + (sei.m_basePicFlag ? (i == 0 ? 0 : prevCoordinateY[i - 1]) : (cnt == 0 ? baseCoordinateY[i] : prevCoordinateY[i]));
        coordinateY.push_back(coordinateYTensor);

        if (sei.m_basePicFlag)
        {
          baseCoordinateY.push_back(coordinateYTensor);
          prevCoordinateY.push_back(coordinateYTensor);
        }
        else
        {
          prevCoordinateY[i] = coordinateYTensor;
        }
        if (m3DCoordinateFlag == 1)
        {
          sei_read_uvlc(pDecodedMessageOutputStream, val, "gfv_coordinate_dz_abs[ i ]");
          int curCoordinateZInt = val;
          double coordinateZTensorAbs = ((double)curCoordinateZInt / (1 << coordinateQuantizationFactor));
          valueSignFlag = 0;
          if (curCoordinateZInt)
          {
            sei_read_flag(pDecodedMessageOutputStream, val, "gfv_coordinate_dz_sign_flag");
            valueSignFlag = val;
          }
          double  coordinateZTensor = (valueSignFlag ? -coordinateZTensorAbs : coordinateZTensorAbs) + (sei.m_basePicFlag ? (i == 0 ? 0 : prevCoordinateZ[i - 1]) : (cnt == 0 ? baseCoordinateZ[i] : prevCoordinateZ[i]));
          coordinateZ.push_back(coordinateZTensor);
          if (sei.m_basePicFlag)
          {
            baseCoordinateZ.push_back(coordinateZTensor);
            prevCoordinateZ.push_back(coordinateZTensor);
          }
          else
          {
            prevCoordinateZ[i] = coordinateZTensor;
          }
        }
      }
    }
    if (pDecodedMessageOutputStream)
    {
      (*pDecodedMessageOutputStream) << "  " << std::setw(55) << "gfv_coordinate_Xtensor" << ": ";
      for (int xi = 0; xi < coordinatePointNum; xi++)
      {
        (*pDecodedMessageOutputStream) << (coordinateX[xi]) << " ";
      }
      (*pDecodedMessageOutputStream) << "\n";
    }
    if (pDecodedMessageOutputStream)
    {
      (*pDecodedMessageOutputStream) << "  " << std::setw(55) << "gfv_coordinate_Ytensor" << ": ";
      for (int yi = 0; yi < coordinatePointNum; yi++)
      {
        (*pDecodedMessageOutputStream) << (coordinateY[yi]) << " ";
      }
      (*pDecodedMessageOutputStream) << "\n";
    }
    if (m3DCoordinateFlag == 1)
    {
      if (pDecodedMessageOutputStream)
      {
        (*pDecodedMessageOutputStream) << "  " << std::setw(55) << "gfv_coordinate_Ztensor" << ": ";
        for (int zi = 0; zi < coordinatePointNum; zi++)
        {
          (*pDecodedMessageOutputStream) << (coordinateZ[zi]) << " ";
        }
        (*pDecodedMessageOutputStream) << "\n";
      }
    }
  }
  else
  {
    m3DCoordinateFlag = 0;
    coordinatePointNum = 0;
  }
  sei_read_flag(pDecodedMessageOutputStream, val, "gfv_matrix_present_flag");
  matrixPresentFlag = val;
  assert(coordinatePresentFlag || matrixPresentFlag);
  if (matrixPresentFlag)
  {
    uint32_t matrixWidth = 0;
    uint32_t matrixHeight = 0;
    uint32_t numMatrices = 0;
    std::vector<uint32_t> matrixWidthVec;
    std::vector<uint32_t> matrixHeightVec;
    std::vector<uint32_t> numMatricesVec;
    if (!sei.m_basePicFlag)
    {
      sei_read_flag(pDecodedMessageOutputStream, val, "gfv_matrix_pred_flag");
      matrixPredFlag = val;
    }
    else
    {
      matrixPredFlag = 0;
    }
    if (!matrixPredFlag)
    {
      sei_read_uvlc(pDecodedMessageOutputStream, val, "gfv_matrix_element_precision_factor_minus1");
      assert(val >= 0 && val <= 31);
      matrixElementPrecisionFactor = val + 1;
      sei_read_uvlc(pDecodedMessageOutputStream, val, "gfv_num_matrix_types_minus1");
      numMatrixType = val + 1;
      if (sei.m_basePicFlag)
      {
        baseMatrixElementPrecisionFactor = matrixElementPrecisionFactor;
        baseNumMatrixType = numMatrixType;
      }
      for (uint32_t matrixId = 0; matrixId < numMatrixType; matrixId++)
      {
        sei_read_code(pDecodedMessageOutputStream, 6, val, "gfv_matrix_type_idx");
        matrixTypeIdx.push_back(val);
        if (matrixTypeIdx[matrixId] == 0 || matrixTypeIdx[matrixId] == 1)
        {
          assert(coordinatePresentFlag != 0);
          sei_read_flag(pDecodedMessageOutputStream, val, "gfv_num_matrices_equal_to_num_kps_flag");
          numMatricestoNumKpsFlag.push_back(val);
          if (!numMatricestoNumKpsFlag[matrixId])
          {
            sei_read_uvlc(pDecodedMessageOutputStream, val, "gfv_num_matrices_info");
            assert(val >= 0 && val <= (1 << 10) - 1);
            numMatricesInfo.push_back(val);
          }
        }
        else if (matrixTypeIdx[matrixId] == 2 || matrixTypeIdx[matrixId] == 3 || matrixTypeIdx[matrixId] >= 7)
        {
          if (matrixTypeIdx[matrixId] >= 7)
          {
            sei_read_uvlc(pDecodedMessageOutputStream, val, "gfv_num_matrices_minus1");
            assert(val >= 0 && val <= (1 << 10) - 1);
            numMatrices = val + 1;
          }
          sei_read_uvlc(pDecodedMessageOutputStream, val, "gfv_matrix_width_minus1");
          assert(val >= 0 && val <= (1 << 10) - 1);
          matrixWidth = val + 1;
          sei_read_uvlc(pDecodedMessageOutputStream, val, "gfv_matrix_height_minus1");
          assert(val >= 0 && val <= (1 << 10) - 1);
          matrixHeight = val + 1;
        }
        else if (matrixTypeIdx[matrixId] >= 4 && matrixTypeIdx[matrixId] <= 6)
        {
          if (!coordinatePresentFlag)
          {
            sei_read_flag(pDecodedMessageOutputStream, val, "gfv_Matrix3DSpaceFlag");
            matrix3DSpaceFlag.push_back(val);
          }
          else
          {
            matrix3DSpaceFlag.push_back(m3DCoordinateFlag);
          }
        }
        if (matrixTypeIdx[matrixId] == 0 || matrixTypeIdx[matrixId] == 1)
        {
          matrixWidth = m3DCoordinateFlag + 2;
          matrixHeight = m3DCoordinateFlag + 2;
        }
        else if (matrixTypeIdx[matrixId] == 4)
        {
          matrixWidth = matrix3DSpaceFlag[matrixId] + 2;
          matrixHeight = matrix3DSpaceFlag[matrixId] + 2;
        }
        else if (matrixTypeIdx[matrixId] == 5 || matrixTypeIdx[matrixId] == 6)
        {
          matrixWidth = 1;
          matrixHeight = matrix3DSpaceFlag[matrixId] + 2;
        }
        if (matrixTypeIdx[matrixId] == 0 || matrixTypeIdx[matrixId] == 1)
        {
          if (coordinatePresentFlag)
          {
            numMatrices = numMatricestoNumKpsFlag[matrixId] ? coordinatePointNum : (numMatricesInfo[matrixId] < (coordinatePointNum - 1) ? (numMatricesInfo[matrixId] + 1) : (numMatricesInfo[matrixId] + 2));
          }
          else
          {
            numMatrices = numMatricesInfo[matrixId] + 1;
          }
        }
        else if (matrixTypeIdx[matrixId] >= 2 && matrixTypeIdx[matrixId] < 7)
        {
          numMatrices = 1;
        }
        if (sei.m_basePicFlag)
        {
          baseNumMatrices.push_back(numMatrices);
          baseMatrixWidth.push_back(matrixWidth);
          baseMatrixHeight.push_back(matrixHeight);
        }
        numMatricesVec.push_back(numMatrices);
        matrixWidthVec.push_back(matrixWidth);
        matrixHeightVec.push_back(matrixHeight);
      }
    }
    else
    {
      matrixElementPrecisionFactor = baseMatrixElementPrecisionFactor;
      numMatrixType = baseNumMatrixType;
      for (uint32_t matrixId = 0; matrixId < numMatrixType; matrixId++)
      {
        numMatrices = baseNumMatrices[matrixId];
        matrixWidth = baseMatrixWidth[matrixId];
        matrixHeight = baseMatrixHeight[matrixId];

        numMatricesVec.push_back(numMatrices);
        matrixWidthVec.push_back(matrixWidth);
        matrixHeightVec.push_back(matrixHeight);
      }
    }
    for (uint32_t matrixId = 0; matrixId < numMatrixType; matrixId++)
    {
      matrixElement.push_back(std::vector<std::vector<std::vector<double>>>());
      for (uint32_t j = 0; j < numMatricesVec[matrixId]; j++)
      {
        matrixElement[matrixId].push_back(std::vector<std::vector<double>>());
        for (uint32_t k = 0; k < matrixHeightVec[matrixId]; k++)
        {
          matrixElement[matrixId][j].push_back(std::vector<double>());
          for (uint32_t l = 0; l < matrixWidthVec[matrixId]; l++)
          {
            if (!matrixPredFlag)
            {
              sei_read_uvlc(pDecodedMessageOutputStream, val, "gfv_matrix_element_int");
              assert(val >=0 && val <= 4294967296 - 2);
              int matrixElementAbsInteger = val;
              sei_read_code(pDecodedMessageOutputStream, matrixElementPrecisionFactor, val, "gfv_matrix_element_dec");
              int curMatrixDecIntValue = val;
              double matrixElementAbsDecimal = ((double)curMatrixDecIntValue) / (1 << matrixElementPrecisionFactor);
              valueSignFlag = 0;
              if (matrixElementAbsInteger || curMatrixDecIntValue)
              {
                sei_read_flag(pDecodedMessageOutputStream, val, "gfv_matrix_element_sign_flag");
                valueSignFlag = val;
              }
              double matrixElementDecimal = valueSignFlag ? -(matrixElementAbsDecimal + matrixElementAbsInteger) : (matrixElementAbsDecimal + matrixElementAbsInteger);
              matrixElement[matrixId][j][k].push_back(matrixElementDecimal);
            }
            else
            {
              sei_read_uvlc(pDecodedMessageOutputStream, val, "gfv_matrix_delta_element_int");
              int matrixElementAbsInteger = val;
              sei_read_uvlc(pDecodedMessageOutputStream, val, "gfv_matrix_delta_element_dec");
              int curMatrixDecIntValue = val;
              double matrixElementAbsDecimal = ((double)curMatrixDecIntValue) / (1 << matrixElementPrecisionFactor);
              valueSignFlag = 0;
              if (matrixElementAbsInteger || curMatrixDecIntValue)
              {
                sei_read_flag(pDecodedMessageOutputStream, val, "gfv_matrix_delta_element_sign_flag");
                valueSignFlag = val;
              }
              assert(!sei.m_basePicFlag);
              double matrixElementDecimal = (valueSignFlag ? -(matrixElementAbsDecimal + matrixElementAbsInteger) : (matrixElementAbsDecimal + matrixElementAbsInteger)) + (cnt == 0 ? baseMatrix[matrixId][j][k][l] : prevMatrix[matrixId][j][k][l]);
              matrixElement[matrixId][j][k].push_back(matrixElementDecimal);
            }
          }
        }
      }
    }
    prevMatrix = matrixElement;
    baseMatrix = matrixElement;
    if (pDecodedMessageOutputStream)
    {
      (*pDecodedMessageOutputStream) << "  " << std::setw(55) << "gfv_MatrixElement" << ": ";
      for (uint32_t mi = 0; mi < numMatrixType; mi++)
      {
        for (uint32_t mj = 0; mj < numMatricesVec[mi]; mj++) // 
        {
          for (uint32_t mk = 0; mk < matrixHeightVec[mi]; mk++)
          {
            for (uint32_t ml = 0; ml < matrixWidthVec[mi]; ml++)
            {
              (*pDecodedMessageOutputStream) << (matrixElement[mi][mj][mk][ml]) << " ";
            }
          }
        }
      }
      (*pDecodedMessageOutputStream) << "\n";
    }
  }
  if (sei.m_nnPresentFlag)
  {
    if (sei.m_nnModeIdc == 0)
    {
      while (m_pcBitstream->getNumBitsRead() % 8 != 0)
      {
        sei_read_flag(pDecodedMessageOutputStream, val, "gfv_nn_alignment_zero_bit_b");
        assert(val == 0);
      }
      int payloadBytesRemaining = getBitstream()->getNumBitsLeft() / 8;
      sei.m_payloadLength = payloadBytesRemaining;
      sei.m_payloadByte = new char[sei.m_payloadLength];
      int code;
      std::string filename = "payloadByte" + std::to_string(id) + ".nnr";
      std::ofstream outFile(filename.c_str(), std::ofstream::binary);
      for (int i = 0; i < payloadBytesRemaining; i++)
      {
        sei_read_scode(pDecodedMessageOutputStream, 8, code, "gfv_nn_payload_byte[i]");
        sei.m_payloadByte[i] = (char)code;
        outFile.write((char*)&code, 1);
      }
      outFile.close();
    }
  }
}
#endif
#if JVET_AK0239_GEFV
void SEIReader::xParseSEIGenerativeFaceVideoEnhancement(SEIGenerativeFaceVideoEnhancement & sei, uint32_t payloadSize, std::ostream* pDecodedMessageOutputStream)
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  uint32_t   val;
  bool       valueSignFlag;
  uint32_t   gfveId;
  uint32_t   gfveGfvCnt;
  uint32_t   gfveGfvId;
  bool       matrixPresentFlag;
  bool       matrixPredFlag;

  sei_read_uvlc(pDecodedMessageOutputStream, val, "gefv_id");
  gfveId = val;
  sei_read_uvlc(pDecodedMessageOutputStream, val, "gefv_gfv_id");
  gfveGfvId = val;
  sei_read_uvlc(pDecodedMessageOutputStream, val, "gefv_gfv_cnt");
  gfveGfvCnt = val;

  if (gfveGfvCnt == 0)
  {
    sei_read_flag(pDecodedMessageOutputStream, val, "gefv_base_picture_flag");
    sei.m_basePicFlag = val;
  }
  else
  {
    sei.m_basePicFlag = 0;
  }
  if (sei.m_basePicFlag)
  {
    sei_read_flag(pDecodedMessageOutputStream, val, "gefv_nn_present_flag");
    sei.m_nnPresentFlag = val;
    if (sei.m_nnPresentFlag)
    {
      sei_read_uvlc(pDecodedMessageOutputStream, val, "gefv_nn_mode_idc");
      sei.m_nnModeIdc = val;
      if (sei.m_nnModeIdc == 1)
      {
        std::string val2;
        while (m_pcBitstream->getNumBitsRead() % 8 != 0)
        {
          sei_read_flag(pDecodedMessageOutputStream, val, "gefv_nn_alignment_zero_bit_a");
          assert(val == 0);
        }
        sei_read_string(pDecodedMessageOutputStream, val2, "gefv_nn_uri_tag");
        sei.m_nnTagURI = val2;
        val2 = "";
        sei_read_string(pDecodedMessageOutputStream, val2, "gefv_nn_uri");
        sei.m_nnURI = val2;
      }
    }
  }
  gfveBaseMatrix.push_back(std::vector<std::vector<double>>());
  gfvePrevMatrix.push_back(std::vector<std::vector<double>>());
  sei_read_flag(pDecodedMessageOutputStream, val, "gefv_matrix_present_flag");
  matrixPresentFlag = val;
  if (matrixPresentFlag)
  {
    uint32_t matrixWidth = 0;
    uint32_t matrixHeight = 0;
    uint32_t numMatrices = 0;
    std::vector<uint32_t> matrixWidthVec;
    std::vector<uint32_t> matrixHeightVec;
    uint32_t   matrixElementPrecisionFactor;
    std::vector<std::vector<std::vector<double>>>   matrixElement;
    if (!sei.m_basePicFlag)
    {
      sei_read_flag(pDecodedMessageOutputStream, val, "gefv_matrix_pred_flag");
      matrixPredFlag = val;
    }
    else
    {
      matrixPredFlag = 0;
    }
    if (!matrixPredFlag)
    {
      sei_read_uvlc(pDecodedMessageOutputStream, val, "gefv_matrix_element_precision_factor_minus1");
      assert(val >= 0 && val <= 31);
      matrixElementPrecisionFactor = val + 1;
      sei_read_uvlc(pDecodedMessageOutputStream, val, "gefv_num_matrices_minus1");
      numMatrices = val + 1;
      if (sei.m_basePicFlag)
      {
        gfveBaseMatrixElementPrecisionFactor = matrixElementPrecisionFactor;
        gfveBaseNumMatrices = numMatrices;
      }
      for (uint32_t j = 0; j < numMatrices; j++)
      {
        sei_read_uvlc(pDecodedMessageOutputStream, val, "gefv_matrix_height_minus1");
        matrixHeight = val + 1;
        sei_read_uvlc(pDecodedMessageOutputStream, val, "gefv_matrix_width_minus1");
        matrixWidth = val + 1;
        matrixHeightVec.push_back(matrixHeight);
        matrixWidthVec.push_back(matrixWidth);
        if (sei.m_basePicFlag)
        {
          gfveBaseMatrixHeight.push_back(matrixHeight);
          gfveBaseMatrixWidth.push_back(matrixWidth);
        }
      }
    }
    else
    {
      matrixElementPrecisionFactor = gfveBaseMatrixElementPrecisionFactor;
      numMatrices = gfveBaseNumMatrices;
      matrixWidthVec = gfveBaseMatrixWidth;
      matrixHeightVec = gfveBaseMatrixHeight;
    }

    for (uint32_t j = 0; j < numMatrices; j++)
    {
      matrixElement.push_back(std::vector<std::vector<double>>());
      for (uint32_t k = 0; k < matrixHeightVec[j]; k++)
      {
        matrixElement[j].push_back(std::vector<double>());
        for (uint32_t l = 0; l < matrixWidthVec[j]; l++)
        {
          if (!matrixPredFlag)
          {
            sei_read_uvlc(pDecodedMessageOutputStream, val, "gefv_matrix_element_int");
            assert(val >= 0 && val <= 4294967296 - 2);
            int matrixElementAbsInteger = val;
            sei_read_code(pDecodedMessageOutputStream, matrixElementPrecisionFactor, val, "gefv_matrix_element_dec");
            int curMatrixDecIntValue = val;
            double matrixElementAbsDecimal = ((double)curMatrixDecIntValue*1.0) / (1 << matrixElementPrecisionFactor);
            valueSignFlag = 0;
            if (matrixElementAbsInteger || curMatrixDecIntValue)
            {
              sei_read_flag(pDecodedMessageOutputStream, val, "gefv_matrix_element_sign_flag");
              valueSignFlag = val;
            }
            double matrixElementDecimal = valueSignFlag ? -(matrixElementAbsDecimal + matrixElementAbsInteger) : (matrixElementAbsDecimal + matrixElementAbsInteger);
            matrixElement[j][k].push_back(matrixElementDecimal);
          }
          else
          {
            sei_read_uvlc(pDecodedMessageOutputStream, val, "gefv_matrix_delta_element_int");
            assert(val >= 0 && val <= 4294967296 - 2);
            int matrixElementAbsInteger = val;
            sei_read_code(pDecodedMessageOutputStream, matrixElementPrecisionFactor, val, "gefv_matrix_delta_element_dec");
            int curMatrixDecIntValue = val;
            double matrixElementAbsDecimal = ((double)curMatrixDecIntValue*1.0) / (1 << matrixElementPrecisionFactor);
            valueSignFlag = 0;
            if (matrixElementAbsInteger || curMatrixDecIntValue)
            {
              sei_read_flag(pDecodedMessageOutputStream, val, "gefv_matrix_delta_element_sign_flag");
              valueSignFlag = val;
            }
            double matrixElementDecimal = (valueSignFlag ? -(matrixElementAbsDecimal + matrixElementAbsInteger) : (matrixElementAbsDecimal + matrixElementAbsInteger)) + (gfveGfvCnt == 0 ? gfveBaseMatrix[j][k][l] : gfvePrevMatrix[j][k][l]);
            matrixElement[j][k].push_back(matrixElementDecimal);
          }
        }
      }
    }
    gfvePrevMatrix = matrixElement;
    if (sei.m_basePicFlag)
    {
      gfveBaseMatrix = matrixElement;
    }
    if (pDecodedMessageOutputStream)
    {
      (*pDecodedMessageOutputStream) << "  " << std::setw(55) << "gefvMatrixNumber" << ": " << numMatrices << "\n";
      (*pDecodedMessageOutputStream) << "  " << std::setw(55) << "gefvMatrixWidthXHeight" << ": ";
      for (uint32_t mj = 0; mj < numMatrices; mj++) // 
      {
        (*pDecodedMessageOutputStream) << (matrixWidthVec[mj]) << "x" << (matrixHeightVec[mj]) << " ";
      }
      (*pDecodedMessageOutputStream) << "\n";
      (*pDecodedMessageOutputStream) << "  " << std::setw(55) << "gefvMatrixElement" << ": ";
      for (uint32_t mj = 0; mj < numMatrices; mj++) // 
      {
        for (uint32_t mk = 0; mk < matrixHeightVec[mj]; mk++)
        {
          for (uint32_t ml = 0; ml < matrixWidthVec[mj]; ml++)
          {
            (*pDecodedMessageOutputStream) << (matrixElement[mj][mk][ml]) << " ";
          }
        }
      }
      (*pDecodedMessageOutputStream) << "\n";
    }
  }

  sei_read_code(pDecodedMessageOutputStream, 2, val, "gefv_pupil_coordinate_present_idx");
  uint32_t pupilPresentIdx = val;
  double   pupilLeftEyeCoordinateX;
  double   pupilLeftEyeCoordinateY;
  double   pupilRightEyeCoordinateX;
  double   pupilRightEyeCoordinateY;
  uint32_t pupilCoordinatePrecisionFactor = 0;
  if (pupilPresentIdx)
  {
    if (sei.m_basePicFlag)
    {
      checkBasePicPupilPresentIdx = true;
      sei_read_uvlc(pDecodedMessageOutputStream, val, "gefv_pupil_coordinate_precision_factor_minus1");
      assert(val >= 0 && val <= 31);
      pupilCoordinatePrecisionFactor = val + 1;
      gfveBasePupilCoordinatePrecisionFactor = pupilCoordinatePrecisionFactor;
    }
    else
    {
      pupilCoordinatePrecisionFactor = gfveBasePupilCoordinatePrecisionFactor;
    }
  }
  if (checkBasePicPupilPresentIdx)
  {
    double gfveLeftPupilCoordinateXRef = 0.0;
    double gfveLeftPupilCoordinateYRef = 0.0;
    double gfveRightPupilCoordinateXRef = 0.0;
    double gfveRightPupilCoordinateYRef = 0.0;
    if (gfveGfvCnt == 0)
    {
      if (!sei.m_basePicFlag)
      {
        gfveLeftPupilCoordinateXRef = baseGfveLeftPupilCoordinateX;
        gfveLeftPupilCoordinateYRef = baseGfveLeftPupilCoordinateY;
        gfveRightPupilCoordinateXRef = baseGfveRightPupilCoordinateX;
        gfveRightPupilCoordinateYRef = baseGfveRightPupilCoordinateY;
      }
    }
    else
    {
      gfveLeftPupilCoordinateXRef = prevGfveLeftPupilCoordinateX;
      gfveLeftPupilCoordinateYRef = prevGfveLeftPupilCoordinateY;
      gfveRightPupilCoordinateXRef = prevGfveRightPupilCoordinateX;
      gfveRightPupilCoordinateYRef = prevGfveRightPupilCoordinateY;
    }
    if (pupilPresentIdx == 1 || pupilPresentIdx == 3)
    {
      pupilLeftEyeCoordinateX = xParseSEIPupilCoordinate(pDecodedMessageOutputStream, gfveLeftPupilCoordinateXRef, pupilCoordinatePrecisionFactor, "left", "x");
      pupilLeftEyeCoordinateY = xParseSEIPupilCoordinate(pDecodedMessageOutputStream, gfveLeftPupilCoordinateYRef, pupilCoordinatePrecisionFactor, "left", "y");
    }
    else
    {
      pupilLeftEyeCoordinateX = gfveLeftPupilCoordinateXRef;
      pupilLeftEyeCoordinateY = gfveLeftPupilCoordinateYRef;
    }
    if (sei.m_basePicFlag)
    {
      gfveRightPupilCoordinateXRef = pupilLeftEyeCoordinateX;
      gfveRightPupilCoordinateYRef = pupilLeftEyeCoordinateY;
    }
    if (pupilPresentIdx == 2 || pupilPresentIdx == 3)
    {
      pupilRightEyeCoordinateX = xParseSEIPupilCoordinate(pDecodedMessageOutputStream, gfveRightPupilCoordinateXRef, pupilCoordinatePrecisionFactor, "right", "x");
      pupilRightEyeCoordinateY = xParseSEIPupilCoordinate(pDecodedMessageOutputStream, gfveRightPupilCoordinateYRef, pupilCoordinatePrecisionFactor, "right", "y");
    }
    else
    {
      pupilRightEyeCoordinateX = gfveRightPupilCoordinateXRef;
      pupilRightEyeCoordinateY = gfveRightPupilCoordinateYRef;
    }

    if (sei.m_basePicFlag)
    {
      baseGfveLeftPupilCoordinateX = pupilLeftEyeCoordinateX;
      baseGfveLeftPupilCoordinateY = pupilLeftEyeCoordinateY;
      baseGfveRightPupilCoordinateX = pupilRightEyeCoordinateX;
      baseGfveRightPupilCoordinateY = pupilRightEyeCoordinateY;
    }
    prevGfveLeftPupilCoordinateX = pupilLeftEyeCoordinateX;
    prevGfveLeftPupilCoordinateY = pupilLeftEyeCoordinateY;
    prevGfveRightPupilCoordinateX = pupilRightEyeCoordinateX;
    prevGfveRightPupilCoordinateY = pupilRightEyeCoordinateY;

    if (pDecodedMessageOutputStream)
    {
      (*pDecodedMessageOutputStream) << "  " << std::setw(55) << "gefv_PupilCoordinate" << ": ";

      (*pDecodedMessageOutputStream) << (pupilLeftEyeCoordinateX) << " ";
      (*pDecodedMessageOutputStream) << (pupilLeftEyeCoordinateY) << " ";
      (*pDecodedMessageOutputStream) << (pupilRightEyeCoordinateX) << " ";
      (*pDecodedMessageOutputStream) << (pupilRightEyeCoordinateY) << " ";

      (*pDecodedMessageOutputStream) << "\n";
    }
  }

  if (sei.m_nnPresentFlag)
  {
    if (sei.m_nnModeIdc == 0)
    {
      while (m_pcBitstream->getNumBitsRead() % 8 != 0)
      {
        sei_read_flag(pDecodedMessageOutputStream, val, "gefv_nn_alignment_zero_bit_b");
        assert(val == 0);
      }
      int payloadBytesRemaining = getBitstream()->getNumBitsLeft() / 8;
      sei.m_payloadLength = payloadBytesRemaining;
      sei.m_payloadByte = new char[sei.m_payloadLength];
      int code;
      std::string filename = "payloadByte_" + std::to_string(gfveId) + "_" + std::to_string(gfveGfvId) + ".nnr";


      std::ofstream outFile(filename.c_str(), std::ofstream::binary);
      for (int i = 0; i < payloadBytesRemaining; i++)
      {
        sei_read_scode(pDecodedMessageOutputStream, 8, code, "gfv_nn_payload_byte[i]");
        sei.m_payloadByte[i] = (char)code;
        outFile.write((char*)&code, 1);
      }
      outFile.close();

    }
  }
}
double SEIReader::xParseSEIPupilCoordinate(std::ostream *pOS, double refCoordinate, int precisionFactor, const char* eye, const char* axis)
{
  uint32_t val;
  bool     valueSignFlag;
  assert(std::string(eye) == "left" || std::string(eye) == "right");
  assert(std::string(axis) == "x" || std::string(axis) == "y");
  std::string checkMessage = "The value of gefv_pupil_" + std::string(eye) + "_eye_d" + std::string(axis) + "_coordinate_abs shall be be 0 to 1 << (gefv_pupil_coordinate_precision_factor_minus1 + 2), inclusive";
  std::string absSymbolName = "gefv_pupil_" + std::string(eye) + "_eye_d" + std::string(axis) + "_coordinate_abs";
  sei_read_uvlc(pOS, val, absSymbolName.c_str());
  assert(val >= 0 && val <=(1 << (precisionFactor + 1)));
  double coordinateAbs = static_cast<double>(val) / (1 << precisionFactor);
  valueSignFlag = 0;
  if (val)
  {
    std::string signSymbolName = "gefv_pupil_" + std::string(eye) + "_eye_d" + std::string(axis) + "_coordinate_sign_flag";
    sei_read_flag(pOS, val, signSymbolName.c_str());
    valueSignFlag = val;
  }
  return (valueSignFlag ? -coordinateAbs : coordinateAbs) + refCoordinate;
}
#endif
#if JVET_AK0140_PACKED_REGIONS_INFORMATION_SEI
void SEIReader::xParsePackedRegionsInfo(SEIPackedRegionsInfo& sei, uint32_t payLoadSize, std::ostream* pDecodedMessageOutputStream)
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payLoadSize);
  uint32_t val;

  sei_read_flag(pDecodedMessageOutputStream, val, "pri_cancel_flag");
  sei.m_cancelFlag = val != 0;
  if (!sei.m_cancelFlag)
  {
    sei_read_flag(pDecodedMessageOutputStream, val, "pri_persistence_flag");
    sei.m_persistenceFlag = val != 0;
    sei_read_uvlc(pDecodedMessageOutputStream, val, "pri_num_regions_minus1");
    CHECK(val > 255, "pri_num_regions_minus1 shall be in the range of 0 to 255, inclusive");
    sei.m_numRegionsMinus1 = val;
    sei_read_flag(pDecodedMessageOutputStream, val, "pri_multilayer_flag");
    sei.m_multilayerFlag = val != 0;
    sei_read_flag(pDecodedMessageOutputStream, val, "pri_use_max_dimensions_flag");
    sei.m_useMaxDimensionsFlag = val != 0;
    sei_read_code(pDecodedMessageOutputStream, 4, val, "pri_log2_unit_size");
    sei.m_log2UnitSize = val;
    sei_read_code(pDecodedMessageOutputStream, 4, val, "pri_region_size_len_minus1");
    sei.m_regionSizeLenMinus1 = val;
    sei_read_flag(pDecodedMessageOutputStream, val, "pri_region_id_present_flag");
    sei.m_regionIdPresentFlag = val != 0;
    sei_read_flag(pDecodedMessageOutputStream, val, "pri_target_pic_params_present_flag");
    sei.m_targetPicParamsPresentFlag = val != 0;
    if (sei.m_targetPicParamsPresentFlag)
    {
      sei_read_code(pDecodedMessageOutputStream, 16, val, "pri_target_pic_width_minus1");
      sei.m_targetPicWidthMinus1 = val;
      sei_read_code(pDecodedMessageOutputStream, 16, val, "pri_target_pic_height_minus1");
      sei.m_targetPicHeightMinus1 = val;
    }
    sei_read_uvlc(pDecodedMessageOutputStream, val, "pri_num_resampling_ratios_minus1");
    CHECK(val > sei.m_numRegionsMinus1, "pri_num_resampling_ratios_minus1 shall be in the range of 0 to pri_num_regions_minus1, inclusive");
    sei.m_numResamplingRatiosMinus1 = val;

    sei.m_resamplingWidthNumMinus1.resize(sei.m_numResamplingRatiosMinus1 + 1);
    sei.m_resamplingWidthDenomMinus1.resize(sei.m_numResamplingRatiosMinus1 + 1);
    sei.m_fixedAspectRatioFlag.resize(sei.m_numResamplingRatiosMinus1 + 1);
    sei.m_resamplingHeightNumMinus1.resize(sei.m_numResamplingRatiosMinus1 + 1);
    sei.m_resamplingHeightDenomMinus1.resize(sei.m_numResamplingRatiosMinus1 + 1);
    sei.m_resamplingWidthNumMinus1[0] = 0;
    sei.m_resamplingWidthDenomMinus1[0] = 0;
    sei.m_fixedAspectRatioFlag[0] = true;
    sei.m_resamplingHeightNumMinus1[0] = 0;
    sei.m_resamplingHeightDenomMinus1[0] = 0;
    for (uint32_t i = 1; i <= sei.m_numResamplingRatiosMinus1; i++)
    {
      sei_read_uvlc(pDecodedMessageOutputStream, val, "pri_resampling_width_num_minus1[i]");
      sei.m_resamplingWidthNumMinus1[i] = val;
      sei_read_uvlc(pDecodedMessageOutputStream, val, "pri_resampling_width_denom_minus1[i]");
      sei.m_resamplingWidthDenomMinus1[i] = val;
      if (sei.m_targetPicParamsPresentFlag)
      {
        double horRatioVal = (double)(sei.m_resamplingWidthNumMinus1[i] + 1) / (sei.m_resamplingWidthDenomMinus1[i] + 1);
        CHECK(horRatioVal < 1.0 / 16.0 || horRatioVal > 16.0, "(pri_resampling_width_num_minus1[i] + 1) / (pri_resampling_width_denom_minus1[i] + 1) shall be in the range of 1/16 to 16, inclusive");
      }
      sei_read_flag(pDecodedMessageOutputStream, val, "pri_fixed_aspect_ratio_flag[i]");
      sei.m_fixedAspectRatioFlag[i] = val != 0;
      if (!sei.m_fixedAspectRatioFlag[i])
      {
        sei_read_uvlc(pDecodedMessageOutputStream, val, "pri_resampling_height_num_minus1[i]");
        sei.m_resamplingHeightNumMinus1[i] = val;
        sei_read_uvlc(pDecodedMessageOutputStream, val, "pri_resampling_height_denom_minus1[i]");
        sei.m_resamplingHeightDenomMinus1[i] = val;
        if (sei.m_targetPicParamsPresentFlag)
        {
          double verRatioVal = (double)(sei.m_resamplingHeightNumMinus1[i] + 1) / (sei.m_resamplingHeightDenomMinus1[i] + 1);
          CHECK(verRatioVal < 1.0 / 16.0 || verRatioVal > 16.0, "(pri_resampling_height_num_minus1[i] + 1) / (pri_resampling_height_denom_minus1[i] + 1) shall be in the range of 1/16 to 16, inclusive");
        }
      }
      else
      {
        sei.m_resamplingHeightNumMinus1[i] = sei.m_resamplingWidthNumMinus1[i];
        sei.m_resamplingHeightDenomMinus1[i] = sei.m_resamplingWidthDenomMinus1[i];
      }
    }

    sei.m_regionId.resize(sei.m_numRegionsMinus1 + 1);
    sei.m_regionLayerId.resize(sei.m_numRegionsMinus1 + 1);
    sei.m_regionIsALayerFlag.resize(sei.m_numRegionsMinus1 + 1);
    sei.m_regionTopLeftInUnitsX.resize(sei.m_numRegionsMinus1 + 1);
    sei.m_regionTopLeftInUnitsY.resize(sei.m_numRegionsMinus1 + 1);
    sei.m_regionWidthInUnitsMinus1.resize(sei.m_numRegionsMinus1 + 1);
    sei.m_regionHeightInUnitsMinus1.resize(sei.m_numRegionsMinus1 + 1);
    sei.m_resamplingRatioIdx.resize(sei.m_numRegionsMinus1 + 1);
    std::fill(sei.m_regionId.begin(), sei.m_regionId.end(), MAX_UINT);
    sei.m_targetRegionTopLeftInUnitsX.resize(sei.m_numRegionsMinus1 + 1);
    sei.m_targetRegionTopLeftInUnitsY.resize(sei.m_numRegionsMinus1 + 1);
    for (uint32_t i = 0; i <= sei.m_numRegionsMinus1; i++)
    {
      if (sei.m_regionIdPresentFlag)
      {
        sei_read_uvlc(pDecodedMessageOutputStream, val, "pri_region_id[i]");
        CHECK(val > 1023, "pri_region_id[i] shall be in the range of 0 to 1023, inclusive");
        sei.m_regionId[i] = val;
      }
      else
      {
        sei.m_regionId[i] = i;
      }
      if (sei.m_multilayerFlag)
      {
        sei_read_uvlc(pDecodedMessageOutputStream, val, "pri_region_layer_id[i]");
        CHECK(val > 2047, "pri_region_layer_id[i] shall be in the range of 0 to 2047, inclusive");
        sei.m_regionLayerId[i] = val;
        sei_read_flag(pDecodedMessageOutputStream, val, "pri_region_is_a_layer_flag[i]");
        sei.m_regionIsALayerFlag[i] = val != 0;
      }
      else
      {
        sei.m_regionLayerId[i] = sei.m_layerId;
        sei.m_regionIsALayerFlag[i] = 0;
      }
      if (!sei.m_regionIsALayerFlag[i])
      {
        sei_read_code(pDecodedMessageOutputStream, sei.m_regionSizeLenMinus1 + 1, val, "pri_region_top_left_in_units_x[i]");
        sei.m_regionTopLeftInUnitsX[i] = val;
        sei_read_code(pDecodedMessageOutputStream, sei.m_regionSizeLenMinus1 + 1, val, "pri_region_top_left_in_units_y[i]");
        sei.m_regionTopLeftInUnitsY[i] = val;
        sei_read_code(pDecodedMessageOutputStream, sei.m_regionSizeLenMinus1 + 1, val, "pri_region_width_in_units_minus1[i]");
        sei.m_regionWidthInUnitsMinus1[i] = val;
        sei_read_code(pDecodedMessageOutputStream, sei.m_regionSizeLenMinus1 + 1, val, "pri_region_height_in_units_minus1[i]");
        sei.m_regionHeightInUnitsMinus1[i] = val;
      }
      else
      {
        sei.m_regionTopLeftInUnitsX[i] = 0;
        sei.m_regionTopLeftInUnitsY[i] = 0;
        sei.m_regionWidthInUnitsMinus1[i] = 0;
        sei.m_regionHeightInUnitsMinus1[i] = 0;
      }
      if (sei.m_numResamplingRatiosMinus1 > 0)
      {
        uint32_t codeLen = 0;
        for (uint32_t i = sei.m_numResamplingRatiosMinus1; i != 0; i >>= 1)
        {
          codeLen++;
        }
        sei_read_code(pDecodedMessageOutputStream, codeLen, val, "pri_resampling_ratio_idx[i]");
        sei.m_resamplingRatioIdx[i] = val;
      }
      else
      {
        sei.m_resamplingRatioIdx[i] = 0;
      }
      if (sei.m_targetPicParamsPresentFlag)
      {
        sei_read_code(pDecodedMessageOutputStream, sei.m_regionSizeLenMinus1 + 1, val, "pri_target_region_top_left_in_units_x[i]");
        sei.m_targetRegionTopLeftInUnitsX[i] = val;
        sei_read_code(pDecodedMessageOutputStream, sei.m_regionSizeLenMinus1 + 1, val, "pri_target_region_top_left_in_units_y[i]");
        sei.m_targetRegionTopLeftInUnitsY[i] = val;
      }
    }
    if (sei.m_regionIdPresentFlag)
    {
      std::vector<uint32_t> tmpVec = sei.m_regionId;
      std::sort(tmpVec.begin(), tmpVec.end());
      auto it = std::unique(tmpVec.begin(), tmpVec.end());
      CHECK(it != tmpVec.end(), "pri_region_id values must be unique");
    }
  }
}
#endif

//! \}
