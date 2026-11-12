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

#include "TLibCommon/TComBitCounter.h"
#include "TLibCommon/TComBitStream.h"
#include "TLibCommon/SEI.h"
#include "TLibCommon/TComSlice.h"
#include "TLibCommon/TComPicYuv.h"
#include "SEIwrite.h"
#if JVET_AK0239_GEFV || JVET_AJ0207_GFV
#include <math.h>
#endif

//! \ingroup TLibEncoder
//! \{

#if ENC_DEC_TRACE
Void  SEIWriter::xTraceSEIHeader()
{
  fprintf( g_hTrace, "=========== SEI message ===========\n");
}

Void  SEIWriter::xTraceSEIMessageType(SEI::PayloadType payloadType)
{
  fprintf( g_hTrace, "=========== %s SEI message ===========\n", SEI::getSEIMessageString(payloadType));
}
#endif

Void SEIWriter::xWriteSEIpayloadData(TComBitIf& bs, const SEI& sei, const TComSPS *sps
#if JCTVC_AD0021_SEI_PREFIX_INDICATION
  , Int SEIPrefixIndicationIdx
#endif
)
{
  switch (sei.payloadType())
  {
  case SEI::BUFFERING_PERIOD:
    xWriteSEIBufferingPeriod(*static_cast<const SEIBufferingPeriod*>(&sei), sps);
    break;
  case SEI::PICTURE_TIMING:
    xWriteSEIPictureTiming(*static_cast<const SEIPictureTiming*>(&sei), sps);
    break;
  case SEI::PAN_SCAN_RECT:
    xWriteSEIPanScanRect(*static_cast<const SEIPanScanRect*>(&sei));
    break;
  case SEI::FILLER_PAYLOAD:
    xWriteSEIFillerPayload(*static_cast<const SEIFillerPayload*>(&sei));
    break;
  case SEI::USER_DATA_REGISTERED_ITU_T_T35:
    xWriteSEIUserDataRegistered(*static_cast<const SEIUserDataRegistered*>(&sei));
    break;
  case SEI::USER_DATA_UNREGISTERED:
    xWriteSEIUserDataUnregistered(*static_cast<const SEIUserDataUnregistered*>(&sei));
    break;
  case SEI::RECOVERY_POINT:
    xWriteSEIRecoveryPoint(*static_cast<const SEIRecoveryPoint*>(&sei));
    break;
  case SEI::SCENE_INFO:
    xWriteSEISceneInfo(*static_cast<const SEISceneInfo*>(&sei));
    break;
  case SEI::PICTURE_SNAPSHOT:
    xWriteSEIPictureSnapshot(*static_cast<const SEIPictureSnapshot*>(&sei));
    break;
  case SEI::PROGRESSIVE_REFINEMENT_SEGMENT_START:
    xWriteSEIProgressiveRefinementSegmentStart(*static_cast<const SEIProgressiveRefinementSegmentStart*>(&sei));
    break;
  case SEI::PROGRESSIVE_REFINEMENT_SEGMENT_END:
    xWriteSEIProgressiveRefinementSegmentEnd(*static_cast<const SEIProgressiveRefinementSegmentEnd*>(&sei));
    break;
  case SEI::FILM_GRAIN_CHARACTERISTICS:
    xWriteSEIFilmGrainCharacteristics(*static_cast<const SEIFilmGrainCharacteristics*>(&sei));
    break;
  case SEI::POST_FILTER_HINT:
    xWriteSEIPostFilterHint(*static_cast<const SEIPostFilterHint*>(&sei), sps);
    break;
  case SEI::TONE_MAPPING_INFO:
    xWriteSEIToneMappingInfo(*static_cast<const SEIToneMappingInfo*>(&sei));
    break;
  case SEI::FRAME_PACKING:
    xWriteSEIFramePacking(*static_cast<const SEIFramePacking*>(&sei)
#if JCTVC_AD0021_SEI_PREFIX_INDICATION
      , SEIPrefixIndicationIdx
#endif
    );
    break;
  case SEI::DISPLAY_ORIENTATION:
    xWriteSEIDisplayOrientation(*static_cast<const SEIDisplayOrientation*>(&sei));
    break;
  case SEI::GREEN_METADATA:
      xWriteSEIGreenMetadataInfo(*static_cast<const SEIGreenMetadataInfo*>(&sei));
    break;
  case SEI::SOP_DESCRIPTION:
    xWriteSEISOPDescription(*static_cast<const SEISOPDescription*>(&sei));
    break;
  case SEI::ACTIVE_PARAMETER_SETS:
    xWriteSEIActiveParameterSets(*static_cast<const SEIActiveParameterSets*>(& sei));
    break;
  case SEI::DECODING_UNIT_INFO:
    xWriteSEIDecodingUnitInfo(*static_cast<const SEIDecodingUnitInfo*>(& sei), sps);
    break;
  case SEI::TEMPORAL_LEVEL0_INDEX:
    xWriteSEITemporalLevel0Index(*static_cast<const SEITemporalLevel0Index*>(&sei));
    break;
  case SEI::DECODED_PICTURE_HASH:
    xWriteSEIDecodedPictureHash(*static_cast<const SEIDecodedPictureHash*>(&sei));
    break;
  case SEI::SCALABLE_NESTING:
    xWriteSEIScalableNesting(bs, *static_cast<const SEIScalableNesting*>(&sei), sps);
    break;
  case SEI::REGION_REFRESH_INFO:
    xWriteSEIRegionRefreshInfo(*static_cast<const SEIRegionRefreshInfo*>(&sei));
    break;
  case SEI::NO_DISPLAY:
    xWriteSEINoDisplay(*static_cast<const SEINoDisplay*>(&sei));
    break;
  case SEI::TIME_CODE:
    xWriteSEITimeCode(*static_cast<const SEITimeCode*>(&sei));
    break;
  case SEI::MASTERING_DISPLAY_COLOUR_VOLUME:
    xWriteSEIMasteringDisplayColourVolume(*static_cast<const SEIMasteringDisplayColourVolume*>(&sei));
    break;
  case SEI::SEGM_RECT_FRAME_PACKING:
    xWriteSEISegmentedRectFramePacking(*static_cast<const SEISegmentedRectFramePacking*>(&sei));
    break;
  case SEI::TEMP_MOTION_CONSTRAINED_TILE_SETS:
    xWriteSEITempMotionConstrainedTileSets(*static_cast<const SEITempMotionConstrainedTileSets*>(&sei));
    break;
#if MCTS_EXTRACTION
  case SEI::MCTS_EXTRACTION_INFO_SET:
    xWriteSEIMCTSExtractionInfoSet(*static_cast<const SEIMCTSExtractionInfoSet*>(&sei));
    break;
#endif
  case SEI::CHROMA_RESAMPLING_FILTER_HINT:
    xWriteSEIChromaResamplingFilterHint(*static_cast<const SEIChromaResamplingFilterHint*>(&sei));
    break;
  case SEI::KNEE_FUNCTION_INFO:
    xWriteSEIKneeFunctionInfo(*static_cast<const SEIKneeFunctionInfo*>(&sei));
    break;
  case SEI::COLOUR_REMAPPING_INFO:
    xWriteSEIColourRemappingInfo(*static_cast<const SEIColourRemappingInfo*>(&sei));
    break;
  case SEI::DEINTERLACE_FIELD_IDENTIFICATION:
    xWriteSEIDeinterlaceFieldIdentification(*static_cast<const SEIDeinterlaceFieldIdentification*>(&sei));
    break;
  case SEI::CONTENT_LIGHT_LEVEL_INFO:
    xWriteSEIContentLightLevelInfo(*static_cast<const SEIContentLightLevelInfo*>(&sei));
    break;
  case SEI::DEPENDENT_RAP_INDICATION:
    xWriteSEIDependentRAPIndication(*static_cast<const SEIDependentRAPIndication*>(&sei));
    break;
  case SEI::CODED_REGION_COMPLETION:
    xWriteSEICodedRegionCompletion(*static_cast<const SEICodedRegionCompletion*>(&sei));
    break;
  case SEI::ALTERNATIVE_TRANSFER_CHARACTERISTICS:
    xWriteSEIAlternativeTransferCharacteristics(*static_cast<const SEIAlternativeTransferCharacteristics*>(&sei));
    break;
  case SEI::AMBIENT_VIEWING_ENVIRONMENT:
    xWriteSEIAmbientViewingEnvironment(*static_cast<const SEIAmbientViewingEnvironment*>(&sei));
    break;
  case SEI::CONTENT_COLOUR_VOLUME:
    xWriteSEIContentColourVolume(*static_cast<const SEIContentColourVolume*>(&sei));
    break;
  case SEI::EQUIRECTANGULAR_PROJECTION:
    xWriteSEIEquirectangularProjection(*static_cast<const SEIEquirectangularProjection*>(&sei)
#if JCTVC_AD0021_SEI_PREFIX_INDICATION
      , SEIPrefixIndicationIdx
#endif   
    );
    break;
  case SEI::SPHERE_ROTATION:
    xWriteSEISphereRotation(*static_cast<const SEISphereRotation*>(&sei)
#if JCTVC_AD0021_SEI_PREFIX_INDICATION
      , SEIPrefixIndicationIdx
#endif 
    );
    break;
  case SEI::OMNI_VIEWPORT:
    xWriteSEIOmniViewport(*static_cast<const SEIOmniViewport*>(&sei));
    break;
  case SEI::CUBEMAP_PROJECTION:
    xWriteSEICubemapProjection(*static_cast<const SEICubemapProjection*>(&sei));
    break;
  case SEI::REGION_WISE_PACKING:
    xWriteSEIRegionWisePacking(*static_cast<const SEIRegionWisePacking*>(&sei)
#if JCTVC_AD0021_SEI_PREFIX_INDICATION
      , SEIPrefixIndicationIdx
#endif 
    );
    break;
  case SEI::FISHEYE_VIDEO_INFO:
    xWriteSEIFisheyeVideoInfo(*static_cast<const SEIFisheyeVideoInfo*>(&sei));
    break;
  case SEI::REGIONAL_NESTING:
    xWriteSEIRegionalNesting(bs, *static_cast<const SEIRegionalNesting*>(&sei), sps);
    break;

  case SEI::ANNOTATED_REGIONS:
    xWriteSEIAnnotatedRegions(*static_cast<const SEIAnnotatedRegions*>(&sei), sps);
    break;

#if SHUTTER_INTERVAL_SEI_MESSAGE
  case SEI::SHUTTER_INTERVAL_INFO:
    xWriteSEIShutterInterval(*static_cast<const SEIShutterIntervalInfo*>(&sei));
    break;
#endif
#if JCTVC_AD0021_SEI_MANIFEST
  case SEI::SEI_MANIFEST:
    xWriteSEISEIManifest(*static_cast<const SEIManifest*>(&sei));
    break;
#endif
#if JCTVC_AD0021_SEI_PREFIX_INDICATION
  case SEI::SEI_PREFIX_INDICATION:
    xWriteSEISEIPrefixIndication(bs, *static_cast<const SEIPrefixIndication*>(&sei), sps);
    break;
#endif
#if NNPFC_SEI_MESSAGE
  case SEI::NEURAL_NETWORK_POST_FILTER_CHARACTERISTICS:
    xWriteSEINeuralNetworkPostFilterCharacteristics(*static_cast<const SEINeuralNetworkPostFilterCharacteristics *>(&sei));
    break;
#endif
#if NNPFA_SEI_MESSAGE
  case SEI::NEURAL_NETWORK_POST_FILTER_ACTIVATION:
    xWriteSEINeuralNetworkPostFilterActivation(*static_cast<const SEINeuralNetworkPostFilterActivation *>(&sei));
    break;
#endif
#if JVET_AE0101_PHASE_INDICATION_SEI_MESSAGE
  case SEI::PayloadType::PHASE_INDICATION:
    xWriteSEIPhaseIndication(*static_cast<const SEIPhaseIndication *>(&sei));
    break;
#endif
#if JVET_AL0062_AI_USAGE_RESTRICTIONS_SEI
  case SEI::PayloadType::AI_USAGE_RESTRICTIONS:
    xWriteSEIAIUsageRestrictions(*static_cast<const SEIAIUsageRestrictions *>(&sei));
    break;
#endif  
#if JVET_AL0061_ENCODER_OPTIMIZATION_INFORMATION_SEI
  case SEI::ENCODER_OPTIMIZATION_INFO:
    xWriteSEIEncoderOptimizationInfo(*static_cast<const SEIEncoderOptimizationInfo *>(&sei));
    break;
#endif
#if JVET_AK0107_MODALITY_INFORMATION
  case SEI::PayloadType::MODALITY_INFORMATION:
    xWriteSEIModalityInfo(*static_cast<const SEIModalityInfo *>(&sei));
    break;
#endif 
#if JVET_AK0194_DSC_SEI
  case SEI::PayloadType::DIGITALLY_SIGNED_CONTENT_INITIALIZATION:
    xWriteSEIDigitallySignedContentInitialization(*static_cast<const SEIDigitallySignedContentInitialization *>(&sei));
    break;
  case SEI::PayloadType::DIGITALLY_SIGNED_CONTENT_SELECTION:
    xWriteSEIDigitallySignedContentSelection(*static_cast<const SEIDigitallySignedContentSelection *>(&sei));
    break;
  case SEI::PayloadType::DIGITALLY_SIGNED_CONTENT_VERIFICATION:
    xWriteSEIDigitallySignedContentVerification(*static_cast<const SEIDigitallySignedContentVerification *>(&sei));
    break;
#endif
#if JVET_AJ0207_GFV
  case SEI::PayloadType::GENERATIVE_FACE_VIDEO:
    xWriteSEIGenerativeFaceVideo(*static_cast<const SEIGenerativeFaceVideo*>(&sei));
    break;
#endif
#if JVET_AK0239_GEFV
  case SEI::PayloadType::GENERATIVE_ENHANCEMENT_FACE_VIDEO:
    xWriteSEIGenerativeFaceVideoEnhancement(*static_cast<const SEIGenerativeFaceVideoEnhancement*>(&sei));
    break;
#endif
#if JVET_AK0140_PACKED_REGIONS_INFORMATION_SEI
  case SEI::PayloadType::PACKED_REGIONS_INFO:
    xWriteSEIPackedRegionsInfo(*static_cast<const SEIPackedRegionsInfo*>(&sei));
    break;
#endif
#if JVET_AK2006_SPTI_SEI_MESSAGE
  case SEI::PayloadType::SOURCE_PICTURE_TIMING_INFO:
    xWriteSEISourcePictureTimingInfo(*static_cast<const SEISourcePictureTimingInfo *>(&sei));
#endif
  default:
    assert(!"Trying to write unhandled SEI message");
    break;
  }
#if JCTVC_AD0021_SEI_PREFIX_INDICATION
  if (SEIPrefixIndicationIdx)
  {
    return;
  }
#endif
  xWriteByteAlign();
}

/**
 * marshal all SEI messages in provided list into one bitstream bs
 */
Void SEIWriter::writeSEImessages(TComBitIf& bs, const SEIMessages &seiList, const TComSPS *sps, Bool isNested)
{
#if ENC_DEC_TRACE
  if (g_HLSTraceEnable)
    xTraceSEIHeader();
#endif

  TComBitCounter bs_count;

  for (SEIMessages::const_iterator sei=seiList.begin(); sei!=seiList.end(); sei++)
  {
    xWriteSEImessage(bs, *sei, sps);
  }
  if (!isNested)
  {
    xWriteRbspTrailingBits();
  }
}

Void SEIWriter::xWriteSEImessage(TComBitIf& bs, const SEI *sei, const TComSPS *sps)
{
#if ENC_DEC_TRACE
  if (g_HLSTraceEnable)
    xTraceSEIHeader();
#endif

  TComBitCounter bs_count;

  // calculate how large the payload data is
  // TODO: this would be far nicer if it used vectored buffers
  bs_count.resetBits();
  setBitstream(&bs_count);

#if ENC_DEC_TRACE
  Bool traceEnable = g_HLSTraceEnable;
  g_HLSTraceEnable = false;
#endif
  xWriteSEIpayloadData(bs_count, *sei, sps);
#if ENC_DEC_TRACE
  g_HLSTraceEnable = traceEnable;
#endif
  UInt payload_data_num_bits = bs_count.getNumberOfWrittenBits();
  assert(0 == payload_data_num_bits % 8);

  setBitstream(&bs);
  UInt payloadType = sei->payloadType();
  for (; payloadType >= 0xff; payloadType -= 0xff)
  {
    WRITE_CODE(0xff, 8, "payload_type");
  }
  WRITE_CODE(payloadType, 8, "payload_type");

  UInt payloadSize = payload_data_num_bits/8;
  for (; payloadSize >= 0xff; payloadSize -= 0xff)
  {
    WRITE_CODE(0xff, 8, "payload_size");
  }
  WRITE_CODE(payloadSize, 8, "payload_size");

  /* payloadData */
#if ENC_DEC_TRACE
  if (g_HLSTraceEnable)
#if MCTS_EXTRACTION //seems like a leftover copy-paste bug
    xTraceSEIMessageType(sei->payloadType());
#else
    //xTraceSEIMessageType((*sei)->payloadType());
#endif

#endif

  xWriteSEIpayloadData(bs, *sei, sps);
}

Void SEIWriter::xWriteSEIBufferingPeriod(const SEIBufferingPeriod& sei, const TComSPS *sps)
{
  Int i, nalOrVcl;
  const TComVUI *vui = sps->getVuiParameters();
  const TComHRD *hrd = vui->getHrdParameters();

  WRITE_UVLC( sei.m_bpSeqParameterSetId, "bp_seq_parameter_set_id" );
  if( !hrd->getSubPicCpbParamsPresentFlag() )
  {
    WRITE_FLAG( sei.m_rapCpbParamsPresentFlag, "irap_cpb_params_present_flag" );
  }
  if( sei.m_rapCpbParamsPresentFlag )
  {
    WRITE_CODE( sei.m_cpbDelayOffset, hrd->getCpbRemovalDelayLengthMinus1() + 1, "cpb_delay_offset" );
    WRITE_CODE( sei.m_dpbDelayOffset, hrd->getDpbOutputDelayLengthMinus1()  + 1, "dpb_delay_offset" );
  }
  WRITE_FLAG( sei.m_concatenationFlag, "concatenation_flag");
  WRITE_CODE( sei.m_auCpbRemovalDelayDelta - 1, ( hrd->getCpbRemovalDelayLengthMinus1() + 1 ), "au_cpb_removal_delay_delta_minus1" );
  for( nalOrVcl = 0; nalOrVcl < 2; nalOrVcl ++ )
  {
    if( ( ( nalOrVcl == 0 ) && ( hrd->getNalHrdParametersPresentFlag() ) ) ||
        ( ( nalOrVcl == 1 ) && ( hrd->getVclHrdParametersPresentFlag() ) ) )
    {
      for( i = 0; i < ( hrd->getCpbCntMinus1( 0 ) + 1 ); i ++ )
      {
        WRITE_CODE( sei.m_initialCpbRemovalDelay[i][nalOrVcl],( hrd->getInitialCpbRemovalDelayLengthMinus1() + 1 ) ,           "initial_cpb_removal_delay" );
        WRITE_CODE( sei.m_initialCpbRemovalDelayOffset[i][nalOrVcl],( hrd->getInitialCpbRemovalDelayLengthMinus1() + 1 ),      "initial_cpb_removal_delay_offset" );
        if( hrd->getSubPicCpbParamsPresentFlag() || sei.m_rapCpbParamsPresentFlag )
        {
          WRITE_CODE( sei.m_initialAltCpbRemovalDelay[i][nalOrVcl], ( hrd->getInitialCpbRemovalDelayLengthMinus1() + 1 ) ,     "initial_alt_cpb_removal_delay" );
          WRITE_CODE( sei.m_initialAltCpbRemovalDelayOffset[i][nalOrVcl], ( hrd->getInitialCpbRemovalDelayLengthMinus1() + 1 ),"initial_alt_cpb_removal_delay_offset" );
        }
      }
    }
  }
}


Void SEIWriter::xWriteSEIPictureTiming(const SEIPictureTiming& sei, const TComSPS *sps)
{
  Int i;
  const TComVUI *vui = sps->getVuiParameters();
  const TComHRD *hrd = vui->getHrdParameters();

  if( vui->getFrameFieldInfoPresentFlag() )
  {
    WRITE_CODE( sei.m_picStruct, 4,              "pic_struct" );
    WRITE_CODE( sei.m_sourceScanType, 2,         "source_scan_type" );
    WRITE_FLAG( sei.m_duplicateFlag ? 1 : 0,     "duplicate_flag" );
  }

  if( hrd->getCpbDpbDelaysPresentFlag() )
  {
    WRITE_CODE( sei.m_auCpbRemovalDelay - 1, ( hrd->getCpbRemovalDelayLengthMinus1() + 1 ),                                         "au_cpb_removal_delay_minus1" );
    WRITE_CODE( sei.m_picDpbOutputDelay, ( hrd->getDpbOutputDelayLengthMinus1() + 1 ),                                          "pic_dpb_output_delay" );
    if(hrd->getSubPicCpbParamsPresentFlag())
    {
      WRITE_CODE(sei.m_picDpbOutputDuDelay, hrd->getDpbOutputDelayDuLengthMinus1()+1, "pic_dpb_output_du_delay" );
    }
    if( hrd->getSubPicCpbParamsPresentFlag() && hrd->getSubPicCpbParamsInPicTimingSEIFlag() )
    {
      WRITE_UVLC( sei.m_numDecodingUnitsMinus1,     "num_decoding_units_minus1" );
      WRITE_FLAG( sei.m_duCommonCpbRemovalDelayFlag, "du_common_cpb_removal_delay_flag" );
      if( sei.m_duCommonCpbRemovalDelayFlag )
      {
        WRITE_CODE( sei.m_duCommonCpbRemovalDelayMinus1, ( hrd->getDuCpbRemovalDelayLengthMinus1() + 1 ),                       "du_common_cpb_removal_delay_minus1" );
      }
      for( i = 0; i <= sei.m_numDecodingUnitsMinus1; i ++ )
      {
        WRITE_UVLC( sei.m_numNalusInDuMinus1[ i ],  "num_nalus_in_du_minus1");
        if( ( !sei.m_duCommonCpbRemovalDelayFlag ) && ( i < sei.m_numDecodingUnitsMinus1 ) )
        {
          WRITE_CODE( sei.m_duCpbRemovalDelayMinus1[ i ], ( hrd->getDuCpbRemovalDelayLengthMinus1() + 1 ),                        "du_cpb_removal_delay_minus1" );
        }
      }
    }
  }
}


Void SEIWriter::xWriteSEIPanScanRect(const SEIPanScanRect &sei)
{
  WRITE_UVLC( sei.m_panScanRectId,         "pan_scan_rect_id" );
  const UInt numRegions = (UInt) sei.m_panScanRectRegions.size();
  if ( !sei.m_panScanRectCancelFlag && numRegions>0 )
  {
    WRITE_FLAG( sei.m_panScanRectCancelFlag, "pan_scan_rect_cancel_flag" );
    WRITE_UVLC( numRegions - 1, "pan_scan_cnt_minus1" );
    for(UInt region=0; region<numRegions; region++)
    {
      const SEIPanScanRect::PanScanRect &rect=sei.m_panScanRectRegions[region];
      WRITE_SVLC( rect.leftOffset,   "pan_scan_rect_left_offset[i]"   );
      WRITE_SVLC( rect.rightOffset,  "pan_scan_rect_rioht_offset[i]"  );
      WRITE_SVLC( rect.topOffset,    "pan_scan_rect_top_offset[i]"    );
      WRITE_SVLC( rect.bottomOffset, "pan_scan_rect_bottom_offset[i]" );
    }
    WRITE_FLAG( sei.m_panScanRectPersistenceFlag, "pan_scan_rect_persistence_flag" );
  }
}


Void SEIWriter::xWriteSEIFillerPayload(const SEIFillerPayload &sei)
{
  for(UInt k=0; k<sei.m_numFillerFFBytes; k++)
  {
    WRITE_CODE( 0xff, 8, "ff_byte" );
  }
}


Void SEIWriter::xWriteSEIUserDataRegistered(const SEIUserDataRegistered &sei)
{
  WRITE_CODE( (sei.m_ituCountryCode>255) ? 0xff : sei.m_ituCountryCode, 8, "itu_t_t35_country_code" );
  if (sei.m_ituCountryCode>=255)
  {
    assert(sei.m_ituCountryCode < 255+256);
    WRITE_CODE( sei.m_ituCountryCode-255, 8, "itu_t_t35_country_code_extension_byte" );
  }
  for(UInt i=0; i<sei.m_userData.size(); i++)
  {
    WRITE_CODE( sei.m_userData[i], 8, "itu_t_t35_payload_byte" );
  }
}


Void SEIWriter::xWriteSEIUserDataUnregistered(const SEIUserDataUnregistered &sei)
{
  for (UInt i = 0; i < ISO_IEC_11578_LEN; i++)
  {
    WRITE_CODE(sei.m_uuid_iso_iec_11578[i], 8 , "sei.uuid_iso_iec_11578[i]");
  }

  for (std::size_t i = 0; i < sei.m_userData.size(); i++)
  {
    WRITE_CODE(sei.m_userData[i], 8 , "user_data");
  }
}


Void SEIWriter::xWriteSEIRecoveryPoint(const SEIRecoveryPoint& sei)
{
  WRITE_SVLC( sei.m_recoveryPocCnt,    "recovery_poc_cnt"    );
  WRITE_FLAG( sei.m_exactMatchingFlag, "exact_matching_flag" );
  WRITE_FLAG( sei.m_brokenLinkFlag,    "broken_link_flag"    );
}


Void SEIWriter::xWriteSEISceneInfo(const SEISceneInfo &sei)
{
  WRITE_FLAG( sei.m_bSceneInfoPresentFlag, "scene_info_present_flag" );
  if (sei.m_bSceneInfoPresentFlag)
  {
    WRITE_FLAG( sei.m_bPrevSceneIdValidFlag, "prev_scene_id_valid_flag" );
    WRITE_UVLC( sei.m_sceneId,               "scene_id" );
    WRITE_UVLC( sei.m_sceneTransitionType,   "scene_transition_type" );
    if (sei.m_sceneTransitionType > 3)
    {
      WRITE_UVLC( sei.m_secondSceneId,       "second_scene_id" );
    }
  }
}


Void SEIWriter::xWriteSEIPictureSnapshot(const SEIPictureSnapshot &sei)
{
  WRITE_UVLC( sei.m_snapshotId, "snapshot_id" );
}


Void SEIWriter::xWriteSEIProgressiveRefinementSegmentStart(const SEIProgressiveRefinementSegmentStart &sei)
{
  WRITE_UVLC( sei.m_progressiveRefinementId, "progressive_refinement_id" );
  WRITE_UVLC( sei.m_picOrderCntDelta,        "pic_order_cnt_delta"       );
}


Void SEIWriter::xWriteSEIProgressiveRefinementSegmentEnd(const SEIProgressiveRefinementSegmentEnd &sei)
{
  WRITE_UVLC( sei.m_progressiveRefinementId, "progressive_refinement_id" );
}


Void SEIWriter::xWriteSEIFilmGrainCharacteristics(const SEIFilmGrainCharacteristics &sei)
{
  WRITE_FLAG( sei.m_filmGrainCharacteristicsCancelFlag, "film_grain_characteristics_cancel_flag" );
  if (!sei.m_filmGrainCharacteristicsCancelFlag)
  {
    WRITE_CODE( sei.m_filmGrainModelId, 2,                  "film_grain_model_id" );
    WRITE_FLAG( sei.m_separateColourDescriptionPresentFlag, "separate_colour_description_present_flag" );
    if (sei.m_separateColourDescriptionPresentFlag)
    {
      WRITE_CODE( sei.m_filmGrainBitDepthLumaMinus8,      3, "film_grain_bit_depth_luma_minus8" );
      WRITE_CODE( sei.m_filmGrainBitDepthChromaMinus8,    3, "film_grain_bit_depth_chroma_minus8" );
      WRITE_FLAG( sei.m_filmGrainFullRangeFlag,              "film_grain_full_range_flag" );
      WRITE_CODE( sei.m_filmGrainColourPrimaries,         8, "film_grain_colour_primaries" );
      WRITE_CODE( sei.m_filmGrainTransferCharacteristics, 8, "film_grain_transfer_characteristics" );
      WRITE_CODE( sei.m_filmGrainMatrixCoeffs,            8, "film_grain_matrix_coeffs" );
    }
    WRITE_CODE( sei.m_blendingModeId,  2, "blending_mode_id" );
    WRITE_CODE( sei.m_log2ScaleFactor, 4, "log2_scale_factor" );
    for(Int c=0; c<3; c++)
    {
      const SEIFilmGrainCharacteristics::CompModel &cm=sei.m_compModel[c];
#if JVET_X0048_X0103_FILM_GRAIN
      const UInt numIntensityIntervals = (UInt) cm.numIntensityIntervals;
#else
      const UInt numIntensityIntervals = (UInt) cm.intensityValues.size();
#endif
      const UInt numModelValues        = cm.numModelValues;
      WRITE_FLAG( sei.m_compModel[c].bPresentFlag && numIntensityIntervals>0 && numModelValues>0, "comp_model_present_flag[c]" );
    }
    for(Int c=0; c<3; c++)
    {
      const SEIFilmGrainCharacteristics::CompModel &cm=sei.m_compModel[c];
#if JVET_X0048_X0103_FILM_GRAIN
      const UInt numIntensityIntervals = (UInt) cm.numIntensityIntervals;
#else
      const UInt numIntensityIntervals = (UInt) cm.intensityValues.size();
#endif
      const UInt numModelValues        = cm.numModelValues;
      if (cm.bPresentFlag && numIntensityIntervals>0 && numModelValues>0)
      {
        assert(numIntensityIntervals<=256);
        assert(numModelValues<=8);
        WRITE_CODE( numIntensityIntervals-1, 8, "num_intensity_intervals_minus1[c]");
        WRITE_CODE( numModelValues-1,        3, "num_model_values_minus1[c]");
        for(UInt interval=0; interval<numIntensityIntervals; interval++)
        {
          const SEIFilmGrainCharacteristics::CompModelIntensityValues &cmiv=cm.intensityValues[interval];
          WRITE_CODE( cmiv.intensityIntervalLowerBound, 8, "intensity_interval_lower_bound[c][i]" );
          WRITE_CODE( cmiv.intensityIntervalUpperBound, 8, "intensity_interval_upper_bound[c][i]" );
#if !JVET_X0048_X0103_FILM_GRAIN
          assert(cmiv.compModelValue.size() == numModelValues);
#endif
          for(UInt j=0; j<cm.numModelValues; j++)
          {
            WRITE_SVLC(cmiv.compModelValue[j], "comp_model_value[c][i]" );
          }
        }
      }
    } // for c
    WRITE_FLAG( sei.m_filmGrainCharacteristicsPersistenceFlag, "film_grain_characteristics_persistence_flag" );
#if JVET_AL0339_SPATIAL_RESOLUTION_FOR_FGC_SEI
    WRITE_FLAG(sei.m_fgSpatialResolutionPresentFlag, "film_grain_spatial_resolution_present_flag");
    if (sei.m_fgSpatialResolutionPresentFlag)
    {
      WRITE_UVLC(sei.m_fgPicWidthInLumaSamples, "fg_pic_width_in_luma_samples");
      WRITE_UVLC(sei.m_fgPicHeightInLumaSamples, "fg_pic_height_in_luma_samples");
      m_SeiExtensionBitsPresentFlag = true;
    }
#endif
  } // cancel flag
}


Void SEIWriter::xWriteSEIPostFilterHint(const SEIPostFilterHint& sei, const TComSPS *sps)
{
  WRITE_UVLC( sei.m_filterHintSizeY,   "filter_hint_size_y" );
  WRITE_UVLC( sei.m_filterHintSizeX,   "filter_hint_size_x" );
  WRITE_CODE( sei.m_filterHintType, 2, "filter_hint_type"   );
  assert( (sps->getChromaFormatIdc() == CHROMA_400) == sei.m_bIsMonochrome );
  const UInt numChromaChannels = sei.m_bIsMonochrome ? 1:3;
  assert( sei.m_filterHintValues.size() == numChromaChannels*sei.m_filterHintSizeX*sei.m_filterHintSizeY );
  for(std::size_t i=0; i<sei.m_filterHintValues.size(); i++)
  {
    WRITE_SVLC( sei.m_filterHintValues[i], "filter_hint_value[][][]" );
  }
}


Void SEIWriter::xWriteSEIToneMappingInfo(const SEIToneMappingInfo& sei)
{
  Int i;
  WRITE_UVLC( sei.m_toneMapId,                    "tone_map_id" );
  WRITE_FLAG( sei.m_toneMapCancelFlag,            "tone_map_cancel_flag" );
  if( !sei.m_toneMapCancelFlag )
  {
    WRITE_FLAG( sei.m_toneMapPersistenceFlag,     "tone_map_persistence_flag" );
    WRITE_CODE( sei.m_codedDataBitDepth,    8,    "coded_data_bit_depth" );
    WRITE_CODE( sei.m_targetBitDepth,       8,    "target_bit_depth" );
    WRITE_UVLC( sei.m_modelId,                    "model_id" );
    switch(sei.m_modelId)
    {
    case 0:
      {
        WRITE_CODE( sei.m_minValue,  32,        "min_value" );
        WRITE_CODE( sei.m_maxValue, 32,         "max_value" );
        break;
      }
    case 1:
      {
        WRITE_CODE( sei.m_sigmoidMidpoint, 32,  "sigmoid_midpoint" );
        WRITE_CODE( sei.m_sigmoidWidth,    32,  "sigmoid_width"    );
        break;
      }
    case 2:
      {
        UInt num = 1u << sei.m_targetBitDepth;
        for(i = 0; i < num; i++)
        {
          WRITE_CODE( sei.m_startOfCodedInterval[i], (( sei.m_codedDataBitDepth + 7 ) >> 3 ) << 3,  "start_of_coded_interval" );
        }
        break;
      }
    case 3:
      {
        WRITE_CODE( sei.m_numPivots, 16,          "num_pivots" );
        for(i = 0; i < sei.m_numPivots; i++ )
        {
          WRITE_CODE( sei.m_codedPivotValue[i], (( sei.m_codedDataBitDepth + 7 ) >> 3 ) << 3,       "coded_pivot_value" );
          WRITE_CODE( sei.m_targetPivotValue[i], (( sei.m_targetBitDepth + 7 ) >> 3 ) << 3,         "target_pivot_value");
        }
        break;
      }
    case 4:
      {
        WRITE_CODE( sei.m_cameraIsoSpeedIdc,    8,    "camera_iso_speed_idc" );
        if( sei.m_cameraIsoSpeedIdc == 255) //Extended_ISO
        {
          WRITE_CODE( sei.m_cameraIsoSpeedValue,    32,    "camera_iso_speed_value" );
        }
        WRITE_CODE( sei.m_exposureIndexIdc,     8,    "exposure_index_idc" );
        if( sei.m_exposureIndexIdc == 255) //Extended_ISO
        {
          WRITE_CODE( sei.m_exposureIndexValue,     32,    "exposure_index_value" );
        }
        WRITE_FLAG( sei.m_exposureCompensationValueSignFlag,           "exposure_compensation_value_sign_flag" );
        WRITE_CODE( sei.m_exposureCompensationValueNumerator,     16,  "exposure_compensation_value_numerator" );
        WRITE_CODE( sei.m_exposureCompensationValueDenomIdc,      16,  "exposure_compensation_value_denom_idc" );
        WRITE_CODE( sei.m_refScreenLuminanceWhite,                32,  "ref_screen_luminance_white" );
        WRITE_CODE( sei.m_extendedRangeWhiteLevel,                32,  "extended_range_white_level" );
        WRITE_CODE( sei.m_nominalBlackLevelLumaCodeValue,         16,  "nominal_black_level_luma_code_value" );
        WRITE_CODE( sei.m_nominalWhiteLevelLumaCodeValue,         16,  "nominal_white_level_luma_code_value" );
        WRITE_CODE( sei.m_extendedWhiteLevelLumaCodeValue,        16,  "extended_white_level_luma_code_value" );
        break;
      }
    default:
      {
        assert(!"Undefined SEIToneMapModelId");
        break;
      }
    }//switch m_modelId
  }//if(!sei.m_toneMapCancelFlag)
}


Void SEIWriter::xWriteSEIFramePacking(const SEIFramePacking& sei
#if JCTVC_AD0021_SEI_PREFIX_INDICATION
  , Int SEIPrefixIndicationIdx
#endif
)
{
#if JCTVC_AD0021_SEI_PREFIX_INDICATION
  if (SEIPrefixIndicationIdx)
  {
    int numBits = 0;
    numBits += getBitsUe(sei.m_arrangementId);
    if (!sei.m_arrangementCancelFlag)
    {
      numBits += 9;
    }
    else
    {
      numBits += 2;
    }
    WRITE_CODE(numBits - 1, 16, "num_bits_in_prefix_indication_minus1");
  }
#endif 
  WRITE_UVLC( sei.m_arrangementId,                  "frame_packing_arrangement_id" );
  WRITE_FLAG( sei.m_arrangementCancelFlag,          "frame_packing_arrangement_cancel_flag" );

  if( sei.m_arrangementCancelFlag == 0 )
  {
    WRITE_CODE( sei.m_arrangementType, 7,           "frame_packing_arrangement_type" );
#if JCTVC_AD0021_SEI_PREFIX_INDICATION
    if (SEIPrefixIndicationIdx)
    {
      return;
    }
#endif
    WRITE_FLAG( sei.m_quincunxSamplingFlag,         "quincunx_sampling_flag" );
    WRITE_CODE( sei.m_contentInterpretationType, 6, "content_interpretation_type" );
    WRITE_FLAG( sei.m_spatialFlippingFlag,          "spatial_flipping_flag" );
    WRITE_FLAG( sei.m_frame0FlippedFlag,            "frame0_flipped_flag" );
    WRITE_FLAG( sei.m_fieldViewsFlag,               "field_views_flag" );
    WRITE_FLAG( sei.m_currentFrameIsFrame0Flag,     "current_frame_is_frame0_flag" );

    WRITE_FLAG( sei.m_frame0SelfContainedFlag,      "frame0_self_contained_flag" );
    WRITE_FLAG( sei.m_frame1SelfContainedFlag,      "frame1_self_contained_flag" );

    if(sei.m_quincunxSamplingFlag == 0 && sei.m_arrangementType != 5)
    {
      WRITE_CODE( sei.m_frame0GridPositionX, 4,     "frame0_grid_position_x" );
      WRITE_CODE( sei.m_frame0GridPositionY, 4,     "frame0_grid_position_y" );
      WRITE_CODE( sei.m_frame1GridPositionX, 4,     "frame1_grid_position_x" );
      WRITE_CODE( sei.m_frame1GridPositionY, 4,     "frame1_grid_position_y" );
    }

    WRITE_CODE( sei.m_arrangementReservedByte, 8,   "frame_packing_arrangement_reserved_byte" );
    WRITE_FLAG( sei.m_arrangementPersistenceFlag,   "frame_packing_arrangement_persistence_flag" );
  }

  WRITE_FLAG( sei.m_upsampledAspectRatio,           "upsampled_aspect_ratio" );
}


Void SEIWriter::xWriteSEIDisplayOrientation(const SEIDisplayOrientation &sei)
{
  WRITE_FLAG( sei.cancelFlag,           "display_orientation_cancel_flag" );
  if( !sei.cancelFlag )
  {
    WRITE_FLAG( sei.horFlip,                   "hor_flip" );
    WRITE_FLAG( sei.verFlip,                   "ver_flip" );
    WRITE_CODE( sei.anticlockwiseRotation, 16, "anticlockwise_rotation" );
    WRITE_FLAG( sei.persistenceFlag,          "display_orientation_persistence_flag" );
  }
}


Void SEIWriter::xWriteSEIGreenMetadataInfo(const SEIGreenMetadataInfo& sei)
{
  WRITE_CODE(sei.m_greenMetadataType, 8, "green_metadata_type");

  WRITE_CODE(sei.m_xsdMetricType, 8, "xsd_metric_type");
  WRITE_CODE(sei.m_xsdMetricValue, 16, "xsd_metric_value");
}


Void SEIWriter::xWriteSEISOPDescription(const SEISOPDescription& sei)
{
  WRITE_UVLC( sei.m_sopSeqParameterSetId,           "sop_seq_parameter_set_id"               );
  WRITE_UVLC( sei.m_numPicsInSopMinus1,             "num_pics_in_sop_minus1"               );
  for (UInt i = 0; i <= sei.m_numPicsInSopMinus1; i++)
  {
    WRITE_CODE( sei.m_sopDescVclNaluType[i], 6, "sop_desc_vcl_nalu_type" );
    WRITE_CODE( sei.m_sopDescTemporalId[i],  3, "sop_desc_temporal_id" );
    if (sei.m_sopDescVclNaluType[i] != NAL_UNIT_CODED_SLICE_IDR_W_RADL && sei.m_sopDescVclNaluType[i] != NAL_UNIT_CODED_SLICE_IDR_N_LP)
    {
      WRITE_UVLC( sei.m_sopDescStRpsIdx[i],           "sop_desc_st_rps_idx"               );
    }
    if (i > 0)
    {
      WRITE_SVLC( sei.m_sopDescPocDelta[i],           "sop_desc_poc_delta"               );
    }
  }
}


Void SEIWriter::xWriteSEIActiveParameterSets(const SEIActiveParameterSets& sei)
{
  WRITE_CODE(sei.activeVPSId,     4,         "active_video_parameter_set_id");
  WRITE_FLAG(sei.m_selfContainedCvsFlag,     "self_contained_cvs_flag");
  WRITE_FLAG(sei.m_noParameterSetUpdateFlag, "no_parameter_set_update_flag");
  WRITE_UVLC(sei.numSpsIdsMinus1,            "num_sps_ids_minus1");

  assert (sei.activeSeqParameterSetId.size() == (sei.numSpsIdsMinus1 + 1));

  for (Int i = 0; i < sei.activeSeqParameterSetId.size(); i++)
  {
    WRITE_UVLC(sei.activeSeqParameterSetId[i], "active_seq_parameter_set_id");
  }
}


Void SEIWriter::xWriteSEIDecodingUnitInfo(const SEIDecodingUnitInfo& sei, const TComSPS *sps)
{
  const TComVUI *vui = sps->getVuiParameters();
  WRITE_UVLC(sei.m_decodingUnitIdx, "decoding_unit_idx");
  if(vui->getHrdParameters()->getSubPicCpbParamsInPicTimingSEIFlag())
  {
    WRITE_CODE( sei.m_duSptCpbRemovalDelay, (vui->getHrdParameters()->getDuCpbRemovalDelayLengthMinus1() + 1), "du_spt_cpb_removal_delay_increment");
  }
  WRITE_FLAG( sei.m_dpbOutputDuDelayPresentFlag, "dpb_output_du_delay_present_flag");
  if(sei.m_dpbOutputDuDelayPresentFlag)
  {
    WRITE_CODE(sei.m_picSptDpbOutputDuDelay, vui->getHrdParameters()->getDpbOutputDelayDuLengthMinus1() + 1, "pic_spt_dpb_output_du_delay");
  }
}


Void SEIWriter::xWriteSEITemporalLevel0Index(const SEITemporalLevel0Index &sei)
{
  WRITE_CODE( sei.tl0Idx, 8 , "tl0_idx" );
  WRITE_CODE( sei.rapIdx, 8 , "rap_idx" );
}


Void SEIWriter::xWriteSEIDecodedPictureHash(const SEIDecodedPictureHash& sei)
{
  const TChar *traceString="\0";
  switch (sei.method)
  {
    case HASHTYPE_MD5: traceString="picture_md5"; break;
    case HASHTYPE_CRC: traceString="picture_crc"; break;
    case HASHTYPE_CHECKSUM: traceString="picture_checksum"; break;
    default: assert(false); break;
  }

  if (traceString != 0) //use of this variable is needed to avoid a compiler error with G++ 4.6.1
  {
    WRITE_CODE(sei.method, 8, "hash_type");
    for(UInt i=0; i<UInt(sei.m_pictureHash.hash.size()); i++)
    {
      WRITE_CODE(sei.m_pictureHash.hash[i], 8, traceString);
    }
  }
}


Void SEIWriter::xWriteSEIScalableNesting(TComBitIf& bs, const SEIScalableNesting& sei, const TComSPS *sps)
{
  WRITE_FLAG( sei.m_bitStreamSubsetFlag,             "bitstream_subset_flag"         );
  WRITE_FLAG( sei.m_nestingOpFlag,                   "nesting_op_flag      "         );
  if (sei.m_nestingOpFlag)
  {
    WRITE_FLAG( sei.m_defaultOpFlag,                 "default_op_flag"               );
    WRITE_UVLC( sei.m_nestingNumOpsMinus1,           "nesting_num_ops_minus1"        );
    for (UInt i = (sei.m_defaultOpFlag ? 1 : 0); i <= sei.m_nestingNumOpsMinus1; i++)
    {
      WRITE_CODE( sei.m_nestingMaxTemporalIdPlus1[i], 3,  "nesting_max_temporal_id_plus1" );
      WRITE_UVLC( sei.m_nestingOpIdx[i],                  "nesting_op_idx"                );
    }
  }
  else
  {
    WRITE_FLAG( sei.m_allLayersFlag,                      "all_layers_flag"               );
    if (!sei.m_allLayersFlag)
    {
      WRITE_CODE( sei.m_nestingNoOpMaxTemporalIdPlus1, 3, "nesting_no_op_max_temporal_id_plus1" );
      WRITE_UVLC( sei.m_nestingNumLayersMinus1,           "nesting_num_layers"                  );
      for (UInt i = 0; i <= sei.m_nestingNumLayersMinus1; i++)
      {
        WRITE_CODE( sei.m_nestingLayerId[i], 6,           "nesting_layer_id"              );
      }
    }
  }

  // byte alignment
  while ( m_pcBitIf->getNumberOfWrittenBits() % 8 != 0 )
  {
    WRITE_FLAG( 0, "nesting_zero_bit" );
  }

  // write nested SEI messages
  writeSEImessages(bs, sei.m_nestedSEIs, sps, true);
}


Void SEIWriter::xWriteSEIRegionRefreshInfo(const SEIRegionRefreshInfo &sei)
{
  WRITE_FLAG( sei.m_gdrForegroundFlag, "gdr_foreground_flag");
}


Void SEIWriter::xWriteSEINoDisplay(const SEINoDisplay& /*sei*/)
{
  // intentionally empty
}


Void SEIWriter::xWriteSEITimeCode(const SEITimeCode& sei)
{
  WRITE_CODE(sei.numClockTs, 2, "num_clock_ts");
  for(Int i = 0; i < sei.numClockTs; i++)
  {
    const TComSEITimeSet &currentTimeSet = sei.timeSetArray[i];
    WRITE_FLAG(currentTimeSet.clockTimeStampFlag, "clock_time_stamp_flag");
    if(currentTimeSet.clockTimeStampFlag)
    {
      WRITE_FLAG(currentTimeSet.numUnitFieldBasedFlag, "units_field_based_flag");
      WRITE_CODE(currentTimeSet.countingType, 5, "counting_type");
      WRITE_FLAG(currentTimeSet.fullTimeStampFlag, "full_timestamp_flag");
      WRITE_FLAG(currentTimeSet.discontinuityFlag, "discontinuity_flag");
      WRITE_FLAG(currentTimeSet.cntDroppedFlag, "cnt_dropped_flag");
      WRITE_CODE(currentTimeSet.numberOfFrames, 9, "n_frames");
      if(currentTimeSet.fullTimeStampFlag)
      {
        WRITE_CODE(currentTimeSet.secondsValue, 6, "seconds_value");
        WRITE_CODE(currentTimeSet.minutesValue, 6, "minutes_value");
        WRITE_CODE(currentTimeSet.hoursValue, 5, "hours_value");
      }
      else
      {
        WRITE_FLAG(currentTimeSet.secondsFlag, "seconds_flag");
        if(currentTimeSet.secondsFlag)
        {
          WRITE_CODE(currentTimeSet.secondsValue, 6, "seconds_value");
          WRITE_FLAG(currentTimeSet.minutesFlag, "minutes_flag");
          if(currentTimeSet.minutesFlag)
          {
            WRITE_CODE(currentTimeSet.minutesValue, 6, "minutes_value");
            WRITE_FLAG(currentTimeSet.hoursFlag, "hours_flag");
            if(currentTimeSet.hoursFlag)
            {
              WRITE_CODE(currentTimeSet.hoursValue, 5, "hours_value");
            }
          }
        }
      }
      WRITE_CODE(currentTimeSet.timeOffsetLength, 5, "time_offset_length");
      if(currentTimeSet.timeOffsetLength > 0)
      {
        if(currentTimeSet.timeOffsetValue >= 0)
        {
          WRITE_CODE((UInt)currentTimeSet.timeOffsetValue, currentTimeSet.timeOffsetLength, "time_offset_value");
        }
        else
        {
          //  Two's complement conversion
          UInt offsetValue = ~(currentTimeSet.timeOffsetValue) + 1;
          offsetValue |= (1 << (currentTimeSet.timeOffsetLength-1));
          WRITE_CODE(offsetValue, currentTimeSet.timeOffsetLength, "time_offset_value");
        }
      }
    }
  }
}


Void SEIWriter::xWriteSEIMasteringDisplayColourVolume(const SEIMasteringDisplayColourVolume& sei)
{
  WRITE_CODE( sei.values.primaries[0][0],  16,  "display_primaries_x[0]" );
  WRITE_CODE( sei.values.primaries[0][1],  16,  "display_primaries_y[0]" );

  WRITE_CODE( sei.values.primaries[1][0],  16,  "display_primaries_x[1]" );
  WRITE_CODE( sei.values.primaries[1][1],  16,  "display_primaries_y[1]" );

  WRITE_CODE( sei.values.primaries[2][0],  16,  "display_primaries_x[2]" );
  WRITE_CODE( sei.values.primaries[2][1],  16,  "display_primaries_y[2]" );

  WRITE_CODE( sei.values.whitePoint[0],    16,  "white_point_x" );
  WRITE_CODE( sei.values.whitePoint[1],    16,  "white_point_y" );

  WRITE_CODE( sei.values.maxLuminance,     32,  "max_display_mastering_luminance" );
  WRITE_CODE( sei.values.minLuminance,     32,  "min_display_mastering_luminance" );
}


Void SEIWriter::xWriteSEISegmentedRectFramePacking(const SEISegmentedRectFramePacking& sei)
{
  WRITE_FLAG( sei.m_arrangementCancelFlag,          "segmented_rect_frame_packing_arrangement_cancel_flag" );
  if( sei.m_arrangementCancelFlag == 0 )
  {
    WRITE_CODE( sei.m_contentInterpretationType, 2, "segmented_rect_content_interpretation_type" );
    WRITE_FLAG( sei.m_arrangementPersistenceFlag,   "segmented_rect_frame_packing_arrangement_persistence" );
  }
}


Void SEIWriter::xWriteSEITempMotionConstrainedTileSets(const SEITempMotionConstrainedTileSets& sei)
{
  //UInt code;
  WRITE_FLAG((sei.m_mc_all_tiles_exact_sample_value_match_flag ? 1 : 0), "mc_all_tiles_exact_sample_value_match_flag");
  WRITE_FLAG((sei.m_each_tile_one_tile_set_flag                ? 1 : 0), "each_tile_one_tile_set_flag"               );

  if(!sei.m_each_tile_one_tile_set_flag)
  {
    WRITE_FLAG((sei.m_limited_tile_set_display_flag ? 1 : 0), "limited_tile_set_display_flag");
    WRITE_UVLC((sei.getNumberOfTileSets() - 1),               "num_sets_in_message_minus1"   );

    if(sei.getNumberOfTileSets() > 0)
    {
      for(Int i = 0; i < sei.getNumberOfTileSets(); i++)
      {
        WRITE_UVLC(sei.tileSetData(i).m_mcts_id, "mcts_id");

        if(sei.m_limited_tile_set_display_flag)
        {
          WRITE_FLAG((sei.tileSetData(i).m_display_tile_set_flag ? 1 : 0), "display_tile_set_flag");
        }

        WRITE_UVLC((sei.tileSetData(i).getNumberOfTileRects() - 1), "num_tile_rects_in_set_minus1");

        for(Int j = 0; j < sei.tileSetData(i).getNumberOfTileRects(); j++)
        {
          WRITE_UVLC(sei.tileSetData(i).topLeftTileIndex    (j), "top_left_tile_index");
          WRITE_UVLC(sei.tileSetData(i).bottomRightTileIndex(j), "bottom_right_tile_index");
        }

        if(!sei.m_mc_all_tiles_exact_sample_value_match_flag)
        {
          WRITE_FLAG((sei.tileSetData(i).m_exact_sample_value_match_flag ? 1 : 0), "exact_sample_value_match_flag");
        }

        WRITE_FLAG((sei.tileSetData(i).m_mcts_tier_level_idc_present_flag ? 1 : 0), "mcts_tier_level_idc_present_flag");

        if(sei.tileSetData(i).m_mcts_tier_level_idc_present_flag)
        {
          WRITE_FLAG((sei.tileSetData(i).m_mcts_tier_flag ? 1 : 0), "mcts_tier_flag");
          WRITE_CODE( sei.tileSetData(i).m_mcts_level_idc, 8,       "mcts_level_idc");
        }
      }
    }
  }
  else
  {
    WRITE_FLAG((sei.m_max_mcs_tier_level_idc_present_flag ? 1 : 0), "max_mcs_tier_level_idc_present_flag");

    if(sei.m_max_mcs_tier_level_idc_present_flag)
    {
      WRITE_FLAG((sei.m_max_mcts_tier_flag ? 1 : 0), "max_mcts_tier_flag");
      WRITE_CODE( sei.m_max_mcts_level_idc, 8,       "max_mcts_level_idc");
    }
  }
}

#if MCTS_EXTRACTION
Void SEIWriter::xWriteSEIMCTSExtractionInfoSet(const SEIMCTSExtractionInfoSet& sei)
{
  assert(!sei.m_MCTSExtractionInfoSets.empty());
  WRITE_UVLC(((UInt)sei.m_MCTSExtractionInfoSets.size() - 1), "num_sets_in_message_minus1");
  for (std::vector<SEIMCTSExtractionInfoSet::MCTSExtractionInfo>::const_iterator MCTSEISiter = sei.m_MCTSExtractionInfoSets.begin();
         MCTSEISiter != sei.m_MCTSExtractionInfoSets.end(); MCTSEISiter++)
  {
    WRITE_UVLC(((UInt)MCTSEISiter->m_idxOfMctsInSet.size() - 1), "num_mcts_sets_minus1[ i ]");
    for ( Int j = 0; j < MCTSEISiter->m_idxOfMctsInSet.size(); j++)
    {
      WRITE_UVLC(((UInt)MCTSEISiter->m_idxOfMctsInSet[j].size() - 1), "num_mcts_in_set_minus1[ i ][ j ]");
      for (Int k = 0; k < MCTSEISiter->m_idxOfMctsInSet[j].size(); k++)
      {
        WRITE_UVLC((MCTSEISiter->m_idxOfMctsInSet[j][k]), "idx_of_mcts_in_set[ i ][ j ][ k ]");
      }
    }
    WRITE_FLAG((MCTSEISiter->m_sliceReorderingEnabledFlag? 1 : 0 ), "slice_reordering_enabled_flag[ i ]");
    if ( MCTSEISiter->m_sliceReorderingEnabledFlag )
    {
      WRITE_UVLC(((UInt)MCTSEISiter->m_outputSliceSegmentAddress.size() - 1), "num_slice_segments_minus1[ i ]");
      for (Int j = 0; j < MCTSEISiter->m_outputSliceSegmentAddress.size(); j++)
      {
        WRITE_UVLC((MCTSEISiter->m_outputSliceSegmentAddress[j]), "output_slice_segment_address[ i ][ j ]");
      }
    }
    WRITE_UVLC(((UInt)MCTSEISiter->m_vpsRbspDataLength.size() - 1), "num_vps_in_info_set_minus1[i]");
    for (Int j = 0; j < MCTSEISiter->m_vpsRbspDataLength.size(); j++)
    {
      WRITE_UVLC((MCTSEISiter->m_vpsRbspDataLength[j]), "vps_rbsp_data_length[i][j]");
    }
    WRITE_UVLC(((UInt)MCTSEISiter->m_spsRbspDataLength.size() - 1), "num_sps_in_info_set_minus1[i]");
    for (Int j = 0; j < MCTSEISiter->m_spsRbspDataLength.size(); j++)
    {
      WRITE_UVLC((MCTSEISiter->m_spsRbspDataLength[j]), "sps_rbsp_data_length[i][j]");
    }
    WRITE_UVLC(((UInt)MCTSEISiter->m_ppsRbspDataLength.size() - 1), "num_sps_in_info_set_minus1[i]");
    for (Int j = 0; j < MCTSEISiter->m_ppsRbspDataLength.size(); j++)
    {
      WRITE_UVLC((MCTSEISiter->m_ppsNuhTemporalIdPlus1[j]), "pps_nuh_temporal_id_plus1[i][j]");
      WRITE_UVLC((MCTSEISiter->m_ppsRbspDataLength[j]), "pps_rbsp_data_length[i][j]");
    }
    // byte alignment
    while (m_pcBitIf->getNumberOfWrittenBits() % 8 != 0)
    {
      WRITE_FLAG(0, "mcts_alignment_bit_equal_to_zero");
    }
    for (Int j = 0; j < MCTSEISiter->m_vpsRbspData.size(); j++)
    {
      for (Int k = 0; k < MCTSEISiter->m_vpsRbspDataLength[j]; k++)
      {
        WRITE_CODE((MCTSEISiter->m_vpsRbspData[j][k]), 8, "vps_rbsp_data_byte[ i ][ j ][ k ]");
      }
    }
    for (Int j = 0; j < MCTSEISiter->m_spsRbspData.size(); j++)
    {
      for (Int k = 0; k < MCTSEISiter->m_spsRbspDataLength[j]; k++)
      {
        WRITE_CODE((MCTSEISiter->m_spsRbspData[j][k]), 8, "sps_rbsp_data_byte[ i ][ j ][ k ]");
      }
    }
    for (Int j = 0; j < MCTSEISiter->m_ppsRbspData.size(); j++)
    {
      for (Int k = 0; k < MCTSEISiter->m_ppsRbspDataLength[j]; k++)
      {
        WRITE_CODE((MCTSEISiter->m_ppsRbspData[j][k]), 8, "pps_rbsp_data_byte[ i ][ j ][ k ]");
      }
    }
  }
}
#endif


Void SEIWriter::xWriteSEIChromaResamplingFilterHint(const SEIChromaResamplingFilterHint &sei)
{
  WRITE_CODE(sei.m_verChromaFilterIdc, 8, "ver_chroma_filter_idc");
  WRITE_CODE(sei.m_horChromaFilterIdc, 8, "hor_chroma_filter_idc");
  WRITE_FLAG(sei.m_verFilteringFieldProcessingFlag, "ver_filtering_field_processing_flag");
  if(sei.m_verChromaFilterIdc == 1 || sei.m_horChromaFilterIdc == 1)
  {
    WRITE_UVLC(sei.m_targetFormatIdc, "target_format_idc");
    if(sei.m_verChromaFilterIdc == 1)
    {
      const Int numVerticalFilter = (Int)sei.m_verFilterCoeff.size();
      WRITE_UVLC(numVerticalFilter, "num_vertical_filters");
      if(numVerticalFilter > 0)
      {
        for(Int i = 0; i < numVerticalFilter; i ++)
        {
          const Int verTapLengthMinus1 = (Int) sei.m_verFilterCoeff[i].size() - 1;
          WRITE_UVLC(verTapLengthMinus1, "ver_tap_length_minus_1");
          for(Int j = 0; j < (verTapLengthMinus1 + 1); j ++)
          {
            WRITE_SVLC(sei.m_verFilterCoeff[i][j], "ver_filter_coeff");
          }
        }
      }
    }
    if(sei.m_horChromaFilterIdc == 1)
    {
      const Int numHorizontalFilter = (Int) sei.m_horFilterCoeff.size();
      WRITE_UVLC(numHorizontalFilter, "num_horizontal_filters");
      if(numHorizontalFilter > 0)
      {
        for(Int i = 0; i < numHorizontalFilter; i ++)
        {
          const Int horTapLengthMinus1 = (Int) sei.m_horFilterCoeff[i].size() - 1;
          WRITE_UVLC(horTapLengthMinus1, "hor_tap_length_minus_1");
          for(Int j = 0; j < (horTapLengthMinus1 + 1); j ++)
          {
            WRITE_SVLC(sei.m_horFilterCoeff[i][j], "hor_filter_coeff");
          }
        }
      }
    }
  }
}


Void SEIWriter::xWriteSEIKneeFunctionInfo(const SEIKneeFunctionInfo &sei)
{
  WRITE_UVLC( sei.m_kneeId, "knee_function_id" );
  WRITE_FLAG( sei.m_kneeCancelFlag, "knee_function_cancel_flag" ); 
  if ( !sei.m_kneeCancelFlag )
  {
    WRITE_FLAG( sei.m_kneePersistenceFlag, "knee_function_persistence_flag" );
    WRITE_CODE( (UInt)sei.m_kneeInputDrange , 32,  "input_d_range" );
    WRITE_CODE( (UInt)sei.m_kneeInputDispLuminance, 32,  "input_disp_luminance" );
    WRITE_CODE( (UInt)sei.m_kneeOutputDrange, 32,  "output_d_range" );
    WRITE_CODE( (UInt)sei.m_kneeOutputDispLuminance, 32,  "output_disp_luminance" );
    WRITE_UVLC( sei.m_kneeNumKneePointsMinus1, "num_knee_points_minus1" );
    for(Int i = 0; i <= sei.m_kneeNumKneePointsMinus1; i++ )
    {
      WRITE_CODE( (UInt)sei.m_kneeInputKneePoint[i], 10,"input_knee_point" );
      WRITE_CODE( (UInt)sei.m_kneeOutputKneePoint[i], 10, "output_knee_point" );
    }
  }
}

Void SEIWriter::xWriteSEIContentColourVolume(const SEIContentColourVolume &sei)
{
  WRITE_FLAG(sei.m_ccvCancelFlag, "ccv_cancel_flag");
  if (!sei.m_ccvCancelFlag)
  {
    WRITE_FLAG(sei.m_ccvPersistenceFlag, "ccv_persistence_flag");
    WRITE_FLAG(sei.m_ccvPrimariesPresentFlag, "ccv_primaries_present_flag");
    WRITE_FLAG(sei.m_ccvMinLuminanceValuePresentFlag, "ccv_min_luminance_value_present_flag");
    WRITE_FLAG(sei.m_ccvMaxLuminanceValuePresentFlag, "ccv_max_luminance_value_present_flag");
    WRITE_FLAG(sei.m_ccvAvgLuminanceValuePresentFlag, "ccv_avg_luminance_value_present_flag");
    
    if (sei.m_ccvPrimariesPresentFlag == true) 
    {
      for (Int i = 0; i < MAX_NUM_COMPONENT; i++) 
      {
        WRITE_SCODE((Int) sei.m_ccvPrimariesX[i], 32, "ccv_primaries_x[i]");
        WRITE_SCODE((Int) sei.m_ccvPrimariesY[i], 32, "ccv_primaries_y[i]");
      }
    }

    if (sei.m_ccvMinLuminanceValuePresentFlag == true) 
    {
      WRITE_CODE( (UInt)sei.m_ccvMinLuminanceValue, 32,  "ccv_min_luminance_value" );
    }
    if (sei.m_ccvMaxLuminanceValuePresentFlag == true) 
    {
      WRITE_CODE( (UInt)sei.m_ccvMaxLuminanceValue, 32,  "ccv_max_luminance_value" );
    }
    if (sei.m_ccvAvgLuminanceValuePresentFlag == true) 
    {
      WRITE_CODE( (UInt)sei.m_ccvAvgLuminanceValue, 32,  "ccv_avg_luminance_value" );
    }
  }
}

#if SHUTTER_INTERVAL_SEI_MESSAGE
Void SEIWriter::xWriteSEIShutterInterval(const SEIShutterIntervalInfo &sei)
{
  WRITE_CODE(sei.m_siiTimeScale, 32, "sii_time_scale");
  WRITE_FLAG(sei.m_siiFixedSIwithinCLVS, "fixed_shutter_interval_within_clvs_flag");
  if (sei.m_siiFixedSIwithinCLVS)
  {
    WRITE_CODE(sei.m_siiNumUnitsInShutterInterval, 32, "sii_num_units_in_shutter_interval");
  }
  else
  {
    WRITE_CODE(sei.m_siiMaxSubLayersMinus1, 3, "sii_max_sub_layers_minus1");
    for (UInt i = 0; i <= sei.m_siiMaxSubLayersMinus1; i++)
    {
      WRITE_CODE(sei.m_siiSubLayerNumUnitsInSI[i], 32, "sub_layer_num_units_in_shutter_interval[ i ]");
    }
  }
}
#endif

#if NNPFC_SEI_MESSAGE
Void SEIWriter::xWriteSEINeuralNetworkPostFilterCharacteristics(const SEINeuralNetworkPostFilterCharacteristics &sei)
{
  WRITE_CODE(sei.m_purpose, 16, "nnpfc_purpose");
  WRITE_UVLC(sei.m_id, "nnpfc_id");
  WRITE_FLAG(sei.m_baseFlag, "nnpfc_base_flag");
  WRITE_UVLC(sei.m_modeIdc, "nnpfc_mode_idc");
  if (sei.m_modeIdc == POST_FILTER_MODE::URI)
  {
    while (m_pcBitIf->getNumBitsUntilByteAligned() != 0)
    {
      WRITE_FLAG(0, "nnpfc_alignment_zero_bit");
    }
    WRITE_STRING(sei.m_uriTag, "nnpfc_uri_tag");
    WRITE_STRING(sei.m_uri, "nnpfc_uri");
  }
  WRITE_FLAG(sei.m_propertyPresentFlag, "nnpfc_property_present_flag");
  if (sei.m_propertyPresentFlag)
  {
    WRITE_UVLC(sei.m_numberInputDecodedPicturesMinus1, "nnpfc_number_of_input_pictures_minus1");

    if (sei.m_numberInputDecodedPicturesMinus1 > 0)
    {
      for (Int i = 0; i <= sei.m_numberInputDecodedPicturesMinus1; ++i)
      {
        WRITE_FLAG(sei.m_inputPicOutputFlag[i], "nnpfc_input_pic_filtering_flag");
      }
      WRITE_FLAG(sei.m_absentInputPicZeroFlag, "nnpfc_absent_input_pic_zero_flag");
    }

    if((sei.m_purpose & NNPC_PurposeType::CHROMA_UPSAMPLING) != 0)
    {
      WRITE_FLAG(sei.m_outSubCFlag, "nnpfc_out_sub_c_flag");
    }
    if((sei.m_purpose & NNPC_PurposeType::COLOURIZATION) != 0)
    {
      WRITE_CODE(uint32_t(sei.m_outColourFormatIdc), 2, "nnpfc_out_colour_format_idc");
    }
    if((sei.m_purpose & NNPC_PurposeType::RESOLUTION_UPSAMPLING) != 0)
    {
      WRITE_UVLC(sei.m_picWidthNumeratorMinus1, "nnpfc_pic_width_num_minus1");
      WRITE_UVLC(sei.m_picWidthDenominatorMinus1, "nnpfc_pic_width_denom_minus1");
      WRITE_UVLC(sei.m_picHeightNumeratorMinus1, "nnpfc_pic_height_num_minus1");
      WRITE_UVLC(sei.m_picHeightDenominatorMinus1, "nnpfc_pic_height_denom_minus1");
    }

    if((sei.m_purpose & NNPC_PurposeType::FRAME_RATE_UPSAMPLING) != 0)
    {
      for (Int i = 0; i < sei.m_numberInputDecodedPicturesMinus1; ++i)
      {
        WRITE_UVLC(sei.m_numberInterpolatedPictures[i], "nnpfc_interpolated_pictures");
      }
    }

    if((sei.m_purpose & NNPC_PurposeType::TEMPORAL_EXTRAPOLATION) != 0)
    {
      WRITE_UVLC(sei.m_numberExtrapolatedPicturesMinus1, "nnpfc_extrapolated_pics_minus1");
    }

    if((sei.m_purpose & NNPC_PurposeType::SPATIAL_EXTRAPOLATION) != 0)
    {
      WRITE_SVLC(sei.m_spatialExtrapolationLeftOffset, "nnpfc_spatial_extrapolation_left_offset");
      WRITE_SVLC(sei.m_spatialExtrapolationRightOffset, "nnpfc_spatial_extrapolation_right_offset");
      WRITE_SVLC(sei.m_spatialExtrapolationTopOffset, "nnpfc_spatial_extrapolation_top_offset");
      WRITE_SVLC(sei.m_spatialExtrapolationBottomOffset, "nnpfc_spatial_extrapolation_right_offset");
    }

    WRITE_FLAG(sei.m_componentLastFlag, "nnpfc_component_last_flag");
    WRITE_UVLC(sei.m_inpFormatIdc, "nnpfc_inp_format_idc");
    WRITE_UVLC(sei.m_auxInpIdc, "nnpfc_auxiliary_inp_idc");
    if ((sei.m_auxInpIdc & 2) > 0)
    {
      WRITE_FLAG(sei.m_inbandPromptFlag, "nnpfc_inband_prompt_flag");
      if (sei.m_inbandPromptFlag)
      {
        while (m_pcBitIf->getNumBitsUntilByteAligned() != 0)
        {
          WRITE_FLAG(0, "nnpfc_alignment_zero_bit_c");
        }
        WRITE_STRING(sei.m_prompt, "nnpfc_prompt");
      }
    }
    if ((sei.m_auxInpIdc & 4) > 0)
    {
      WRITE_FLAG(sei.m_inbandSeedFlag, "nnpfc_inband_seed_flag");
      if (sei.m_inbandSeedFlag)
      {
        WRITE_CODE(sei.m_seed, 16, "nnpfc_seed");
      }
    }
    WRITE_UVLC(sei.m_inpOrderIdc, "nnpfc_inp_order_idc");
    if (sei.m_inpFormatIdc == 1)
    {
      if (sei.m_inpOrderIdc != 1)
      {
        WRITE_UVLC(sei.m_inpTensorBitDepthLumaMinus8, "nnpfc_inp_tensor_luma_bitdepth_minus8");
      }
      if (sei.m_inpOrderIdc != 0)
      {
        WRITE_UVLC(sei.m_inpTensorBitDepthChromaMinus8, "nnpfc_inp_tensor_chroma_bitdepth_minus8");
      }
    }
    WRITE_UVLC(sei.m_outFormatIdc, "nnpfc_out_format_idc");
    WRITE_UVLC(sei.m_outOrderIdc, "nnpfc_out_order_idc");
    if (sei.m_outFormatIdc == 1)
    {
      if (sei.m_outOrderIdc != 1)
      {
        WRITE_UVLC(sei.m_outTensorBitDepthLumaMinus8, "nnpfc_out_tensor_luma_bitdepth_minus8");
      }
      if (sei.m_outOrderIdc != 0)
      {
        WRITE_UVLC(sei.m_outTensorBitDepthChromaMinus8, "nnpfc_out_tensor_chroma_bitdepth_minus8");
      }
    }

    WRITE_FLAG(sei.m_sepColDescriptionFlag, "nnpfc_sep_col_desc_flag");

    if (sei.m_sepColDescriptionFlag)
    {
      WRITE_CODE(sei.m_colPrimaries, 8, "nnpfc_col_primaries");
      WRITE_CODE(sei.m_transCharacteristics, 8, "nnpfc_trans_characteristics");
      if (sei.m_outFormatIdc == 1)
      {
        WRITE_CODE(sei.m_matrixCoeffs, 8, "nnpfc_matrix_coeffs");
      }
    }
    if (sei.m_sepColDescriptionFlag && (sei.m_outFormatIdc == 1))
    {
      WRITE_FLAG(sei.m_fullRangeFlag, "nnpfc_full_range_flag");
    }
    
    if (sei.m_outOrderIdc != 0)
    {   
      WRITE_FLAG(sei.m_chromaLocInfoPresentFlag, "nnpfc_chroma_loc_info_present_flag");
    }

    if(sei.m_chromaLocInfoPresentFlag)
    {
      WRITE_UVLC(static_cast<UInt>(sei.m_chromaSampleLocTypeFrame), "nnpfc_chroma_sample_loc_type_frame");
    }
    
    if((sei.m_purpose & NNPC_PurposeType::SPATIAL_EXTRAPOLATION) == 0)
    {
      WRITE_UVLC(sei.m_overlap, "nnpfc_overlap");
      WRITE_FLAG(sei.m_constantPatchSizeFlag, "nnpfc_constant_patch_size_flag");
    }
    if (sei.m_constantPatchSizeFlag)
    {
      WRITE_UVLC(sei.m_patchWidthMinus1, "nnpfc_patch_width_minus1");
      WRITE_UVLC(sei.m_patchHeightMinus1, "nnpfc_patch_height_minus1");
    }
    else
    {
      WRITE_UVLC(sei.m_extendedPatchWidthCdDeltaMinus1, "extended_nnpfc_patch_width_cd_delta_minus1");
      WRITE_UVLC(sei.m_extendedPatchHeightCdDeltaMinus1, "extended_nnpfc_patch_height_cd_delta_minus1");
    }
    WRITE_UVLC(sei.m_paddingType, "nnpfc_padding_type");
    if (sei.m_paddingType == NNPC_PaddingType::FIXED_PADDING)
    {
      if (sei.m_inpOrderIdc != 1)
      {
        WRITE_UVLC(sei.m_lumaPadding, "nnpfc_luma_padding_val");
      }
      if (sei.m_inpOrderIdc != 0)
      {
        WRITE_UVLC(sei.m_cbPadding, "nnpfc_cb_padding_val");
        WRITE_UVLC(sei.m_crPadding, "nnpfc_cr_padding_val");
      }
    }

    WRITE_FLAG(sei.m_complexityInfoPresentFlag, "nnpfc_complexity_info_present_flag");
    if (sei.m_complexityInfoPresentFlag)
    {
      WRITE_CODE(sei.m_parameterTypeIdc, 2, "nnpfc_parameter_type_idc");
      if (sei.m_parameterTypeIdc != 2)
      {
        WRITE_CODE(sei.m_log2ParameterBitLengthMinus3, 2, "nnpfc_log2_parameter_bit_length_minus3");
      }
      WRITE_CODE(sei.m_numParametersIdc, 6, "nnpfc_num_parameters_idc");
      WRITE_UVLC(sei.m_numKmacOperationsIdc, "nnpfc_num_kmac_operations_idc");
      WRITE_UVLC(sei.m_totalKilobyteSize, "nnpfc_total_kilobyte_size");
    }
    UInt metadataExtensionNumBits = 0;
    if (sei.m_purpose == 0 || sei.m_forHumanViewingIdc != 0 || sei.m_forMachineAnalysisIdc != 0)
    {
      if (sei.m_purpose == 0)
      {
        metadataExtensionNumBits++;
        if (sei.m_applicationPurposeTagUriPresentFlag)
        {
          metadataExtensionNumBits +=  (static_cast<UInt>(sei.m_applicationPurposeTagUri.length() + 1) * 8);
        }
      }
      metadataExtensionNumBits += 4;  // nnpfc_for_human_viewing_idc and nnpfc_for_machine_analysis_idc bits
      WRITE_UVLC(metadataExtensionNumBits, "nnpfc_metadata_extension_num_bits");
      if (sei.m_purpose == 0)
      {
        WRITE_FLAG(sei.m_applicationPurposeTagUriPresentFlag, "nnpfc_application_purpose_tag_uri_present_flag");
        if ( sei.m_applicationPurposeTagUriPresentFlag )
        {
          while (m_pcBitIf->getNumBitsUntilByteAligned() != 0)
          {
            WRITE_FLAG(0, "nnpfc_metadata_alignment_zero_bit");
          }
          WRITE_STRING(sei.m_applicationPurposeTagUri, "nnpfc_application_purpose_tag_uri"); 
        }
      }
      if ((sei.m_purpose & NNPC_PurposeType::SPATIAL_EXTRAPOLATION) != 0 || (sei.m_purpose & NNPC_PurposeType::RESOLUTION_UPSAMPLING) != 0)
      {
        WRITE_CODE(sei.m_scanTypeIdc, 2, "nnpfc_scan_type_idc");
      }
      WRITE_CODE(sei.m_forHumanViewingIdc, 2, "nnpfc_for_human_viewing_idc");
      WRITE_CODE(sei.m_forMachineAnalysisIdc, 2, "nnpfc_for_machine_analysis_idc");
    }
    else
    {
      WRITE_UVLC(metadataExtensionNumBits, "nnpfc_metadata_extension_num_bits");  
    }
  }
  if (sei.m_modeIdc == POST_FILTER_MODE::ISO_IEC_15938_17)
  {
    while (m_pcBitIf->getNumBitsUntilByteAligned() != 0)
    {
      WRITE_FLAG(0, "nnpfc_alignment_zero_bit");
    }
    for (UInt i = 0; i < sei.m_payloadLength; i++)
    {
      WRITE_SCODE(sei.m_payloadByte[i], 8, "nnpfc_payload_byte[i]");
    }
  }
}
#endif

#if NNPFA_SEI_MESSAGE
Void SEIWriter::xWriteSEINeuralNetworkPostFilterActivation(const SEINeuralNetworkPostFilterActivation &sei)
{
  WRITE_UVLC(sei.m_targetId, "nnpfa_target_id");
  WRITE_FLAG(sei.m_cancelFlag, "nnpfa_cancel_flag");
  if(!sei.m_cancelFlag)
  {
    WRITE_FLAG(sei.m_persistenceFlag, "nnpfa_persistence_flag");
    WRITE_FLAG(sei.m_targetBaseFlag, "nnpfa_target_base_flag");
    WRITE_FLAG(sei.m_noPrevCLVSFlag, "nnpfa_no_prev_clvs_flag");
    if (sei.m_persistenceFlag)
    {
      WRITE_FLAG(sei.m_noFollCLVSFlag, "nnpfa_no_foll_clvs_flag");
    }
    WRITE_UVLC((UInt)sei.m_outputFlag.size(), "nnpfa_num_output_entries");
    for (UInt i = 0; i < (UInt)sei.m_outputFlag.size(); i++)
    {
      WRITE_FLAG(sei.m_outputFlag[i], "nnpfa_output_flag");
    }
    WRITE_FLAG(sei.m_promptUpdateFlag, "nnpfa_prompt_update_flag");
    if (sei.m_promptUpdateFlag)
    {
      while (m_pcBitIf->getNumBitsUntilByteAligned() != 0)
      {
        WRITE_FLAG(0, "nnpfa_alignment_zero_bit");
      }
      WRITE_STRING(sei.m_prompt, "nnpfa_prompt");
    }
    WRITE_FLAG(sei.m_seedUpdateFlag, "nnpfa_seed_update_flag");
    if (sei.m_seedUpdateFlag)
    {
      WRITE_CODE(sei.m_seed, 16, "nnpfa_seed");
    }
  }
}
#endif

#if JVET_AE0101_PHASE_INDICATION_SEI_MESSAGE
void SEIWriter::xWriteSEIPhaseIndication(const SEIPhaseIndication& sei)
{
  WRITE_CODE((uint32_t)sei.m_horPhaseNum, 8, "hor_phase_num");
  WRITE_CODE((uint32_t)sei.m_horPhaseDenMinus1, 8, "hor_phase_den_minus1");
  WRITE_CODE((uint32_t)sei.m_verPhaseNum, 8, "ver_phase_num");
  WRITE_CODE((uint32_t)sei.m_verPhaseDenMinus1, 8, "ver_phase_den_minus1");
}
#endif
#if JVET_AL0061_ENCODER_OPTIMIZATION_INFORMATION_SEI
void SEIWriter::xWriteSEIEncoderOptimizationInfo(const SEIEncoderOptimizationInfo &sei)
{
  WRITE_FLAG(sei.m_cancelFlag, "eoi_cancel_flag");
  if (!sei.m_cancelFlag)
  {
    WRITE_FLAG(sei.m_persistenceFlag, "eoi_persistence_flag");
    WRITE_CODE(sei.m_forHumanViewingIdc, 2, "eoi_for_human_viewing_idc");
    WRITE_CODE(sei.m_forMachineAnalysisIdc, 2, "eoi_for_machine_analysis_idc");
    WRITE_CODE(0, 2, "eoi_reserved_zero_2bits");
    WRITE_CODE(sei.m_type, 16, "eoi_type");


    if ((sei.m_type & EOI_OptimizationType::OBJECT_BASED_OPTIMIZATION) != 0)
    {
      WRITE_CODE(sei.m_objectBasedIdc, 16, "eoi_object_based_idc");
      if (sei.m_objectBasedIdc & EOI_OBJECT_BASED::COARSER_QUANTIZATION)
      {
        WRITE_UVLC(sei.m_quantThresholdDelta, "eoi_quant_threshold_delta");
        if (sei.m_quantThresholdDelta > 0)
        {
          WRITE_FLAG(sei.m_picQuantObjectFlag, "eoi_pic_quant_object_flag");
        }
      }
    }
    if ((sei.m_type & EOI_OptimizationType::TEMPORAL_RESAMPLING) != 0)
    {
      WRITE_FLAG(sei.m_temporalResamplingTypeFlag, "eoi_temporal_resampling_type_flag");
      WRITE_UVLC(sei.m_numIntPics, "eoi_num_int_pics");
      if (sei.m_temporalResamplingTypeFlag && sei.m_numIntPics > 0)
      {
        WRITE_FLAG(sei.m_srcPicFlag, "eoi_src_pic_flag");
      }
    }
    if ((sei.m_type & EOI_OptimizationType::SPATIAL_RESAMPLING) != 0)
    {
      WRITE_FLAG(sei.m_origPicDimensionsFlag, "eoi_orig_pic_dimensions_flag");
      if (sei.m_origPicDimensionsFlag)
      {
        WRITE_CODE(sei.m_origPicWidth, 16, "eoi_orig_pic_width");
        WRITE_CODE(sei.m_origPicHeight, 16, "eoi_orig_pic_height");
      }
      else
      {
        WRITE_FLAG(sei.m_spatialResamplingTypeFlag, "eoi_spatial_resampling_type_flag");
      }
    }
    if ((sei.m_type & EOI_OptimizationType::PRIVACY_PROTECTION_OPTIMIZATION) != 0)
    {
      WRITE_CODE(sei.m_privacyProtectionTypeIdc, 16, "eoi_privacy_protection_type_idc");
      WRITE_CODE(sei.m_privacyProtectedInfoType, 8, "eoi_privacy_protected_info_type");
    }
  }
}
#endif

#if JVET_AK0107_MODALITY_INFORMATION
Void SEIWriter::xWriteSEIModalityInfo(const SEIModalityInfo& sei)
{
  WRITE_FLAG( sei.m_miCancelFlag,                                  "modality_info_cancel_flag" );
  if(!sei.m_miCancelFlag)
  {
    WRITE_FLAG( sei.m_miPersistenceFlag,                           "modality_info_persistence_flag" );
    WRITE_CODE( sei.m_miModalityType,                  5,          "modality_type");
    WRITE_FLAG( sei.m_miSpectrumRangePresentFlag,                  "spectrum_range_present_flag" );
    if (sei.m_miSpectrumRangePresentFlag)
    {
      WRITE_CODE( sei.m_miMinWavelengthMantissa,      11,          "min_wavelength_mantissa ");
      WRITE_CODE( sei.m_miMinWavelengthExponentPlus15, 5,          "min_wavelength_exponent_plus15 ");
      WRITE_CODE( sei.m_miMaxWavelengthMantissa,      11,          "max_wavelength_mantissa ");
      WRITE_CODE( sei.m_miMaxWavelengthExponentPlus15, 5,          "max_wavelength_exponent_plus15 ");
    }
    WRITE_UVLC(0, "modality_type_extension_bits");   // mi_modality_type_extension_bits shall be equal to 0 in the current edition 
  }
}
#endif 

Void SEIWriter::xWriteSEIEquirectangularProjection(const SEIEquirectangularProjection &sei
#if JCTVC_AD0021_SEI_PREFIX_INDICATION
  , Int SEIPrefixIndicationIdx
#endif
)
{
#if JCTVC_AD0021_SEI_PREFIX_INDICATION
  if (SEIPrefixIndicationIdx)
  {
    int numBits = 5;
    if (sei.m_erpGuardBandFlag)
    {
      numBits += 19;
    }
    WRITE_CODE(numBits - 1, 16, "num_bits_in_prefix_indication_minus1");
  }
#endif
  WRITE_FLAG( sei.m_erpCancelFlag, "erp_cancel_flag" );
  if( !sei.m_erpCancelFlag )
  {
    WRITE_FLAG( sei.m_erpPersistenceFlag, "erp_persistence_flag" );
    WRITE_FLAG( sei.m_erpGuardBandFlag,   "erp_guard_band_flag" );
    WRITE_CODE( 0, 2, "erp_reserved_zero_2bits" );
    if ( sei.m_erpGuardBandFlag == 1)
    {
      WRITE_CODE( sei.m_erpGuardBandType,       3, "erp_guard_band_type" );  
      WRITE_CODE( sei.m_erpLeftGuardBandWidth,  8, "erp_left_guard_band_width" );  
      WRITE_CODE( sei.m_erpRightGuardBandWidth, 8, "erp_right_guard_band_width" );  
    }
  }
}

Void SEIWriter::xWriteSEISphereRotation(const SEISphereRotation &sei
#if JCTVC_AD0021_SEI_PREFIX_INDICATION
  , Int SEIPrefixIndicationIdx
#endif
)
{
#if JCTVC_AD0021_SEI_PREFIX_INDICATION
  if (SEIPrefixIndicationIdx)
  {
    if (sei.m_sphereRotationCancelFlag)
    {
      WRITE_CODE(0, 8, "num_sei_prefix_indications_minus1");
    }
    else
    {
      WRITE_CODE(1, 8, "num_sei_prefix_indications_minus1");
    }
    int numBits = 8;
    WRITE_CODE(numBits - 1, 16, "num_bits_in_prefix_indication_minus1");
  }
#endif
  WRITE_FLAG( sei.m_sphereRotationCancelFlag, "sphere_rotation_cancel_flag" );
  if( !sei.m_sphereRotationCancelFlag )
  {
    WRITE_FLAG( sei.m_sphereRotationPersistenceFlag,    "sphere_rotation_persistence_flag" );
    WRITE_CODE( 0,                                   6, "sphere_rotation_reserved_zero_6bits" );
#if JCTVC_AD0021_SEI_PREFIX_INDICATION
    if (SEIPrefixIndicationIdx >= 2)
    {
      xWriteSEIPrefixIndicationByteAlign();
      int numBits2 = 32 + 32 + 32 + 8;
      WRITE_CODE(numBits2 - 1, 16, "num_bits_in_prefix_indication_minus1");
      WRITE_FLAG(sei.m_sphereRotationCancelFlag, "sphere_rotation_cancel_flag");
      WRITE_FLAG(sei.m_sphereRotationPersistenceFlag, "sphere_rotation_persistence_flag");
      WRITE_CODE(0, 6, "sphere_rotation_reserved_zero_6bits");
    }
#endif
    WRITE_SCODE(sei.m_sphereRotationYaw,            32, "sphere_rotation_yaw" );  
    WRITE_SCODE(sei.m_sphereRotationPitch,          32, "sphere_rotation_pitch" );  
    WRITE_SCODE(sei.m_sphereRotationRoll,           32, "sphere_rotation_roll" );  
  }
}

Void SEIWriter::xWriteSEIOmniViewport(const SEIOmniViewport &sei)
{
  WRITE_CODE( sei.m_omniViewportId,     10,    "omni_viewport_id" );
  WRITE_FLAG( sei.m_omniViewportCancelFlag, "omni_viewport_cancel_flag" );
  if ( !sei.m_omniViewportCancelFlag )
  {
    WRITE_FLAG( sei.m_omniViewportPersistenceFlag, "omni_viewport_persistence_flag" );
    const UInt numRegions = (UInt) sei.m_omniViewportRegions.size();
    WRITE_CODE( numRegions - 1, 4, "omni_viewport_cnt_minus1" );
    for(UInt region=0; region<numRegions; region++)
    {
      const SEIOmniViewport::OmniViewport &viewport=sei.m_omniViewportRegions[region];
      WRITE_SCODE( viewport.azimuthCentre,     32,  "omni_viewport_azimuth_centre"   );  
      WRITE_SCODE( viewport.elevationCentre,   32,  "omni_viewport_elevation_centre" );  
      WRITE_SCODE( viewport.tiltCentre,        32,  "omni_viewport_tilt_center" );  
      WRITE_CODE( viewport.horRange,           32, "omni_viewport_hor_range[i]" );
      WRITE_CODE( viewport.verRange,           32, "omni_viewport_ver_range[i]" );
    }
  }
}
Void SEIWriter::xWriteSEICubemapProjection(const SEICubemapProjection &sei)
{
  WRITE_FLAG(sei.m_cmpCancelFlag, "cmp_cancel_flag");
  if (!sei.m_cmpCancelFlag)
  {
    WRITE_FLAG(sei.m_cmpPersistenceFlag, "cmp_persistence_flag");
  }
}
Void SEIWriter::xWriteSEIRegionWisePacking(const SEIRegionWisePacking &sei
#if JCTVC_AD0021_SEI_PREFIX_INDICATION
  , Int SEIPrefixIndicationIdx
#endif
)
{
#if JCTVC_AD0021_SEI_PREFIX_INDICATION
  if (SEIPrefixIndicationIdx)
  {
    WRITE_CODE(0, 8, "num_sei_prefix_indications_minus1");
    int numBits = 1;
    if (!sei.m_rwpCancelFlag)
    {
      numBits += 111;
    }
    WRITE_CODE(numBits - 1, 16, "num_bits_in_prefix_indication_minus1");
  }
#endif
  WRITE_FLAG( sei.m_rwpCancelFlag,                                           "rwp_cancel_flag" );
  if(!sei.m_rwpCancelFlag)
  {
    WRITE_FLAG( sei.m_rwpPersistenceFlag,                                    "rwp_persistence_flag" );
    WRITE_FLAG( sei.m_constituentPictureMatchingFlag,                        "constituent_picture_matching_flag" );
    WRITE_CODE( 0, 5,                                                        "rwp_reserved_zero_5bits" );
    WRITE_CODE( (UInt)sei.m_numPackedRegions,                 8,             "num_packed_regions" );
    WRITE_CODE( (UInt)sei.m_projPictureWidth,                 32,            "proj_picture_width" );
    WRITE_CODE( (UInt)sei.m_projPictureHeight,                32,            "proj_picture_height" );
    WRITE_CODE( (UInt)sei.m_packedPictureWidth,               16,            "packed_picture_width" );
    WRITE_CODE( (UInt)sei.m_packedPictureHeight,              16,            "packed_picture_height" );
#if JCTVC_AD0021_SEI_PREFIX_INDICATION
    if (SEIPrefixIndicationIdx)
    {
      // don't write full message
      return;
    }
#endif
    for( Int i=0; i < sei.m_numPackedRegions; i++ )
    { 
      WRITE_CODE( 0, 4,                                                      "rwp_reserved_zero_4bits" );
      WRITE_CODE( (UInt)sei.m_rwpTransformType[i],            3,             "rwp_tTransform_type" );
      WRITE_FLAG( sei.m_rwpGuardBandFlag[i],                                 "rwp_guard_band_flag" );
      WRITE_CODE( (UInt)sei.m_projRegionWidth[i],             32,            "proj_region_width" );
      WRITE_CODE( (UInt)sei.m_projRegionHeight[i],            32,            "proj_region_height" );
      WRITE_CODE( (UInt)sei.m_rwpProjRegionTop[i],            32,            "rwp_proj_regionTop" );
      WRITE_CODE( (UInt)sei.m_projRegionLeft[i],              32,            "proj_region_left" );
      WRITE_CODE( (UInt)sei.m_packedRegionWidth[i],           16,            "packed_region_width" );
      WRITE_CODE( (UInt)sei.m_packedRegionHeight[i],          16,            "packed_region_height" );
      WRITE_CODE( (UInt)sei.m_packedRegionTop[i],             16,            "packed_region_top" );
      WRITE_CODE( (UInt)sei.m_packedRegionLeft[i],            16,            "packed_region_left" );
      if( sei.m_rwpGuardBandFlag[i] )
      {
        WRITE_CODE( (UInt)sei.m_rwpLeftGuardBandWidth[i],     8,             "rwp_left_guard_band_width");
        WRITE_CODE( (UInt)sei.m_rwpRightGuardBandWidth[i],    8,             "rwp_right_guard_band_width");
        WRITE_CODE( (UInt)sei.m_rwpTopGuardBandHeight[i],     8,             "rwp_top_guard_band_height");
        WRITE_CODE( (UInt)sei. m_rwpBottomGuardBandHeight[i], 8,             "rwp_bottom_guard_band_height");
        WRITE_FLAG( sei.m_rwpGuardBandNotUsedForPredFlag[i],                 "rwp_guard_band_not_used_forPred_flag" );
        for( Int j=0; j < 4; j++ )
        {
          WRITE_CODE( (UInt)sei.m_rwpGuardBandType[i*4 + j],  3,             "rwp_guard_band_type");
        }
        WRITE_CODE( 0, 3,                                                    "rwp_guard_band_reserved_zero_3bits" );
      }
    }
  }
}

Void SEIWriter::xWriteSEIFisheyeVideoInfo(const SEIFisheyeVideoInfo &sei)
{
  const TComSEIFisheyeVideoInfo &info=sei.values;
  WRITE_FLAG(info.m_fisheyeCancelFlag, "fisheye_cancel_flag");
  if (!info.m_fisheyeCancelFlag)
  {
    WRITE_FLAG(info.m_fisheyePersistenceFlag, "fisheye_persistence_flag");
    WRITE_CODE((UInt)info.m_fisheyeViewDimensionIdc, 3, "fisheye_view_dimension_idc");
    WRITE_CODE(0, 3, "fisheye_reserved_zero_3bits");
    assert(info.m_fisheyeActiveAreas.size());
    WRITE_CODE((UInt)info.m_fisheyeActiveAreas.size()-1, 8, "fisheye_num_active_area_minus1");

    for (std::size_t i = 0; i < info.m_fisheyeActiveAreas.size(); i++)
    {
      const TComSEIFisheyeVideoInfo::ActiveAreaInfo &area=info.m_fisheyeActiveAreas[i];
      
      WRITE_CODE((UInt)area.m_fisheyeCircularRegionCentreX, 32, "fisheye_circular_region_centre_x[i]");
      WRITE_CODE((UInt)area.m_fisheyeCircularRegionCentreY, 32, "fisheye_circular_region_centre_y[i]");
      WRITE_CODE((UInt)area.m_fisheyeRectRegionTop, 32, "fisheye_rect_region_top[i]");
      WRITE_CODE((UInt)area.m_fisheyeRectRegionLeft, 32, "fisheye_rect_region_left[i]");
      WRITE_CODE((UInt)area.m_fisheyeRectRegionWidth, 32, "fisheye_rect_region_width[i]");
      WRITE_CODE((UInt)area.m_fisheyeRectRegionHeight, 32, "fisheye_rect_region_Height[i]");
      WRITE_CODE((UInt)area.m_fisheyeCircularRegionRadius, 32, "fisheye_circular_region_radius[i]");
      WRITE_CODE((UInt)area.m_fisheyeSceneRadius, 32, "fisheye_scene_radius[i]");

      WRITE_SCODE((Int)area.m_fisheyeCameraCentreAzimuth, 32, "fisheye_camera_centre_azimuth[i]");
      WRITE_SCODE((Int)area.m_fisheyeCameraCentreElevation, 32, "fisheye_camera_centre_elevation[i]");
      WRITE_SCODE((Int)area.m_fisheyeCameraCentreTilt, 32, "fisheye_camera_centre_tilt[i]");

      WRITE_CODE((UInt)area.m_fisheyeCameraCentreOffsetX, 32, "fisheye_camera_centre_offset_x[i]");
      WRITE_CODE((UInt)area.m_fisheyeCameraCentreOffsetY, 32, "fisheye_camera_centre_offset_x[i]");
      WRITE_CODE((UInt)area.m_fisheyeCameraCentreOffsetZ, 32, "fisheye_camera_centre_offset_z[i]");
      WRITE_CODE((UInt)area.m_fisheyeFieldOfView, 32, "fisheye_field_of_view[i]");
      WRITE_CODE((UInt)area.m_fisheyePolynomialCoeff.size(), 16, "fisheye_num_polynomial_coeffs[i]");

      for (std::size_t j = 0; j < area.m_fisheyePolynomialCoeff.size(); j++)
      {
        WRITE_SCODE((Int)area.m_fisheyePolynomialCoeff[j], 32, "fisheye_polynomial_coeff[i][j]");
      }
    }
  }
}

Void SEIWriter::xWriteSEIColourRemappingInfo(const SEIColourRemappingInfo& sei)
{
  WRITE_UVLC( sei.m_colourRemapId,                             "colour_remap_id" );
  WRITE_FLAG( sei.m_colourRemapCancelFlag,                     "colour_remap_cancel_flag" );
  if( !sei.m_colourRemapCancelFlag ) 
  {
    WRITE_FLAG( sei.m_colourRemapPersistenceFlag,              "colour_remap_persistence_flag" );
    WRITE_FLAG( sei.m_colourRemapVideoSignalInfoPresentFlag,   "colour_remap_video_signal_info_present_flag" );
    if ( sei.m_colourRemapVideoSignalInfoPresentFlag )
    {
      WRITE_FLAG( sei.m_colourRemapFullRangeFlag,              "colour_remap_full_range_flag" );
      WRITE_CODE( sei.m_colourRemapPrimaries,               8, "colour_remap_primaries" );
      WRITE_CODE( sei.m_colourRemapTransferFunction,        8, "colour_remap_transfer_function" );
      WRITE_CODE( sei.m_colourRemapMatrixCoefficients,      8, "colour_remap_matrix_coefficients" );
    }
    WRITE_CODE( sei.m_colourRemapInputBitDepth,             8, "colour_remap_input_bit_depth" );
    WRITE_CODE( sei.m_colourRemapBitDepth,                  8, "colour_remap_bit_depth" );
    for( Int c=0 ; c<3 ; c++ )
    {
      WRITE_CODE( sei.m_preLutNumValMinus1[c],              8, "pre_lut_num_val_minus1[c]" );
      if( sei.m_preLutNumValMinus1[c]>0 )
      {
        for( Int i=0 ; i<=sei.m_preLutNumValMinus1[c] ; i++ )
        {
          WRITE_CODE( sei.m_preLut[c][i].codedValue,  (( sei.m_colourRemapInputBitDepth + 7 ) >> 3 ) << 3, "pre_lut_coded_value[c][i]" );
          WRITE_CODE( sei.m_preLut[c][i].targetValue, (( sei.m_colourRemapBitDepth      + 7 ) >> 3 ) << 3, "pre_lut_target_value[c][i]" );
        }
      }
    }
    WRITE_FLAG( sei.m_colourRemapMatrixPresentFlag,            "colour_remap_matrix_present_flag" );
    if( sei.m_colourRemapMatrixPresentFlag )
    {
      WRITE_CODE( sei.m_log2MatrixDenom,                    4, "log2_matrix_denom" );
      for( Int c=0 ; c<3 ; c++ )
      {
        for( Int i=0 ; i<3 ; i++ )
        {
          WRITE_SVLC( sei.m_colourRemapCoeffs[c][i],           "colour_remap_coeffs[c][i]" );
        }
      }
    }

    for( Int c=0 ; c<3 ; c++ )
    {
      WRITE_CODE( sei.m_postLutNumValMinus1[c],             8, "m_postLutNumValMinus1[c]" );
      if( sei.m_postLutNumValMinus1[c]>0 )
      {
        for( Int i=0 ; i<=sei.m_postLutNumValMinus1[c] ; i++ )
        {
          WRITE_CODE( sei.m_postLut[c][i].codedValue, (( sei.m_colourRemapBitDepth + 7 ) >> 3 ) << 3, "post_lut_coded_value[c][i]" );
          WRITE_CODE( sei.m_postLut[c][i].targetValue, (( sei.m_colourRemapBitDepth + 7 ) >> 3 ) << 3, "post_lut_target_value[c][i]" );
        }
      }
    }
  }
}


Void SEIWriter::xWriteSEIDeinterlaceFieldIdentification(const SEIDeinterlaceFieldIdentification& sei)
{
  WRITE_FLAG( sei.m_deinterlacedPictureSourceParityFlag, "deinterlaced_picture_source_parity_flag" );
}


Void SEIWriter::xWriteSEIContentLightLevelInfo(const SEIContentLightLevelInfo& sei)
{
  WRITE_CODE( sei.m_maxContentLightLevel,    16, "max_content_light_level"     );
  WRITE_CODE( sei.m_maxPicAverageLightLevel, 16, "max_pic_average_light_level" );
}


Void SEIWriter::xWriteSEIDependentRAPIndication(const SEIDependentRAPIndication& /*sei*/)
{
  // intentionally empty
}


Void SEIWriter::xWriteSEICodedRegionCompletion(const SEICodedRegionCompletion& sei)
{
  WRITE_UVLC( sei.m_nextSegmentAddress, "next_segment_address" );
  if (sei.m_nextSegmentAddress)
  {
    WRITE_FLAG( sei.m_independentSliceSegmentFlag, "independent_slice_segment_flag" );
  }
}


Void SEIWriter::xWriteSEIAlternativeTransferCharacteristics(const SEIAlternativeTransferCharacteristics& sei)
{
  WRITE_CODE(sei.m_preferredTransferCharacteristics, 8, "preferred_transfer_characteristics");
}


Void SEIWriter::xWriteSEIAmbientViewingEnvironment(const SEIAmbientViewingEnvironment& sei)
{
  WRITE_CODE(sei.m_ambientIlluminance, 32, "ambient_illuminance" );
  WRITE_CODE(sei.m_ambientLightX,      16, "ambient_light_x" );
  WRITE_CODE(sei.m_ambientLightY,      16, "ambient_light_y" );
}

Void SEIWriter::xWriteSEIRegionalNesting(TComBitIf& bs, const SEIRegionalNesting& sei, const TComSPS *sps)
{
  WRITE_CODE(sei.getRNId(),            16, "regional_nesting_id");
  WRITE_CODE(sei.getNumRectRegions(),   8, "regional_nesting_num_rect_regions");
  const RNSEIWindowVec regions = sei.getRegions();
  for(RNSEIWindowVec::const_iterator it = regions.begin(); it != regions.end(); it++)
  {
    assert((*it).getWindowEnabledFlag());
    WRITE_CODE((*it).getRegionId(),            8, "regional_nesting_rect_region_id[i]");
    WRITE_CODE((*it).getWindowLeftOffset(),   16, "regional_nesting_rect_left_offset[i]");
    WRITE_CODE((*it).getWindowRightOffset(),  16, "regional_nesting_rect_right_offset[i]");
    WRITE_CODE((*it).getWindowTopOffset(),    16, "regional_nesting_rect_top_offset[i]");
    WRITE_CODE((*it).getWindowBottomOffset(), 16, "regional_nesting_rect_bottom_offset[i]");
  }
  assert(sei.getNumRnSEIMessage() >= 1);
  WRITE_CODE(sei.getNumRnSEIMessage()-1,   8, "num_sei_messages_in_regional_nesting_minus1");
  const std::vector<SEIRegionalNesting::SEIListOfIndices> seiMessages = sei.getRnSEIMessages();
  std::vector<SEIRegionalNesting::SEIListOfIndices>::const_iterator it;
  for(it = seiMessages.begin(); it != seiMessages.end(); it++)
  {
    std::vector<UInt> listOfRegions = (*it).m_listOfIndices;
    SEI *nestedSEI = (*it).m_seiMessage;
    WRITE_CODE((UInt)listOfRegions.size(),       8, "num_regions_for_sei_message[i]");
    for(Int j = 0; j < listOfRegions.size(); j++)
    {
      WRITE_CODE(listOfRegions[j],               8, "regional_nesting_sei_region_idx[i][j]");
    }
    xWriteSEImessage(bs, nestedSEI, sps);
  }
}

Void SEIWriter::xWriteSEIAnnotatedRegions(const SEIAnnotatedRegions &sei, const TComSPS *sps)
{
  WRITE_FLAG(sei.m_hdr.m_cancelFlag, "ar_cancel_flag");
  if (!sei.m_hdr.m_cancelFlag)
  {
    WRITE_FLAG(sei.m_hdr.m_notOptimizedForViewingFlag, "ar_not_optimized_for_viewing_flag");
    WRITE_FLAG(sei.m_hdr.m_trueMotionFlag, "ar_true_motion_flag");
    WRITE_FLAG(sei.m_hdr.m_occludedObjectFlag, "ar_occluded_object_flag");
    WRITE_FLAG(sei.m_hdr.m_partialObjectFlagPresentFlag, "ar_partial_object_flag_present_flag");
    WRITE_FLAG(sei.m_hdr.m_objectLabelPresentFlag, "ar_object_label_present_flag");
    WRITE_FLAG(sei.m_hdr.m_objectConfidenceInfoPresentFlag, "ar_object_confidence_info_present_flag");
    if (sei.m_hdr.m_objectConfidenceInfoPresentFlag)
    {
      assert(sei.m_hdr.m_objectConfidenceLength <= 16 && sei.m_hdr.m_objectConfidenceLength>0);
      WRITE_CODE((sei.m_hdr.m_objectConfidenceLength - 1), 4, "ar_object_confidence_length_minus_1");
    }
    if (sei.m_hdr.m_objectLabelPresentFlag)
    {
      WRITE_FLAG(sei.m_hdr.m_objectLabelLanguagePresentFlag, "ar_object_label_language_present_flag");
      if (sei.m_hdr.m_objectLabelLanguagePresentFlag)
      {
        xWriteByteAlign();
        assert(sei.m_hdr.m_annotatedRegionsObjectLabelLang.size()<256);
        for (UInt j = 0; j < sei.m_hdr.m_annotatedRegionsObjectLabelLang.size(); j++)
        {
          UChar ch = sei.m_hdr.m_annotatedRegionsObjectLabelLang[j];
          WRITE_CODE(ch, 8, "ar_object_label_language");
        }
        WRITE_CODE('\0', 8, "ar_label_language");
      }
    }
    WRITE_UVLC((UInt)sei.m_annotatedLabels.size(), "ar_num_label_updates");
    assert(sei.m_annotatedLabels.size()<256);
    for(auto it=sei.m_annotatedLabels.begin(); it!=sei.m_annotatedLabels.end(); it++)
    {
      assert(it->first < 256);
      WRITE_UVLC(it->first, "ar_label_idx[]");
      const SEIAnnotatedRegions::AnnotatedRegionLabel &ar=it->second;
      WRITE_FLAG(!ar.labelValid, "ar_label_cancel_flag");
      if (ar.labelValid)
      {
        xWriteByteAlign();
        assert(ar.label.size()<256);
        for (UInt j = 0; j < ar.label.size(); j++)
        {
          UChar ch = ar.label[j];
          WRITE_CODE(ch, 8, "ar_label[]");
        }
        WRITE_CODE('\0', 8, "ar_label[]");
      }
    }
    WRITE_UVLC((UInt)sei.m_annotatedRegions.size(), "ar_num_object_updates");
    assert(sei.m_annotatedRegions.size()<256);
    for (auto it=sei.m_annotatedRegions.begin(); it!=sei.m_annotatedRegions.end(); it++)
    {
      const SEIAnnotatedRegions::AnnotatedRegionObject &ar = it->second;
      WRITE_UVLC(it->first, "ar_object_idx");
      WRITE_FLAG(ar.objectCancelFlag, "ar_object_cancel_flag");
      if (!ar.objectCancelFlag)
      {
        if (sei.m_hdr.m_objectLabelPresentFlag)
        {
          WRITE_FLAG(ar.objectLabelValid, "ar_object_label_update_flag");
          if (ar.objectLabelValid)
          {
            assert(ar.objLabelIdx<256);
            WRITE_UVLC(ar.objLabelIdx, "ar_object_label_idx");
          }
        }
        WRITE_FLAG(ar.boundingBoxValid, "ar_object_bounding_box_update_flag");
        if (ar.boundingBoxValid)
        {
          WRITE_FLAG(ar.boundingBoxCancelFlag, "ar_object_bounding_box_cancel_flag");
          if (!ar.boundingBoxCancelFlag)
          {
            WRITE_CODE(ar.boundingBoxTop,   16, "ar_bounding_box_top");
            WRITE_CODE(ar.boundingBoxLeft,  16, "ar_bounding_box_left");
            WRITE_CODE(ar.boundingBoxWidth, 16, "ar_bounding_box_width");
            WRITE_CODE(ar.boundingBoxHeight,16, "ar_bounding_box_height");
            if (sei.m_hdr.m_partialObjectFlagPresentFlag)
            {
              WRITE_UVLC(ar.partialObjectFlag, "ar_partial_object_flag");
            }
            if (sei.m_hdr.m_objectConfidenceInfoPresentFlag)
            {
              assert(ar.objectConfidence < (1<<sei.m_hdr.m_objectConfidenceLength));
              WRITE_CODE(ar.objectConfidence, sei.m_hdr.m_objectConfidenceLength, "ar_object_confidence");
            }
          }
        }
      }
    }
  }
}

#if JCTVC_AD0021_SEI_MANIFEST
Void SEIWriter::xWriteSEISEIManifest(const SEIManifest& sei)
{
  WRITE_CODE(sei.m_manifestNumSeiMsgTypes, 16, "manifest_num_sei_msg_types");
  for (int i = 0; i < sei.m_manifestNumSeiMsgTypes; i++)
  {
    WRITE_CODE(sei.m_manifestSeiPayloadType[i], 16, "manifest_sei_payload_types");
    WRITE_CODE(sei.m_manifestSeiDescription[i], 8, "manifest_sei_description");
  }
}
#endif 

#if JCTVC_AD0021_SEI_PREFIX_INDICATION
//SEI prefix indication
Void SEIWriter::xWriteSEISEIPrefixIndication(TComBitIf& bs, const SEIPrefixIndication& sei, const TComSPS* sps)
{
  WRITE_CODE(sei.m_prefixSeiPayloadType, 16, "prefix_sei_payload_type");
  int idx = sei.m_numSeiPrefixIndicationsMinus1 + 1;
  //If num_sei_prefix_indication cannot be determined during initialization, then determine when writing prefix databits
  if (idx <= 1)
  {
    WRITE_CODE(sei.m_numSeiPrefixIndicationsMinus1, 8, "num_sei_prefix_indications_minus1");
  }
  // By writing SEI prefix indication recursively, you only need to pass in SEIPrefixIndicationIdx in the corresponding
  // function and add the SEI prefix syntax elements. At present, only part of SEI can be written in SEI prefix
  // indication. If it needs to be added later, the corresponding databit should be determined
  xWriteSEIpayloadData(bs, *static_cast<const SEI*>(sei.m_payload), sps, idx);
  xWriteSEIPrefixIndicationByteAlign();
}

Void SEIWriter::xWriteSEIPrefixIndicationByteAlign() {
  while (m_pcBitIf->getNumberOfWrittenBits() % 8 != 0)
  {
    WRITE_FLAG(1, "byte_alignment_bit_equal_to_one");
  }
}
// ~SEI prefix indication
#endif

Void SEIWriter::xWriteByteAlign()
{
#if JVET_AL0339_SPATIAL_RESOLUTION_FOR_FGC_SEI
  if (m_pcBitIf->getNumberOfWrittenBits() % 8 != 0 || m_SeiExtensionBitsPresentFlag)
#else
  if (m_pcBitIf->getNumberOfWrittenBits() % 8 != 0)
#endif
  {
    WRITE_FLAG(1, "payload_bit_equal_to_one");
    while (m_pcBitIf->getNumberOfWrittenBits() % 8 != 0)
    {
      WRITE_FLAG(0, "payload_bit_equal_to_zero");
    }
  }
}

#if JVET_AL0062_AI_USAGE_RESTRICTIONS_SEI
void SEIWriter::xWriteSEIAIUsageRestrictions(const SEIAIUsageRestrictions &sei)
{
  WRITE_FLAG(sei.m_cancelFlag, "aur_cancel_flag");
  
  if (!sei.m_cancelFlag)
  {
    WRITE_FLAG(sei.m_persistenceFlag, "aur_persistence_flag");
    WRITE_UVLC(sei.m_numRestrictionsMinus1, "aur_num_restrictions_minus1");
    for (uint32_t i = 0; i <= sei.m_numRestrictionsMinus1; i++)
    {
      WRITE_UVLC(sei.m_restrictions[i], "aur_restriction");
      WRITE_FLAG(sei.m_contextPresentFlag[i], "aur_context_present_flag");
      if (sei.m_contextPresentFlag[i])
        WRITE_UVLC(sei.m_context[i], "aur_context");
    }
  }
}
#endif

#if JVET_AK0194_DSC_SEI
void SEIWriter::xWriteSEIDigitallySignedContentInitialization(const SEIDigitallySignedContentInitialization &sei)
{
  WRITE_CODE(sei.dsciId, 8, "dsci_id");
  WRITE_CODE(sei.dsciHashMethodType, 8, "dsci_hash_method_type");
  WRITE_UVLC(sei.dsciKeyRetrievalModeIdc, "dsci_key_retrieval_mode_idc");
  if (sei.dsciKeyRetrievalModeIdc == 1)
  {
    WRITE_FLAG(sei.dsciUseKeyRegisterIdxFlag, "dsci_use_key_register_idx_flag");
    if( sei.dsciUseKeyRegisterIdxFlag )
    {
      WRITE_UVLC(sei.dsciKeyRegisterIdx, "dsci_key_register_idx");
    }
  }
  WRITE_FLAG(sei.dsciContentUuidPresentFlag, "dsci_content_uuid_present_flag");
  if (sei.dsciContentUuidPresentFlag)
  {
    for (int i=0; i<16; i++)
    {
      WRITE_CODE(sei.dsciContentUuid[i], 8, "dsci_content_uuid");
    }
  }
  CHECK (sei.dsciNumVerificationSubstreams < 1, "Number of DSC verification substreams has to be greater than zero");
  WRITE_UVLC(sei.dsciNumVerificationSubstreams - 1, "dsci_num_verification_substreams_minus1");
  for (int i = 1; i < sei.dsciNumVerificationSubstreams; i++)
  {
    for (int j = 0; j < i; j++)
    {
      WRITE_FLAG(sei.dsciRefSubstreamFlag[i][j], "dsci_ref_substream_flag");
    }
  }
  WRITE_FLAG(sei.dsciVSSImplicitAssociationModeFlag, "dsci_vss_implicit_association_mode_flag");
  WRITE_FLAG(sei.dsciSignedContentStartFlag, "dsci_signed_content_start_flag");
  WRITE_FLAG(sei.dsciSEISigningFlag, "dsci_sei_signing_flag");
  while (!isByteAligned())
  {
    WRITE_FLAG(0, "dsci_alignment_zero_bit");
  }
  WRITE_STRING(sei.dsciKeySourceUri, "dsci_key_source_uri");}

void SEIWriter::xWriteSEIDigitallySignedContentSelection(const SEIDigitallySignedContentSelection &sei)
{
  WRITE_CODE(sei.dscsId, 8, "dscs_id");
  WRITE_CODE(sei.dscsVerificationSubstreamId, 8, "dscs_verification_substream_id");
}

void SEIWriter::xWriteSEIDigitallySignedContentVerification(const SEIDigitallySignedContentVerification &sei)
{
  WRITE_CODE(sei.dscvId, 8, "dscv_id");
  WRITE_CODE(sei.dscvVerificationSubstreamId, 8, "dscv_verification_substream_id");
  CHECK (sei.dscvSignatureLengthInOctets < 1, "Length of signature has to be greater than zero");
  WRITE_CODE(sei.dscvSignatureLengthInOctets - 1, 24, "dscv_signature_length_in_octets_minus1");
  CHECK (sei.dscvSignatureLengthInOctets != sei.dscvSignature.size(), "Signature length incosistent");
  for (int i=0; i< sei.dscvSignature.size(); i++)
  {
    WRITE_CODE(sei.dscvSignature[i], 8, "dscv_signature");
  }
  WRITE_FLAG(sei.dscvSignedContentEndFlag, "dscv_signed_content_end_flag");
}

#endif

#if JVET_AK0140_PACKED_REGIONS_INFORMATION_SEI
void SEIWriter::xWriteSEIPackedRegionsInfo(const SEIPackedRegionsInfo& sei)
{
  WRITE_FLAG(sei.m_cancelFlag, "pri_cancel_flag");
  if (!sei.m_cancelFlag)
  {
    WRITE_FLAG(sei.m_persistenceFlag, "pri_persistence_flag");
    WRITE_UVLC(sei.m_numRegionsMinus1, "pri_num_regions_minus1");
    WRITE_FLAG(sei.m_multilayerFlag, "pri_multilayer_flag");
    WRITE_FLAG(sei.m_useMaxDimensionsFlag, "pri_use_max_dimensions_flag");
    WRITE_CODE(sei.m_log2UnitSize, 4, "pri_log2_unit_size");
    WRITE_CODE(sei.m_regionSizeLenMinus1, 4, "pri_region_size_len_minus1");
    WRITE_FLAG(sei.m_regionIdPresentFlag, "pri_region_id_present_flag");
    WRITE_FLAG(sei.m_targetPicParamsPresentFlag, "pri_target_pic_params_present_flag");
    if (sei.m_targetPicParamsPresentFlag)
    {
      WRITE_CODE(sei.m_targetPicWidthMinus1, 16, "pri_target_pic_width_minus1");
      WRITE_CODE(sei.m_targetPicHeightMinus1, 16, "pri_target_pic_height_minus1");
    }
    WRITE_UVLC(sei.m_numResamplingRatiosMinus1, "pri_num_resampling_ratios_minus1");
    for (uint32_t i = 1; i <= sei.m_numResamplingRatiosMinus1; i++)
    {
      WRITE_UVLC(sei.m_resamplingWidthNumMinus1[i], "pri_resampling_width_num_minus1[i]");
      WRITE_UVLC(sei.m_resamplingWidthDenomMinus1[i], "pri_resampling_width_denom_minus1[i]");
      WRITE_FLAG(sei.m_fixedAspectRatioFlag[i], "pri_fixed_aspect_ratio_flag[i]");
      if (!sei.m_fixedAspectRatioFlag[i])
      {
        WRITE_UVLC(sei.m_resamplingHeightNumMinus1[i], "pri_resampling_height_num_minus1[i]");
        WRITE_UVLC(sei.m_resamplingHeightDenomMinus1[i], "pri_resampling_height_denom_minus1[i]");
      }
    }
    for (uint32_t i = 0; i <= sei.m_numRegionsMinus1; i++)
    {
      if (sei.m_regionIdPresentFlag)
      {
        WRITE_UVLC(sei.m_regionId[i], "pri_region_id[i]");
      }
      if (sei.m_multilayerFlag)
      {
        WRITE_UVLC(sei.m_regionLayerId[i], "pri_region_layer_id[i]");
        WRITE_FLAG(sei.m_regionIsALayerFlag[i], "pri_region_is_a_layer_flag[i]");
      }
      if (!sei.m_regionIsALayerFlag[i])
      {
        WRITE_CODE(sei.m_regionTopLeftInUnitsX[i], sei.m_regionSizeLenMinus1 + 1, "pri_region_top_left_in_units_x[i]");
        WRITE_CODE(sei.m_regionTopLeftInUnitsY[i], sei.m_regionSizeLenMinus1 + 1, "pri_region_top_left_in_units_y[i]");
        WRITE_CODE(sei.m_regionWidthInUnitsMinus1[i], sei.m_regionSizeLenMinus1 + 1, "pri_region_width_in_units_minus1[i]");
        WRITE_CODE(sei.m_regionHeightInUnitsMinus1[i], sei.m_regionSizeLenMinus1 + 1, "pri_region_height_in_units_minus1[i]");
      }
      if (sei.m_numResamplingRatiosMinus1 > 0)
      {
        uint32_t codeLen = 0;
        for (uint32_t i = sei.m_numResamplingRatiosMinus1; i != 0; i >>= 1)
        {
          codeLen++;
        }
        WRITE_CODE(sei.m_resamplingRatioIdx[i], codeLen, "pri_resampling_ratio_idx[i]");
      }
      if (sei.m_targetPicParamsPresentFlag)
      {
        WRITE_CODE(sei.m_targetRegionTopLeftInUnitsX[i], sei.m_regionSizeLenMinus1 + 1, "pri_target_region_top_left_in_units_x[i]");
        WRITE_CODE(sei.m_targetRegionTopLeftInUnitsY[i], sei.m_regionSizeLenMinus1 + 1, "pri_target_region_top_left_in_units_y[i]");
      }
    }
  }
}
#endif
#if JVET_AJ0207_GFV
void SEIWriter::xWriteSEIGenerativeFaceVideo(const SEIGenerativeFaceVideo &sei)
{
  uint32_t basePicFlag = 0;
  std::vector<double>  coordinateXRec;
  std::vector<double>  coordinateYRec;
  std::vector<double>  coordinateZRec;
  std::vector<std::vector<std::vector<std::vector<double>>>>   matrixElementRec;
  WRITE_UVLC(sei.m_id, "gfv_id");
  WRITE_UVLC(sei.m_cnt, "gfv_cnt");
  if (sei.m_cnt == 0)
  {
    WRITE_FLAG(sei.m_basePicFlag, "gfv_base_picture_flag");
    basePicFlag = sei.m_basePicFlag;
  }
  else
  {
    basePicFlag = 0;
  }
  if (basePicFlag == 1)
  {
    WRITE_FLAG(sei.m_nnPresentFlag, "gfv_nn_present_flag");
    if (sei.m_nnPresentFlag)
    {
      WRITE_UVLC(sei.m_nnModeIdc, "gfv_mode_idc");
      if (sei.m_nnModeIdc == 1)
      {
        while (m_pcBitIf->getNumberOfWrittenBits() % 8 != 0)
        {
          WRITE_FLAG(0, "gfv_reserved_zero_bit_a");
        }
        WRITE_STRING(sei.m_nnTagURI, "gfv_uri_tag");
        WRITE_STRING(sei.m_nnURI, "gfv_uri");
      }
    }
    WRITE_FLAG(sei.m_chromaKeyInfoPresentFlag, "gfv_chroma_key_info_presentFlag");
    if (sei.m_chromaKeyInfoPresentFlag)
    {
      for (uint32_t chromac = 0; chromac < 3; chromac++)
      {
        WRITE_FLAG(sei.m_chromaKeyValuePresentFlag[chromac], "gfv_chroma_key_value_present_flag[c]");
        if (sei.m_chromaKeyValuePresentFlag[chromac])
        {
          WRITE_CODE(sei.m_chromaKeyValue[chromac], 8, "gfv_chroma_key_value[chromac]");
        }
      }
      for (uint32_t chromai = 0; chromai < 2; chromai++)
      {
        WRITE_FLAG(sei.m_chromaKeyThrPresentFlag[chromai], "gfv_chroma_key_thr_present_flag[i]");
        if (sei.m_chromaKeyThrPresentFlag[chromai])
        {
          WRITE_UVLC(sei.m_chromaKeyThrValue[chromai], "gfv_chroma_key_thr_value[i]");
        }
      }
    }
  }
  else
  {
    WRITE_FLAG(sei.m_drivePicFusionFlag, "gfv_drive_picture_fusion_flag");
  }
  WRITE_FLAG(sei.m_lowConfidenceFaceParameterFlag, "gfv_low_confidence_face_parameter_flag");
  WRITE_FLAG(sei.m_coordinatePresentFlag, "gfv_coordinate_present_flag");
  if (sei.m_coordinatePresentFlag)
  {
    WRITE_FLAG(sei.m_coordinatePredFlag, "gfv_kps_pred_flag");
    if (basePicFlag || !sei.m_coordinatePredFlag)
    {
      uint32_t gfvCoordinatePrecisionFactorMinus1 = sei.m_coordinateQuantizationFactor - 1;
      assert(gfvCoordinatePrecisionFactorMinus1 >= 0 && gfvCoordinatePrecisionFactorMinus1 <= 31);
      WRITE_UVLC(gfvCoordinatePrecisionFactorMinus1, "gfv_coordinate_precision_factor_minus1");
      uint32_t gfvNumKpsMinus1 = sei.m_coordinatePointNum - 1;
      WRITE_UVLC(gfvNumKpsMinus1, "gfv_num_kps_minus1");
      WRITE_FLAG(sei.m_3DCoordinateFlag, "gfv_coordinate_z_present_flag");
      if (sei.m_3DCoordinateFlag == 1)
      {
        uint32_t gfvCoordinateZMaxValueMinus1 = sei.m_coordinateZMaxValue - 1;
        assert(gfvCoordinateZMaxValueMinus1 >= 0 && gfvCoordinateZMaxValueMinus1 <= (1 << 16) - 1);
        WRITE_UVLC(gfvCoordinateZMaxValueMinus1, "gfv_coordinate_z_max_value_minus1");
      }
    }
    //X_coordinate_tensor && Y_coordinate_tensor  && Z_coordinate_tensor
    for (uint32_t i = 0; i < sei.m_coordinatePointNum; i++)
    {
      if (!sei.m_coordinatePredFlag)
      {
        // X_coordinate_tensor
        int curCoordinateXInt = (int)(sei.m_coordinateX[i] * (1 << sei.m_coordinateQuantizationFactor) + 0.5);
        int curCoordinateXIntAbs = abs(curCoordinateXInt);
        WRITE_UVLC(curCoordinateXIntAbs, "gfv_coordinate_x_abs[ i ]");
        if (curCoordinateXIntAbs)
        {
          const int signflag = curCoordinateXInt <= 0;
          WRITE_FLAG(signflag, "gfv_coordinate_x_sign_flag[ i ]");
        }
        double coordinateXTensorAbsRec = ((double)curCoordinateXInt) / (1 << sei.m_coordinateQuantizationFactor);
        coordinateXRec.push_back(coordinateXTensorAbsRec);
        // Y_coordinate_tensor
        int curCoordinateYInt = (int)(sei.m_coordinateY[i] * (1 << sei.m_coordinateQuantizationFactor) + 0.5);
        int curCoordinateYIntAbs = abs(curCoordinateYInt);
        WRITE_UVLC(curCoordinateYIntAbs, "gfv_coordinate_y_abs[ i ]");
        if (curCoordinateYIntAbs)
        {
          const int signflag = curCoordinateYInt <= 0;
          WRITE_FLAG(signflag, "gfv_coordinate_y_sign_flag[ i ]");
        }
        double coordinateYTensorAbsRec = ((double)curCoordinateYInt) / (1 << sei.m_coordinateQuantizationFactor);
        coordinateYRec.push_back(coordinateYTensorAbsRec);
        // Z_coordinate_tensor
        if (sei.m_3DCoordinateFlag == 1)
        {
          int curCoordinateZInt = (int)(sei.m_coordinateZ[i] * (1 << sei.m_coordinateQuantizationFactor) + 0.5);
          int curCoordinateZIntAbs = abs(curCoordinateZInt);
          WRITE_UVLC(curCoordinateZIntAbs, "gfv_coordinate_z_abs[ i ]");
          if (curCoordinateZIntAbs)
          {
            const int signflag = curCoordinateZInt <= 0;
            WRITE_FLAG(signflag, "gfv_coordinate_z_sign_flag[ i ]");
          }
          double coordinateZTensorAbsRec = (((double)curCoordinateZInt / (1 << sei.m_coordinateQuantizationFactor)));
          coordinateZRec.push_back(coordinateZTensorAbsRec);
        }
      }
      else
      {
        //Inter-frame difference
        int curCoordinateXInt = (int)((sei.m_coordinateX[i] - (basePicFlag ? (i == 0 ? 0 : coordinateXRec[i - 1]) : (sei.m_cnt == 0 ? baseCoordinateXRec[i] : prevcoordinateXRec[i]))) * (1 << sei.m_coordinateQuantizationFactor) + 0.5);
        double coordinateXTensorAbsRec = ((double)curCoordinateXInt) / (1 << sei.m_coordinateQuantizationFactor);
        coordinateXRec.push_back(coordinateXTensorAbsRec + (basePicFlag ? (i == 0 ? 0 : coordinateXRec[i - 1]) : (sei.m_cnt == 0 ? baseCoordinateXRec[i] : prevcoordinateXRec[i])));
        int curCoordinateXIntAbs = abs(curCoordinateXInt);
        WRITE_UVLC(curCoordinateXIntAbs, "gfv_coordinate_dx_abs[ i ]");
        if (curCoordinateXIntAbs)
        {
          const int signflag = curCoordinateXInt <= 0;
          WRITE_FLAG(signflag, "gfv_coordinate_dx_sign_flag[ i ]");
        }
        int curCoordinateYInt = (int)((sei.m_coordinateY[i] - (basePicFlag ? (i == 0 ? 0 : coordinateYRec[i - 1]) : (sei.m_cnt == 0 ? baseCoordinateYRec[i] : prevcoordinateYRec[i]))) * (1 << sei.m_coordinateQuantizationFactor) + 0.5);
        double coordinateYTensorAbsRec = ((double)curCoordinateYInt) / (1 << sei.m_coordinateQuantizationFactor);
        coordinateYRec.push_back(coordinateYTensorAbsRec + (basePicFlag ? (i == 0 ? 0 : coordinateYRec[i - 1]) : (sei.m_cnt == 0 ? baseCoordinateYRec[i] : prevcoordinateYRec[i])));
        int curCoordinateYIntAbs = abs(curCoordinateYInt);
        WRITE_UVLC(curCoordinateYIntAbs, "gfv_coordinate_dy_abs[ i ]");
        if (curCoordinateYIntAbs)
        {
          const int signflag = curCoordinateYInt <= 0;
          WRITE_FLAG(signflag, "gfv_coordinate_dy_sign_flag[ i ]");
        }
        if (sei.m_3DCoordinateFlag == 1)
        {
          int curCoordinateZInt = (int)((sei.m_coordinateZ[i] - (basePicFlag ? (i == 0 ? 0 : coordinateZRec[i - 1]) : (sei.m_cnt == 0 ? baseCoordinateZRec[i] : prevcoordinateZRec[i]))) * (1 << sei.m_coordinateQuantizationFactor) + 0.5);
          double coordinateZTensorAbsRec = ((double)curCoordinateZInt / (1 << sei.m_coordinateQuantizationFactor));
          coordinateZRec.push_back(coordinateZTensorAbsRec + (basePicFlag ? (i == 0 ? 0 : coordinateZRec[i - 1]) : (sei.m_cnt == 0 ? baseCoordinateZRec[i] : prevcoordinateZRec[i])));
          int curCoordinateZIntAbs = abs(curCoordinateZInt);
          WRITE_UVLC(curCoordinateZIntAbs, "gfv_coordinate_dz_abs[ i ]");
          if (curCoordinateZIntAbs)
          {
            const int signflag = curCoordinateZInt <= 0;
            WRITE_FLAG(signflag, "gfv_coordinate_dz_sign_flag[ i ]");
          }
        }
      }
    }
    if (doUpdateGFVcoordinate)
    {
      prevcoordinateXRec = coordinateXRec;
      prevcoordinateYRec = coordinateYRec;
      if (sei.m_3DCoordinateFlag == 1)
      {
        prevcoordinateZRec = coordinateZRec;
      }
      if (sei.m_basePicFlag)
      {
        baseCoordinateXRec = coordinateXRec;
        baseCoordinateYRec = coordinateYRec;
        if (sei.m_3DCoordinateFlag == 1)
        {
          baseCoordinateZRec = coordinateZRec;
        }
      }
      doUpdateGFVcoordinate = false;
    }
    else
    {
      doUpdateGFVcoordinate = true;
    }
  }
  // Matrix Parameters
  assert(sei.m_coordinatePresentFlag || sei.m_matrixPresentFlag);
  WRITE_FLAG(sei.m_matrixPresentFlag, "gfv_matrix_present_flag");

  if (sei.m_matrixPresentFlag)
  {
    std::vector<uint32_t> matrixWidthVec;
    std::vector<uint32_t> matrixHeightVec;
    std::vector<uint32_t> numMatricesVec;
    if (!basePicFlag)
    {
      WRITE_FLAG(sei.m_matrixPredFlag, "gfv_matrix_pred_flag");
    }
    if (!sei.m_matrixPredFlag)
    {
      uint32_t gfvMatrixElementPrecisionFactorMinus1 = sei.m_matrixElementPrecisionFactor - 1;
      assert(gfvMatrixElementPrecisionFactorMinus1 >= 0 && gfvMatrixElementPrecisionFactorMinus1 <= 31);
      WRITE_UVLC(gfvMatrixElementPrecisionFactorMinus1, "gfv_matrix_element_precision_factor_minus1");
      uint32_t gfvNumMatrixTypesMinus1 = sei.m_numMatrixType - 1;
      WRITE_UVLC(gfvNumMatrixTypesMinus1, "gfv_num_matrix_types_minus1");

      for (uint32_t matrixId = 0; matrixId < sei.m_numMatrixType; matrixId++)
      {
        assert(sei.m_matrixTypeIdx[matrixId] >= 0 && sei.m_matrixTypeIdx[matrixId] <= 63);
        WRITE_CODE(sei.m_matrixTypeIdx[matrixId], 6, "gfv_matrix_type_idx");
        if (sei.m_matrixTypeIdx[matrixId] == 0 || sei.m_matrixTypeIdx[matrixId] == 1)
        {
          assert(sei.m_coordinatePresentFlag != 0);
          WRITE_FLAG(sei.m_numMatricestonumKpsFlag[matrixId], "gfv_num_matrices_equal_to_num_kps_flag");
          if (!sei.m_numMatricestonumKpsFlag[matrixId])
          {
            assert(sei.m_numMatricesInfo[matrixId] >= 0 && sei.m_numMatricesInfo[matrixId] <= (1 << 10) - 1);
            WRITE_UVLC(sei.m_numMatricesInfo[matrixId], "gfv_num_matrices_info");
          }
        }
        else if (sei.m_matrixTypeIdx[matrixId] == 2 || sei.m_matrixTypeIdx[matrixId] == 3 || sei.m_matrixTypeIdx[matrixId] >= 7)
        {
          if (sei.m_matrixTypeIdx[matrixId] >= 7)
          {
            uint32_t gfvNumMatricesMinus1 = sei.m_numMatrices[matrixId] - 1;
            assert(gfvNumMatricesMinus1 >= 0 && gfvNumMatricesMinus1 <=(1 << 10) - 1);
            WRITE_UVLC(gfvNumMatricesMinus1, "gfv_num_matrices_minus1");
          }
          uint32_t gfvMatrixWidthMinus1 = sei.m_matrixWidth[matrixId] - 1;
          WRITE_UVLC(gfvMatrixWidthMinus1, "gfv_matrix_width_minus1");
          assert(gfvMatrixWidthMinus1 >= 0 && gfvMatrixWidthMinus1 <=(1 << 10) - 1);
          uint32_t gfvMatrixHeightMinus1 = sei.m_matrixHeight[matrixId] - 1;
          WRITE_UVLC(gfvMatrixHeightMinus1, "gfv_matrix_height_minus1");
          assert(gfvMatrixHeightMinus1 >= 0 && gfvMatrixHeightMinus1 <=(1 << 10) - 1);
        }
        else if (sei.m_matrixTypeIdx[matrixId] >= 4 && sei.m_matrixTypeIdx[matrixId] <= 6)
        {
          if (!sei.m_coordinatePresentFlag)
          {
            WRITE_FLAG(sei.m_matrix3DSpaceFlag[matrixId], "gfv_Matrix3DSpaceFlag");
          }
        }
      }
    }
    if (sei.m_matrixPredFlag)
    {
      numMatricesVec = baseNumMatricesVec;
      matrixHeightVec = baseMatrixHeightVec;
      matrixWidthVec = baseMatrixWidthVec;
    }
    else
    {
      numMatricesVec = sei.m_numMatricesstore;
      matrixHeightVec = sei.m_matrixHeightstore;
      matrixWidthVec = sei.m_matrixWidthstore;
    }
    for (uint32_t matrixId = 0; matrixId < sei.m_numMatrixType; matrixId++)
    {
      matrixElementRec.push_back(std::vector<std::vector<std::vector<double>>>());
      for (uint32_t j = 0; j < numMatricesVec[matrixId]; j++)
      {
        matrixElementRec[matrixId].push_back(std::vector<std::vector<double>>());
        for (uint32_t k = 0; k < matrixHeightVec[matrixId]; k++)
        {
          matrixElementRec[matrixId][j].push_back(std::vector<double>());
          for (uint32_t l = 0; l < matrixWidthVec[matrixId]; l++)
          {
            if (!sei.m_matrixPredFlag)
            {
              double curMatrixElementAbs = fabs(sei.m_matrixElement[matrixId][j][k][l]);
              int curMatrixElementAbsInt = (int)(curMatrixElementAbs);
              WRITE_UVLC(curMatrixElementAbsInt, "gfv_matrix_element_int");
              double curMatrixElementAbsDecimal = curMatrixElementAbs - curMatrixElementAbsInt;
              assert(curMatrixElementAbsDecimal >= 0);
              int curMatrixElementAbsDecIntValue = Clip3(0, (1 << sei.m_matrixElementPrecisionFactor) - 1, (int)(curMatrixElementAbsDecimal * (1 << sei.m_matrixElementPrecisionFactor) + 0.5));
              WRITE_CODE(curMatrixElementAbsDecIntValue, sei.m_matrixElementPrecisionFactor, "gfv_matrix_element_dec");
              const int signflag = sei.m_matrixElement[matrixId][j][k][l] < 0;
              if (curMatrixElementAbsInt || curMatrixElementAbsDecIntValue)
              {
                WRITE_FLAG(signflag, "gfv_matrix_element_sign_flag");
              }
              double matrixElementAbsRec = (double)(curMatrixElementAbsInt + (((double)curMatrixElementAbsDecIntValue) / (1 << sei.m_matrixElementPrecisionFactor)));
              matrixElementRec[matrixId][j][k].push_back(signflag ? -matrixElementAbsRec : matrixElementAbsRec);

            }
            else
            {
              double curMatrixElementAbs = fabs(sei.m_matrixElement[matrixId][j][k][l] - (sei.m_cnt == 0 ? baseMatrixRec[matrixId][j][k][l] : prevMatrixRec[matrixId][j][k][l]));
              int curMatrixElementAbsInt = (int)curMatrixElementAbs;
              WRITE_UVLC(curMatrixElementAbsInt, "gfv_matrix_delta_element_int");
              double curMatrixElementAbsDecimal = curMatrixElementAbs - curMatrixElementAbsInt;
              assert(curMatrixElementAbsDecimal >= 0);
              int curMatrixElementAbsDecIntValue = (int)(curMatrixElementAbsDecimal* (1 << sei.m_matrixElementPrecisionFactor) + 0.5);
              WRITE_UVLC(curMatrixElementAbsDecIntValue, "gfv_matrix_element_dec");
              const int signflag = (sei.m_matrixElement[matrixId][j][k][l] - (sei.m_cnt == 0 ? baseMatrixRec[matrixId][j][k][l] : prevMatrixRec[matrixId][j][k][l])) < 0;
              if (curMatrixElementAbsInt || curMatrixElementAbsDecIntValue)
              {
                WRITE_FLAG(signflag, "gfv_matrix_delta_element_sign_flag");
              }
              double matrixElementAbsRec = (double)(curMatrixElementAbsInt + (((double)curMatrixElementAbsDecIntValue) / (1 << sei.m_matrixElementPrecisionFactor)));
              matrixElementRec[matrixId][j][k].push_back((signflag ? -matrixElementAbsRec : matrixElementAbsRec) + (sei.m_cnt == 0 ? baseMatrixRec[matrixId][j][k][l] : prevMatrixRec[matrixId][j][k][l]));
            }
          }
        }
      }
    }
    if (doUpdateGFVmatrix)
    {
      prevMatrixRec = matrixElementRec;
      if (basePicFlag)
      {
        baseMatrixRec = matrixElementRec;
        baseNumMatricesVec = numMatricesVec;
        baseMatrixHeightVec = matrixHeightVec;
        baseMatrixWidthVec = matrixWidthVec;
      }
      doUpdateGFVmatrix = false;
    }
    else
    {
      doUpdateGFVmatrix = true;
    }
  }
  if (sei.m_nnPresentFlag)
  {
    if (sei.m_nnModeIdc == 0)
    {
      while (m_pcBitIf->getNumberOfWrittenBits() % 8 != 0)
      {
        WRITE_FLAG(0, "gfv_reserved_zero_bit_b");
      }
      for (long i = 0; i < sei.m_payloadLength; i++)
      {
        WRITE_SCODE(sei.m_payloadByte[i], 8, "gfv_nn_payload_byte[i]");
      }
    }
  }
}
#endif
#if JVET_AK0239_GEFV
Void SEIWriter::xWriteSEIGenerativeFaceVideoEnhancement(const SEIGenerativeFaceVideoEnhancement &sei)
{
  uint32_t basePicFlag = 0;
  WRITE_UVLC(sei.m_id, "gefv_id");
  WRITE_UVLC(sei.m_gfvid, "gefv_gfv_id");
  WRITE_UVLC(sei.m_gfvcnt, "gefv_gfv_cnt");

  if (sei.m_gfvcnt == 0)
  {
    WRITE_FLAG(sei.m_basePicFlag, "gefv_base_picture_flag");
    basePicFlag = sei.m_basePicFlag;
  }
  else
  {
    basePicFlag = 0;
  }
  if (basePicFlag)
  {
    WRITE_FLAG(sei.m_nnPresentFlag, "gefv_nnPresentFlag");
    if (sei.m_nnPresentFlag)
    {
      WRITE_UVLC(sei.m_nnModeIdc, "gefv_mode_idc");
      if (sei.m_nnModeIdc == 1)
      {
        while (m_pcBitIf->getNumberOfWrittenBits() % 8 != 0)
        {
          WRITE_FLAG(0, "gefv_nn_alignment_zero_bit_a");
        }
        WRITE_STRING(sei.m_nnTagURI, "gefv_uri_tag");
        WRITE_STRING(sei.m_nnURI, "gefv_uri");
      }
    }
  }
  // Matrix Parameters
  WRITE_FLAG(sei.m_matrixPresentFlag, "gefv_matrix_present_flag");
  if (sei.m_matrixPresentFlag)
  {
    std::vector<std::vector<std::vector<double>>>   gfveMatrixElementRec;
    uint32_t numMatrices = 0;
    uint32_t   matrixElementPrecisionFactor = 0;
    std::vector<uint32_t> matrixHeightVec;
    std::vector<uint32_t> matrixWidthVec;
    if (!basePicFlag)
    {
      WRITE_FLAG(sei.m_matrixPredFlag, "gefv_matrix_pred_flag");
    }
    if (!sei.m_matrixPredFlag)
    {
      uint32_t gfveMatrixElementPrecisionFactorMinus1 = sei.m_matrixElementPrecisionFactor - 1;
      assert(gfveMatrixElementPrecisionFactorMinus1 >= 0 && gfveMatrixElementPrecisionFactorMinus1 <= 31);
      WRITE_UVLC(gfveMatrixElementPrecisionFactorMinus1, "gefv_matrix_element_precision_factor_minus1");
      uint32_t gfveNumMatricesMinus1 = sei.m_numMatrices - 1;
      assert(gfveNumMatricesMinus1 >= 0 && gfveNumMatricesMinus1 <=(1 << 10) - 1);
      WRITE_UVLC(gfveNumMatricesMinus1, "gefv_num_matrices_minus1");
      numMatrices = gfveNumMatricesMinus1 + 1;
      matrixElementPrecisionFactor = gfveMatrixElementPrecisionFactorMinus1 + 1;
      if (basePicFlag)
      {
        baseGfveNumMatrices = gfveNumMatricesMinus1 + 1;
        baseMatrixElementPrecisionFactor = gfveMatrixElementPrecisionFactorMinus1 + 1;
      }
      for (uint32_t j = 0; j <= gfveNumMatricesMinus1; j++)
      {
        uint32_t gfveMatrixHeightMinus1 = sei.m_matrixHeight[j] - 1;
        WRITE_UVLC(gfveMatrixHeightMinus1, "gefv_matrix_height_minus1");
        uint32_t gfveMatrixWidthMinus1 = sei.m_matrixWidth[j] - 1;
        WRITE_UVLC(gfveMatrixWidthMinus1, "gefv_matrix_width_minus1");
        matrixHeightVec.push_back(sei.m_matrixHeight[j]);
        matrixWidthVec.push_back(sei.m_matrixWidth[j]);
        if (basePicFlag && doUpdateGFVEmatrix)
        {
          baseGfveMatrixHeightVec.push_back(sei.m_matrixHeight[j]);
          baseGfveMatrixWidthVec.push_back(sei.m_matrixWidth[j]);
        }
      }
    }
    else
    {
      numMatrices = baseGfveNumMatrices;
      matrixElementPrecisionFactor = baseMatrixElementPrecisionFactor;
      matrixHeightVec = baseGfveMatrixHeightVec;
      matrixWidthVec = baseGfveMatrixWidthVec;
    }
    for (uint32_t j = 0; j < numMatrices; j++)
    {
      gfveMatrixElementRec.push_back(std::vector<std::vector<double>>());
      for (uint32_t k = 0; k < matrixHeightVec[j]; k++)
      {
        gfveMatrixElementRec[j].push_back(std::vector<double>());
        for (uint32_t l = 0; l < matrixWidthVec[j]; l++)
        {
          if (!sei.m_matrixPredFlag)
          {
            double curMatrixElementAbs = fabs(sei.m_matrixElement[j][k][l]);
            uint32_t curMatrixElementAbsInt = (int)(curMatrixElementAbs);
            assert(curMatrixElementAbsInt >= 0 && curMatrixElementAbsInt <= 4294967294);
            WRITE_UVLC(curMatrixElementAbsInt, "gefv_matrix_element_int");
            double curMatrixElementAbsDecimal = curMatrixElementAbs - curMatrixElementAbsInt;
            assert(curMatrixElementAbsDecimal >= 0);
            int curMatrixElementAbsDecIntValue = Clip3(0, (1 << matrixElementPrecisionFactor) - 1, (int)(curMatrixElementAbsDecimal * (1 << matrixElementPrecisionFactor) + 0.5));
            WRITE_CODE(curMatrixElementAbsDecIntValue, matrixElementPrecisionFactor, "gefv_matrix_element_dec");
            const int signflag = sei.m_matrixElement[j][k][l] < 0;
            if (curMatrixElementAbsInt || curMatrixElementAbsDecIntValue)
            {
              WRITE_FLAG(signflag, "gefv_matrix_element_sign_flag");
            }
            double matrixElementAbsRec = (double)(curMatrixElementAbsInt + (((double)curMatrixElementAbsDecIntValue) / (1 << matrixElementPrecisionFactor)));
            gfveMatrixElementRec[j][k].push_back(signflag ? -matrixElementAbsRec : matrixElementAbsRec);
          }
          else
          {
            double curMatrixElementAbs = fabs(sei.m_matrixElement[j][k][l] - (sei.m_gfvcnt == 0 ? baseGfveMatrixRec[j][k][l] : prevGfveMatrixRec[j][k][l]));
            uint32_t curMatrixElementAbsInt = (int)curMatrixElementAbs;
            assert(curMatrixElementAbsInt >= 0 && curMatrixElementAbsInt <= 4294967294);
            WRITE_UVLC(curMatrixElementAbsInt, "gefv_matrix_delta_element_int");
            double curMatrixElementAbsDecimal = curMatrixElementAbs - curMatrixElementAbsInt;
            assert(curMatrixElementAbsDecimal >= 0);
            int curMatrixElementAbsDecIntValue = (int)(curMatrixElementAbsDecimal* (1 << matrixElementPrecisionFactor) + 0.5);
            WRITE_CODE(curMatrixElementAbsDecIntValue, matrixElementPrecisionFactor, "gefv_matrix_delta_element_dec");
            const int signflag = (sei.m_matrixElement[j][k][l] - (sei.m_gfvcnt == 0 ? baseGfveMatrixRec[j][k][l] : prevGfveMatrixRec[j][k][l])) < 0;
            if (curMatrixElementAbsInt || curMatrixElementAbsDecIntValue)
            {
              WRITE_FLAG(signflag, "gefv_matrix_delta_element_sign_flag");
            }
            double matrixElementAbsRec = (double)(curMatrixElementAbsInt + (((double)curMatrixElementAbsDecIntValue) / (1 << baseMatrixElementPrecisionFactor)));
            gfveMatrixElementRec[j][k].push_back((signflag ? -matrixElementAbsRec : matrixElementAbsRec) + (sei.m_gfvcnt == 0 ? baseGfveMatrixRec[j][k][l] : prevGfveMatrixRec[j][k][l]));
          }
        }
      }
    }
    if (doUpdateGFVEmatrix)
    {
      prevGfveMatrixRec = gfveMatrixElementRec;
      if (basePicFlag)
      {
        baseGfveMatrixRec = gfveMatrixElementRec;
      }
      doUpdateGFVEmatrix = false;
    }
    else
    {
      doUpdateGFVEmatrix = true;
    }
  }
  double gfveLeftPupilCoordinateXRec;
  double gfveLeftPupilCoordinateYRec;
  double gfveRightPupilCoordinateXRec;
  double gfveRightPupilCoordinateYRec;
  assert(sei.m_pupilPresentIdx >= 0 && sei.m_pupilPresentIdx <= 3);
  WRITE_CODE(sei.m_pupilPresentIdx, 2, "gefv_pupil_coordinate_present_idx");
  if (sei.m_pupilPresentIdx != 0)
  {
    if (basePicFlag)
    {
      checkBasePicPupilPresentIdx = true;

      uint32_t gfvePupilCoordinatePrecisionFactorMinus1 = sei.m_pupilCoordinatePrecisionFactor - 1;
      assert(gfvePupilCoordinatePrecisionFactorMinus1 >= 0 && gfvePupilCoordinatePrecisionFactorMinus1 <= 31);
      WRITE_UVLC(gfvePupilCoordinatePrecisionFactorMinus1, "gefv_pupil_coordinate_precision_factor_minus1");
    }
    assert(checkBasePicPupilPresentIdx);
  }
  if (checkBasePicPupilPresentIdx)
  {
    double gfveLeftPupilCoordinateXRef = 0.0;
    double gfveLeftPupilCoordinateYRef = 0.0;
    double gfveRightPupilCoordinateXRef = 0.0;
    double gfveRightPupilCoordinateYRef = 0.0;
    if (sei.m_gfvcnt == 0)
    {
      if (!basePicFlag)
      {
        gfveLeftPupilCoordinateXRef = basegfveLeftPupilCoordinateX;
        gfveLeftPupilCoordinateYRef = basegfveLeftPupilCoordinateY;
        gfveRightPupilCoordinateXRef = basegfveRightPupilCoordinateX;
        gfveRightPupilCoordinateYRef = basegfveRightPupilCoordinateY;
      }
    }
    else
    {
      gfveLeftPupilCoordinateXRef = prevgfveLeftPupilCoordinateX;
      gfveLeftPupilCoordinateYRef = prevgfveLeftPupilCoordinateY;
      gfveRightPupilCoordinateXRef = prevgfveRightPupilCoordinateX;
      gfveRightPupilCoordinateYRef = prevgfveRightPupilCoordinateY;
    }
    if (sei.m_pupilPresentIdx == 1 || sei.m_pupilPresentIdx == 3)
    {
      gfveLeftPupilCoordinateXRec = xWriteSEIPupilCoordinate(sei.m_pupilLeftEyeCoordinateX, gfveLeftPupilCoordinateXRef, sei.m_pupilCoordinatePrecisionFactor, "left", "x");
      gfveLeftPupilCoordinateYRec = xWriteSEIPupilCoordinate(sei.m_pupilLeftEyeCoordinateY, gfveLeftPupilCoordinateYRef, sei.m_pupilCoordinatePrecisionFactor, "left", "y");
    }
    else
    {
      gfveLeftPupilCoordinateXRec = gfveLeftPupilCoordinateXRef;
      gfveLeftPupilCoordinateYRec = gfveLeftPupilCoordinateYRef;
    }
    if (basePicFlag)
    {
      gfveRightPupilCoordinateXRef = gfveLeftPupilCoordinateXRec;
      gfveRightPupilCoordinateYRef = gfveLeftPupilCoordinateYRec;
    }
    if (2 == sei.m_pupilPresentIdx || 3 == sei.m_pupilPresentIdx)
    {
      gfveRightPupilCoordinateXRec = xWriteSEIPupilCoordinate(sei.m_pupilRightEyeCoordinateX, gfveRightPupilCoordinateXRef, sei.m_pupilCoordinatePrecisionFactor, "right", "x");
      gfveRightPupilCoordinateYRec = xWriteSEIPupilCoordinate(sei.m_pupilRightEyeCoordinateY, gfveRightPupilCoordinateYRef, sei.m_pupilCoordinatePrecisionFactor, "right", "y");
    }
    else
    {
      gfveRightPupilCoordinateXRec = gfveRightPupilCoordinateXRef;
      gfveRightPupilCoordinateYRec = gfveRightPupilCoordinateYRef;
    }
    if (doUpdateGFVPupilCoordinate)
    {
      if (basePicFlag)
      {
        basegfveLeftPupilCoordinateX = gfveLeftPupilCoordinateXRec;
        basegfveLeftPupilCoordinateY = gfveLeftPupilCoordinateYRec;
        basegfveRightPupilCoordinateX = gfveRightPupilCoordinateXRec;
        basegfveRightPupilCoordinateY = gfveRightPupilCoordinateYRec;
      }
      prevgfveLeftPupilCoordinateX = gfveLeftPupilCoordinateXRec;
      prevgfveLeftPupilCoordinateY = gfveLeftPupilCoordinateYRec;
      prevgfveRightPupilCoordinateX = gfveRightPupilCoordinateXRec;
      prevgfveRightPupilCoordinateY = gfveRightPupilCoordinateYRec;
      doUpdateGFVPupilCoordinate = false;
    }
    else
    {
      doUpdateGFVPupilCoordinate = true;
    }
  }
  if (sei.m_nnPresentFlag)
  {
    if (sei.m_nnModeIdc == 0)
    {
      while (m_pcBitIf->getNumberOfWrittenBits() % 8 != 0)
      {
        WRITE_FLAG(0, "gefv_nn_alignment_zero_bit_b");
      }
      for (long i = 0; i < sei.m_payloadLength; i++)
      {
        WRITE_SCODE(sei.m_payloadByte[i], 8, "gefv_nn_payload_byte[i]");
      }
    }
  }
}
double SEIWriter::xWriteSEIPupilCoordinate(double coordinate, double refCoordinate, int precisionFactor, const char* eye, const char* axis)
{
  double deltaAbs = fabs(coordinate - refCoordinate);
  int absIntValue;
  absIntValue = static_cast<int>(deltaAbs * (1 << precisionFactor) + 0.5);

  assert(std::string(eye) == "left" || std::string(eye) == "right");
  assert(std::string(axis) == "x" || std::string(axis) == "y");
  std::string checkMessage = "The value of gefv_pupil_" + std::string(eye) + "_eye_d" + std::string(axis) + "_coordinate_abs shall be be 0 to 1 << (gefv_pupil_coordinate_precision_factor_minus1 + 2), inclusive";
  assert(absIntValue >= 0 && absIntValue <=(1 << (precisionFactor + 1)));
  std::string absSymbolName = "gefv_pupil_" + std::string(eye) + "_eye_d" + std::string(axis) + "_coordinate_abs";
  WRITE_UVLC(absIntValue, absSymbolName.c_str());

  const int signFlag = (coordinate - refCoordinate < 0) ? 1 : 0;
  if (absIntValue)
  {
  
    std::string signSymbolName = "gefv_pupil_" + std::string(eye) + "_eye_d" + std::string(axis) + "_coordinate_sign_flag";
    WRITE_FLAG(signFlag, signSymbolName.c_str());
  }
  double deltaAbsRec = static_cast<double>(absIntValue) / (1 << precisionFactor);
  return (signFlag ? -deltaAbsRec : deltaAbsRec) + refCoordinate;
}
#endif
#if JVET_AK2006_SPTI_SEI_MESSAGE
void SEIWriter::xWriteSEISourcePictureTimingInfo(const SEISourcePictureTimingInfo &sei) 
{
  WRITE_FLAG(sei.m_sptiCancelFlag, "spti_cancel_flag");
  if (!sei.m_sptiCancelFlag)
  {
    WRITE_FLAG(sei.m_sptiPersistenceFlag, "spti_persistance_flag");
    WRITE_FLAG(sei.m_sptiSourceTimingEqualsOutputTimingFlag, "spti_source_timing_equals_output_timing_flag");
    if (!sei.m_sptiSourceTimingEqualsOutputTimingFlag)
    {
      WRITE_FLAG(sei.m_sptiSourceTypePresentFlag,"spti_source_type_present_flag");
      if (sei.m_sptiSourceTypePresentFlag)
      {
        WRITE_CODE(sei.m_sptiSourceType, 16, "spti_source_type");
      }
      WRITE_CODE(sei.m_sptiTimeScale, 32, "spti_time_scale");
      WRITE_CODE(sei.m_sptiNumUnitsInElementalInterval, 32, "spti_num_units_in_elemental_interval");
      WRITE_FLAG(sei.m_sptiDirectionFlag, "spti_direction_flag");
      if (sei.m_sptiPersistenceFlag)
      {
        WRITE_CODE(sei.m_sptiMaxSublayersMinus1, 3, "spti_max_sublayers_minus_1");
      }
      int sptiMinTemporalSublayer = (sei.m_sptiPersistenceFlag ? 0 : sei.m_sptiMaxSublayersMinus1);
      for (int i = sptiMinTemporalSublayer; i <= sei.m_sptiMaxSublayersMinus1; i++)
      {
        WRITE_UVLC(sei.m_sptiSublayerIntervalScaleFactor[i],"spti_sublayer_interval_scale_factor");
        WRITE_FLAG(sei.m_sptiSublayerSynthesizedPictureFlag[i],"spti_sublayer_synthesized_picture_flag");
      }
    }
  }
}

#endif


//! \}
