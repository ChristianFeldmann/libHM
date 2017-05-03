/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2014, ITU/ISO/IEC
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

/** \file     TEncSlice.cpp
    \brief    slice encoder class
*/

#include "TEncTop.h"
#include "TEncSlice.h"
#include <math.h>

//! \ingroup TLibEncoder
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

TEncSlice::TEncSlice()
{
  m_apcPicYuvPred = NULL;
  m_apcPicYuvResi = NULL;

  m_pdRdPicLambda = NULL;
  m_pdRdPicQp     = NULL;
  m_piRdPicQp     = NULL;
  m_pcBufferSbacCoders    = NULL;
  m_pcBufferBinCoderCABACs  = NULL;
  m_pcBufferLowLatSbacCoders    = NULL;
  m_pcBufferLowLatBinCoderCABACs  = NULL;
}

TEncSlice::~TEncSlice()
{
  for (std::vector<TEncSbac*>::iterator i = CTXMem.begin(); i != CTXMem.end(); i++)
  {
    delete (*i);
  }
}

Void TEncSlice::initCtxMem(  UInt i )
{
  for (std::vector<TEncSbac*>::iterator j = CTXMem.begin(); j != CTXMem.end(); j++)
  {
    delete (*j);
  }
  CTXMem.clear();
  CTXMem.resize(i);
}

Void TEncSlice::create( Int iWidth, Int iHeight, ChromaFormat chromaFormat, UInt iMaxCUWidth, UInt iMaxCUHeight, UChar uhTotalDepth )
{
  // create prediction picture
  if ( m_apcPicYuvPred == NULL )
  {
    m_apcPicYuvPred  = new TComPicYuv;
    m_apcPicYuvPred->create( iWidth, iHeight, chromaFormat, iMaxCUWidth, iMaxCUHeight, uhTotalDepth );
  }

  // create residual picture
  if( m_apcPicYuvResi == NULL )
  {
    m_apcPicYuvResi  = new TComPicYuv;
    m_apcPicYuvResi->create( iWidth, iHeight, chromaFormat, iMaxCUWidth, iMaxCUHeight, uhTotalDepth );
  }
}

Void TEncSlice::destroy()
{
  // destroy prediction picture
  if ( m_apcPicYuvPred )
  {
    m_apcPicYuvPred->destroy();
    delete m_apcPicYuvPred;
    m_apcPicYuvPred  = NULL;
  }

  // destroy residual picture
  if ( m_apcPicYuvResi )
  {
    m_apcPicYuvResi->destroy();
    delete m_apcPicYuvResi;
    m_apcPicYuvResi  = NULL;
  }

  // free lambda and QP arrays
  if ( m_pdRdPicLambda ) { xFree( m_pdRdPicLambda ); m_pdRdPicLambda = NULL; }
  if ( m_pdRdPicQp     ) { xFree( m_pdRdPicQp     ); m_pdRdPicQp     = NULL; }
  if ( m_piRdPicQp     ) { xFree( m_piRdPicQp     ); m_piRdPicQp     = NULL; }

  if ( m_pcBufferSbacCoders )
  {
    delete[] m_pcBufferSbacCoders;
  }
  if ( m_pcBufferBinCoderCABACs )
  {
    delete[] m_pcBufferBinCoderCABACs;
  }
  if ( m_pcBufferLowLatSbacCoders )
    delete[] m_pcBufferLowLatSbacCoders;
  if ( m_pcBufferLowLatBinCoderCABACs )
    delete[] m_pcBufferLowLatBinCoderCABACs;
}

Void TEncSlice::init( TEncTop* pcEncTop )
{
  m_pcCfg             = pcEncTop;
  m_pcListPic         = pcEncTop->getListPic();

  m_pcGOPEncoder      = pcEncTop->getGOPEncoder();
  m_pcCuEncoder       = pcEncTop->getCuEncoder();
  m_pcPredSearch      = pcEncTop->getPredSearch();

  m_pcEntropyCoder    = pcEncTop->getEntropyCoder();
  m_pcCavlcCoder      = pcEncTop->getCavlcCoder();
  m_pcSbacCoder       = pcEncTop->getSbacCoder();
  m_pcBinCABAC        = pcEncTop->getBinCABAC();
  m_pcTrQuant         = pcEncTop->getTrQuant();

  m_pcBitCounter      = pcEncTop->getBitCounter();
  m_pcRdCost          = pcEncTop->getRdCost();
  m_pppcRDSbacCoder   = pcEncTop->getRDSbacCoder();
  m_pcRDGoOnSbacCoder = pcEncTop->getRDGoOnSbacCoder();

  // create lambda and QP arrays
  m_pdRdPicLambda     = (Double*)xMalloc( Double, m_pcCfg->getDeltaQpRD() * 2 + 1 );
  m_pdRdPicQp         = (Double*)xMalloc( Double, m_pcCfg->getDeltaQpRD() * 2 + 1 );
  m_piRdPicQp         = (Int*   )xMalloc( Int,    m_pcCfg->getDeltaQpRD() * 2 + 1 );
  m_pcRateCtrl        = pcEncTop->getRateCtrl();
}



Void
TEncSlice::setUpLambda(TComSlice* slice, const Double dLambda, Int iQP)
{
  // store lambda
  m_pcRdCost ->setLambda( dLambda );

  // for RDO
  // in RdCost there is only one lambda because the luma and chroma bits are not separated, instead we weight the distortion of chroma.
  Double dLambdas[MAX_NUM_COMPONENT] = { dLambda };
  for(UInt compIdx=1; compIdx<MAX_NUM_COMPONENT; compIdx++)
  {
    const ComponentID compID=ComponentID(compIdx);
    Int chromaQPOffset = slice->getPPS()->getQpOffset(compID) + slice->getSliceChromaQpDelta(compID);
    Int qpc=(iQP + chromaQPOffset < 0) ? iQP : getScaledChromaQP(iQP + chromaQPOffset, m_pcCfg->getChromaFormatIdc());
    Double tmpWeight = pow( 2.0, (iQP-qpc)/3.0 );  // takes into account of the chroma qp mapping and chroma qp Offset
    m_pcRdCost->setDistortionWeight(compID, tmpWeight);
    dLambdas[compIdx]=dLambda/tmpWeight;
  }

#if RDOQ_CHROMA_LAMBDA
// for RDOQ
  m_pcTrQuant->setLambdas( dLambdas );
#else
  m_pcTrQuant->setLambda( dLambda );
#endif

// For SAO
  slice   ->setLambdas( dLambdas );
}



/**
 - non-referenced frame marking
 - QP computation based on temporal structure
 - lambda computation based on QP
 - set temporal layer ID and the parameter sets
 .
 \param pcPic         picture class
 \param pocLast      POC of last picture
 \param pocCurr     current POC
 \param iNumPicRcvd   number of received pictures
 \param iTimeOffset   POC offset for hierarchical structure
 \param iDepth        temporal layer depth
 \param rpcSlice      slice header class
 \param pSPS          SPS associated with the slice
 \param pPPS          PPS associated with the slice
 */

Void TEncSlice::initEncSlice( TComPic* pcPic, Int pocLast, Int pocCurr, Int iNumPicRcvd, Int iGOPid, TComSlice*& rpcSlice, TComSPS* pSPS, TComPPS *pPPS, Bool isField )
{
  Double dQP;
  Double dLambda;

  rpcSlice = pcPic->getSlice(0);
  rpcSlice->setSPS( pSPS );
  rpcSlice->setPPS( pPPS );
  rpcSlice->setSliceBits(0);
  rpcSlice->setPic( pcPic );
  rpcSlice->initSlice();
  rpcSlice->setPicOutputFlag( true );
  rpcSlice->setPOC( pocCurr );

  // depth computation based on GOP size
  Int depth;
  {
#if FIX_FIELD_DEPTH    
    Int poc = rpcSlice->getPOC();
    if(isField)
    {
      poc = (poc/2) % (m_pcCfg->getGOPSize()/2);
    }
    else
    {
      poc = poc % m_pcCfg->getGOPSize();   
    }
#else
    Int poc = rpcSlice->getPOC()%m_pcCfg->getGOPSize();
#endif

    if ( poc == 0 )
    {
      depth = 0;
    }
    else
    {
      Int step = m_pcCfg->getGOPSize();
      depth    = 0;
      for( Int i=step>>1; i>=1; i>>=1 )
      {
        for ( Int j=i; j<m_pcCfg->getGOPSize(); j+=step )
        {
          if ( j == poc )
          {
            i=0;
            break;
          }
        }
        step >>= 1;
        depth++;
      }
    }

#if FIX_FIELD_DEPTH  
#if HARMONIZE_GOP_FIRST_FIELD_COUPLE
    if(poc != 0)
    {
#endif
      if (isField && ((rpcSlice->getPOC() % 2) == 1))
      {
        depth ++;
      }
#if HARMONIZE_GOP_FIRST_FIELD_COUPLE
    }
#endif
#endif
  }

  // slice type
  SliceType eSliceType;

  eSliceType=B_SLICE;
#if EFFICIENT_FIELD_IRAP
  if(!(isField && pocLast == 1))
  {
#endif // EFFICIENT_FIELD_IRAP
#if ALLOW_RECOVERY_POINT_AS_RAP
    if(m_pcCfg->getDecodingRefreshType() == 3)
    {
      eSliceType = (pocLast == 0 || pocCurr % m_pcCfg->getIntraPeriod() == 0             || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
    }
    else
    {
#endif
      eSliceType = (pocLast == 0 || (pocCurr - (isField ? 1 : 0)) % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
#if ALLOW_RECOVERY_POINT_AS_RAP
    }
#endif
#if EFFICIENT_FIELD_IRAP
  }
#endif

  rpcSlice->setSliceType    ( eSliceType );

  // ------------------------------------------------------------------------------------------------------------------
  // Non-referenced frame marking
  // ------------------------------------------------------------------------------------------------------------------

  if(pocLast == 0)
  {
    rpcSlice->setTemporalLayerNonReferenceFlag(false);
  }
  else
  {
    rpcSlice->setTemporalLayerNonReferenceFlag(!m_pcCfg->getGOPEntry(iGOPid).m_refPic);
  }
  rpcSlice->setReferenced(true);

  // ------------------------------------------------------------------------------------------------------------------
  // QP setting
  // ------------------------------------------------------------------------------------------------------------------

  dQP = m_pcCfg->getQP();
  if(eSliceType!=I_SLICE)
  {
    if (!(( m_pcCfg->getMaxDeltaQP() == 0 ) && (dQP == -rpcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA) ) && (rpcSlice->getPPS()->getTransquantBypassEnableFlag())))
    {
      dQP += m_pcCfg->getGOPEntry(iGOPid).m_QPOffset;
    }
  }

  // modify QP
  Int* pdQPs = m_pcCfg->getdQPs();
  if ( pdQPs )
  {
    dQP += pdQPs[ rpcSlice->getPOC() ];
  }

  if (m_pcCfg->getCostMode()==COST_LOSSLESS_CODING)
  {
    dQP=RExt__LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP;
    m_pcCfg->setDeltaQpRD(0);
  }

  // ------------------------------------------------------------------------------------------------------------------
  // Lambda computation
  // ------------------------------------------------------------------------------------------------------------------

  Int iQP;
  Double dOrigQP = dQP;

  // pre-compute lambda and QP values for all possible QP candidates
  for ( Int iDQpIdx = 0; iDQpIdx < 2 * m_pcCfg->getDeltaQpRD() + 1; iDQpIdx++ )
  {
    // compute QP value
    dQP = dOrigQP + ((iDQpIdx+1)>>1)*(iDQpIdx%2 ? -1 : 1);

    // compute lambda value
    Int    NumberBFrames = ( m_pcCfg->getGOPSize() - 1 );
    Int    SHIFT_QP = 12;

    Double dLambda_scale = 1.0 - Clip3( 0.0, 0.5, 0.05*(Double)(isField ? NumberBFrames/2 : NumberBFrames) );

#if FULL_NBIT
    Int    bitdepth_luma_qp_scale = 6 * (g_bitDepth[CHANNEL_TYPE_LUMA] - 8);
#else
    Int    bitdepth_luma_qp_scale = 0;
#endif
    Double qp_temp = (Double) dQP + bitdepth_luma_qp_scale - SHIFT_QP;
#if FULL_NBIT
    Double qp_temp_orig = (Double) dQP - SHIFT_QP;
#endif
    // Case #1: I or P-slices (key-frame)
    Double dQPFactor = m_pcCfg->getGOPEntry(iGOPid).m_QPFactor;
    if ( eSliceType==I_SLICE )
    {
      dQPFactor=0.57*dLambda_scale;
    }
    dLambda = dQPFactor*pow( 2.0, qp_temp/3.0 );

    if ( depth>0 )
    {
#if FULL_NBIT
        dLambda *= Clip3( 2.00, 4.00, (qp_temp_orig / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
#else
        dLambda *= Clip3( 2.00, 4.00, (qp_temp / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
#endif
    }

    // if hadamard is used in ME process
    if ( !m_pcCfg->getUseHADME() && rpcSlice->getSliceType( ) != I_SLICE )
    {
      dLambda *= 0.95;
    }

    iQP = max( -pSPS->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );

    m_pdRdPicLambda[iDQpIdx] = dLambda;
    m_pdRdPicQp    [iDQpIdx] = dQP;
    m_piRdPicQp    [iDQpIdx] = iQP;
  }

  // obtain dQP = 0 case
  dLambda = m_pdRdPicLambda[0];
  dQP     = m_pdRdPicQp    [0];
  iQP     = m_piRdPicQp    [0];

  if( rpcSlice->getSliceType( ) != I_SLICE )
  {
    dLambda *= m_pcCfg->getLambdaModifier( m_pcCfg->getGOPEntry(iGOPid).m_temporalId );
  }

  setUpLambda(rpcSlice, dLambda, iQP);

#if HB_LAMBDA_FOR_LDC
  // restore original slice type

#if EFFICIENT_FIELD_IRAP
  if(!(isField && pocLast == 1))
  {
#endif // EFFICIENT_FIELD_IRAP
#if ALLOW_RECOVERY_POINT_AS_RAP
    if(m_pcCfg->getDecodingRefreshType() == 3)
    {
      eSliceType = (pocLast == 0 || (pocCurr)                     % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
    }
    else
    {
#endif
      eSliceType = (pocLast == 0 || (pocCurr - (isField ? 1 : 0)) % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
#if ALLOW_RECOVERY_POINT_AS_RAP
    }
#endif
#if EFFICIENT_FIELD_IRAP
  }
#endif // EFFICIENT_FIELD_IRAP

  rpcSlice->setSliceType        ( eSliceType );
#endif

  if (m_pcCfg->getUseRecalculateQPAccordingToLambda())
  {
    dQP = xGetQPValueAccordingToLambda( dLambda );
    iQP = max( -pSPS->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );
  }

  rpcSlice->setSliceQp           ( iQP );
#if ADAPTIVE_QP_SELECTION
  rpcSlice->setSliceQpBase       ( iQP );
#endif
  rpcSlice->setSliceQpDelta      ( 0 );
  rpcSlice->setSliceChromaQpDelta( COMPONENT_Cb, 0 );
  rpcSlice->setSliceChromaQpDelta( COMPONENT_Cr, 0 );
  rpcSlice->setUseChromaQpAdj( pPPS->getChromaQpAdjTableSize() > 0 );
  rpcSlice->setNumRefIdx(REF_PIC_LIST_0,m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive);
  rpcSlice->setNumRefIdx(REF_PIC_LIST_1,m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive);

  if ( m_pcCfg->getDeblockingFilterMetric() )
  {
    rpcSlice->setDeblockingFilterOverrideFlag(true);
    rpcSlice->setDeblockingFilterDisable(false);
    rpcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
    rpcSlice->setDeblockingFilterTcOffsetDiv2( 0 );
  }
  else if (rpcSlice->getPPS()->getDeblockingFilterControlPresentFlag())
  {
    rpcSlice->getPPS()->setDeblockingFilterOverrideEnabledFlag( !m_pcCfg->getLoopFilterOffsetInPPS() );
    rpcSlice->setDeblockingFilterOverrideFlag( !m_pcCfg->getLoopFilterOffsetInPPS() );
    rpcSlice->getPPS()->setPicDisableDeblockingFilterFlag( m_pcCfg->getLoopFilterDisable() );
    rpcSlice->setDeblockingFilterDisable( m_pcCfg->getLoopFilterDisable() );
    if ( !rpcSlice->getDeblockingFilterDisable())
    {
      if ( !m_pcCfg->getLoopFilterOffsetInPPS() && eSliceType!=I_SLICE)
      {
        rpcSlice->getPPS()->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_betaOffsetDiv2 + m_pcCfg->getLoopFilterBetaOffset() );
        rpcSlice->getPPS()->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_tcOffsetDiv2 + m_pcCfg->getLoopFilterTcOffset() );
        rpcSlice->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_betaOffsetDiv2 + m_pcCfg->getLoopFilterBetaOffset()  );
        rpcSlice->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_tcOffsetDiv2 + m_pcCfg->getLoopFilterTcOffset() );
      }
      else
      {
        rpcSlice->getPPS()->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getLoopFilterBetaOffset() );
        rpcSlice->getPPS()->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getLoopFilterTcOffset() );
        rpcSlice->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getLoopFilterBetaOffset() );
        rpcSlice->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getLoopFilterTcOffset() );
      }
    }
  }
  else
  {
    rpcSlice->setDeblockingFilterOverrideFlag( false );
    rpcSlice->setDeblockingFilterDisable( false );
    rpcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
    rpcSlice->setDeblockingFilterTcOffsetDiv2( 0 );
  }

  rpcSlice->setDepth            ( depth );

  pcPic->setTLayer( m_pcCfg->getGOPEntry(iGOPid).m_temporalId );
  if(eSliceType==I_SLICE)
  {
    pcPic->setTLayer(0);
  }
  rpcSlice->setTLayer( pcPic->getTLayer() );

  assert( m_apcPicYuvPred );
  assert( m_apcPicYuvResi );

  pcPic->setPicYuvPred( m_apcPicYuvPred );
  pcPic->setPicYuvResi( m_apcPicYuvResi );
  rpcSlice->setSliceMode            ( m_pcCfg->getSliceMode()            );
  rpcSlice->setSliceArgument        ( m_pcCfg->getSliceArgument()        );
  rpcSlice->setSliceSegmentMode     ( m_pcCfg->getSliceSegmentMode()     );
  rpcSlice->setSliceSegmentArgument ( m_pcCfg->getSliceSegmentArgument() );
  rpcSlice->setMaxNumMergeCand        ( m_pcCfg->getMaxNumMergeCand()        );
  xStoreWPparam( pPPS->getUseWP(), pPPS->getWPBiPred() );
}

Void TEncSlice::resetQP( TComPic* pic, Int sliceQP, Double lambda )
{
  TComSlice* slice = pic->getSlice(0);

  // store lambda
  slice->setSliceQp( sliceQP );
#if ADAPTIVE_QP_SELECTION
  slice->setSliceQpBase ( sliceQP );
#endif
  setUpLambda(slice, lambda, sliceQP);
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

Void TEncSlice::setSearchRange( TComSlice* pcSlice )
{
  Int iCurrPOC = pcSlice->getPOC();
  Int iRefPOC;
  Int iGOPSize = m_pcCfg->getGOPSize();
  Int iOffset = (iGOPSize >> 1);
  Int iMaxSR = m_pcCfg->getSearchRange();
  Int iNumPredDir = pcSlice->isInterP() ? 1 : 2;

  for (Int iDir = 0; iDir <= iNumPredDir; iDir++)
  {
    //RefPicList e = (RefPicList)iDir;
    RefPicList  e = ( iDir ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );
    for (Int iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx(e); iRefIdx++)
    {
      iRefPOC = pcSlice->getRefPic(e, iRefIdx)->getPOC();
      Int iNewSR = Clip3(8, iMaxSR, (iMaxSR*ADAPT_SR_SCALE*abs(iCurrPOC - iRefPOC)+iOffset)/iGOPSize);
      m_pcPredSearch->setAdaptiveSearchRange(iDir, iRefIdx, iNewSR);
    }
  }
}

/**
 - multi-loop slice encoding for different slice QP
 .
 \param rpcPic    picture class
 */
Void TEncSlice::precompressSlice( TComPic* pcPic )
{
  // if deltaQP RD is not used, simply return
  if ( m_pcCfg->getDeltaQpRD() == 0 )
  {
    return;
  }

  if ( m_pcCfg->getUseRateCtrl() )
  {
    printf( "\nMultiple QP optimization is not allowed when rate control is enabled." );
    assert(0);
  }

  TComSlice* pcSlice        = pcPic->getSlice(getSliceIdx());
  Double     dPicRdCostBest = MAX_DOUBLE;
  UInt       uiQpIdxBest = 0;

  Double dFrameLambda;
#if FULL_NBIT
  Int    SHIFT_QP = 12 + 6 * (g_bitDepth[CHANNEL_TYPE_LUMA] - 8);
#else
  Int    SHIFT_QP = 12;
#endif

  // set frame lambda
  if (m_pcCfg->getGOPSize() > 1)
  {
    dFrameLambda = 0.68 * pow (2, (m_piRdPicQp[0]  - SHIFT_QP) / 3.0) * (pcSlice->isInterB()? 2 : 1);
  }
  else
  {
    dFrameLambda = 0.68 * pow (2, (m_piRdPicQp[0] - SHIFT_QP) / 3.0);
  }
  m_pcRdCost      ->setFrameLambda(dFrameLambda);

  // for each QP candidate
  for ( UInt uiQpIdx = 0; uiQpIdx < 2 * m_pcCfg->getDeltaQpRD() + 1; uiQpIdx++ )
  {
    pcSlice       ->setSliceQp             ( m_piRdPicQp    [uiQpIdx] );
#if ADAPTIVE_QP_SELECTION
    pcSlice       ->setSliceQpBase         ( m_piRdPicQp    [uiQpIdx] );
#endif
    setUpLambda(pcSlice, m_pdRdPicLambda[uiQpIdx], m_piRdPicQp    [uiQpIdx]);

    // try compress
    compressSlice   ( pcPic );

    Double dPicRdCost;
    UInt64 uiPicDist        = m_uiPicDist;
    UInt64 uiALFBits        = 0;

    m_pcGOPEncoder->preLoopFilterPicAll( pcPic, uiPicDist, uiALFBits );

    // compute RD cost and choose the best
    dPicRdCost = m_pcRdCost->calcRdCost64( m_uiPicTotalBits + uiALFBits, uiPicDist, true, DF_SSE_FRAME);

    if ( dPicRdCost < dPicRdCostBest )
    {
      uiQpIdxBest    = uiQpIdx;
      dPicRdCostBest = dPicRdCost;
    }
  }

  // set best values
  pcSlice       ->setSliceQp             ( m_piRdPicQp    [uiQpIdxBest] );
#if ADAPTIVE_QP_SELECTION
  pcSlice       ->setSliceQpBase         ( m_piRdPicQp    [uiQpIdxBest] );
#endif
  setUpLambda(pcSlice, m_pdRdPicLambda[uiQpIdxBest], m_piRdPicQp    [uiQpIdxBest]);
}

Void TEncSlice::calCostSliceI(TComPic* pcPic)
{
  UInt    ctuRsAddr;
  UInt    startCtuTsAddr;
  UInt    boundingCtuTsAddr;
  Int     iSumHad, shift = g_bitDepth[CHANNEL_TYPE_LUMA]-8, offset = (shift>0)?(1<<(shift-1)):0;;
  Double  iSumHadSlice = 0;

  pcPic->getSlice(getSliceIdx())->setSliceSegmentBits(0);
  TComSlice* pcSlice            = pcPic->getSlice(getSliceIdx());
  xDetermineStartAndBoundingCtuTsAddr ( startCtuTsAddr, boundingCtuTsAddr, pcPic, false );

  UInt ctuTsAddr;
  ctuRsAddr = pcPic->getPicSym()->getCtuTsToRsAddrMap( startCtuTsAddr);
  for( ctuTsAddr = startCtuTsAddr; ctuTsAddr < boundingCtuTsAddr; ctuRsAddr = pcPic->getPicSym()->getCtuTsToRsAddrMap(++ctuTsAddr) )
  {
    // initialize CU encoder
    TComDataCU* pCtu = pcPic->getCtu( ctuRsAddr );
    pCtu->initCtu( pcPic, ctuRsAddr );

    Int height  = min( pcSlice->getSPS()->getMaxCUHeight(),pcSlice->getSPS()->getPicHeightInLumaSamples() - ctuRsAddr / pcPic->getFrameWidthInCtus() * pcSlice->getSPS()->getMaxCUHeight() );
    Int width   = min( pcSlice->getSPS()->getMaxCUWidth(),pcSlice->getSPS()->getPicWidthInLumaSamples() - ctuRsAddr % pcPic->getFrameWidthInCtus() * pcSlice->getSPS()->getMaxCUWidth() );

    iSumHad = m_pcCuEncoder->updateCtuDataISlice(pCtu, width, height);

    (m_pcRateCtrl->getRCPic()->getLCU(ctuRsAddr)).m_costIntra=(iSumHad+offset)>>shift;
    iSumHadSlice += (m_pcRateCtrl->getRCPic()->getLCU(ctuRsAddr)).m_costIntra;

  }
  m_pcRateCtrl->getRCPic()->setTotalIntraCost(iSumHadSlice);
}

/** \param rpcPic   picture class
 */
Void TEncSlice::compressSlice( TComPic* pcPic )
{
  UInt   startCtuTsAddr;
  UInt   boundingCtuTsAddr;
  pcPic->getSlice(getSliceIdx())->setSliceSegmentBits(0);
  TEncBinCABAC* pppcRDSbacCoder = NULL;
  TComSlice* pcSlice            = pcPic->getSlice(getSliceIdx());
  xDetermineStartAndBoundingCtuTsAddr ( startCtuTsAddr, boundingCtuTsAddr, pcPic, false );

  // initialize cost values
  m_uiPicTotalBits  = 0;
  m_dPicRdCost      = 0;
  m_uiPicDist       = 0;

  // set entropy coder
  m_pcSbacCoder->init( m_pcBinCABAC );
  m_pcEntropyCoder->setEntropyCoder   ( m_pcSbacCoder, pcSlice );
  m_pcEntropyCoder->resetEntropy      ();
  m_pppcRDSbacCoder[0][CI_CURR_BEST]->load(m_pcSbacCoder);
  pppcRDSbacCoder = (TEncBinCABAC *) m_pppcRDSbacCoder[0][CI_CURR_BEST]->getEncBinIf();
  pppcRDSbacCoder->setBinCountingEnableFlag( false );
  pppcRDSbacCoder->setBinsCoded( 0 );

  //------------------------------------------------------------------------------
  //  Weighted Prediction parameters estimation.
  //------------------------------------------------------------------------------
  // calculate AC/DC values for current picture
  if( pcSlice->getPPS()->getUseWP() || pcSlice->getPPS()->getWPBiPred() )
  {
    xCalcACDCParamSlice(pcSlice);
  }

  Bool bWp_explicit = (pcSlice->getSliceType()==P_SLICE && pcSlice->getPPS()->getUseWP()) || (pcSlice->getSliceType()==B_SLICE && pcSlice->getPPS()->getWPBiPred());

  if ( bWp_explicit )
  {
    //------------------------------------------------------------------------------
    //  Weighted Prediction implemented at Slice level. SliceMode=2 is not supported yet.
    //------------------------------------------------------------------------------
    if ( pcSlice->getSliceMode()==FIXED_NUMBER_OF_BYTES || pcSlice->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES )
    {
      printf("Weighted Prediction is not supported with slice mode determined by max number of bins.\n"); exit(0);
    }

    xEstimateWPParamSlice( pcSlice );
    pcSlice->initWpScaling();

    // check WP on/off
    xCheckWPEnable( pcSlice );
  }

#if ADAPTIVE_QP_SELECTION
  if( m_pcCfg->getUseAdaptQpSelect() )
  {
    m_pcTrQuant->clearSliceARLCnt();
    if(pcSlice->getSliceType()!=I_SLICE)
    {
      Int qpBase = pcSlice->getSliceQpBase();
      pcSlice->setSliceQp(qpBase + m_pcTrQuant->getQpDelta(qpBase));
    }
  }
#endif
  TEncTop* pcEncTop = (TEncTop*) m_pcCfg;
  TEncSbac**** ppppcRDSbacCoders    = pcEncTop->getRDSbacCoders();
  TComBitCounter* pcBitCounters     = pcEncTop->getBitCounters();
  const Int  iNumSubstreams = pcSlice->getPPS()->getNumSubstreams();
  const UInt uiTilesAcross  = pcPic->getPicSym()->getNumTileColumnsMinus1()+1;
  delete[] m_pcBufferSbacCoders;
  delete[] m_pcBufferBinCoderCABACs;
  m_pcBufferSbacCoders     = new TEncSbac    [uiTilesAcross];
  m_pcBufferBinCoderCABACs = new TEncBinCABAC[uiTilesAcross];
  for (Int ui = 0; ui < uiTilesAcross; ui++)
  {
    m_pcBufferSbacCoders[ui].init( &m_pcBufferBinCoderCABACs[ui] );
  }
  for (UInt ui = 0; ui < uiTilesAcross; ui++)
  {
    m_pcBufferSbacCoders[ui].load(m_pppcRDSbacCoder[0][CI_CURR_BEST]);  //init. state
  }

  for ( UInt ui = 0 ; ui < iNumSubstreams ; ui++ ) //init all sbac coders for RD optimization
  {
    ppppcRDSbacCoders[ui][0][CI_CURR_BEST]->load(m_pppcRDSbacCoder[0][CI_CURR_BEST]);
  }
  delete[] m_pcBufferLowLatSbacCoders;
  delete[] m_pcBufferLowLatBinCoderCABACs;
  m_pcBufferLowLatSbacCoders     = new TEncSbac    [uiTilesAcross];
  m_pcBufferLowLatBinCoderCABACs = new TEncBinCABAC[uiTilesAcross];
  for (Int ui = 0; ui < uiTilesAcross; ui++)
  {
    m_pcBufferLowLatSbacCoders[ui].init( &m_pcBufferLowLatBinCoderCABACs[ui] );
  }
  for (UInt ui = 0; ui < uiTilesAcross; ui++)
    m_pcBufferLowLatSbacCoders[ui].load(m_pppcRDSbacCoder[0][CI_CURR_BEST]);  //init. state

  const UInt      frameWidthInCtus        = pcPic->getPicSym()->getFrameWidthInCtus();
  UInt      tileColumnNumber        = 0;
  Bool      depSliceSegmentsEnabled = pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag();
  UInt      ctuRsAddr               = pcPic->getPicSym()->getCtuTsToRsAddrMap( startCtuTsAddr);
  UInt      currentTileIdx          = pcPic->getPicSym()->getTileIdxMap(ctuRsAddr);
  TComTile *pCurrentTile            = pcPic->getPicSym()->getTComTile(currentTileIdx);
  if( depSliceSegmentsEnabled )
  {
    const UInt      firstCtuRsAddrOfTile    = pCurrentTile->getFirstCtuRsAddr();
    if((pcSlice->getSliceSegmentCurStartCtuTsAddr()!= pcSlice->getSliceCurStartCtuTsAddr())&&(ctuRsAddr != firstCtuRsAddrOfTile))
    {
      UInt uiSubStrm=0;
      if( m_pcCfg->getWaveFrontsynchro() )
      {
        tileColumnNumber = currentTileIdx % (pcPic->getPicSym()->getNumTileColumnsMinus1()+1);
        m_pcBufferSbacCoders[tileColumnNumber].loadContexts( CTXMem[1] );
        //uiCUAddr = rpcPic->getPicSym()->getCUOrderMap( uiStartCUAddr /rpcPic->getNumPartInCU());
        uiSubStrm=pcPic->getSubstreamForCtuAddr(ctuRsAddr, true, pcSlice);
        if ( pCurrentTile->getTileWidthInCtus() < 2)
        {
          CTXMem[0]->loadContexts(m_pcSbacCoder);
        }
      }
      m_pppcRDSbacCoder[0][CI_CURR_BEST]->loadContexts( CTXMem[0] );
      ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST]->loadContexts( CTXMem[0] );
    }
    else
    {
      if(m_pcCfg->getWaveFrontsynchro())
      {
        CTXMem[1]->loadContexts(m_pcSbacCoder);
      }
      CTXMem[0]->loadContexts(m_pcSbacCoder);
    }
  }
  // for every CU in slice
  for( UInt ctuTsAddr = startCtuTsAddr; ctuTsAddr < boundingCtuTsAddr; ctuRsAddr = pcPic->getPicSym()->getCtuTsToRsAddrMap(++ctuTsAddr) )
  {
    // initialize CU encoder
    TComDataCU* pCtu = pcPic->getCtu( ctuRsAddr );
    pCtu->initCtu( pcPic, ctuRsAddr );

    // inherit from TR if necessary, select substream to use.
    tileColumnNumber = pcPic->getPicSym()->getTileIdxMap(ctuRsAddr) % (pcPic->getPicSym()->getNumTileColumnsMinus1()+1); // what column of tiles are we in?
    const UInt firstCtuRsAddrOfTile = pcPic->getPicSym()->getTComTile(pcPic->getPicSym()->getTileIdxMap(ctuRsAddr))->getFirstCtuRsAddr();
    const UInt tileXPosInCtus = firstCtuRsAddrOfTile % frameWidthInCtus;
    const UInt ctuXPosInCtus  = ctuRsAddr % frameWidthInCtus;
    const UInt uiSubStrm=pcPic->getSubstreamForCtuAddr(ctuRsAddr, true, pcSlice);
    
    if ( ((iNumSubstreams > 1) || depSliceSegmentsEnabled ) && (ctuXPosInCtus == tileXPosInCtus) && m_pcCfg->getWaveFrontsynchro())
    {
      // We'll sync if the TR is available.
      TComDataCU *pCtuUp = pCtu->getCtuAbove();
      TComDataCU *pCtuTR = NULL;
      if ( pCtuUp && ((ctuRsAddr%frameWidthInCtus+1) < frameWidthInCtus)  )
      {
        pCtuTR = pcPic->getCtu( ctuRsAddr - frameWidthInCtus + 1 );
      }
      if ( !pCtu->CUIsFromSameSliceAndTile(pCtuTR) )
      {
        // TR not available.
      }
      else
      {
        // TR is available, we use it.
        ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST]->loadContexts( &m_pcBufferSbacCoders[tileColumnNumber] );
      }
    }
    m_pppcRDSbacCoder[0][CI_CURR_BEST]->load( ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST] ); //this load is used to simplify the code

    // reset the entropy coder
    if( ctuRsAddr == pcPic->getPicSym()->getTComTile(pcPic->getPicSym()->getTileIdxMap(ctuRsAddr))->getFirstCtuRsAddr() &&                    // must be first CU of tile
        ctuRsAddr!=0 &&                                                                                                                       // cannot be first CU of picture
        ctuRsAddr!=pcPic->getPicSym()->getCtuTsToRsAddrMap(pcPic->getSlice(pcPic->getCurrSliceIdx())->getSliceSegmentCurStartCtuTsAddr()) &&
        ctuRsAddr!=pcPic->getPicSym()->getCtuTsToRsAddrMap(pcPic->getSlice(pcPic->getCurrSliceIdx())->getSliceCurStartCtuTsAddr()))           // cannot be first CU of slice
    {
      SliceType sliceType = pcSlice->getSliceType();
      if (!pcSlice->isIntra() && pcSlice->getPPS()->getCabacInitPresentFlag() && pcSlice->getPPS()->getEncCABACTableIdx()!=I_SLICE)
      {
        sliceType = (SliceType) pcSlice->getPPS()->getEncCABACTableIdx();
      }
      m_pcEntropyCoder->updateContextTables ( sliceType, pcSlice->getSliceQp(), false );
      m_pcEntropyCoder->setEntropyCoder     ( m_pppcRDSbacCoder[0][CI_CURR_BEST], pcSlice );
      m_pcEntropyCoder->updateContextTables ( sliceType, pcSlice->getSliceQp() );
      m_pcEntropyCoder->setEntropyCoder     ( m_pcSbacCoder, pcSlice );
    }

    // set go-on entropy coder
    m_pcEntropyCoder->setEntropyCoder ( m_pcRDGoOnSbacCoder, pcSlice );
    m_pcEntropyCoder->setBitstream( &pcBitCounters[uiSubStrm] );

    ((TEncBinCABAC*)m_pcRDGoOnSbacCoder->getEncBinIf())->setBinCountingEnableFlag(true);

    Double oldLambda = m_pcRdCost->getLambda();
    if ( m_pcCfg->getUseRateCtrl() )
    {
      Int estQP        = pcSlice->getSliceQp();
      Double estLambda = -1.0;
      Double bpp       = -1.0;

      if ( ( pcPic->getSlice( 0 )->getSliceType() == I_SLICE && m_pcCfg->getForceIntraQP() ) || !m_pcCfg->getLCULevelRC() )
      {
        estQP = pcSlice->getSliceQp();
      }
      else
      {
        bpp = m_pcRateCtrl->getRCPic()->getLCUTargetBpp(pcSlice->getSliceType());
        if ( pcPic->getSlice( 0 )->getSliceType() == I_SLICE)
        {
          estLambda = m_pcRateCtrl->getRCPic()->getLCUEstLambdaAndQP(bpp, pcSlice->getSliceQp(), &estQP);
        }
        else
        {
          estLambda = m_pcRateCtrl->getRCPic()->getLCUEstLambda( bpp );
          estQP     = m_pcRateCtrl->getRCPic()->getLCUEstQP    ( estLambda, pcSlice->getSliceQp() );
        }

        estQP     = Clip3( -pcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, estQP );

        m_pcRdCost->setLambda(estLambda);

#if RDOQ_CHROMA_LAMBDA
        // set lambda for RDOQ
        const Double chromaLambda = estLambda / m_pcRdCost->getChromaWeight();
        const Double lambdaArray[MAX_NUM_COMPONENT] = { estLambda, chromaLambda, chromaLambda };
        m_pcTrQuant->setLambdas( lambdaArray );
#else
        m_pcTrQuant->setLambda( estLambda );
#endif
      }

      m_pcRateCtrl->setRCQP( estQP );
#if ADAPTIVE_QP_SELECTION
      pCtu->getSlice()->setSliceQpBase( estQP );
#endif
    }

    // run CU encoder
    m_pcCuEncoder->compressCtu( pCtu );

    // restore entropy coder to an initial stage
    m_pcEntropyCoder->setEntropyCoder ( m_pppcRDSbacCoder[0][CI_CURR_BEST], pcSlice );
    m_pcEntropyCoder->setBitstream( &pcBitCounters[uiSubStrm] );
    m_pcCuEncoder->setBitCounter( &pcBitCounters[uiSubStrm] );
    m_pcBitCounter = &pcBitCounters[uiSubStrm];
    pppcRDSbacCoder->setBinCountingEnableFlag( true );
    m_pcBitCounter->resetBits();
    pppcRDSbacCoder->setBinsCoded( 0 );
    m_pcCuEncoder->encodeCtu( pCtu );

    pppcRDSbacCoder->setBinCountingEnableFlag( false );
    if (m_pcCfg->getSliceMode()==FIXED_NUMBER_OF_BYTES && ( ( pcSlice->getSliceBits() + m_pcEntropyCoder->getNumberOfWrittenBits() ) ) > m_pcCfg->getSliceArgument()<<3)
    {
      pcSlice->setNextSlice( true ); // Value of slice-segment has lower priority. TODO: replace NextSlice and NextSliceSegment with an enumeration.
      break;
    }
    if (m_pcCfg->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES &&
        pcSlice->getSliceSegmentBits()+m_pcEntropyCoder->getNumberOfWrittenBits() > (m_pcCfg->getSliceSegmentArgument() << 3) &&
        pcSlice->getSliceCurEndCtuTsAddr()!=pcSlice->getSliceSegmentCurEndCtuTsAddr())
    {
      pcSlice->setNextSliceSegment( true );
      pcSlice->setNextSlice( false ); // NextSlice may have been already set true (due to slice-mode settings), but at this point, the next NAL unit will be a slice-segment, not a slice.
      break;
    }

    ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST]->load( m_pppcRDSbacCoder[0][CI_CURR_BEST] );

    //Store probabilties of second CTU in line into buffer
    if ( ( ctuXPosInCtus == tileXPosInCtus+1) && (depSliceSegmentsEnabled || (iNumSubstreams > 1)) && m_pcCfg->getWaveFrontsynchro())
    {
      m_pcBufferSbacCoders[tileColumnNumber].loadContexts(ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST]);
    }

    if ( m_pcCfg->getUseRateCtrl() )
    {
      Int actualQP        = g_RCInvalidQPValue;
      Double actualLambda = m_pcRdCost->getLambda();
      Int actualBits      = pCtu->getTotalBits();
      Int numberOfEffectivePixels    = 0;
      for ( Int idx = 0; idx < pcPic->getNumPartitionsInCtu(); idx++ )
      {
        if ( pCtu->getPredictionMode( idx ) != NUMBER_OF_PREDICTION_MODES && ( !pCtu->isSkipped( idx ) ) )
        {
          numberOfEffectivePixels = numberOfEffectivePixels + 16;
          break;
        }
      }

      if ( numberOfEffectivePixels == 0 )
      {
        actualQP = g_RCInvalidQPValue;
      }
      else
      {
        actualQP = pCtu->getQP( 0 );
      }
      m_pcRdCost->setLambda(oldLambda);
      m_pcRateCtrl->getRCPic()->updateAfterCTU( m_pcRateCtrl->getRCPic()->getLCUCoded(), actualBits, actualQP, actualLambda,
          pCtu->getSlice()->getSliceType() == I_SLICE ? 0 : m_pcCfg->getLCULevelRC() );
    }

    m_uiPicTotalBits += pCtu->getTotalBits();
    m_dPicRdCost     += pCtu->getTotalCost();
    m_uiPicDist      += pCtu->getTotalDistortion();
  }
  if ((iNumSubstreams > 1) && !depSliceSegmentsEnabled)
  {
    pcSlice->setNextSlice( true );
  }
  if(m_pcCfg->getSliceMode()==FIXED_NUMBER_OF_BYTES || m_pcCfg->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES)
  {
    if(pcSlice->getSliceCurEndCtuTsAddr()<=pcSlice->getSliceSegmentCurEndCtuTsAddr())
    {
       pcSlice->setNextSlice( true );
    }
    else
    {
       pcSlice->setNextSliceSegment( true );
    }
  }
  if( depSliceSegmentsEnabled )
  {
    if (m_pcCfg->getWaveFrontsynchro())
    {
      CTXMem[1]->loadContexts( &m_pcBufferSbacCoders[tileColumnNumber] );//ctx 2.CTU
    }
    CTXMem[0]->loadContexts( m_pppcRDSbacCoder[0][CI_CURR_BEST] );//ctx end of dep.slice
  }
  xRestoreWPparam( pcSlice );
}

/**
 \param  rpcPic        picture class
 \retval rpcBitstream  bitstream class
 */
Void TEncSlice::encodeSlice   ( TComPic* pcPic, TComOutputBitstream* pcSubstreams )
{
  UInt       ctuRsAddr;
  UInt       startCtuTsAddr;
  UInt       boundingCtuTsAddr;
  TComSlice* pcSlice = pcPic->getSlice(getSliceIdx());

  startCtuTsAddr    = pcSlice->getSliceSegmentCurStartCtuTsAddr();
  boundingCtuTsAddr = pcSlice->getSliceSegmentCurEndCtuTsAddr();
  // choose entropy coder
  {
    m_pcSbacCoder->init( (TEncBinIf*)m_pcBinCABAC );
    m_pcEntropyCoder->setEntropyCoder ( m_pcSbacCoder, pcSlice );
  }

  m_pcCuEncoder->setBitCounter( NULL );
  m_pcBitCounter = NULL;
  // Appropriate substream bitstream is switched later.
  // for every CU
#if ENC_DEC_TRACE
  g_bJustDoIt = g_bEncDecTraceEnable;
#endif
  DTRACE_CABAC_VL( g_nSymbolCounter++ );
  DTRACE_CABAC_T( "\tPOC: " );
  DTRACE_CABAC_V( rpcPic->getPOC() );
  DTRACE_CABAC_T( "\n" );
#if ENC_DEC_TRACE
  g_bJustDoIt = g_bEncDecTraceDisable;
#endif

  TEncTop* pcEncTop = (TEncTop*) m_pcCfg;
  TEncSbac* pcSbacCoders = pcEncTop->getSbacCoders(); //coder for each substream
  const Int iNumSubstreams = pcSlice->getPPS()->getNumSubstreams();
  UInt uiBitsOriginallyInSubstreams = 0;
  {
    UInt uiTilesAcross = pcPic->getPicSym()->getNumTileColumnsMinus1()+1;
    for (UInt ui = 0; ui < uiTilesAcross; ui++)
    {
      m_pcBufferSbacCoders[ui].load(m_pcSbacCoder); //init. state
    }

    for (Int iSubstrmIdx=0; iSubstrmIdx < iNumSubstreams; iSubstrmIdx++)
    {
      uiBitsOriginallyInSubstreams += pcSubstreams[iSubstrmIdx].getNumberOfWrittenBits();
    }

    for (UInt ui = 0; ui < uiTilesAcross; ui++)
    {
      m_pcBufferLowLatSbacCoders[ui].load(m_pcSbacCoder);  //init. state
    }
  }

  const UInt frameWidthInCtus  = pcPic->getPicSym()->getFrameWidthInCtus();
  UInt uiSubStrm=0;
  UInt tileColumnNumber      = 0;
  const Bool depSliceSegmentsEnabled = pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag();
  ctuRsAddr        = pcPic->getPicSym()->getCtuTsToRsAddrMap( startCtuTsAddr );
  UInt currentTileIdx=pcPic->getPicSym()->getTileIdxMap(ctuRsAddr);
  if( depSliceSegmentsEnabled )
  {
    const TComTile *pCurrentTile=pcPic->getPicSym()->getTComTile(currentTileIdx);
    const UInt firstCtuRsAddrOfTile = pCurrentTile->getFirstCtuRsAddr();
    if( pcSlice->isNextSlice()|| ctuRsAddr == firstCtuRsAddrOfTile)
    {
      if(m_pcCfg->getWaveFrontsynchro())
      {
        CTXMem[1]->loadContexts(m_pcSbacCoder);
      }
      CTXMem[0]->loadContexts(m_pcSbacCoder);
    }
    else
    {
      if( m_pcCfg->getWaveFrontsynchro() )
      {
        tileColumnNumber = currentTileIdx % (pcPic->getPicSym()->getNumTileColumnsMinus1()+1);
        m_pcBufferSbacCoders[tileColumnNumber].loadContexts( CTXMem[1] );
        uiSubStrm=pcPic->getSubstreamForCtuAddr(ctuRsAddr, true, pcSlice);
        if ( pCurrentTile->getTileWidthInCtus() < 2)
        {
          CTXMem[0]->loadContexts(m_pcSbacCoder);
        }
      }
      pcSbacCoders[uiSubStrm].loadContexts( CTXMem[0] );
    }
  }

  for( UInt ctuTsAddr = startCtuTsAddr; ctuTsAddr < boundingCtuTsAddr; ctuRsAddr = pcPic->getPicSym()->getCtuTsToRsAddrMap(++ctuTsAddr) )
  {
    tileColumnNumber     = pcPic->getPicSym()->getTileIdxMap(ctuRsAddr) % (pcPic->getPicSym()->getNumTileColumnsMinus1()+1); // what column of tiles are we in?
    const UInt firstCtuRsAddrOfTile = pcPic->getPicSym()->getTComTile(pcPic->getPicSym()->getTileIdxMap(ctuRsAddr))->getFirstCtuRsAddr();
    const UInt tileXPosInCtus       = firstCtuRsAddrOfTile % frameWidthInCtus;
    const UInt ctuXPosInCtus        = ctuRsAddr % frameWidthInCtus;
    uiSubStrm=pcPic->getSubstreamForCtuAddr(ctuRsAddr, true, pcSlice);

    m_pcEntropyCoder->setBitstream( &pcSubstreams[uiSubStrm] );
    // Synchronize cabac probabilities with upper-right CTU if it's available and we're at the start of a line.
    if (((iNumSubstreams > 1) || depSliceSegmentsEnabled) && (ctuXPosInCtus == tileXPosInCtus) && m_pcCfg->getWaveFrontsynchro())
    {
      // We'll sync if the TR is available.
      TComDataCU *pCtuUp = pcPic->getCtu( ctuRsAddr )->getCtuAbove();
      TComDataCU *pCtuTR = NULL;
      if ( pCtuUp && ((ctuRsAddr%frameWidthInCtus+1) < frameWidthInCtus)  )
      {
        pCtuTR = pcPic->getCtu( ctuRsAddr - frameWidthInCtus + 1 );
      }
      if ( true/*bEnforceSliceRestriction*/ && !pcPic->getCtu( ctuRsAddr )->CUIsFromSameSliceAndTile(pCtuTR) )
      {
        // TR not available.
      }
      else
      {
        // TR is available, we use it.
        pcSbacCoders[uiSubStrm].loadContexts( &m_pcBufferSbacCoders[tileColumnNumber] );
      }
    }
    m_pcSbacCoder->load(&pcSbacCoders[uiSubStrm]);  //this load is used to simplify the code (avoid to change all the call to m_pcSbacCoder)

    // reset the entropy coder
    if( ctuRsAddr == pcPic->getPicSym()->getTComTile(pcPic->getPicSym()->getTileIdxMap(ctuRsAddr))->getFirstCtuRsAddr() && // must be first CU of tile
        ctuRsAddr!=0 &&                                                                                                    // cannot be first CU of picture
        ctuTsAddr!=pcPic->getSlice(pcPic->getCurrSliceIdx())->getSliceSegmentCurStartCtuTsAddr() &&
        ctuTsAddr!=pcPic->getSlice(pcPic->getCurrSliceIdx())->getSliceCurStartCtuTsAddr())                                 // cannot be first CU of slice
    {
      // We're crossing into another tile, tiles are independent.
      // When tiles are independent, we have "substreams per tile".  Each substream has already been terminated, and we no longer
      // have to perform it here.
      if (iNumSubstreams <= 1)
      {
        SliceType sliceType  = pcSlice->getSliceType();
        if (!pcSlice->isIntra() && pcSlice->getPPS()->getCabacInitPresentFlag() && pcSlice->getPPS()->getEncCABACTableIdx()!=I_SLICE)
        {
          sliceType = (SliceType) pcSlice->getPPS()->getEncCABACTableIdx();
        }
        m_pcEntropyCoder->updateContextTables( sliceType, pcSlice->getSliceQp() );

        // Byte-alignment in slice_data() when new tile
        pcSubstreams[uiSubStrm].writeByteAlignment();
      }

      {
        UInt numStartCodeEmulations = pcSubstreams[uiSubStrm].countStartCodeEmulations();
        UInt uiAccumulatedSubstreamLength = 0;
        for (Int iSubstrmIdx=0; iSubstrmIdx < iNumSubstreams; iSubstrmIdx++)
        {
          uiAccumulatedSubstreamLength += pcSubstreams[iSubstrmIdx].getNumberOfWrittenBits();
        }
        // add bits coded in previous dependent slices + bits coded so far
        // add number of emulation prevention byte count in the tile
        pcSlice->addTileLocation( ((pcSlice->getTileOffstForMultES() + uiAccumulatedSubstreamLength - uiBitsOriginallyInSubstreams) >> 3) + numStartCodeEmulations );
      }
    }

    TComDataCU* pCtu = pcPic->getCtu( ctuRsAddr );

    if ( pcSlice->getSPS()->getUseSAO() )
    {
      Bool bIsSAOSliceEnabled = false;
      Bool sliceEnabled[MAX_NUM_COMPONENT];
      for(Int comp=0; comp < MAX_NUM_COMPONENT; comp++)
      {
        ComponentID compId=ComponentID(comp);
        sliceEnabled[compId] = pcSlice->getSaoEnabledFlag(toChannelType(compId)) && (comp < pcPic->getNumberValidComponents());
        if (sliceEnabled[compId]) bIsSAOSliceEnabled=true;
      }
      if (bIsSAOSliceEnabled)
      {
        SAOBlkParam& saoblkParam = (pcPic->getPicSym()->getSAOBlkParam())[ctuRsAddr];

        Bool leftMergeAvail = false;
        Bool aboveMergeAvail= false;
        //merge left condition
        Int rx = (ctuRsAddr % frameWidthInCtus);
        if(rx > 0)
        {
          leftMergeAvail = pcPic->getSAOMergeAvailability(ctuRsAddr, ctuRsAddr-1);
        }

        //merge up condition
        Int ry = (ctuRsAddr / frameWidthInCtus);
        if(ry > 0)
        {
          aboveMergeAvail = pcPic->getSAOMergeAvailability(ctuRsAddr, ctuRsAddr-frameWidthInCtus);
        }

        m_pcEntropyCoder->encodeSAOBlkParam(saoblkParam, sliceEnabled, leftMergeAvail, aboveMergeAvail);
      }
    }

#if ENC_DEC_TRACE
    g_bJustDoIt = g_bEncDecTraceEnable;
#endif
      m_pcCuEncoder->encodeCtu( pCtu );
#if ENC_DEC_TRACE
    g_bJustDoIt = g_bEncDecTraceDisable;
#endif
    pcSbacCoders[uiSubStrm].load(m_pcSbacCoder);   //load back status of the entropy coder after encoding the CTU into relevant bitstream entropy coder

    //Store probabilities of second CTU in line into buffer
    if ( (depSliceSegmentsEnabled || (iNumSubstreams > 1)) && (ctuXPosInCtus == tileXPosInCtus+1) && m_pcCfg->getWaveFrontsynchro())
    {
      m_pcBufferSbacCoders[tileColumnNumber].loadContexts( &pcSbacCoders[uiSubStrm] );
    }
  }
  if( depSliceSegmentsEnabled )
  {
    if (m_pcCfg->getWaveFrontsynchro())
    {
      CTXMem[1]->loadContexts( &m_pcBufferSbacCoders[tileColumnNumber] );//ctx 2.CTU
    }
    CTXMem[0]->loadContexts( m_pcSbacCoder );//ctx end of dep.slice
  }
#if ADAPTIVE_QP_SELECTION
  if( m_pcCfg->getUseAdaptQpSelect() )
  {
    m_pcTrQuant->storeSliceQpNext(pcSlice);
  }
#endif
  if (pcSlice->getPPS()->getCabacInitPresentFlag())
  {
    if (pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag())
    {
      pcSlice->getPPS()->setEncCABACTableIdx( pcSlice->getSliceType() );
    }
    else
    {
      m_pcEntropyCoder->determineCabacInitIdx();
    }
  }
}

Void TEncSlice::calculateBoundingCtuTsAddrForSlice(UInt &startCtuTSAddrSlice, UInt &boundingCtuTSAddrSlice, Bool &haveReachedTileBoundary,
                                                   TComPic* pcPic, const Bool encodingSlice, const Int sliceMode, const Int sliceArgument, const UInt sliceCurEndCtuTSAddr)
{
  TComSlice* pcSlice = pcPic->getSlice(getSliceIdx());
  const UInt numberOfCtusInFrame = pcPic->getNumberOfCtusInFrame();
  boundingCtuTSAddrSlice=0;
  haveReachedTileBoundary=false;

  switch (sliceMode)
  {
    case FIXED_NUMBER_OF_CTU:
      {
        UInt ctuAddrIncrement    = sliceArgument;
        boundingCtuTSAddrSlice  = ((startCtuTSAddrSlice + ctuAddrIncrement) < numberOfCtusInFrame) ? (startCtuTSAddrSlice + ctuAddrIncrement) : numberOfCtusInFrame;
      }
      break;
    case FIXED_NUMBER_OF_BYTES:
      if (encodingSlice)
        boundingCtuTSAddrSlice  = sliceCurEndCtuTSAddr;
      else
        boundingCtuTSAddrSlice  = numberOfCtusInFrame;
      break;
    case FIXED_NUMBER_OF_TILES:
      {
        const UInt tileIdx        = pcPic->getPicSym()->getTileIdxMap( pcPic->getPicSym()->getCtuTsToRsAddrMap(startCtuTSAddrSlice) );
        const UInt tileTotalCount = (pcPic->getPicSym()->getNumTileColumnsMinus1()+1) * (pcPic->getPicSym()->getNumTileRowsMinus1()+1);
        UInt ctuAddrIncrement   = 0;

        for(UInt tileIdxIncrement = 0; tileIdxIncrement < sliceArgument; tileIdxIncrement++)
        {
          if((tileIdx + tileIdxIncrement) < tileTotalCount)
          {
            UInt tileWidthInCtus   = pcPic->getPicSym()->getTComTile(tileIdx + tileIdxIncrement)->getTileWidthInCtus();
            UInt tileHeightInCtus  = pcPic->getPicSym()->getTComTile(tileIdx + tileIdxIncrement)->getTileHeightInCtus();
            ctuAddrIncrement    += (tileWidthInCtus * tileHeightInCtus);
          }
        }

        boundingCtuTSAddrSlice  = ((startCtuTSAddrSlice + ctuAddrIncrement) < numberOfCtusInFrame) ? (startCtuTSAddrSlice + ctuAddrIncrement) : numberOfCtusInFrame;
      }
      break;
    default:
      boundingCtuTSAddrSlice    = numberOfCtusInFrame;
      break;
  }

  // Adjust for tiles and wavefronts.
  if ((sliceMode == FIXED_NUMBER_OF_CTU || sliceMode == FIXED_NUMBER_OF_BYTES) &&
      (m_pcCfg->getNumRowsMinus1() > 0 || m_pcCfg->getNumColumnsMinus1() > 0))
  {
    const UInt ctuRSAddr                  = pcPic->getPicSym()->getCtuTsToRsAddrMap(startCtuTSAddrSlice);
    const UInt startTileIdx               = pcPic->getPicSym()->getTileIdxMap(ctuRSAddr);
    const Bool wavefrontsAreEnabled       = m_pcCfg->getWaveFrontsynchro();

    const TComTile *pStartingTile         = pcPic->getPicSym()->getTComTile(startTileIdx);
    const UInt tileStartTsAddr            = pcPic->getPicSym()->getCtuRsToTsAddrMap(pStartingTile->getFirstCtuRsAddr());
    const UInt tileStartWidth             = pStartingTile->getTileWidthInCtus();
    const UInt tileStartHeight            = pStartingTile->getTileHeightInCtus();
    const UInt tileLastTsAddr_excl        = tileStartTsAddr + tileStartWidth*tileStartHeight;
    const UInt tileBoundingCtuTsAddrSlice = tileLastTsAddr_excl;

    const UInt ctuColumnOfStartingTile    = ((startCtuTSAddrSlice-tileStartTsAddr)%tileStartWidth);
    if (wavefrontsAreEnabled && ctuColumnOfStartingTile!=0)
    {
      // WPP: if a slice does not start at the beginning of a CTB row, it must end within the same CTB row
      const UInt numberOfCTUsToEndOfRow            = tileStartWidth - ctuColumnOfStartingTile;
      const UInt wavefrontTileBoundingCtuAddrSlice = startCtuTSAddrSlice + numberOfCTUsToEndOfRow;
      if (wavefrontTileBoundingCtuAddrSlice < boundingCtuTSAddrSlice)
      {
        boundingCtuTSAddrSlice = wavefrontTileBoundingCtuAddrSlice;
      }
    }

    if (tileBoundingCtuTsAddrSlice < boundingCtuTSAddrSlice)
    {
      boundingCtuTSAddrSlice = tileBoundingCtuTsAddrSlice;
      haveReachedTileBoundary = true;
    }
  }
  else if ((sliceMode == FIXED_NUMBER_OF_CTU || sliceMode == FIXED_NUMBER_OF_BYTES) && pcSlice->getPPS()->getNumSubstreams() > 1 && ((startCtuTSAddrSlice % pcPic->getFrameWidthInCtus()) != 0))
  {
    // Adjust for wavefronts (no tiles).
    // WPP: if a slice does not start at the beginning of a CTB row, it must end within the same CTB row
    boundingCtuTSAddrSlice = min(boundingCtuTSAddrSlice, startCtuTSAddrSlice - (startCtuTSAddrSlice % pcPic->getFrameWidthInCtus()) + (pcPic->getFrameWidthInCtus()));
  }
}

/** Determines the starting and bounding CTU address of current slice / dependent slice
 * \param bEncodeSlice Identifies if the calling function is compressSlice() [false] or encodeSlice() [true]
 * \returns Updates startCtuTsAddr, boundingCtuTsAddr with appropriate CTU address
 */
Void TEncSlice::xDetermineStartAndBoundingCtuTsAddr  ( UInt& startCtuTsAddr, UInt& boundingCtuTsAddr, TComPic* pcPic, const Bool encodingSlice )
{
  TComSlice* pcSlice                 = pcPic->getSlice(getSliceIdx());

  // Non-dependent slice
  UInt startCtuTsAddrSlice           = pcSlice->getSliceCurStartCtuTsAddr();
  Bool haveReachedTileBoundarySlice  = false;
  UInt boundingCtuTsAddrSlice;
  calculateBoundingCtuTsAddrForSlice(startCtuTsAddrSlice, boundingCtuTsAddrSlice, haveReachedTileBoundarySlice, pcPic,
                                     encodingSlice, m_pcCfg->getSliceMode(), m_pcCfg->getSliceArgument(), pcSlice->getSliceCurEndCtuTsAddr());
  pcSlice->setSliceCurEndCtuTsAddr(   boundingCtuTsAddrSlice );
  pcSlice->setSliceCurStartCtuTsAddr( startCtuTsAddrSlice    );

  // Dependent slice
  UInt startCtuTsAddrSliceSegment          = pcSlice->getSliceSegmentCurStartCtuTsAddr();
  Bool haveReachedTileBoundarySliceSegment = false;
  UInt boundingCtuTsAddrSliceSegment;
  calculateBoundingCtuTsAddrForSlice(startCtuTsAddrSliceSegment, boundingCtuTsAddrSliceSegment, haveReachedTileBoundarySliceSegment, pcPic,
                                     encodingSlice, m_pcCfg->getSliceSegmentMode(), m_pcCfg->getSliceSegmentArgument(), pcSlice->getSliceSegmentCurEndCtuTsAddr());
  if (boundingCtuTsAddrSliceSegment>boundingCtuTsAddrSlice)
  {
    boundingCtuTsAddrSliceSegment = boundingCtuTsAddrSlice;
  }
  pcSlice->setSliceSegmentCurEndCtuTsAddr( boundingCtuTsAddrSliceSegment );
  pcSlice->setSliceSegmentCurStartCtuTsAddr(startCtuTsAddrSliceSegment);

  // Make a joint decision based on reconstruction and dependent slice bounds
  startCtuTsAddr    = max(startCtuTsAddrSlice   , startCtuTsAddrSliceSegment   );
  boundingCtuTsAddr = min(boundingCtuTsAddrSlice, boundingCtuTsAddrSliceSegment);

  if (!encodingSlice)
  {
    // For fixed number of CTU within an entropy and reconstruction slice we already know whether we will encounter end of entropy and/or reconstruction slice
    // first. Set the flags accordingly.
    if ( (  (m_pcCfg->getSliceMode()!=FIXED_NUMBER_OF_BYTES && m_pcCfg->getSliceSegmentMode()!=FIXED_NUMBER_OF_BYTES) &&
           !(m_pcCfg->getSliceMode()==NO_SLICES && m_pcCfg->getSliceSegmentMode()==NO_SLICES)
         )
        || haveReachedTileBoundarySlice || haveReachedTileBoundarySliceSegment )
    {
      if (boundingCtuTsAddrSlice > boundingCtuTsAddrSliceSegment)
      {
        pcSlice->setNextSlice       ( false );
        pcSlice->setNextSliceSegment( true );
      }
      else
      {
        pcSlice->setNextSlice       ( true );
        pcSlice->setNextSliceSegment( true );
      }
    }
    else
    {
      pcSlice->setNextSlice       ( false );
      pcSlice->setNextSliceSegment( false );
    }
  }
}

Double TEncSlice::xGetQPValueAccordingToLambda ( Double lambda )
{
  return 4.2005*log(lambda) + 13.7122;
}

//! \}
