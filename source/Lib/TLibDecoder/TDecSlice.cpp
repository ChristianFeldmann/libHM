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

/** \file     TDecSlice.cpp
    \brief    slice decoder class
*/

#include "TDecSlice.h"

//! \ingroup TLibDecoder
//! \{

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

TDecSlice::TDecSlice()
{
  m_pcBufferSbacDecoders = NULL;
  m_pcBufferBinCABACs    = NULL;
  m_pcBufferLowLatSbacDecoders = NULL;
  m_pcBufferLowLatBinCABACs    = NULL;
}

TDecSlice::~TDecSlice()
{
  for (std::vector<TDecSbac*>::iterator i = CTXMem.begin(); i != CTXMem.end(); i++)
  {
    delete (*i);
  }
  CTXMem.clear();
}

Void TDecSlice::initCtxMem(  UInt i )
{
  for (std::vector<TDecSbac*>::iterator j = CTXMem.begin(); j != CTXMem.end(); j++)
  {
    delete (*j);
  }
  CTXMem.clear();
  CTXMem.resize(i);
}

Void TDecSlice::create()
{
}

Void TDecSlice::destroy()
{
  if ( m_pcBufferSbacDecoders )
  {
    delete[] m_pcBufferSbacDecoders;
    m_pcBufferSbacDecoders = NULL;
  }
  if ( m_pcBufferBinCABACs )
  {
    delete[] m_pcBufferBinCABACs;
    m_pcBufferBinCABACs = NULL;
  }
  if ( m_pcBufferLowLatSbacDecoders )
  {
    delete[] m_pcBufferLowLatSbacDecoders;
    m_pcBufferLowLatSbacDecoders = NULL;
  }
  if ( m_pcBufferLowLatBinCABACs )
  {
    delete[] m_pcBufferLowLatBinCABACs;
    m_pcBufferLowLatBinCABACs = NULL;
  }
}

Void TDecSlice::init(TDecEntropy* pcEntropyDecoder, TDecCu* pcCuDecoder)
{
  m_pcEntropyDecoder  = pcEntropyDecoder;
  m_pcCuDecoder       = pcCuDecoder;
}

Void TDecSlice::decompressSlice(TComInputBitstream** ppcSubstreams, TComPic* pcPic, TDecSbac* pcSbacDecoder, TDecSbac* pcSbacDecoders)
{
  UInt        uiIsLast = 0;
  TComSlice*  pcSlice = pcPic->getSlice(pcPic->getCurrSliceIdx());
  const Int   startCtuTsAddr = max(pcSlice->getSliceCurStartCtuTsAddr(), pcSlice->getSliceSegmentCurStartCtuTsAddr());
  const Int   startCtuRsAddr = pcPic->getPicSym()->getCtuTsToRsAddrMap(startCtuTsAddr);

  // decoder doesn't need prediction & residual frame buffer
  pcPic->setPicYuvPred( 0 );
  pcPic->setPicYuvResi( 0 );

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

  UInt uiTilesAcross   = pcPic->getPicSym()->getNumTileColumnsMinus1()+1;

  // delete decoders if already allocated in previous slice
  if (m_pcBufferSbacDecoders)
  {
    delete [] m_pcBufferSbacDecoders;
  }
  if (m_pcBufferBinCABACs)
  {
    delete [] m_pcBufferBinCABACs;
  }
  // allocate new decoders based on tile numbaer
  m_pcBufferSbacDecoders = new TDecSbac    [uiTilesAcross];
  m_pcBufferBinCABACs    = new TDecBinCABAC[uiTilesAcross];
  for (UInt ui = 0; ui < uiTilesAcross; ui++)
  {
    m_pcBufferSbacDecoders[ui].init(&m_pcBufferBinCABACs[ui]);
  }
  //save init. state
  for (UInt ui = 0; ui < uiTilesAcross; ui++)
  {
    m_pcBufferSbacDecoders[ui].load(pcSbacDecoder);
  }

  // free memory if already allocated in previous call
  if (m_pcBufferLowLatSbacDecoders)
  {
    delete [] m_pcBufferLowLatSbacDecoders;
  }
  if (m_pcBufferLowLatBinCABACs)
  {
    delete [] m_pcBufferLowLatBinCABACs;
  }
  m_pcBufferLowLatSbacDecoders = new TDecSbac    [uiTilesAcross];
  m_pcBufferLowLatBinCABACs    = new TDecBinCABAC[uiTilesAcross];
  for (UInt ui = 0; ui < uiTilesAcross; ui++)
  {
    m_pcBufferLowLatSbacDecoders[ui].init(&m_pcBufferLowLatBinCABACs[ui]);
  }
  //save init. state
  for (UInt ui = 0; ui < uiTilesAcross; ui++)
  {
    m_pcBufferLowLatSbacDecoders[ui].load(pcSbacDecoder);
  }

  const UInt frameWidthInCtus  = pcPic->getPicSym()->getFrameWidthInCtus();
  const Bool depSliceSegmentsEnabled = pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag();
  const UInt startTileIdx=pcPic->getPicSym()->getTileIdxMap(startCtuRsAddr);

  // The first CTU of the slice is the first coded substream, but the global substream number, as calculated by getSubstreamForCtuAddr may be higher.
  // This calculates the common offset for all substreams in this slice.
  const UInt subStreamOffset=pcPic->getSubstreamForCtuAddr(startCtuRsAddr, true, pcSlice);
  if( depSliceSegmentsEnabled )
  {
    const TComTile *pCurrentTile=pcPic->getPicSym()->getTComTile(startTileIdx);
    UInt firstCtuRsAddrOfTile = pCurrentTile->getFirstCtuRsAddr();
    if( (!pcSlice->isNextSlice()) && startCtuRsAddr != firstCtuRsAddrOfTile) // Is this a dependent slice segment and not the start of a tile?
    {
      if(pcSlice->getPPS()->getEntropyCodingSyncEnabledFlag())
      {
        UInt tileColumnNumber = startTileIdx % (pcPic->getPicSym()->getNumTileColumnsMinus1()+1);
        m_pcBufferSbacDecoders[tileColumnNumber].loadContexts( CTXMem[1]  );//2.CTU
        if ( pCurrentTile->getTileWidthInCtus() < 2)
        {
          CTXMem[0]->loadContexts(pcSbacDecoder); // If tile width is less than 2, need to ensure CTX states get initialised to un-adapted CABAC. Set here, to load a few lines later (!)
        }
      }
      pcSbacDecoder->loadContexts(CTXMem[0] ); //end of depSlice-1
      pcSbacDecoders[0].loadContexts(pcSbacDecoder); // The first substream used for the slice will always be 0. (The original code was equivalent)
    }
    else
    {
      if(pcSlice->getPPS()->getEntropyCodingSyncEnabledFlag())
      {
        CTXMem[1]->loadContexts(pcSbacDecoder);
      }
      CTXMem[0]->loadContexts(pcSbacDecoder);
    }
  }

  for( Int ctuRsAddr = startCtuRsAddr ;!uiIsLast && ctuRsAddr < pcPic->getNumberOfCtusInFrame(); ctuRsAddr = pcPic->getPicSym()->getCtuTsToRsAddrMap(pcPic->getPicSym()->getCtuRsToTsAddrMap(ctuRsAddr)+1) )
  {
    TComDataCU* pCtu = pcPic->getCtu( ctuRsAddr );
    pCtu->initCtu( pcPic, ctuRsAddr );
    const UInt tileColumnNumber = pcPic->getPicSym()->getTileIdxMap(ctuRsAddr) % (pcPic->getPicSym()->getNumTileColumnsMinus1()+1); // what column of tiles are we in?
    const UInt firstCtuRsAddrOfTile = pcPic->getPicSym()->getTComTile(pcPic->getPicSym()->getTileIdxMap(ctuRsAddr))->getFirstCtuRsAddr();
    const UInt tileXPosInCtus = firstCtuRsAddrOfTile % frameWidthInCtus;
    const UInt ctuXPosInCtus  = ctuRsAddr % frameWidthInCtus;
    const UInt uiSubStrm=pcPic->getSubstreamForCtuAddr(ctuRsAddr, true, pcSlice)-subStreamOffset;
    // inherit from TR if necessary, select substream to use.
    if( (pcSlice->getPPS()->getNumSubstreams() > 1) || ( depSliceSegmentsEnabled  && (ctuXPosInCtus == tileXPosInCtus)&&(pcSlice->getPPS()->getEntropyCodingSyncEnabledFlag()) ))
    {
      m_pcEntropyDecoder->setBitstream( ppcSubstreams[uiSubStrm] );
      // Synchronize cabac probabilities with upper-right CTU if it's available and we're at the start of a line.
      if (((pcSlice->getPPS()->getNumSubstreams() > 1) || depSliceSegmentsEnabled ) && (ctuXPosInCtus == tileXPosInCtus)&&(pcSlice->getPPS()->getEntropyCodingSyncEnabledFlag()))
      {
        // We'll sync if the TR is available.
        TComDataCU *pCtuUp = pCtu->getCtuAbove();
        TComDataCU *pCtuTR = NULL;
        if ( pCtuUp && ((ctuRsAddr%frameWidthInCtus+1) < frameWidthInCtus)  )
        {
          pCtuTR = pcPic->getCtu( ctuRsAddr - frameWidthInCtus + 1 );
        }
        if ( (true/*bEnforceSliceRestriction*/ && !pCtu->CUIsFromSameSliceAndTile(pCtuTR)) )
        {
          // TR not available.
        }
        else
        {
          // TR is available, we use it.
          pcSbacDecoders[uiSubStrm].loadContexts( &m_pcBufferSbacDecoders[tileColumnNumber] );
        }
      }
      pcSbacDecoder->load(&pcSbacDecoders[uiSubStrm]);  //this load is used to simplify the code (avoid to change all the call to pcSbacDecoders)
    }
    if ( (ctuRsAddr == pcPic->getPicSym()->getTComTile(pcPic->getPicSym()->getTileIdxMap(ctuRsAddr))->getFirstCtuRsAddr()) // It is first in tile.
         && (ctuRsAddr!=0) // !first in frame
         && (ctuRsAddr!=pcPic->getPicSym()->getCtuTsToRsAddrMap(pcSlice->getSliceCurStartCtuTsAddr()))         // !first in slice
         && (ctuRsAddr!=pcPic->getPicSym()->getCtuTsToRsAddrMap(pcSlice->getSliceSegmentCurStartCtuTsAddr()))  // !first in slice segment
         )
    {
      if (pcSlice->getPPS()->getNumSubstreams() > 1)
      {
        // We're crossing into another tile, tiles are independent.
        // When tiles are independent, we have "substreams per tile".  Each substream has already been terminated, and we no longer
        // have to perform it here.
        // For TILES_DECODER, there can be a header at the start of the 1st substream in a tile.  These are read when the substreams
        // are extracted, not here.
      }
      else
      {
        SliceType sliceType  = pcSlice->getSliceType();
        if (pcSlice->getCabacInitFlag())
        {
          switch (sliceType)
          {
          case P_SLICE:           // change initialization table to B_SLICE intialization
            sliceType = B_SLICE;
            break;
          case B_SLICE:           // change initialization table to P_SLICE intialization
            sliceType = P_SLICE;
            break;
          default     :           // should not occur
            assert(0);
            break;
          }
        }
        m_pcEntropyDecoder->updateContextTables( sliceType, pcSlice->getSliceQp() );
      }

    }

#if ENC_DEC_TRACE
    g_bJustDoIt = g_bEncDecTraceEnable;
#endif

    if ( pcSlice->getSPS()->getUseSAO() )
    {
      SAOBlkParam& saoblkParam = (pcPic->getPicSym()->getSAOBlkParam())[ctuRsAddr];
      Bool bIsSAOSliceEnabled = false;
      Bool sliceEnabled[MAX_NUM_COMPONENT];
      for(Int comp=0; comp < MAX_NUM_COMPONENT; comp++)
      {
        ComponentID compId=ComponentID(comp);
        sliceEnabled[compId] = pcSlice->getSaoEnabledFlag(toChannelType(compId)) && (comp < pcPic->getNumberValidComponents());
        if (sliceEnabled[compId]) bIsSAOSliceEnabled=true;
        saoblkParam[compId].modeIdc = SAO_MODE_OFF;
      }
      if (bIsSAOSliceEnabled)
      {
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

        pcSbacDecoder->parseSAOBlkParam( saoblkParam, sliceEnabled, leftMergeAvail, aboveMergeAvail);
      }
    }

    m_pcCuDecoder->decodeCtu     ( pCtu, uiIsLast );
    m_pcCuDecoder->decompressCtu ( pCtu );

#if ENC_DEC_TRACE
    g_bJustDoIt = g_bEncDecTraceDisable;
#endif
    pcSbacDecoders[uiSubStrm].load(pcSbacDecoder);

    if ( ctuXPosInCtus == pcPic->getPicSym()->getTComTile(pcPic->getPicSym()->getTileIdxMap(ctuRsAddr))->getRightEdgePosInCtus()
        && pcSlice->getPPS()->getEntropyCodingSyncEnabledFlag()
        && !uiIsLast )
    {
      // Parse end_of_substream_one_bit for WPP case
      UInt binVal;
      pcSbacDecoder->parseTerminatingBit( binVal );
      assert( binVal );
    }

    //Store probabilities of second CTU in line into buffer
    if ( (ctuXPosInCtus == tileXPosInCtus+1)&& (depSliceSegmentsEnabled || (pcSlice->getPPS()->getNumSubstreams() > 1)) && (pcSlice->getPPS()->getEntropyCodingSyncEnabledFlag()) )
    {
      m_pcBufferSbacDecoders[tileColumnNumber].loadContexts( &pcSbacDecoders[uiSubStrm] );
    }
    if( uiIsLast && depSliceSegmentsEnabled )
    {
      if (pcSlice->getPPS()->getEntropyCodingSyncEnabledFlag())
      {
        CTXMem[1]->loadContexts( &m_pcBufferSbacDecoders[tileColumnNumber] );//ctx 2.CTU
      }
      CTXMem[0]->loadContexts( pcSbacDecoder );//ctx end of dep.slice
      return;
    }
  }
}

ParameterSetManagerDecoder::ParameterSetManagerDecoder()
: m_vpsBuffer(MAX_NUM_VPS)
, m_spsBuffer(MAX_NUM_SPS)
, m_ppsBuffer(MAX_NUM_PPS)
{
}

ParameterSetManagerDecoder::~ParameterSetManagerDecoder()
{

}

TComVPS* ParameterSetManagerDecoder::getPrefetchedVPS  (Int vpsId)
{
  if (m_vpsBuffer.getPS(vpsId) != NULL )
  {
    return m_vpsBuffer.getPS(vpsId);
  }
  else
  {
    return getVPS(vpsId);
  }
}


TComSPS* ParameterSetManagerDecoder::getPrefetchedSPS  (Int spsId)
{
  if (m_spsBuffer.getPS(spsId) != NULL )
  {
    return m_spsBuffer.getPS(spsId);
  }
  else
  {
    return getSPS(spsId);
  }
}

TComPPS* ParameterSetManagerDecoder::getPrefetchedPPS  (Int ppsId)
{
  if (m_ppsBuffer.getPS(ppsId) != NULL )
  {
    return m_ppsBuffer.getPS(ppsId);
  }
  else
  {
    return getPPS(ppsId);
  }
}

Void     ParameterSetManagerDecoder::applyPrefetchedPS()
{
  m_vpsMap.mergePSList(m_vpsBuffer);
  m_ppsMap.mergePSList(m_ppsBuffer);
  m_spsMap.mergePSList(m_spsBuffer);
}

//! \}
