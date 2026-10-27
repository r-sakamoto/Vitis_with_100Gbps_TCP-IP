/*
 * Copyright (c) 2019, Systems Group, ETH Zurich
 * Copyright (c) 2016, Xilinx, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "rx_sar_table.hpp"

using namespace hls;

/** @ingroup rx_sar_table
 * 	This data structure stores the RX(receiving) sliding window
 *  and handles concurrent access from the @ref rx_engine, @ref rx_app_if
 *  and @ref tx_engine
 *  @param[in]		rxEng2rxSar_upd_req
 *  @param[in]		rxApp2rxSar_upd_req
 *  @param[in]		txEng2rxSar_upd_req
 *  @param[out]		rxSar2rxEng_upd_rsp
 *  @param[out]		rxSar2rxApp_upd_rsp
 *  @param[out]		rxSar2txEng_upd_rsp
 */
void rx_sar_table(	stream<rxSarRecvd>&			rxEng2rxSar_upd_req,
					stream<rxSarAppd>&			rxApp2rxSar_upd_req,
					stream<ap_uint<17> >&		txEng2rxSar_req, //read; [16] = consume the D-SACK block
					stream<rxSarEngReply>&		rxSar2rxEng_upd_rsp,
					stream<rxSarAppd>&			rxSar2rxApp_upd_rsp,
					stream<rxSarReply>&			rxSar2txEng_rsp)
{

	static rxSarEntry rx_table[MAX_SESSIONS];

#pragma HLS PIPELINE II=1

#pragma HLS bind_storage variable=rx_table type=RAM_T2P impl=BRAM
#pragma HLS DEPENDENCE variable=rx_table inter false

	// Access from the Tx Engine: read, plus a conditional advance of the
	// SWS-avoidance advertised edge
	if(!txEng2rxSar_req.empty())
	{
		ap_uint<17> req = txEng2rxSar_req.read();
		ap_uint<16> addr = req(15, 0);
		rxSarEntry entry = rx_table[addr];
		rxSarEntry updEntry = entry;
		rxSarReply reply (entry);

		//Pre-calculated usedLength, windowSize to improve timing in metaLoader
#if (WINDOW_SCALE)
		ap_uint<WINDOW_BITS> actualWindowSize = (entry.appd - ((ap_uint<WINDOW_BITS>)entry.recvd)) - 1; // This works even for wrap around
#else
		ap_uint<WINDOW_BITS> actualWindowSize = (entry.appd - ((ap_uint<16>)entry.recvd)) - 1; // This works even for wrap around
#endif
		//receiver SWS avoidance (RFC 1122 4.2.3.3): advance the advertised
		//right edge only in steps of at least one MSS; the edge itself never
		//moves left, the window still shrinks naturally as recvd advances
		ap_uint<32> newEdge = entry.recvd + actualWindowSize;
		ap_uint<32> advEdge = entry.advEdge;
		bool advance = seq_geq(newEdge, advEdge + (ap_uint<32>)MSS);
		if (advance)
		{
			advEdge = newEdge;
			updEntry.advEdge = newEdge;
		}
		//req[16]: this read builds an ACK, which reports (consumes) the D-SACK block
		bool dsackConsume = req[16] && entry.dsackValid;
		updEntry.dsackValid = entry.dsackValid && !req[16];
		if (advance || dsackConsume)
		{
			rx_table[addr] = updEntry;
		}
		ap_uint<WINDOW_BITS> advWindow = seq_lt(advEdge, entry.recvd)
				? (ap_uint<WINDOW_BITS>)0
				: (ap_uint<WINDOW_BITS>)(advEdge - entry.recvd);
#if (WINDOW_SCALE)
		reply.windowSize = advWindow >> entry.win_shift;
#else
		reply.windowSize = advWindow;
#endif

		rxSar2txEng_rsp.write(reply);
	}
	// Read or Write access from the Rx App I/F to update the application pointer
	else if(!rxApp2rxSar_upd_req.empty())
	{
		rxSarAppd in_appd = rxApp2rxSar_upd_req.read();
		if(in_appd.write)
		{
			rx_table[in_appd.sessionID].appd = in_appd.appd;
		}
		else
		{
			rxSar2rxApp_upd_rsp.write(rxSarAppd(in_appd.sessionID, rx_table[in_appd.sessionID].appd));
		}
	}
	// Read or Write access from the Rx Engine
	else if(!rxEng2rxSar_upd_req.empty())
	{
		rxSarRecvd in_recvd = rxEng2rxSar_upd_req.read();
		if (in_recvd.write || in_recvd.tsWrite || in_recvd.eceWrite || in_recvd.dsackWrite)
		{
			if (in_recvd.dsackWrite)
			{
				//RFC 2883: duplicate segment received
				rx_table[in_recvd.sessionID].dsackValid = true;
				rx_table[in_recvd.sessionID].dsackStart = in_recvd.dsackStart;
				rx_table[in_recvd.sessionID].dsackLen = in_recvd.dsackLen;
			}
			if (in_recvd.tsWrite)
			{
				//RFC 7323: advance TS.Recent (also possible on pure ACKs)
				rx_table[in_recvd.sessionID].tsRecent = in_recvd.tsRecent;
			}
			if (in_recvd.eceWrite)
			{
				//RFC 3168: CE received / CWR received
				rx_table[in_recvd.sessionID].ecePending = in_recvd.ece;
			}
			if (in_recvd.write)
			{
				rx_table[in_recvd.sessionID].recvd = in_recvd.recvd;
				rx_table[in_recvd.sessionID].head = in_recvd.head;
				rx_table[in_recvd.sessionID].offset = in_recvd.offset;
				rx_table[in_recvd.sessionID].gap = in_recvd.gap;
				rx_table[in_recvd.sessionID].head2 = in_recvd.head2;
				rx_table[in_recvd.sessionID].offset2 = in_recvd.offset2;
				rx_table[in_recvd.sessionID].gap2 = in_recvd.gap2;
				if (in_recvd.init)
				{
					rx_table[in_recvd.sessionID].appd = in_recvd.recvd;
					rx_table[in_recvd.sessionID].sackEnabled = in_recvd.sackEnabled;
					rx_table[in_recvd.sessionID].tsEnabled = in_recvd.tsEnabled;
					rx_table[in_recvd.sessionID].ecnEnabled = in_recvd.ecnEnabled;
					rx_table[in_recvd.sessionID].ecePending = false;
					rx_table[in_recvd.sessionID].dsackValid = false;
					//initial advertised edge: the whole (empty) buffer
					rx_table[in_recvd.sessionID].advEdge = in_recvd.recvd + (BUFFER_SIZE - 1);
#if (WINDOW_SCALE)
					rx_table[in_recvd.sessionID].win_shift = in_recvd.win_shift;
#endif
				}
			}
		}
		else
		{
			rxSar2rxEng_upd_rsp.write(rxSarEngReply(rx_table[in_recvd.sessionID]));
		}
	}
}
