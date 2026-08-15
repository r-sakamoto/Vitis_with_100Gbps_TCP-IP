/************************************************
Copyright (c) 2016, Xilinx, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software
without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.// Copyright (c) 2015 Xilinx, Inc.
************************************************/

#include "probe_timer.hpp"

using namespace hls;

/** @ingroup probe_timer
 *  Zero-window persist timer (RFC 1122 4.2.2.17). A session is armed when
 *  the peer advertises a zero window (or, without TCP_NODELAY, when the TX
 *  engine could not send). After TIME_50ms a probe event is fired:
 *  - TCP_NODELAY: a KEEPALIVE event, i.e. a pure ACK with seq = snd_nxt - 1
 *    that forces the peer to answer with its current window
 *  - otherwise: a TX event that resumes transmission
 *  A clear (window opened) fires the same event once so transmission resumes
 *  immediately.
 *
 *  Structured as a single read->modify->write path so that all BRAM writes
 *  commit in invocation order (a direct early-stage write in a branch is
 *  otherwise overtaken by the stale write-back of a sweep in flight).
 *	@param[in]		rxEng2timer_clearProbeTimer
 *	@param[in]		txEng2timer_setProbeTimer
 *	@param[out]		probeTimer2eventEng_setEvent
 */
void probe_timer(	stream<ap_uint<16> >&		rxEng2timer_clearProbeTimer,
					stream<ap_uint<16> >&		txEng2timer_setProbeTimer,
					stream<event>&				probeTimer2eventEng_setEvent)
{
#pragma HLS aggregate  variable=txEng2timer_setProbeTimer compact=bit
#pragma HLS aggregate  variable=probeTimer2eventEng_setEvent compact=bit

#pragma HLS PIPELINE II=1

	static probe_timer_entry probeTimerTable[MAX_SESSIONS];
	#pragma HLS bind_storage variable=probeTimerTable type=RAM_T2P impl=BRAM
	#pragma HLS aggregate  variable=probeTimerTable compact=bit
	#pragma HLS DEPENDENCE variable=probeTimerTable inter false

	static ap_uint<16> pt_position = 0;

	probe_timer_entry currEntry;
	ap_uint<16> currID;
	bool isSet = false;
	bool isClear = false;

	if (!txEng2timer_setProbeTimer.empty())
	{
		currID = txEng2timer_setProbeTimer.read();
		isSet = true;
	}
	else if (!rxEng2timer_clearProbeTimer.empty())
	{
		currID = rxEng2timer_clearProbeTimer.read();
		isClear = true;
	}
	else
	{
		currID = pt_position;
		pt_position++;
		if (pt_position >= MAX_SESSIONS)
		{
			pt_position = 0;
		}
	}

	if (isSet || isClear)
	{
		//RMW-hazard guard (wrap-safe, mod MAX_SESSIONS): skip the sweep past
		//an entry it would otherwise read before this update commits
		ap_uint<16> dist = (currID >= pt_position)
				? (ap_uint<16>)(currID - pt_position)
				: (ap_uint<16>)(currID + MAX_SESSIONS - pt_position);
		if (dist < 3)
		{
			pt_position += 5;
			if (pt_position >= MAX_SESSIONS)
			{
				pt_position -= MAX_SESSIONS;
			}
		}
	}

	currEntry = probeTimerTable[currID];
	bool doWrite = true;
	if (isSet)
	{
		currEntry.time = TIME_50ms;
		currEntry.active = true;
	}
	else if (isClear)
	{
		//window opened: resume transmission immediately (only if armed)
		if (currEntry.active && !probeTimer2eventEng_setEvent.full())
		{
#if !(TCP_NODELAY)
			probeTimer2eventEng_setEvent.write(event(TX, currID));
#else
			probeTimer2eventEng_setEvent.write(event(KEEPALIVE, currID));
#endif
		}
		currEntry.time = 0;
		currEntry.active = false;
	}
	else if (currEntry.active)
	{
		if (currEntry.time == 0)
		{
			if (!probeTimer2eventEng_setEvent.full())
			{
				currEntry.active = false;
				//zero-window probe; re-armed by the rx engine as long as the
				//peer keeps advertising a zero window
#if !(TCP_NODELAY)
				probeTimer2eventEng_setEvent.write(event(TX, currID));
#else
				probeTimer2eventEng_setEvent.write(event(KEEPALIVE, currID));
#endif
			}
		}
		else
		{
			currEntry.time -= 1;
		}
	}
	else
	{
		doWrite = false;
	}

	if (doWrite)
	{
		probeTimerTable[currID] = currEntry;
	}
}
