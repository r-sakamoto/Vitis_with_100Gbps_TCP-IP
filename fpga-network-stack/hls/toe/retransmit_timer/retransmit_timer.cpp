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

#include "retransmit_timer.hpp"

using namespace hls;

/** @ingroup retransmit_timer
 *  The @ref tx_engine sends the Session-ID and Eventy type through the @param txRetransmitTimerFifoIn.
 *  If the timer is unactivated for this session it is activated, the time-out interval is set depending on
 *  how often the session already time-outed.
 *  The @ref rx_engine indicates when a timer for a specific session has to be stopped.
 *	If a timer times-out the corresponding EVent is fired back to the @ref tx_engine. If a session times-out more than 4 times
 *	in a row, it is aborted. The session is released through @param retransmitTimerReleaseFifoOut and the application is
 *	notified through @param timerNotificationFifoOut.
 *	Currently the II is 2 which means that the interval take twice as much time as defined.
 *  @param[in]		rxEng2timer_clearRetransmitTimer
 *  @param[in]		txEng2timer_setRetransmitTimer
 *  @param[out]		rtTimer2eventEng_setEvent
 *  @param[out]		rtTimer2stateTable_releaseState
 *  @param[out]		rtTimer2rxApp_notification
 */
void retransmit_timer(	stream<rxRetransmitTimerUpdate>&	rxEng2timer_clearRetransmitTimer,
						stream<txRetransmitTimerSet>&		txEng2timer_setRetransmitTimer,
						stream<rtoUpdate>&					rtt2timer_rtoUpdate,
						stream<event>&						rtTimer2eventEng_setEvent,
						stream<ap_uint<16> >&				rtTimer2stateTable_releaseState,
						stream<appNotification>&			rtTimer2rxApp_notification,
						stream<openStatus>&					rtTimer2txApp_notification)
{
#pragma HLS PIPELINE II=1
//#pragma HLS INLINE

#pragma HLS aggregate  variable=rxEng2timer_clearRetransmitTimer compact=bit
#pragma HLS aggregate  variable=txEng2timer_setRetransmitTimer compact=bit
#pragma HLS aggregate  variable=rtTimer2eventEng_setEvent compact=bit
#pragma HLS aggregate  variable=rtTimer2stateTable_releaseState compact=bit
#pragma HLS aggregate  variable=rtTimer2rxApp_notification compact=bit

	static retransmitTimerEntry retransmitTimerTable[MAX_SESSIONS];
	#pragma HLS bind_storage variable=retransmitTimerTable type=RAM_T2P impl=BRAM
	#pragma HLS aggregate  variable=retransmitTimerTable compact=bit
	#pragma HLS DEPENDENCE variable=retransmitTimerTable inter false

	static ap_uint<16>			rt_position = 0;
	static bool							rt_waitForWrite = false;
	static ap_uint<16>					rt_prevPosition = 0;
	static rxRetransmitTimerUpdate	rt_update;

	retransmitTimerEntry	currEntry;
	ap_uint<1> operationSwitch = 0;
	txRetransmitTimerSet	set;
	ap_uint<16>				currID;

	if (rt_waitForWrite && rt_update.sessionID != rt_prevPosition) //TODO maybe prevprev too
	{
		if (!rt_update.stop)
		{
			// partial ACK: restart with the measured RTO (0 = not yet measured)
			ap_uint<32> entryRto = retransmitTimerTable[rt_update.sessionID].rto;
			retransmitTimerTable[rt_update.sessionID].time = (entryRto == 0) ? TIME_1s : entryRto;
		}
		else
		{
			retransmitTimerTable[rt_update.sessionID].time = 0;
			retransmitTimerTable[rt_update.sessionID].active = false;
			retransmitTimerTable[rt_update.sessionID].activeOpen = false;
		}
		retransmitTimerTable[rt_update.sessionID].retries = 0;
		rt_waitForWrite = false;
	}
	else if (!rxEng2timer_clearRetransmitTimer.empty() && !rt_waitForWrite) //FIXME rx path has priority over tx path
	{
		rxEng2timer_clearRetransmitTimer.read(rt_update);
		rt_waitForWrite = true;
	}
	else if (!rtt2timer_rtoUpdate.empty())
	{
		// smoothed RTO from the RTT estimator (RFC 6298)
		rtoUpdate rttUpd = rtt2timer_rtoUpdate.read();
		retransmitTimerTable[rttUpd.sessionID].rto = rttUpd.rto;
	}
	else
	{
		currID = rt_position;

		if (!txEng2timer_setRetransmitTimer.empty())
		{
			txEng2timer_setRetransmitTimer.read(set);
			currID = set.sessionID;
			operationSwitch = 1;
			//RMW-hazard guard, wrap-safe: the naive sessionID-3 comparison
			//never fires for IDs 0..2, losing timer writes to those sessions
			{
				ap_uint<16> setDist = (set.sessionID >= rt_position)
						? (ap_uint<16>)(set.sessionID - rt_position)
						: (ap_uint<16>)(set.sessionID + MAX_SESSIONS - rt_position);
				if (setDist < 3)
				{
					rt_position += 5;
					if (rt_position >= MAX_SESSIONS)
					{
						rt_position -= MAX_SESSIONS;
					}
				}
			}
		}
		else
		{
			// increment position
			rt_position++;
			if (rt_position >= MAX_SESSIONS)
			{
				rt_position = 0;
			}
			operationSwitch = 0;
		}

		// Get entry from table
		currEntry = retransmitTimerTable[currID];
		switch (operationSwitch)
		{
		case 1:
			// a SYN-ACK replacing a running SYN timer is a simultaneous open;
			// SYN-ACK retransmissions keep the flag
			if (set.type == SYN)
			{
				currEntry.activeOpen = true;
			}
			else if (set.type == SYN_ACK)
			{
				if (currEntry.type == SYN && currEntry.active)
				{
					currEntry.activeOpen = true;
				}
				else if (currEntry.type != SYN_ACK)
				{
					currEntry.activeOpen = false;
				}
			}
			else
			{
				currEntry.activeOpen = false;
			}
			currEntry.type = set.type;
			if (!currEntry.active)
			{
				// RFC 6298: measured RTO with exponential backoff (2^retries),
				// clamped to RTO_MAX. rto == 0 means no RTT sample yet -> 1s.
				ap_uint<32> baseRto = (currEntry.rto == 0) ? TIME_1s : currEntry.rto;
				ap_uint<32> backedOff = baseRto << currEntry.retries;
				currEntry.time = (backedOff > TIME_30s) ? TIME_30s : backedOff;
			}
			currEntry.active = true;
			break;
		case 0:
			if (currEntry.active)
			{
				if (currEntry.time > 0)
				{
					currEntry.time--;
				}
				// We need to check if we can generate another event, otherwise we might end up in a Deadlock,
				// since the TX Engine will not be able to set new retransmit timers
				else if (!rtTimer2eventEng_setEvent.full())
				{
					currEntry.time = 0;
					currEntry.active = false;
					// data connections retry longer (Linux tcp_retries2 ~ 15;
					// with the 30s RTO cap 7 retries already span >1 minute),
					// handshake segments keep the short ladder
					ap_uint<3> maxRetries = (currEntry.type == SYN || currEntry.type == SYN_ACK) ? 4 : 7;
					if (currEntry.retries < maxRetries)
					{
						currEntry.retries++;
						rtTimer2eventEng_setEvent.write(event(currEntry.type, currID, currEntry.retries));
					}
					else
					{
						currEntry.retries = 0;
						rtTimer2stateTable_releaseState.write(currID);
						bool openFailed = (currEntry.type == SYN) || (currEntry.type == SYN_ACK && currEntry.activeOpen);
						currEntry.activeOpen = false;
						if (openFailed)
						{
							rtTimer2txApp_notification.write(openStatus(currID, false));
						}
						else if (currEntry.type != SYN_ACK)
						{
							// a half-open passive session was never visible to
							// the application: release it silently (as Linux
							// drops an unanswered request socket)
							rtTimer2rxApp_notification.write(appNotification(currID, true)); //TIME_OUT
						}
					}
				}
			} //if active
			break;
		} //switch
		// write entry back
		retransmitTimerTable[currID] = currEntry;
		rt_prevPosition = currID;
	}
}
