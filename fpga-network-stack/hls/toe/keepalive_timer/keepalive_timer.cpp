#include "keepalive_timer.hpp"

using namespace hls;

void keepalive_timer(	stream<ap_uint<16> >&		rxEng2timer_resetKeepalive,
						stream<ap_uint<16> >&		stateTable2timer_releaseKeepalive,
						stream<event>&				keepalive2eventEng_setEvent,
						stream<ap_uint<16> >&		keepalive2stateTable_releaseState,
						stream<appNotification>&	keepalive2rxApp_notification)
{
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

#pragma HLS aggregate variable=keepalive2eventEng_setEvent compact=bit
#pragma HLS aggregate variable=keepalive2rxApp_notification compact=bit

	static keepaliveTimerEntry keepaliveTimerTable[MAX_SESSIONS];
	#pragma HLS bind_storage variable=keepaliveTimerTable type=RAM_T2P impl=BRAM
	#pragma HLS aggregate variable=keepaliveTimerTable compact=bit
	#pragma HLS DEPENDENCE variable=keepaliveTimerTable inter false

	static ap_uint<16> ka_position = 0;

	keepaliveTimerEntry currEntry;
	ap_uint<16> currID;

	if (!stateTable2timer_releaseKeepalive.empty())
	{
		//session torn down: stop probing it
		ap_uint<16> releaseID = stateTable2timer_releaseKeepalive.read();
		keepaliveTimerTable[releaseID].active = false;
		keepaliveTimerTable[releaseID].time = 0;
		keepaliveTimerTable[releaseID].probes = 0;
		if (releaseID-3 < ka_position && ka_position <= releaseID)
		{
			ka_position += 5;
		}
	}
	else if (!rxEng2timer_resetKeepalive.empty())
	{
		//activity on the session: restart the idle period
		ap_uint<16> resetID = rxEng2timer_resetKeepalive.read();
		keepaliveTimerTable[resetID].active = true;
		keepaliveTimerTable[resetID].time = KEEPALIVE_IDLE;
		keepaliveTimerTable[resetID].probes = 0;
		if (resetID-3 < ka_position && ka_position <= resetID)
		{
			ka_position += 5;
		}
	}
	else
	{
		currID = ka_position;
		ka_position++;
		if (ka_position >= MAX_SESSIONS)
		{
			ka_position = 0;
		}

		currEntry = keepaliveTimerTable[currID];
		if (currEntry.active)
		{
			if (currEntry.time > 0)
			{
				currEntry.time--;
			}
			else if (!keepalive2eventEng_setEvent.full()
					&& !keepalive2stateTable_releaseState.full()
					&& !keepalive2rxApp_notification.full())
			{
				if (currEntry.probes < KEEPALIVE_MAX_PROBES)
				{
					currEntry.probes++;
					currEntry.time = KEEPALIVE_INTERVAL;
					keepalive2eventEng_setEvent.write(event(KEEPALIVE, currID));
				}
				else
				{
					//peer is unresponsive: abort the connection
					currEntry.active = false;
					currEntry.probes = 0;
					keepalive2stateTable_releaseState.write(currID);
					keepalive2rxApp_notification.write(appNotification(currID, true)); //TIME_OUT
				}
			}
			keepaliveTimerTable[currID] = currEntry;
		}
	}
}
