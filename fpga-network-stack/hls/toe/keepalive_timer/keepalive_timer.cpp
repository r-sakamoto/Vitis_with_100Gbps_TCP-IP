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
	bool isRelease = false;
	bool isReset = false;

	if (!stateTable2timer_releaseKeepalive.empty())
	{
		//session torn down: stop probing it
		currID = stateTable2timer_releaseKeepalive.read();
		isRelease = true;
	}
	else if (!rxEng2timer_resetKeepalive.empty())
	{
		//activity on the session: restart the idle period
		currID = rxEng2timer_resetKeepalive.read();
		isReset = true;
	}
	else
	{
		currID = ka_position;
		ka_position++;
		if (ka_position >= MAX_SESSIONS)
		{
			ka_position = 0;
		}
	}

	if (isRelease || isReset)
	{
		//RMW-hazard guard (wrap-safe, mod MAX_SESSIONS): if the sweep would
		//read this entry within the next few cycles it would still see the
		//pre-update value (DEPENDENCE inter false) and its write-back would
		//clobber this update - skip the sweep past the entry instead
		ap_uint<16> dist = (currID >= ka_position)
				? (ap_uint<16>)(currID - ka_position)
				: (ap_uint<16>)(currID + MAX_SESSIONS - ka_position);
		if (dist < 3)
		{
			ka_position += 5;
			if (ka_position >= MAX_SESSIONS)
			{
				ka_position -= MAX_SESSIONS;
			}
		}
	}

	//Single read -> modify -> write path for ALL operations. With II=1 every
	//invocation then commits its BRAM write at the same pipeline stage, i.e.
	//in invocation order. (A direct early-stage write in the reset branch was
	//silently overtaken by the later-stage stale write-back of a sweep
	//invocation already in flight.)
	currEntry = keepaliveTimerTable[currID];
	bool doWrite = true;
	if (isRelease)
	{
		currEntry.active = false;
		currEntry.time = 0;
		currEntry.probes = 0;
	}
	else if (isReset)
	{
		currEntry.active = true;
		currEntry.time = KEEPALIVE_IDLE;
		currEntry.probes = 0;
	}
	else if (currEntry.active)
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
	}
	else
	{
		doWrite = false;
	}

	if (doWrite)
	{
		keepaliveTimerTable[currID] = currEntry;
	}
}
