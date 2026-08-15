/*
 * TCP keepalive timer (RFC 1122 4.2.3.6).
 *
 * Every received in-window segment resets a per-session idle counter. When a
 * session stays idle for KEEPALIVE_IDLE, a KEEPALIVE event is fired; the TX
 * engine turns it into an ACK probe with seq = snd_nxt - 1, which forces the
 * peer to answer with an ACK (which in turn resets the idle counter through
 * the normal RX path). After KEEPALIVE_MAX_PROBES unanswered probes spaced
 * KEEPALIVE_INTERVAL apart, the session is aborted: the state table entry is
 * released and the application receives a closed notification.
 */
#pragma once

#include "../toe.hpp"
#include "../toe_internals.hpp"

// NOTE: #define, not static const ap_uint<32> = TIME_60s - Vitis HLS folds a
// chained ap_uint static initializer (ap_uint = ap_uint) to 0 in synthesis.
#define KEEPALIVE_IDLE		TIME_60s
#define KEEPALIVE_INTERVAL	TIME_10s
static const ap_uint<3> KEEPALIVE_MAX_PROBES = 4;

struct keepaliveTimerEntry
{
	ap_uint<32>	time;
	ap_uint<3>	probes;
	bool		active;
};

void keepalive_timer(	hls::stream<ap_uint<16> >&		rxEng2timer_resetKeepalive,
						hls::stream<ap_uint<16> >&		stateTable2timer_releaseKeepalive,
						hls::stream<event>&				keepalive2eventEng_setEvent,
						hls::stream<ap_uint<16> >&		keepalive2stateTable_releaseState,
						hls::stream<appNotification>&	keepalive2rxApp_notification);
