/*
 * RTT estimator (RFC 6298) using Karn's algorithm - no timestamps option.
 *
 * One RTT sample at a time per session: the TX engine reports the end
 * sequence number of newly sent data (kind 0) and invalidates the sample on
 * any retransmission (kind 1, Karn) or connection (re)initialization
 * (kind 2). The RX engine reports cumulative ACK numbers; when the sampled
 * sequence number is covered, SRTT/RTTVAR are updated and the resulting
 * RTO = clamp(SRTT + max(1, 4*RTTVAR), TIME_250ms, TIME_30s) is pushed to
 * the retransmit timer.
 *
 * Time base: one tick per MAX_SESSIONS clock cycles - the same unit in which
 * the retransmit timer decrements a session's counter.
 */
#pragma once

#include "../toe_internals.hpp"

struct rttSampleEntry
{
	ap_uint<32>	seq;
	ap_uint<32>	time;
	bool		valid;
};

struct rttStatsEntry
{
	ap_uint<32>	srtt;
	ap_uint<32>	rttvar;
	bool		valid;
};

void rtt_estimator(	hls::stream<rttEvent>&		txEng2rtt_event,
					hls::stream<rttAck>&		rxEng2rtt_ack,
					hls::stream<rtoUpdate>&		rtt2timer_rtoUpdate);
