#include "rtt_estimator.hpp"

using namespace hls;

void rtt_estimator(	stream<rttEvent>&		txEng2rtt_event,
					stream<rttAck>&			rxEng2rtt_ack,
					stream<rtoUpdate>&		rtt2timer_rtoUpdate)
{
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

#pragma HLS aggregate variable=txEng2rtt_event compact=bit
#pragma HLS aggregate variable=rxEng2rtt_ack compact=bit
#pragma HLS aggregate variable=rtt2timer_rtoUpdate compact=bit

	static rttSampleEntry sampleTable[MAX_SESSIONS];
	#pragma HLS bind_storage variable=sampleTable type=RAM_T2P impl=BRAM
	#pragma HLS aggregate variable=sampleTable compact=bit
	#pragma HLS DEPENDENCE variable=sampleTable inter false
	static rttStatsEntry statsTable[MAX_SESSIONS];
	#pragma HLS bind_storage variable=statsTable type=RAM_T2P impl=BRAM
	#pragma HLS aggregate variable=statsTable compact=bit
	#pragma HLS DEPENDENCE variable=statsTable inter false

	// time base: one tick per MAX_SESSIONS cycles (retransmit-timer units)
	static ap_uint<32> rttNow = 0;
	static ap_uint<16> rttSubTick = 0;

	if (rttSubTick == MAX_SESSIONS-1)
	{
		rttNow++;
		rttSubTick = 0;
	}
	else
	{
		rttSubTick++;
	}

	if (!txEng2rtt_event.empty())
	{
		rttEvent ev = txEng2rtt_event.read();
		if (ev.kind == 0) //new data sent: start a sample if none outstanding
		{
			rttSampleEntry sample = sampleTable[ev.sessionID];
			if (!sample.valid)
			{
				sample.valid = true;
				sample.seq = ev.seq;
				sample.time = rttNow;
				sampleTable[ev.sessionID] = sample;
			}
		}
		else if (ev.kind == 1) //retransmission: discard the sample (Karn)
		{
			sampleTable[ev.sessionID].valid = false;
		}
		else //init: new connection reuses the session ID
		{
			sampleTable[ev.sessionID].valid = false;
			statsTable[ev.sessionID].valid = false;
			rtt2timer_rtoUpdate.write(rtoUpdate(ev.sessionID, TIME_1s)); //RFC 6298 initial RTO
		}
	}
	else if (!rxEng2rtt_ack.empty())
	{
		rttAck ack = rxEng2rtt_ack.read();
		rttSampleEntry sample = sampleTable[ack.sessionID];
		if (sample.valid && seq_geq(ack.ackNumb, sample.seq))
		{
			ap_uint<32> rtt = rttNow - sample.time;
			rttStatsEntry stats = statsTable[ack.sessionID];
			if (!stats.valid) //first measurement (RFC 6298 2.2)
			{
				stats.srtt = rtt;
				stats.rttvar = rtt >> 1;
				stats.valid = true;
			}
			else //RFC 6298 2.3: alpha = 1/8, beta = 1/4
			{
				ap_uint<32> diff = (stats.srtt > rtt) ? (ap_uint<32>)(stats.srtt - rtt)
													  : (ap_uint<32>)(rtt - stats.srtt);
				stats.rttvar = stats.rttvar - (stats.rttvar >> 2) + (diff >> 2);
				stats.srtt = stats.srtt - (stats.srtt >> 3) + (rtt >> 3);
			}
			statsTable[ack.sessionID] = stats;
			sampleTable[ack.sessionID].valid = false;

			ap_uint<32> var4 = stats.rttvar << 2;
			ap_uint<32> rto = stats.srtt + ((var4 == 0) ? (ap_uint<32>)1 : var4);
			if (rto < TIME_250ms) //RTO_MIN (Linux uses 200ms)
			{
				rto = TIME_250ms;
			}
			if (rto > TIME_30s) //RTO_MAX
			{
				rto = TIME_30s;
			}
			rtt2timer_rtoUpdate.write(rtoUpdate(ack.sessionID, rto));
		}
	}
}
