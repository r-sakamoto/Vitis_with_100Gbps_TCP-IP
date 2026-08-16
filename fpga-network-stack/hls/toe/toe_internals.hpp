#pragma once

#include "toe_config.hpp"
#include "../axi_utils.hpp"
#include "../packet.hpp"

#ifndef __SYNTHESIS__
static const ap_uint<32> TIME_64us		= 1;
static const ap_uint<32> TIME_128us		= 1;
static const ap_uint<32> TIME_1ms		= 1;
static const ap_uint<32> TIME_5ms		= 1;
static const ap_uint<32> TIME_25ms		= 1;
static const ap_uint<32> TIME_50ms		= 1;
static const ap_uint<32> TIME_100ms		= 1;
static const ap_uint<32> TIME_250ms		= 1;
static const ap_uint<32> TIME_500ms		= 1;
static const ap_uint<32> TIME_1s		= 1;
static const ap_uint<32> TIME_5s		= 1;
static const ap_uint<32> TIME_7s		= 2;
static const ap_uint<32> TIME_10s		= 3;
static const ap_uint<32> TIME_15s		= 4;
static const ap_uint<32> TIME_20s		= 5;
static const ap_uint<32> TIME_30s		= 6;
static const ap_uint<32> TIME_60s		= 60;
static const ap_uint<32> TIME_120s		= 120;
#else
// TCP_SIM_TIMER_SCALE (default 1) shrinks all protocol timers by that factor.
// Only meant for RTL-simulation builds so that RTO / persist / TIME_WAIT can
// be observed in a practical number of cycles. Must be 1 for hardware builds.
#ifndef TCP_SIM_TIMER_SCALE
#define TCP_SIM_TIMER_SCALE 1
#endif
static const ap_uint<32> TIME_64us		= (       64.0/0.0064/MAX_SESSIONS/TCP_SIM_TIMER_SCALE) + 1;
static const ap_uint<32> TIME_128us		= (      128.0/0.0064/MAX_SESSIONS/TCP_SIM_TIMER_SCALE) + 1;
static const ap_uint<32> TIME_1ms		= (     1000.0/0.0064/MAX_SESSIONS/TCP_SIM_TIMER_SCALE) + 1;
static const ap_uint<32> TIME_5ms		= (     5000.0/0.0064/MAX_SESSIONS/TCP_SIM_TIMER_SCALE) + 1;
static const ap_uint<32> TIME_25ms		= (    25000.0/0.0064/MAX_SESSIONS/TCP_SIM_TIMER_SCALE) + 1;
static const ap_uint<32> TIME_50ms		= (    50000.0/0.0064/MAX_SESSIONS/TCP_SIM_TIMER_SCALE) + 1;
static const ap_uint<32> TIME_100ms		= (   100000.0/0.0064/MAX_SESSIONS/TCP_SIM_TIMER_SCALE) + 1;
static const ap_uint<32> TIME_250ms		= (   250000.0/0.0064/MAX_SESSIONS/TCP_SIM_TIMER_SCALE) + 1;
static const ap_uint<32> TIME_500ms		= (   500000.0/0.0064/MAX_SESSIONS/TCP_SIM_TIMER_SCALE) + 1;
static const ap_uint<32> TIME_1s		= (  1000000.0/0.0064/MAX_SESSIONS/TCP_SIM_TIMER_SCALE) + 1;
static const ap_uint<32> TIME_5s		= (  5000000.0/0.0064/MAX_SESSIONS/TCP_SIM_TIMER_SCALE) + 1;
static const ap_uint<32> TIME_7s		= (  7000000.0/0.0064/MAX_SESSIONS/TCP_SIM_TIMER_SCALE) + 1;
static const ap_uint<32> TIME_10s		= ( 10000000.0/0.0064/MAX_SESSIONS/TCP_SIM_TIMER_SCALE) + 1;
static const ap_uint<32> TIME_15s		= ( 15000000.0/0.0064/MAX_SESSIONS/TCP_SIM_TIMER_SCALE) + 1;
static const ap_uint<32> TIME_20s		= ( 20000000.0/0.0064/MAX_SESSIONS/TCP_SIM_TIMER_SCALE) + 1;
static const ap_uint<32> TIME_30s		= ( 30000000.0/0.0064/MAX_SESSIONS/TCP_SIM_TIMER_SCALE) + 1;
static const ap_uint<32> TIME_60s		= ( 60000000.0/0.0064/MAX_SESSIONS/TCP_SIM_TIMER_SCALE) + 1;
static const ap_uint<32> TIME_120s		= (120000000.0/0.0064/MAX_SESSIONS/TCP_SIM_TIMER_SCALE) + 1;
#endif


// RFC 7323 timestamps: full support (option on every segment including
// payload-carrying ones) is only implemented for the narrow datapath; the
// 512-bit header builder embeds options in the first beat and cannot shift
// the body yet.
#define TCP_TS_OPTION (DATA_WIDTH <= 256)

// Modulo-2^32 TCP sequence number comparisons (RFC 793 / RFC 1982 style).
// Valid as long as the two values are less than 2^31 apart.
inline bool seq_lt (ap_uint<32> a, ap_uint<32> b) { return ((ap_int<32>)(ap_uint<32>)(a - b)) < 0; }
inline bool seq_leq(ap_uint<32> a, ap_uint<32> b) { return ((ap_int<32>)(ap_uint<32>)(a - b)) <= 0; }
inline bool seq_gt (ap_uint<32> a, ap_uint<32> b) { return ((ap_int<32>)(ap_uint<32>)(a - b)) > 0; }
inline bool seq_geq(ap_uint<32> a, ap_uint<32> b) { return ((ap_int<32>)(ap_uint<32>)(a - b)) >= 0; }

enum eventType {TX, RT, ACK, SYN, SYN_ACK, FIN, RST, ACK_NODELAY, KEEPALIVE};
/*
 * There is no explicit LISTEN state (merged into CLOSED).
 * CLOSE_WAIT and FIN_WAIT_2 are supported (half-close): on a peer FIN in
 * ESTABLISHED the session enters CLOSE_WAIT and the application may keep
 * sending until it closes; the ACK of our own FIN moves FIN_WAIT_1 to
 * FIN_WAIT_2.
 */
enum sessionState {CLOSED, SYN_SENT, SYN_RECEIVED, ESTABLISHED, FIN_WAIT_1, FIN_WAIT_2, CLOSING, TIME_WAIT, LAST_ACK, CLOSE_WAIT};


struct fourTuple
{
	ap_uint<32> srcIp;
	ap_uint<32> dstIp;
	ap_uint<16> srcPort;
	ap_uint<16> dstPort;
	fourTuple() {}
	fourTuple(ap_uint<32> srcIp, ap_uint<32> dstIp, ap_uint<16> srcPort, ap_uint<16> dstPort)
			  : srcIp(srcIp), dstIp(dstIp), srcPort(srcPort), dstPort(dstPort) {}
};

inline bool operator < (fourTuple const& lhs, fourTuple const& rhs) {
		return lhs.dstIp < rhs.dstIp || (lhs.dstIp == rhs.dstIp && lhs.srcIp < rhs.srcIp);
	}

struct sessionLookupQuery
{
	fourTuple	tuple;
	bool		allowCreation;
	sessionLookupQuery() {}
	sessionLookupQuery(fourTuple tuple, bool allowCreation)
			:tuple(tuple), allowCreation(allowCreation) {}
};

struct sessionLookupReply
{
	ap_uint<16> sessionID;
	bool		hit;
	sessionLookupReply() {}
	sessionLookupReply(ap_uint<16> id, bool hit)
			:sessionID(id), hit(hit) {}
};


struct stateQuery
{
	ap_uint<16> sessionID;
	sessionState state;
	ap_uint<1>	write;
	stateQuery() {}
	stateQuery(ap_uint<16> id)
				:sessionID(id), state(CLOSED), write(0){}
	stateQuery(ap_uint<16> id, sessionState state, ap_uint<1> write)
				:sessionID(id), state(state), write(write) {}
};

/** @ingroup rx_sar_table
 *  @ingroup rx_engine
 *  @ingroup tx_engine
 *  This struct defines an entry of the @ref rx_sar_table
 */

/*	A gap caused by out-of-order packet:
*	                v- appd         v- recvd    v- offset        v- head
*	|  <- undef ->  |  <committed>  |   <gap>   |  <pre-mature>  |  <- undef ->  |
*/
struct rxSarEntry
{
	ap_uint<32> recvd; 
	ap_uint<WINDOW_BITS> appd;
#if (WINDOW_SCALE)
	ap_uint<4>	win_shift;
#endif
	ap_uint<32> head;
	ap_uint<32> offset;
	bool gap;
	//second out-of-order range (seq order: recvd < offset < head < offset2 < head2)
	ap_uint<32> head2;
	ap_uint<32> offset2;
	bool gap2;
	bool sackEnabled; //SACK permitted was negotiated on this session
	bool tsEnabled;   //RFC 7323 timestamps negotiated on this session
	ap_uint<32> tsRecent; //TS.Recent: last valid peer TSval
	//receiver SWS avoidance (RFC 1122 4.2.3.3): the advertised right window
	//edge only advances in steps of at least one MSS
	ap_uint<32> advEdge;
};

struct rxSarReply
{
	ap_uint<32> recvd;
#if (WINDOW_SCALE)
	ap_uint<4>	win_shift;
#endif
	ap_uint<16>	windowSize;
	ap_uint<WINDOW_BITS>	usedLength;
	//for SACK block generation on duplicate ACKs (single gap)
	ap_uint<32> head;
	ap_uint<32> offset;
	bool		gap;
	ap_uint<32> head2;
	ap_uint<32> offset2;
	bool		gap2;
	bool		sackEnabled;
	bool		tsEnabled;
	ap_uint<32>	tsRecent; //echoed as TSecr on outgoing segments
	rxSarReply() {}
	rxSarReply(rxSarEntry entry)
#if !(WINDOW_SCALE)
		:recvd(entry.recvd), head(entry.head), offset(entry.offset), gap(entry.gap), head2(entry.head2), offset2(entry.offset2), gap2(entry.gap2), sackEnabled(entry.sackEnabled), tsEnabled(entry.tsEnabled), tsRecent(entry.tsRecent) {}
#else
		:recvd(entry.recvd), win_shift(entry.win_shift), head(entry.head), offset(entry.offset), gap(entry.gap), head2(entry.head2), offset2(entry.offset2), gap2(entry.gap2), sackEnabled(entry.sackEnabled), tsEnabled(entry.tsEnabled), tsRecent(entry.tsRecent) {}
#endif
};

struct rxSarRecvd
{
	ap_uint<16> sessionID;
	ap_uint<32> recvd;
	ap_uint<4>	win_shift; //TODO name
	bool		sackEnabled; //only valid on init
	bool		tsEnabled;   //only valid on init (RFC 7323)
	ap_uint<1>	tsWrite;     //update tsRecent with the value below
	ap_uint<32>	tsRecent;
	ap_uint<1> write;
	ap_uint<1> init;
	ap_uint<32> head;
	ap_uint<32> offset;
	bool gap;
	ap_uint<32> head2;
	ap_uint<32> offset2;
	bool gap2; //set explicitly by the rx FSM after construction
	rxSarRecvd() {}
	rxSarRecvd(ap_uint<16> id)
				:sessionID(id), recvd(0), tsEnabled(false), tsWrite(0), write(0), init(0), gap2(false) {}
	rxSarRecvd(ap_uint<16> id, ap_uint<32> recvd)
				:sessionID(id), recvd(recvd), tsEnabled(false), tsWrite(0), head(0), offset(0), gap(0), write(1), init(0), gap2(false) {}
	// rxSarRecvd(ap_uint<16> id, ap_uint<32> recvd, ap_uint<4> win_shift)
	// 				:sessionID(id), recvd(recvd), win_shift(win_shift), write(1), init(1) {}

	rxSarRecvd(ap_uint<16> id, ap_uint<32> recvd, ap_uint<32> head, ap_uint<32>offset, bool gap)
				:sessionID(id), recvd(recvd), tsEnabled(false), tsWrite(0), head(head), offset(offset), gap(gap), write(1), init(0), gap2(false) {}
	rxSarRecvd(ap_uint<16> id, ap_uint<32> recvd, ap_uint<32> head, ap_uint<32>offset, bool gap, ap_uint<4> win_shift)
				:sessionID(id), recvd(recvd), head(head), offset(offset), gap(gap), win_shift(win_shift), sackEnabled(false), tsEnabled(false), tsWrite(0), write(1), init(1), gap2(false) {}
	rxSarRecvd(ap_uint<16> id, ap_uint<32> recvd, ap_uint<32> head, ap_uint<32>offset, bool gap, ap_uint<4> win_shift, bool sackEnabled)
				:sessionID(id), recvd(recvd), head(head), offset(offset), gap(gap), win_shift(win_shift), sackEnabled(sackEnabled), tsEnabled(false), tsWrite(0), write(1), init(1), gap2(false) {}
};

struct rxSarAppd
{
	ap_uint<16> sessionID;
	ap_uint<WINDOW_BITS> appd;
	ap_uint<1>	write;
	rxSarAppd() {}
	rxSarAppd(ap_uint<16> id)
				:sessionID(id), appd(0), write(0) {}
	rxSarAppd(ap_uint<16> id, ap_uint<WINDOW_BITS> appd)
				:sessionID(id), appd(appd), write(1) {}
};


struct txSarEntry
{
	ap_uint<32> ackd;
	ap_uint<32> not_ackd;
	ap_uint<16> recv_window;
	ap_uint<16> mss; //effective segment size: min(peer MSS, local MSS)
#if (WINDOW_SCALE)
	ap_uint<4>	win_shift;
#endif
	ap_uint<WINDOW_BITS> cong_window;
	ap_uint<WINDOW_BITS> slowstart_threshold;
	ap_uint<WINDOW_BITS> app;
	ap_uint<2>	count;
	bool		fastRetransmitted;
	bool		finReady;
	bool		finSent;
	//latest SACK block advertised by the peer (RFC 2018)
	ap_uint<32>	peerSackStart;
	ap_uint<32>	peerSackEnd;
	bool		peerSackValid;
};

struct rxTxSarQuery
{
	ap_uint<16> sessionID;
	ap_uint<32> ackd;
	ap_uint<16> recv_window;
	ap_uint<WINDOW_BITS>	cong_window;
	ap_uint<2>  count;
	bool		fastRetransmitted;
#if (WINDOW_SCALE)
	ap_uint<4>	win_shift;
#endif
	ap_uint<16> mss; //only valid on init
	ap_uint<WINDOW_BITS> ssthresh; //only applied when writeSsthresh is set
	ap_uint<1> writeSsthresh;
	//peer SACK block state after this ACK (stored verbatim)
	ap_uint<32>	peerSackStart;
	ap_uint<32>	peerSackEnd;
	ap_uint<1>	peerSackValid;
	ap_uint<1> write;
	ap_uint<1>	init;
	rxTxSarQuery () {}
	rxTxSarQuery(ap_uint<16> id)
				:sessionID(id), ackd(0), recv_window(0), cong_window(0), count(0), fastRetransmitted(false), mss(MSS), ssthresh(0), writeSsthresh(0), peerSackStart(0), peerSackEnd(0), peerSackValid(0), write(0), init(0) {}
	rxTxSarQuery(ap_uint<16> id, ap_uint<32> ackd, ap_uint<16> recv_win, ap_uint<WINDOW_BITS> cong_win, ap_uint<2> count, bool fastRetransmitted)
				:sessionID(id), ackd(ackd), recv_window(recv_win), cong_window(cong_win), count(count), fastRetransmitted(fastRetransmitted), mss(MSS), ssthresh(0), writeSsthresh(0), peerSackStart(0), peerSackEnd(0), peerSackValid(0), write(1), init(0) {}
#if (WINDOW_SCALE)
	rxTxSarQuery(ap_uint<16> id, ap_uint<32> ackd, ap_uint<16> recv_win, ap_uint<WINDOW_BITS> cong_win, ap_uint<2> count, bool fastRetransmitted, ap_uint<4> win_shift)
				:sessionID(id), ackd(ackd), recv_window(recv_win), cong_window(cong_win), count(count), fastRetransmitted(fastRetransmitted), win_shift(win_shift), mss(MSS), ssthresh(0), writeSsthresh(0), peerSackStart(0), peerSackEnd(0), peerSackValid(0), write(1), init(1) {}
	rxTxSarQuery(ap_uint<16> id, ap_uint<32> ackd, ap_uint<16> recv_win, ap_uint<WINDOW_BITS> cong_win, ap_uint<2> count, bool fastRetransmitted, ap_uint<4> win_shift, ap_uint<16> mss)
				:sessionID(id), ackd(ackd), recv_window(recv_win), cong_window(cong_win), count(count), fastRetransmitted(fastRetransmitted), win_shift(win_shift), mss(mss), ssthresh(0), writeSsthresh(0), peerSackStart(0), peerSackEnd(0), peerSackValid(0), write(1), init(1) {}
#endif
};

struct txTxSarQuery
{
	ap_uint<16> sessionID;
	ap_uint<32> not_ackd;
	ap_uint<1>	write;
	ap_uint<1>	init;
	bool		finReady;
	bool		finSent;
	bool		isRtQuery;
	txTxSarQuery() {}
	txTxSarQuery(ap_uint<16> id)
				:sessionID(id), not_ackd(0), write(0), init(0), finReady(false), finSent(false), isRtQuery(false) {}
	txTxSarQuery(ap_uint<16> id, ap_uint<32> not_ackd, ap_uint<1> write)
				:sessionID(id), not_ackd(not_ackd), write(write), init(0), finReady(false), finSent(false), isRtQuery(false) {}
	txTxSarQuery(ap_uint<16> id, ap_uint<32> not_ackd, ap_uint<1> write, ap_uint<1> init)
				:sessionID(id), not_ackd(not_ackd), write(write), init(init), finReady(false), finSent(false), isRtQuery(false) {}
	txTxSarQuery(ap_uint<16> id, ap_uint<32> not_ackd, ap_uint<1> write, ap_uint<1> init, bool finReady, bool finSent)
				:sessionID(id), not_ackd(not_ackd), write(write), init(init), finReady(finReady), finSent(finSent), isRtQuery(false) {}
	txTxSarQuery(ap_uint<16> id, ap_uint<32> not_ackd, ap_uint<1> write, ap_uint<1> init, bool finReady, bool finSent, bool isRt)
				:sessionID(id), not_ackd(not_ackd), write(write), init(init), finReady(finReady), finSent(finSent), isRtQuery(isRt) {}
};

struct txTxSarRtQuery : public txTxSarQuery
{
	txTxSarRtQuery() {}
	txTxSarRtQuery(const txTxSarQuery& q)
			:txTxSarQuery(q.sessionID, q.not_ackd, q.write, q.init, q.finReady, q.finSent, q.isRtQuery) {}
	txTxSarRtQuery(ap_uint<16> id, ap_uint<WINDOW_BITS> ssthresh)
			:txTxSarQuery(id, ssthresh, 1, 0, false, false, true) {}
	ap_uint<WINDOW_BITS> getThreshold()
	{
	return not_ackd(WINDOW_BITS-1, 0);
	}
};

struct txAppTxSarQuery
{
	ap_uint<16> sessionID;
	//ap_uint<16> ackd;
	ap_uint<WINDOW_BITS> mempt;
	bool		write;
	txAppTxSarQuery() {}
	txAppTxSarQuery(ap_uint<16> id)
				:sessionID(id), mempt(0), write(false) {}
	txAppTxSarQuery(ap_uint<16> id, ap_uint<WINDOW_BITS> pt)
			:sessionID(id), mempt(pt), write(true) {}
};

struct rxTxSarReply
{
	ap_uint<32>	prevAck;
	ap_uint<32> nextByte;
	ap_uint<WINDOW_BITS>	cong_window;
	ap_uint<WINDOW_BITS> slowstart_threshold;
	ap_uint<2>	count;
	bool		fastRetransmitted;
	ap_uint<16>	mss; //effective segment size: min(peer MSS, local MSS)
	ap_uint<32>	peerSackStart;
	ap_uint<32>	peerSackEnd;
	bool		peerSackValid;
	rxTxSarReply() {}
	rxTxSarReply(ap_uint<32> ack, ap_uint<32> next, ap_uint<WINDOW_BITS> cong_win, ap_uint<WINDOW_BITS> sstresh, ap_uint<2> count, bool fastRetransmitted)
			:prevAck(ack), nextByte(next), cong_window(cong_win), slowstart_threshold(sstresh), count(count), fastRetransmitted(fastRetransmitted), mss(MSS), peerSackStart(0), peerSackEnd(0), peerSackValid(false) {}
};

struct txAppTxSarReply
{
	ap_uint<16> sessionID;
	ap_uint<WINDOW_BITS> ackd;
	ap_uint<WINDOW_BITS> mempt;
#if (TCP_NODELAY)
	ap_uint<WINDOW_BITS> min_window;
#endif
	txAppTxSarReply() {}
#if !(TCP_NODELAY)
	txAppTxSarReply(ap_uint<16> id, ap_uint<16> ackd, ap_uint<16> pt)
		:sessionID(id), ackd(ackd), mempt(pt) {}
#else
	txAppTxSarReply(ap_uint<16> id, ap_uint<WINDOW_BITS> ackd, ap_uint<WINDOW_BITS> pt, ap_uint<WINDOW_BITS> min_window)
		:sessionID(id), ackd(ackd), mempt(pt), min_window(min_window) {}
#endif
};

struct txAppTxSarPush
{
	ap_uint<16> sessionID;
	ap_uint<WINDOW_BITS> app;
	txAppTxSarPush() {}
	txAppTxSarPush(ap_uint<16> id, ap_uint<WINDOW_BITS> app)
			:sessionID(id), app(app) {}
};

struct txSarAckPush
{
	ap_uint<16> sessionID;
	ap_uint<WINDOW_BITS> ackd;
#if (TCP_NODELAY)
	ap_uint<WINDOW_BITS> min_window;
#endif
	ap_uint<1>	init;
	txSarAckPush() {}
#if !(TCP_NODELAY)
	txSarAckPush(ap_uint<16> id, ap_uint<WINDOW_BITS> ackd)
		:sessionID(id), ackd(ackd), init(0) {}
	txSarAckPush(ap_uint<16> id, ap_uint<WINDOW_BITS> ackd, ap_uint<1> init)
		:sessionID(id), ackd(ackd), init(init) {}
#else
	txSarAckPush(ap_uint<16> id, ap_uint<WINDOW_BITS> ackd, ap_uint<WINDOW_BITS> min_window)
		:sessionID(id), ackd(ackd), min_window(min_window), init(0) {}
	txSarAckPush(ap_uint<16> id, ap_uint<WINDOW_BITS> ackd, ap_uint<WINDOW_BITS> min_window, ap_uint<1> init)
		:sessionID(id), ackd(ackd), min_window(min_window), init(init) {}
#endif
};

struct txTxSarReply
{
	ap_uint<32> ackd;
	ap_uint<32> not_ackd;
	ap_uint<WINDOW_BITS> usableWindow;
	ap_uint<WINDOW_BITS> app;
	ap_uint<WINDOW_BITS> usedLength;
	bool		finReady;
	bool		finSent;
//#if (WINDOW_SCALE)
	ap_uint<4>	win_shift;
//#endif
	ap_uint<16> mss; //effective segment size: min(peer MSS, local MSS)
	ap_uint<32>	peerSackStart;
	ap_uint<32>	peerSackEnd;
	bool		peerSackValid;
	txTxSarReply() {}
	txTxSarReply(ap_uint<32> ack, ap_uint<32> nack, ap_uint<WINDOW_BITS> usableWindow, ap_uint<WINDOW_BITS> app, ap_uint<WINDOW_BITS> usedLength, bool finReady, bool finSent)
		:ackd(ack), not_ackd(nack), usableWindow(usableWindow), app(app), usedLength(usedLength), finReady(finReady), finSent(finSent), mss(MSS) {}
};

/** RTT estimation (RFC 6298, Karn's algorithm - no timestamps option) */
struct rttEvent
{
	ap_uint<16> sessionID;
	ap_uint<32> seq;   //for kind==0: seq number whose ACK completes the sample
	ap_uint<2>  kind;  //0=sample new data, 1=invalidate (retransmission), 2=init (new connection)
	rttEvent() {}
	rttEvent(ap_uint<16> id, ap_uint<32> seq, ap_uint<2> kind)
			:sessionID(id), seq(seq), kind(kind) {}
};

struct rttAck
{
	ap_uint<16> sessionID;
	ap_uint<32> ackNumb;
	rttAck() {}
	rttAck(ap_uint<16> id, ap_uint<32> ack)
			:sessionID(id), ackNumb(ack) {}
};

struct rtoUpdate
{
	ap_uint<16> sessionID;
	ap_uint<32> rto; //in retransmit-timer ticks (1 tick = MAX_SESSIONS cycles)
	rtoUpdate() {}
	rtoUpdate(ap_uint<16> id, ap_uint<32> rto)
			:sessionID(id), rto(rto) {}
};

struct rxRetransmitTimerUpdate {
	ap_uint<16> sessionID;
	bool		stop;
	rxRetransmitTimerUpdate() {}
	rxRetransmitTimerUpdate(ap_uint<16> id)
				:sessionID(id), stop(0) {}
	rxRetransmitTimerUpdate(ap_uint<16> id, bool stop)
				:sessionID(id), stop(stop) {}
};

struct txRetransmitTimerSet {
	ap_uint<16> sessionID;
	eventType	type;
	txRetransmitTimerSet() {}
	txRetransmitTimerSet(ap_uint<16> id)
				:sessionID(id), type(RT) {} //FIXME??
	txRetransmitTimerSet(ap_uint<16> id, eventType type)
				:sessionID(id), type(type) {}
};

struct event
{
	eventType	type;
	ap_uint<16>	sessionID;
	ap_uint<WINDOW_BITS> address;
	ap_uint<16> length;
	ap_uint<3>	rt_count;
	event() {}
	//event(const event&) {}
	event(eventType type, ap_uint<16> id)
			:type(type), sessionID(id), address(0), length(0), rt_count(0) {}
	event(eventType type, ap_uint<16> id, ap_uint<3> rt_count)
			:type(type), sessionID(id), address(0), length(0), rt_count(rt_count) {}
	event(eventType type, ap_uint<16> id, ap_uint<WINDOW_BITS> addr, ap_uint<16> len)
			:type(type), sessionID(id), address(addr), length(len), rt_count(0) {}
	event(eventType type, ap_uint<16> id, ap_uint<WINDOW_BITS> addr, ap_uint<16> len, ap_uint<3> rt_count)
			:type(type), sessionID(id), address(addr), length(len), rt_count(rt_count) {}
};

struct extendedEvent : public event
{
	fourTuple	tuple;
	extendedEvent() {}
	extendedEvent(const event& ev)
			:event(ev.type, ev.sessionID, ev.address, ev.length, ev.rt_count) {}
	extendedEvent(const event& ev, fourTuple tuple)
			:event(ev.type, ev.sessionID, ev.address, ev.length, ev.rt_count), tuple(tuple) {}
};

struct rstEvent : public event
{
	rstEvent() {}
	rstEvent(const event& ev)
		:event(ev.type, ev.sessionID, ev.address, ev.length, ev.rt_count) {}
	rstEvent(ap_uint<32> seq)
			//:event(RST, 0, false), seq(seq) {}
			:event(RST, 0, seq(31, 16), seq(15, 0), 0) {}
	rstEvent(ap_uint<16> id, ap_uint<32> seq)
			:event(RST, id, seq(31, 16), seq(15, 0), 1) {}
			//:event(RST, id, true), seq(seq) {}
	rstEvent(ap_uint<16> id, ap_uint<32> seq, bool hasSessionID)
		:event(RST, id, seq(31, 16), seq(15, 0), hasSessionID) {}
		//:event(RST, id, hasSessionID), seq(seq) {}
	ap_uint<32> getAckNumb()
	{
		ap_uint<32> seq;
		seq(31, 16) = address;
		seq(15, 0) = length;
		return seq;
	}
	bool hasSessionID()
	{
		return (rt_count != 0);
	}
};

const uint32_t TCP_PSEUDO_HEADER_SIZE = 96;
const uint32_t TCP_FULL_PSEUDO_HEADER_SIZE = 256;
const uint32_t TCP_HEADER_SIZE = 160;
const uint32_t TCP_OPTIONAL_HEADER_SIZE = 320;


/**
 * [31:4] = source address;
 * [63:32] = destination address;
 * [71:64] = zeros;
 * [79:72] = protocol;
 * [95:80] = length;
 */
template <int W>
class tcpPseudoHeader : public packetHeader<W, TCP_PSEUDO_HEADER_SIZE> {
	using packetHeader<W, TCP_PSEUDO_HEADER_SIZE>::header;

public:
	tcpPseudoHeader()
	{
		header(71, 64) = 0x00; // zeros
		header(79, 72) = 0x06; // protocol
	}

	void setSrcAddr(const ap_uint<32>& addr)
	{
		header(31,0) = addr;
	}
	ap_uint<32> getSrcAddr()
	{
		return header(31,0);
	}
	void setDstAddr(const ap_uint<32>& addr)
	{
		header(63,32) = addr;
	}
	ap_uint<32> getDstAddr()
	{
		return header(63,32);
	}
	void setLength(const ap_uint<16> len)
	{
		header(95,80) = reverse(len);
	}
	ap_uint<16> getLength()
	{
		return reverse((ap_uint<16>)header(95,80));
	}
	void setProtocol(const ap_uint<8>& protocol)
	{
		header(79, 72) = protocol;
	}
	ap_uint<8> getProtocol()
	{
		return header(79, 72);
	}
};

/**
 * [31:0] = source address
 * [63:32] = destination address
 * [71:64] = zeros
 * [79:72] = protocol
 * [95:80] = length
 * [111:96] = source port
 * [127:112] = destination port
 * [159:128] = sequence number
 * [191:160] = acknowledgement number
 * [192] = NS flag
 * [195:193] = reserved
 * [199:196] = data offset
 * [200] = FIN flag
 * [201] = SYN flag
 * [202] = RST flag
 * [203] = PSH flag
 * [204] = ACK flag
 * [205] = URG flag
 * [206] = ECE flag
 * [207] = CWR flag
 * [223:208] = window
 * [239:224] = checksum
 * [255:240] = urgent pointer
 * [263:256] = option kind
 * [271:264] = option length
 * [287:272] = MSS
 */
template <int W>
class tcpFullPseudoHeader : public packetHeader<W, TCP_FULL_PSEUDO_HEADER_SIZE> {
	using packetHeader<W, TCP_FULL_PSEUDO_HEADER_SIZE>::header;

public:
	tcpFullPseudoHeader()
	{
		header(71, 64) = 0x00; // zeros
		header(79, 72) = 0x06; // protocol
		header[192] = 0; //NS bit
		header(195,193) = 0; // reserved
		header[203] = 0;//flag
		header(207,205) = 0; //flags
		header(255,240) = 0; //urgent pointer
	}

	void setSrcAddr(const ap_uint<32>& addr)
	{
		header(31,0) = addr;
	}
	ap_uint<32> getSrcAddr()
	{
		return header(31,0);
	}
	void setDstAddr(const ap_uint<32>& addr)
	{
		header(63,32) = addr;
	}
	ap_uint<32> getDstAddr()
	{
		return header(63,32);
	}
	void setProtocol(const ap_uint<8>& protocol)
	{
		header(79, 72) = protocol;
	}
	ap_uint<8> getProtocol()
	{
		return header(79, 72);
	}
	void setLength(const ap_uint<16> len)
	{
		header(95,80) = reverse(len);
	}
	ap_uint<16> getLength()
	{
		return reverse((ap_uint<16>)header(95,80));
	}
	void setSrcPort(const ap_uint<16>& port)
	{
		header(111,96) = port;
	}
	ap_uint<16> getSrcPort()
	{
		return header(111,96);
	}
	void setDstPort(const ap_uint<16>& port)
	{
		header(127,112) = port;
	}
	ap_uint<16> getDstPort()
	{
		return header(127,112);
	}
	void setSeqNumb(const ap_uint<32>& seq)
	{
		header(159,128) = reverse(seq);
	}
	ap_uint<32> getSeqNumb()
	{
		return reverse((ap_uint<32>)header(159,128));
	}
	void setAckNumb(const ap_uint<32>& ack)
	{
		header(191,160) = reverse(ack);
	}
	ap_uint<32> getAckNumb()
	{
		return reverse((ap_uint<32>)header(191,160));
	}
	void setDataOffset(ap_uint<4> offset)
	{
		header(199,196) = offset;
	}
	ap_uint<4> getDataOffset()
	{
		return header(199,196);
	}
	ap_uint<1> getAckFlag()
	{
		return header[204];
	}
	void setAckFlag(ap_uint<1> flag)
	{
		header[204] = flag;
	}
	ap_uint<1> getRstFlag()
	{
		return header[202];
	}
	void setRstFlag(ap_uint<1> flag)
	{
		header[202] = flag;
	}
	ap_uint<1> getSynFlag()
	{
		return header[201];
	}
	void setSynFlag(ap_uint<1> flag)
	{
		header[201] = flag;
	}
	ap_uint<1> getFinFlag()
	{
		return header[200];
	}
	void setFinFlag(ap_uint<1> flag)
	{
		header[200] = flag;
	}
	void setWindowSize(const ap_uint<16>& size)
	{
		header(223, 208) = reverse(size);
	}
	ap_uint<16> getWindowSize()
	{
		return reverse((ap_uint<16>)header(223,208));
	}
	void setChecksum(const ap_uint<16>& checksum)
	{
		header(239, 224) = checksum;
	}
	ap_uint<16> getChecksum()
	{
		return header(239,224);
	}
	//TODO how to use this
	/*void setMssOption(ap_uint<16> MSS)
	{
		header(263,256) = 0x02;
		header(271,264) = 0x04;
		header(287,272) = reverse(MSS);
	}*/
};


/**
 * [15:0] source port
 * [31:16] = destination port
 * [63:32] = sequence number
 * [95:64] = acknowledgement number
 * [96] = NS flag
 * [99:97] = reserved
 * [103:100] = data offset
 * [104] = FIN flag
 * [105] = SYN flag
 * [106] = RST flag
 * [107] = PSH flag
 * [108] = ACK flag
 * [109] = URG flag
 * [110] = ECE flag
 * [111] = CWR flag
 * [127:112] = window size
 * [143:128] = checksum
 * [159:144] = urgent pointer
 */
template <int W>
class tcpHeader : public packetHeader<W, TCP_HEADER_SIZE> {
	using packetHeader<W, TCP_HEADER_SIZE>::header;

public:
	tcpHeader()
	{
		//header(71, 64) = 0x00; // zeros
		//header(79, 72) = 0x06; // protocol
		header(99,97) = 0; //reserved
	}

	void setSrcPort(const ap_uint<16>& port)
	{
		header(15,0) = port;
	}
	ap_uint<16> getSrcPort()
	{
		return header(15,0);
	}
	void setDstPort(const ap_uint<16>& port)
	{
		header(31,16) = port;
	}
	ap_uint<16> getDstPort()
	{
		return header(31,16);
	}
	void setSeqNumb(const ap_uint<32>& seq)
	{
		header(63,32) = reverse(seq);
	}
	ap_uint<32> getSeqNumb()
	{
		return reverse((ap_uint<32>)header(63,32));
	}
	void setAckNumb(const ap_uint<32>& ack)
	{
		header(95,64) = reverse(ack);
	}
	ap_uint<32> getAckNumb()
	{
		return reverse((ap_uint<32>)header(95,64));
	}
	ap_uint<4> getDataOffset()
	{
		return header(103,100);
	}
	ap_uint<1> getAckFlag()
	{
		return header[108];
	}
	ap_uint<1> getRstFlag()
	{
		return header[106];
	}
	ap_uint<1> getSynFlag()
	{
		return header[105];
	}
	ap_uint<1> getFinFlag()
	{
		return header[104];
	}
	void setWindowSize(const ap_uint<16>& size)
	{
		header(127, 112) = reverse(size);
	}
	ap_uint<16> getWindowSize()
	{
		return reverse((ap_uint<16>)header(127,112));
	}
	void setChecksum(const ap_uint<16>& checksum)
	{
		header(143, 128) = checksum;
	}
	ap_uint<16> getChecksum()
	{
		return header(143,128);
	}

};

template <int W>
class tcpOptionalHeader : public packetHeader<W, TCP_OPTIONAL_HEADER_SIZE> {
	using packetHeader<W, TCP_OPTIONAL_HEADER_SIZE>::header;

public:
	tcpOptionalHeader() {}
};