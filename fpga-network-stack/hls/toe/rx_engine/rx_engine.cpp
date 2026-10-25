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

#include "toe_config.hpp"
#include "rx_engine.hpp"
#include "../../ipv4/ipv4.hpp"
#include "../two_complement_subchecksums.hpp"


//parse IP header, and remove it
template <int WIDTH>
void toe_process_ipv4(	stream<net_axis<WIDTH> >&		dataIn,
					stream<ap_uint<4> >&	process2dropLengthFifo,
					stream<pseudoMeta>&		metaOut,
					stream<ap_uint<1> >&	ceOut,
					stream<net_axis<WIDTH> >&		dataOut)
{
#pragma HLS INLINE off
#pragma HLS pipeline II=1

	static ipv4Header<WIDTH> header;
	static ap_uint<4> headerWordsDropped = 0;
	static bool metaWritten = false;

	if (!dataIn.empty())
	{
		net_axis<WIDTH> currWord = dataIn.read();
		// std::cout << "IP process: ";
		// printLE(std::cout, currWord);
		// std::cout << std::endl;
		header.parseWord(currWord.data);

		if (header.isReady())
		{
			dataOut.write(currWord);
			if (!metaWritten)
			{
				#ifndef __SYNTHESIS__
				std::cout << std::dec << "RX length: " << header.getLength() << ", header length: " << header.getHeaderLength() << std::endl;
				#endif
				process2dropLengthFifo.write(header.getHeaderLength() - headerWordsDropped);
				metaOut.write(pseudoMeta(header.getSrcAddr(), header.getDstAddr(), header.getLength() - (header.getHeaderLength()*4)));
				// ECN (RFC 3168): Congestion Experienced codepoint, one entry
				// per packet for processPseudoHeader
				ceOut.write(header.getECN() == 3);
				metaWritten = true;
			}
		}
		headerWordsDropped += (WIDTH/32);
		if (currWord.last)
		{
			metaWritten = false;
			header.clear();
			headerWordsDropped = 0;
		}
	}
}

template <int WIDTH>
void constructPseudoHeader(	hls::stream<pseudoMeta>&	ipMetaIn,
							hls::stream<net_axis<WIDTH> >&	headerOut)
{
	#pragma HLS PIPELINE II=1
	#pragma HLS INLINE off

	enum fsmState{META, HEADER};
	static fsmState state = META;
	static tcpPseudoHeader<WIDTH> header; //size 96bits

	switch (state)
	{
	case META:
		if (!ipMetaIn.empty())
		{
			pseudoMeta meta = ipMetaIn.read();

			header.clear();
			header.setSrcAddr(meta.their_address);
			header.setDstAddr(meta.our_address);
			header.setLength(meta.length);
			state = HEADER;
		}
		break;
	case HEADER:
		net_axis<WIDTH> sendWord;
		ap_uint<8> remainingLength = header.consumeWord(sendWord.data);
		sendWord.keep = ~0;
		sendWord.last = (remainingLength == 0);
		headerOut.write(sendWord);

		if (sendWord.last)
		{
			state = META;
		}
		break;
	}//swtich
}

template <int WIDTH>
void prependPseudoHeader(	hls::stream<net_axis<WIDTH> >&		headerIn,
							hls::stream<net_axis<WIDTH> >&		packetIn,
							hls::stream<net_axis<WIDTH> >&		output)
{
	#pragma HLS PIPELINE II=1
	#pragma HLS INLINE off

	enum fsmState{HEADER, PAYLOAD};
	static fsmState state = HEADER;
	static net_axis<WIDTH> prevWord;
	static bool firstPayload = true;

	switch (state)
	{
	case HEADER:
		if (!headerIn.empty())
		{
			headerIn.read(prevWord);

			if (!prevWord.last)
			{
				output.write(prevWord);
			}
			else
			{
				prevWord.last = 0;
				firstPayload = true;
				state = PAYLOAD;
			}

		}
		break;
	case PAYLOAD:
		if (!packetIn.empty())
		{
			net_axis<WIDTH> currWord = packetIn.read();

			net_axis<WIDTH> sendWord = currWord;
			if (firstPayload)
			{
				if (WIDTH == 64)
				{
					sendWord.data(31, 0) = prevWord.data(31,0);
				}
				if (WIDTH > 64)
				{
					sendWord.data(95, 0) = prevWord.data(95,0);
				}
				firstPayload = false;
			}

			output.write(sendWord);

			if (currWord.last)
			{
				// std::cout << "PREPEND" << std::endl;
				state = HEADER;
			}
		}
		break;
	}
}

/** @ingroup rx_engine
 *  Checks the TCP checksum writes valid into @p validBuffer
 *  Additionally it extracts some metadata and the IP tuples from
 *  the TCP packet and writes it to @p metaDataFifoOut
 *  and @p tupleFifoOut
 *  It also sends the destination port number to the @ref port_table
 *  to check if the port is open.
 *  @param[in]		dataIn
 *  @param[out]		dataOut
 *  @param[out]		validFifoOut
 *  @param[out]		metaDataFifoOut
 *  @param[out]		tupleFifoOut
 *  @param[out]		portTableOut
 */

void rxCheckTCPchecksum(stream<net_axis<64> >&					dataIn,
							stream<net_axis<64> >&				dataOut,
							stream<bool>&					validFifoOut,
							stream<rxEngineMetaData>&		metaDataFifoOut,
							stream<fourTuple>&				tupleFifoOut,
							stream<ap_uint<16> >&			portTableOut)
{
#pragma HLS INLINE off
#pragma HLS pipeline II=1

	static ap_uint<17> csa_tcp_sums[4] = {0, 0, 0, 0};
	static ap_uint<8> csa_dataOffset = 0xFF;
	static ap_uint<16> csa_wordCount = 0;
	static fourTuple csa_sessionTuple;
	static bool csa_shift = false;
	static bool csa_wasLast = false;
	static bool csa_checkChecksum = false;
	static ap_uint<36> halfWord; 
	net_axis<64> currWord, sendWord;
	static rxEngineMetaData csa_meta;
	static ap_uint<16> csa_port;

	static ap_uint<3> csa_cc_state = 0;

	//currWord.last = 0; //mighnt no be necessary any more FIXME to you want to risk it ;)
	if (!dataIn.empty() && !csa_checkChecksum)
	{
		dataIn.read(currWord);
		switch (csa_wordCount)
		{
		case 0:
			csa_dataOffset = 0xFF;
			csa_shift = false;
				// We don't switch bytes, internally we store it Most Significant Byte Last
				csa_sessionTuple.srcIp = currWord.data(31, 0);
				csa_sessionTuple.dstIp = currWord.data(63, 32);
				sendWord.last = currWord.last;

			break;
		case 1:
			// Get length
			csa_meta.length(7, 0) = currWord.data(31, 24);
			csa_meta.length(15, 8) = currWord.data(23, 16);
			// We don't switch bytes, internally we store it Most Significant Byte Last
			csa_sessionTuple.srcPort = currWord.data(47, 32);
			csa_sessionTuple.dstPort = currWord.data(63, 48);
			csa_port = currWord.data(63, 48);
			sendWord.last = currWord.last;
			break;
		case 2:
			// GET SEQ and ACK number
			csa_meta.seqNumb(7, 0) = currWord.data(31, 24);
			csa_meta.seqNumb(15, 8) = currWord.data(23, 16);
			csa_meta.seqNumb(23, 16) = currWord.data(15, 8);
			csa_meta.seqNumb(31, 24) = currWord.data(7, 0);
			csa_meta.ackNumb(7, 0) = currWord.data(63, 56);
			csa_meta.ackNumb(15, 8) = currWord.data(55, 48);
			csa_meta.ackNumb(23, 16) = currWord.data(47, 40);
			csa_meta.ackNumb(31, 24) = currWord.data(39, 32);
			sendWord.last = currWord.last;
			break;
		case 3:
			csa_dataOffset = currWord.data.range(7, 4);
			csa_meta.length -= (csa_dataOffset * 4);
			//csa_dataOffset -= 5; //FIXME, do -5
			/* Control bits:
			 * [8] == FIN
			 * [9] == SYN
			 * [10] == RST
			 * [11] == PSH
			 * [12] == ACK
			 * [13] == URG
			 */
			csa_meta.ack = currWord.data[12];
			csa_meta.rst = currWord.data[10];
			csa_meta.syn = currWord.data[9];
			csa_meta.fin = currWord.data[8];
			csa_meta.winSize(7, 0) = currWord.data(31, 24);
			csa_meta.winSize(15, 8) = currWord.data(23, 16);
			// We add checksum as well and check for cs == 0
			sendWord.last = currWord.last;
			break;
		default:
			if (csa_dataOffset > 6)
			{
				csa_dataOffset -= 2;
			}
			else if (csa_dataOffset == 6)
			{
				csa_dataOffset = 5;
				csa_shift = true;
				halfWord.range(31, 0) = currWord.data.range(63, 32);
				halfWord.range(35, 32) = currWord.keep.range(7, 4);
				sendWord.last = (currWord.keep[4] == 0);
			}
			else // == 5 (or less)
			{
				if (!csa_shift)
				{
					sendWord = currWord;
					dataOut.write(sendWord);
				}
				else
				{
					sendWord.data.range(31, 0) = halfWord.range(31, 0);
					sendWord.data.range(63, 32) = currWord.data.range(31, 0);
					sendWord.keep.range(3, 0) = halfWord.range(35, 32);
					sendWord.keep.range(7, 4) = currWord.keep.range(3, 0);
					sendWord.last = (currWord.keep[4] == 0);
					/*if (currWord.last && currWord.strb.range(7, 4) != 0)
					{
						sendWord.last = 0;
					}*/
					dataOut.write(sendWord);
					halfWord.range(31, 0) = currWord.data.range(63, 32);
					halfWord.range(35, 32) = currWord.keep.range(7, 4);
				}
			}
			break;
		} // switch
		for (int i = 0; i < 4; i++)
		{
#pragma HLS UNROLL
			ap_uint<16> temp;
			if (currWord.keep.range(i*2+1, i*2) == 0x3)
			{
				temp(7, 0) = currWord.data.range(i*16+15, i*16+8);
				temp(15, 8) = currWord.data.range(i*16+7, i*16);
				csa_tcp_sums[i] += temp;
				csa_tcp_sums[i] = (csa_tcp_sums[i] + (csa_tcp_sums[i] >> 16)) & 0xFFFF;
			}
			else if (currWord.keep[i*2] == 0x1)
			{
				temp(7, 0) = 0;
				temp(15, 8) = currWord.data.range(i*16+7, i*16);
				csa_tcp_sums[i] += temp;
				csa_tcp_sums[i] = (csa_tcp_sums[i] + (csa_tcp_sums[i] >> 16)) & 0xFFFF;
			}
		}
		csa_wordCount++;
		if(currWord.last == 1)
		{
			csa_wordCount = 0;
			csa_wasLast = !sendWord.last; // moved length test down
			csa_checkChecksum = true;
		}
	}
	/*if (currWord.last == 1)
	{
		csa_wordCount = 0;
		csa_checkChecksum = true;
	}*/
	else if(csa_wasLast) //make if
	{
		if (csa_meta.length != 0)
		{
			sendWord.data.range(31, 0) = halfWord.range(31, 0);
			sendWord.data.range(63, 32) = 0;
			sendWord.keep.range(3, 0) = halfWord.range(35, 32);
			sendWord.keep.range(7, 4) = 0;
			sendWord.last = 1;
			dataOut.write(sendWord);
		}
		csa_wasLast = false;
	}
	else if (csa_checkChecksum) //make if?
	{
		switch (csa_cc_state)
		{
		case 0:
			csa_tcp_sums[0] = (csa_tcp_sums[0] + (csa_tcp_sums[0] >> 16)) & 0xFFFF;
			csa_tcp_sums[1] = (csa_tcp_sums[1] + (csa_tcp_sums[1] >> 16)) & 0xFFFF;
			csa_tcp_sums[2] = (csa_tcp_sums[2] + (csa_tcp_sums[2] >> 16)) & 0xFFFF;
			csa_tcp_sums[3] = (csa_tcp_sums[3] + (csa_tcp_sums[3] >> 16)) & 0xFFFF;
			csa_cc_state++;
			break;
		case 1:
			csa_tcp_sums[0] += csa_tcp_sums[2];
			csa_tcp_sums[1] += csa_tcp_sums[3];
			csa_tcp_sums[0] = (csa_tcp_sums[0] + (csa_tcp_sums[0] >> 16)) & 0xFFFF;
			csa_tcp_sums[1] = (csa_tcp_sums[1] + (csa_tcp_sums[1] >> 16)) & 0xFFFF;
			csa_cc_state++;
			break;
		case 2:
			csa_tcp_sums[0] += csa_tcp_sums[1];
			csa_tcp_sums[0] = (csa_tcp_sums[0] + (csa_tcp_sums[0] >> 16)) & 0xFFFF;
			csa_cc_state++;
			break;
		case 3:
			csa_tcp_sums[0] = ~csa_tcp_sums[0];
			csa_cc_state++;
			break;
		case 4:
			// If summation == 0 then checksum is correct
			if (csa_tcp_sums[0](15, 0) == 0)
			{
				// Since pkg is valid, write out metadata, 4-tuple and check port
				metaDataFifoOut.write(csa_meta);
				portTableOut.write(csa_port);
				tupleFifoOut.write(csa_sessionTuple);
				if (csa_meta.length != 0)
				{
					validFifoOut.write(true);
					#ifndef __SYNTHESIS__
					std::cout<<"checksum correct"<<std::endl;
					#endif
				}
			}
			else if(csa_meta.length != 0)
			{
				validFifoOut.write(false);
				#ifndef __SYNTHESIS__
				std::cout<<"checksum not correct!!!"<<std::endl;
				#endif
			}
			csa_checkChecksum = false;
			csa_tcp_sums[0] = 0;
			csa_tcp_sums[1] = 0;
			csa_tcp_sums[2] = 0;
			csa_tcp_sums[3] = 0;
			csa_cc_state = 0;
			break;
		}

	}
}

template <int WIDTH>
void processPseudoHeader(stream<net_axis<WIDTH> >&					dataIn,
							stream<net_axis<WIDTH> >&				dataOut,
							stream<bool>&					validFifoIn,
							stream<ap_uint<1> >&			ceFifoIn,
							stream<rxEngineMetaData>&		metaDataFifoOut,
							stream<fourTuple>&				tupleFifoOut,
							stream<ap_uint<16> >&			portTableOut,
							stream<optionalFieldsMeta>&	ofMetaOut)
{
	#pragma HLS PIPELINE II=1
	#pragma HLS INLINE off

	static tcpFullPseudoHeader<WIDTH>	header; //width: 256bit
	static bool firstWord = true;
	static bool pkgValid = false;
	static bool metaWritten = false;
	static ap_uint<1> pkgCe = 0;

	if (!dataIn.empty() && (!firstWord || (!validFifoIn.empty() && !ceFifoIn.empty())))
	{
		net_axis<WIDTH> word = dataIn.read();
		// std::cout << "PROCESS TCP: ";
		// printLE(std::cout, word);
		// std::cout << std::endl;
		header.parseWord(word.data);
		if (firstWord)
		{
			validFifoIn.read(pkgValid);
			ceFifoIn.read(pkgCe);
			firstWord = false;
		}

		if (metaWritten && pkgValid && WIDTH <= TCP_FULL_PSEUDO_HEADER_SIZE)
		{
			dataOut.write(word);
		}

		if (header.isReady())
		{
			if (pkgValid && WIDTH > TCP_FULL_PSEUDO_HEADER_SIZE && ((header.getLength() - (header.getDataOffset() * 4) != 0 || (header.getDataOffset() > 5))))// && WIDTH == 512)
			{
				dataOut.write(word);
			}
			if (!metaWritten)
			{
				rxEngineMetaData meta;
				meta.seqNumb = header.getSeqNumb();
				meta.ackNumb = header.getAckNumb();
				meta.winSize = header.getWindowSize();
				meta.length = header.getLength() - (header.getDataOffset() * 4);
				meta.ack = header.getAckFlag();
				meta.rst = header.getRstFlag();
				meta.syn = header.getSynFlag();
				meta.fin = header.getFinFlag();
				meta.ece = header.getEceFlag();
				meta.cwr = header.getCwrFlag();
				meta.ce = pkgCe;
				meta.dataOffset = header.getDataOffset();
				if (pkgValid)
				{
					metaDataFifoOut.write(meta);
					portTableOut.write(header.getDstPort());
					tupleFifoOut.write(fourTuple(header.getSrcAddr(), header.getDstAddr(), header.getSrcPort(), header.getDstPort()));
					if (meta.length != 0 || header.getDataOffset() > 5)
					{
						ofMetaOut.write(optionalFieldsMeta(header.getDataOffset() - 5, header.getSynFlag()));
					}
				}
				if (meta.length != 0)
				{
					// std::cout << "VALID WRITE: " << std::dec << meta.length << ", " << header.getLength() << ", " << header.getDataOffset() << std::endl;
				}

				metaWritten = true;
			}
		}

		if (word.last)
		{
			header.clear();
			firstWord = true;
			metaWritten = false;
		}
	}

}

//data offset specifies the number of 32bit words
//the header is between 20 (5 words) and 60 (15 words). This means the optional fields are between 0 and 10 words (0 and 40 bytes)
template <int WIDTH>
void drop_optional_header_fields(	hls::stream<optionalFieldsMeta>&		metaIn,
									hls::stream<net_axis<WIDTH> >&	dataIn,
#if (WINDOW_SCALE)
									hls::stream<ap_uint<4> >&			dataOffsetOut,
									hls::stream<ap_uint<320> >&		optionalHeaderFieldsOut,
#endif
									hls::stream<net_axis<WIDTH> >&	dataOut)
{
	#pragma HLS PIPELINE II=1
	#pragma HLS INLINE off

	static ap_uint<2> state = 0;
	static ap_uint<4> dataOffset = 0;
	static net_axis<WIDTH> prevWord;
	static tcpOptionalHeader<WIDTH> optionalHeader;
	static bool parseHeader = false;
	static bool headerWritten = false;

	switch (state)
	{
	case 0:
		if (!metaIn.empty() && !dataIn.empty())
		{
			optionalHeader.clear();
			optionalFieldsMeta meta =  metaIn.read();
			net_axis<WIDTH> currWord = dataIn.read();
			dataOffset = meta.dataOffset;// - 5;
			// std::cout << "OFFSET: " << dataOffset << std::endl;
			// std::cout << "DROP OO: ";
			// printLE(std::cout, currWord);
			// std::cout << std::endl;

			optionalHeader.parseWord(currWord.data);
			parseHeader = false;
			if (dataOffset == 0)
			{
				dataOut.write(currWord);
			}
			else
			{
				//TODO only works for 512
#if (WINDOW_SCALE)
				//parse options on every segment that carries them (SYN for
				//MSS/WS/SACK-Permitted, ACKs for SACK blocks)
				parseHeader = true;
				dataOffsetOut.write(dataOffset);
				if (optionalHeader.isReady() || currWord.last)
				{
					optionalHeaderFieldsOut.write(optionalHeader.getRawHeader());
				}
#endif
			}

			state = 1;
			prevWord = currWord;
			headerWritten = false;
			if (currWord.last)
			{
				state = 0;
				if (dataOffset != 0 && (dataOffset*4 < WIDTH/8) && currWord.keep[dataOffset*4] != 0)
				{
					state = 2;
				}
			}
		}
		break;
	case 1:
		if (!dataIn.empty())
		{
			net_axis<WIDTH> currWord = dataIn.read();
			net_axis<WIDTH> sendWord;
			sendWord.last = 0;

#if (WINDOW_SCALE)
			optionalHeader.parseWord(currWord.data);
			if ((optionalHeader.isReady() || currWord.last) && parseHeader && !headerWritten)
			{
				optionalHeaderFieldsOut.write(optionalHeader.getRawHeader());
				headerWritten = true;
			}
#endif
			if (dataOffset >= WIDTH/32)
			{
				dataOffset -= WIDTH/32;
			}
			else if (dataOffset == 0)
			{
				sendWord = currWord;
				dataOut.write(sendWord);
			}
			else //if (dataOffset == 1)
			{
				sendWord.data(WIDTH - (dataOffset*32) -1, 0) = prevWord.data(WIDTH-1, dataOffset*32);
				sendWord.keep((WIDTH/8) - (dataOffset*4) -1, 0) = prevWord.keep(WIDTH/8-1, dataOffset*4);
				sendWord.data(WIDTH-1, WIDTH - (dataOffset*32)) = currWord.data(dataOffset*32-1, 0);
				sendWord.keep(WIDTH/8-1, (WIDTH/8) - (dataOffset*4)) = currWord.keep(dataOffset*4-1, 0);
				sendWord.last = (currWord.keep[dataOffset*4] == 0);
				dataOut.write(sendWord);
			}

			prevWord = currWord;
			if (currWord.last)
			{
				state = 0;
				if (currWord.keep[dataOffset*4] != 0 && dataOffset != 0)
				{
					state = 2;
				}
			}
		}
		break;
	case 2:
	{
		net_axis<WIDTH> sendWord;
		sendWord.data(WIDTH - (dataOffset*32) -1, 0) = prevWord.data(WIDTH-1, dataOffset*32);
		sendWord.keep((WIDTH/8) - (dataOffset*4) -1, 0) = prevWord.keep(WIDTH/8-1, dataOffset*4);
		sendWord.data(WIDTH-1, WIDTH - (dataOffset*32)) = 0;
		sendWord.keep(WIDTH/8-1, (WIDTH/8) - (dataOffset*4)) = 0;
		sendWord.last = 1;
		dataOut.write(sendWord);
		state = 0;
		break;
	}
	}//switch

}

/**
 * Optional Header Fields are only parse on connection establishment to determine
 * the window scaling factor
 * Available options (during SYN):
 * kind | length | description
 *   0  |     1B | End of options list
 *   1  |     1B | NOP/Padding
 *   2  |     4B | MSS (Maximum segment size)
 *   3  |     3B | Window scale
 *   4  |     2B | SACK permitted (Selective Acknowledgment)
 */
// Result is packed as {sack2End[215:184], sack2Start[183:152],
// sack2Valid[151], tsEcr[150:119], tsVal[118:87], tsValid[86],
// sackEnd[85:54], sackStart[53:22], sackValid[21], sackPermitted[20],
// mss[19:4], windowScale[3:0]}. MSS defaults to 536 (RFC 1122) when the
// peer's SYN carries no MSS option.
void parse_optional_header_fields(	hls::stream<ap_uint<4> >&		dataOffsetIn,
												hls::stream<ap_uint<320> >&	optionalHeaderFieldsIn,
												hls::stream<ap_uint<216> >&		parsedOptionsOut)
{
	#pragma HLS PIPELINE II=1
	#pragma HLS INLINE off

	enum fsmStateType {IDLE, PARSE};
	static fsmStateType state = IDLE;
	// remaining option bytes: dataOffsetIn carries (dataOffset-5) in 32bit
	// words (0..10), option space is therefore 0..40 bytes (issue #43: the
	// old code compared the word count against byte lengths directly)
	static ap_uint<6> remainingBytes;
	static ap_uint<320> fields;
	static ap_uint<4> wsVal;
	static ap_uint<16> mssVal;
	static ap_uint<1> sackPermVal;
	static ap_uint<1> sackVal;
	static ap_uint<32> sackStartVal;
	static ap_uint<32> sackEndVal;
	static ap_uint<1> tsValidVal;
	static ap_uint<32> tsValVal;
	static ap_uint<32> tsEcrVal;
	static ap_uint<1> sack2Val;
	static ap_uint<32> sack2StartVal;
	static ap_uint<32> sack2EndVal;

	switch (state)
	{
	case IDLE:
		if (!dataOffsetIn.empty() && !optionalHeaderFieldsIn.empty())
		{
			ap_uint<4> dataOffset;
			dataOffsetIn.read(dataOffset);
			optionalHeaderFieldsIn.read(fields);
			remainingBytes = ((ap_uint<6>)dataOffset) * 4;
			wsVal = 0;
			mssVal = 536;
			sackPermVal = 0;
			sackVal = 0;
			sackStartVal = 0;
			sackEndVal = 0;
			tsValidVal = 0;
			tsValVal = 0;
			tsEcrVal = 0;
			sack2Val = 0;
			sack2StartVal = 0;
			sack2EndVal = 0;
			state = PARSE;
		}
		break;
	case PARSE:
	{
		ap_uint<8> optionKind = fields(7, 0);
		ap_uint<8> optionLength = fields(15, 8);
		bool done = false;

		if (remainingBytes == 0)
		{
			done = true; //walked the whole option list
		}
		else
		{
			switch (optionKind)
			{
			case 0: //End of option list
				done = true;
				break;
			case 1: //NOP
				optionLength = 1;
				break;
			case 2: //MSS (value is big-endian on the wire)
				if (optionLength == 4 && remainingBytes >= 4)
				{
					mssVal = (fields(23, 16), fields(31, 24));
				}
				else
				{
					done = true; //malformed MSS option, stop parsing
				}
				break;
			case 5: //SACK blocks (we use the first two)
				if (optionLength >= 10 && optionLength <= remainingBytes && (optionLength & 0x7) == 2)
				{
					sackVal = 1;
					//values are big-endian on the wire, fields is byte0-at-LSB
					sackStartVal = (fields(23, 16), fields(31, 24), fields(39, 32), fields(47, 40));
					sackEndVal = (fields(55, 48), fields(63, 56), fields(71, 64), fields(79, 72));
					if (optionLength >= 18)
					{
						sack2Val = 1;
						sack2StartVal = (fields(87, 80), fields(95, 88), fields(103, 96), fields(111, 104));
						sack2EndVal = (fields(119, 112), fields(127, 120), fields(135, 128), fields(143, 136));
					}
				}
				else
				{
					done = true; //malformed SACK option, stop parsing
				}
				break;
			case 8: //Timestamps (RFC 7323), TSval + TSecr big-endian on the wire
				if (optionLength == 10 && remainingBytes >= 10)
				{
					tsValidVal = 1;
					tsValVal = (fields(23, 16), fields(31, 24), fields(39, 32), fields(47, 40));
					tsEcrVal = (fields(55, 48), fields(63, 56), fields(71, 64), fields(79, 72));
				}
				else
				{
					done = true; //malformed timestamps option, stop parsing
				}
				break;
			case 4: //SACK permitted
				if (optionLength == 2 && remainingBytes >= 2)
				{
					sackPermVal = 1;
				}
				else
				{
					done = true; //malformed SACK-permitted option, stop parsing
				}
				break;
			case 3: //Window scale
				if (optionLength == 3 && remainingBytes >= 3)
				{
					// RFC 7323: shift counts larger than 14 must be clamped
					// to 14 (issue #42: unclamped values overflow the
					// scaled-window bit range in tx_sar_table)
					ap_uint<4> ws = fields(19, 16);
					wsVal = (ws > 14) ? (ap_uint<4>)14 : ws;
				}
				else
				{
					done = true; //malformed WS option, stop parsing
				}
				break;
			default:
				// unknown option: a valid length is >= 2 and must fit into
				// the remaining option space; anything else is malformed and
				// would previously spin this FSM forever (len 0 -> shift by 0)
				if (optionLength < 2 || optionLength > remainingBytes)
				{
					done = true;
				}
				break;
			}//switch
		}

		if (done)
		{
			parsedOptionsOut.write((sack2EndVal, sack2StartVal, sack2Val, tsEcrVal, tsValVal, tsValidVal, sackEndVal, sackStartVal, sackVal, sackPermVal, mssVal, wsVal));
			state = IDLE;
		}
		else
		{
			remainingBytes -= optionLength;
			fields = (fields >> (optionLength*8));
		}
		break;
	}
	}//switch
}

void merge_header_meta(hls::stream<ap_uint<216> >&			rxEng_winScaleFifo,
								hls::stream<rxEngineMetaData>&	rxEng_headerMetaFifo,
								hls::stream<rxEngineMetaData>& 	rxEng_metaDataFifo)
{
	#pragma HLS PIPELINE II=1
	#pragma HLS INLINE off

	static ap_uint<1> state = 0;
	static rxEngineMetaData meta;

	switch (state)
	{
	case 0:
		if (!rxEng_headerMetaFifo.empty())
		{
			// std::cout << "META MERGE 0" << std::endl;
			rxEng_headerMetaFifo.read(meta);
			if (meta.dataOffset > 5)
			{
				state = 1;
			}
			else
			{
				meta.winScale = 0;
				meta.mss = 536; //RFC 1122 default when no MSS option present
				meta.sackPermitted = 0;
				meta.sackValid = 0;
				meta.sackValid2 = 0;
				meta.tsValid = 0;
				rxEng_metaDataFifo.write(meta);
			}
		}
		break;
	case 1:
		if (!rxEng_winScaleFifo.empty())
		{
			// std::cout << "META MERGE 1" << std::endl;
			ap_uint<216> parsedOptions = rxEng_winScaleFifo.read();
			meta.winScale = parsedOptions(3, 0);
			meta.mss = parsedOptions(19, 4);
			meta.sackPermitted = parsedOptions[20];
			meta.sackValid = parsedOptions[21];
			meta.sackStart = parsedOptions(53, 22);
			meta.sackEnd = parsedOptions(85, 54);
			meta.tsValid = parsedOptions[86];
			meta.tsVal = parsedOptions(118, 87);
			meta.tsEcr = parsedOptions(150, 119);
			meta.sackValid2 = parsedOptions[151];
			meta.sackStart2 = parsedOptions(183, 152);
			meta.sackEnd2 = parsedOptions(215, 184);
			rxEng_metaDataFifo.write(meta);
			state = 0;
		}
		break;
	}//switch
}



/** @ingroup rx_engine
 *  For each packet it reads the valid value from @param validFifoIn
 *  If the packet is valid the data stream is passed on
 *  If it is not valid it is dropped
 *  @param[in]		dataIn, incoming data stream
 *  @param[in]		validFifoIn, Valid FIFO indicating if current packet is valid
 *  @param[out]		dataOut, outgoing data stream
 */
//TODO remove
/*template <int WIDTH>
void rxTcpInvalidDropper(stream<net_axis<WIDTH> >&				dataIn,
							stream<bool>&				validFifoIn,
							stream<net_axis<WIDTH> >&			dataOut)
{
#pragma HLS INLINE off
#pragma HLS pipeline II=1

	enum rtid_StateType {GET_VALID, FWD, DROP};
	static rtid_StateType rtid_state = GET_VALID;

	net_axis<WIDTH> currWord;
	bool valid;


	switch (rtid_state) {
	case GET_VALID: //Drop1
		if (!validFifoIn.empty())
		{
			#ifndef __SYNTHESIS__
			std::cout << "INVALID IN" << std::endl;
			#endif
			validFifoIn.read(valid);
			if (valid)
			{
				rtid_state = FWD;
			}
			else
			{
				rtid_state = DROP;
			}
		}
		break;
	case FWD:
		if(!dataIn.empty())
		{
			dataIn.read(currWord);
			dataOut.write(currWord);
			if (currWord.last)
			{
				#ifndef __SYNTHESIS__
				std::cout << "INVALID 0" << std::endl;
				#endif
				rtid_state = GET_VALID;
			}
		}
		break;
	case DROP:
		if(!dataIn.empty())
		{
			dataIn.read(currWord);
			if (currWord.last)
			{
				#ifndef __SYNTHESIS__
				std::cout << "INVALID 1" << std::endl;
				#endif
				rtid_state = GET_VALID;
			}
		}
		break;
	} // switch
}*/




/** @ingroup rx_engine
 * The module contains 2 state machines nested into each other. The outer state machine
 * loads the metadata and does the session lookup. The inner state machin then evaluates all
 * this data. This inner state machine mostly represents the TCP state machine and contains
 * all the logic how to update the metadata, what events are triggered and so on. It is the key
 * part of the @ref rx_engine.
 * @param[in]	metaDataFifoIn
 * @param[in]	sLookup2rxEng_rsp
 * @param[in]	stateTable2rxEng_upd_rsp
 * @param[in]	portTable2rxEng_rsp
 * @param[in]	tupleBufferIn
 * @param[in]	rxSar2rxEng_upd_rsp
 * @param[in]	txSar2rxEng_upd_rsp
 * @param[out]	rxEng2sLookup_req
 * @param[out]	rxEng2stateTable_req
 * @param[out]	rxEng2rxSar_upd_req
 * @param[out]	rxEng2txSar_upd_req
 * @param[out]	rxEng2timer_clearRetransmitTimer
 * @param[out]	rxEng2timer_setCloseTimer
 * @param[out]	openConStatusOut
 * @param[out]	rxEng2eventEng_setEvent
 * @param[out]	dropDataFifoOut
 * @param[out]	rxBufferWriteCmd
 * @param[out]	rxEng2rxApp_notification
 */
void rxMetadataHandler(	stream<rxEngineMetaData>&				metaDataFifoIn,
						stream<sessionLookupReply>&				sLookup2rxEng_rsp,
						stream<bool>&							portTable2rxEng_rsp,
						stream<fourTuple>&						tupleBufferIn,
						stream<sessionLookupQuery>&				rxEng2sLookup_req,
						stream<extendedEvent>&					rxEng2eventEng_setEvent,
						stream<bool>&							dropDataFifoOut,
						stream<rxFsmMetaData>&					fsmMetaDataFifo)
{
#pragma HLS INLINE off
#pragma HLS pipeline II=1

	static rxEngineMetaData mh_meta;
	static sessionLookupReply mh_lup;
	enum mhStateType {META, LOOKUP};
	static mhStateType mh_state = META;
	static ap_uint<32> mh_srcIpAddress;
	static ap_uint<16> mh_dstIpPort;
	static ap_uint<16> mh_srcIpPort;

	fourTuple tuple;
	bool portIsOpen;

	switch (mh_state)
	{
	case META:
		if (!metaDataFifoIn.empty() && !portTable2rxEng_rsp.empty() && !tupleBufferIn.empty())
		{
			metaDataFifoIn.read(mh_meta);
			portTable2rxEng_rsp.read(portIsOpen);
			tupleBufferIn.read(tuple);
			mh_srcIpAddress(7, 0) = tuple.srcIp(31, 24);
			mh_srcIpAddress(15, 8) = tuple.srcIp(23, 16);
			mh_srcIpAddress(23, 16) = tuple.srcIp(15, 8);
			mh_srcIpAddress(31, 24) = tuple.srcIp(7, 0);
			mh_dstIpPort(7, 0) = tuple.dstPort(15, 8);
			mh_dstIpPort(15, 8) = tuple.dstPort(7, 0);
			mh_srcIpPort(7,0) = tuple.srcPort(15,8);
			mh_srcIpPort(15,8) = tuple.srcPort(7,0);
			// CHeck if port is closed
			if (!portIsOpen)
			{
				// SEND RST+ACK
				if (!mh_meta.rst)
				{
					// send necesssary tuple through event
					fourTuple switchedTuple;
					switchedTuple.srcIp = tuple.dstIp;
					switchedTuple.dstIp = tuple.srcIp;
					switchedTuple.srcPort = tuple.dstPort;
					switchedTuple.dstPort = tuple.srcPort;
					if (mh_meta.syn || mh_meta.fin)
					{
						rxEng2eventEng_setEvent.write(extendedEvent(rstEvent(mh_meta.seqNumb+mh_meta.length+1), switchedTuple)); //always 0
					}
					else
					{
						rxEng2eventEng_setEvent.write(extendedEvent(rstEvent(mh_meta.seqNumb+mh_meta.length), switchedTuple));
					}
				}
				//else ignore => do nothing
				if (mh_meta.length != 0)
				{
					dropDataFifoOut.write(true);
				}
			}
			else
			{
				// Make session lookup, only allow creation of new entry when SYN or SYN_ACK
				rxEng2sLookup_req.write(sessionLookupQuery(tuple, (mh_meta.syn && !mh_meta.rst && !mh_meta.fin)));
				mh_state = LOOKUP;
			}
		}
		break;
	case LOOKUP: //BIG delay here, waiting for LOOKup
		if (!sLookup2rxEng_rsp.empty())
		{
			sLookup2rxEng_rsp.read(mh_lup);
			if (mh_lup.hit)
			{
				//Write out lup and meta
				fsmMetaDataFifo.write(rxFsmMetaData(mh_lup.sessionID, mh_srcIpAddress, mh_dstIpPort, mh_meta, mh_srcIpPort));
			}
			if (mh_meta.length != 0)
			{
				dropDataFifoOut.write(!mh_lup.hit);
			}
			/*if (!mh_lup.hit)
			{
				// Port is Open, but we have no sessionID, that matches or is free
				// For SYN we should time out, for everything else sent RST TODO
				if (mh_meta.length != 0)
				{
					dropDataFifoOut.write(true); // always write???
				}
				//mh_state = META;
			}
			else
			{
				//Write out lup and meta
				fsmMetaDataFifo.write(rxFsmMetaData(mh_lup.sessionID, mh_srcIpAddress, mh_dstIpPort, mh_meta));

				// read state
				/*rxEng2stateTable_upd_req.write(stateQuery(mh_lup.sessionID));
				// read rxSar & txSar
				if (!(mh_meta.syn && !mh_meta.rst && !mh_meta.fin)) // Do not read rx_sar for SYN(+ACK)(+ANYTHING) => (!syn || rst || fin
				{
					rxEng2rxSar_upd_req.write(rxSarRecvd(mh_lup.sessionID));
				}
				if (mh_meta.ack) // Do not read for SYN (ACK+ANYTHING)
				{
					rxEng2txSar_upd_req.write(rxTxSarQuery(mh_lup.sessionID));
				}*/
				//mh_state = META;
			//}
			mh_state = META;
		}

		break;
	}//switch
}

void rxTcpFSM(			stream<rxFsmMetaData>&					fsmMetaDataFifo,
						stream<sessionState>&					stateTable2rxEng_upd_rsp,
						stream<rxSarEntry>&						rxSar2rxEng_upd_rsp,
						stream<rxTxSarReply>&					txSar2rxEng_upd_rsp,
						stream<stateQuery>&						rxEng2stateTable_upd_req,
						stream<rxSarRecvd>&						rxEng2rxSar_upd_req,
						stream<rxTxSarQuery>&					rxEng2txSar_upd_req,
						stream<rxRetransmitTimerUpdate>&		rxEng2timer_clearRetransmitTimer,
						stream<ap_uint<16> >&					rxEng2timer_clearProbeTimer,
						stream<ap_uint<16> >&					rxEng2timer_setProbeTimer,
						stream<ap_uint<16> >&					rxEng2timer_setCloseTimer,
						stream<openStatus>&						openConStatusOut,
						stream<event>&							rxEng2eventEng_setEvent,
						stream<ap_uint<17> >&					dropDataFifoOut, //[0] drop, [16:1] head bytes to trim
						stream<rttAck>&							rxEng2rtt_ack,
						stream<ap_uint<16> >&					rxEng2timer_resetKeepalive,
						stream<bool>&						stateTable2rxEng_synBacklogFull,
#if !(RX_DDR_BYPASS)
						stream<mmCmd>&							rxBufferWriteCmd,
						stream<appNotification>&				rxEng2rxApp_notification)
#else
						stream<appNotification>&				rxEng2rxApp_notification,
						ap_uint<16>						rxbuffer_data_count,
						ap_uint<16>						rxbuffer_max_data_count)
#endif
{
#pragma HLS INLINE off
#pragma HLS pipeline II=1


	enum fsmStateType {LOAD, TRANSITION};
	static fsmStateType fsm_state = LOAD;

	static rxFsmMetaData fsm_meta;
	static bool fsm_txSarRequest = false;
	// RFC 5961 challenge ACK rate limit (global)
	static ap_uint<28> fsm_challengeTimer = 0;
	static ap_uint<11> fsm_challengeCount = 0;
	bool challengeOk = (fsm_challengeCount < CHALLENGE_ACK_LIMIT);
	bool challengeSent = false;
	// SYN backlog full (published by the state table on change)
	static bool fsm_synBacklogFull = false;
	if (!stateTable2rxEng_synBacklogFull.empty())
	{
		fsm_synBacklogFull = stateTable2rxEng_synBacklogFull.read();
	}


	ap_uint<4> control_bits = 0;
	sessionState tcpState;
	rxSarEntry rxSar;
	rxTxSarReply txSar;


	switch(fsm_state)
	{
	case LOAD:
		if (!fsmMetaDataFifo.empty())
		{
			fsmMetaDataFifo.read(fsm_meta);
			// read state
			rxEng2stateTable_upd_req.write(stateQuery(fsm_meta.sessionID));
			// Always read rxSar, even though not required for SYN-ACK
			rxEng2rxSar_upd_req.write(rxSarRecvd(fsm_meta.sessionID));
			// read txSar
			// (also for a pure FIN without ACK, so the FIN handler can run;
			//  the TRANSITION state waits for the response, and the default
			//  handler consumes it with read_nb for odd flag combinations)
			// (and for a SYN: a simultaneous open answers with the ISS of
			//  its own SYN)
			if (fsm_meta.meta.ack || fsm_meta.meta.fin || fsm_meta.meta.syn)
			{
				rxEng2txSar_upd_req.write(rxTxSarQuery(fsm_meta.sessionID));
				fsm_txSarRequest  = true;
			}
			fsm_state = TRANSITION;
		}
		break;
	case TRANSITION:
		// Check if transition to LOAD occurs
		if (!stateTable2rxEng_upd_rsp.empty() && !rxSar2rxEng_upd_rsp.empty()
						&& !(fsm_txSarRequest && txSar2rxEng_upd_rsp.empty()))
		{
			fsm_state = LOAD;
			fsm_txSarRequest = false;
		}

		control_bits[0] = fsm_meta.meta.ack;
		control_bits[1] = fsm_meta.meta.syn;
		control_bits[2] = fsm_meta.meta.fin;
		control_bits[3] = fsm_meta.meta.rst;
		switch (control_bits)
		{
		case 1: //ACK
			//if (!rxSar2rxEng_upd_rsp.empty() && !stateTable2rxEng_upd_rsp.empty() && !txSar2rxEng_upd_rsp.empty())
			if (fsm_state == LOAD)
			{
				stateTable2rxEng_upd_rsp.read(tcpState);
				rxSar2rxEng_upd_rsp.read(rxSar);
				txSar2rxEng_upd_rsp.read(txSar);
				// RFC 7323 PAWS: a segment whose timestamp is older than
				// TS.Recent is a duplicate from a previous sequence-number
				// wrap; drop it and answer with a duplicate ACK (5.3 R2)
				bool pawsReject = rxSar.tsEnabled && fsm_meta.meta.tsValid
						&& seq_lt(fsm_meta.meta.tsVal, rxSar.tsRecent)
						&& (tcpState == ESTABLISHED || tcpState == SYN_RECEIVED || tcpState == FIN_WAIT_1
							|| tcpState == FIN_WAIT_2 || tcpState == CLOSE_WAIT
							|| tcpState == CLOSING || tcpState == LAST_ACK);
				// RFC 7323 TS.Recent update rule: fresh timestamp on a segment
				// that covers (or precedes) the left window edge
				bool tsUpdate = rxSar.tsEnabled && fsm_meta.meta.tsValid && !pawsReject
						&& seq_geq(fsm_meta.meta.tsVal, rxSar.tsRecent)
						&& seq_leq(fsm_meta.meta.seqNumb, rxSar.recvd);
				// RFC 3168 6.1.3: a CE mark on a data segment is echoed with
				// ECE until the peer confirms it with CWR
				bool ceMark = rxSar.ecnEnabled && fsm_meta.meta.ce && (fsm_meta.meta.length != 0);
				bool eceNext = ceMark || (rxSar.ecePending && !fsm_meta.meta.cwr);
				bool eceChange = rxSar.ecnEnabled && (eceNext != rxSar.ecePending);
				// the first CE mark is acknowledged at once (cf. Linux quickack)
				bool ceQuickAck = ceMark && !rxSar.ecePending;
				// RFC 5961 5.2 ACK validation in the synchronized states:
				// SND.UNA - MAX.SND.WND <= SEG.ACK <= SND.NXT. An ACK for data
				// never sent (RFC 793) or one too old to belong to this
				// connection's recent history is dropped together with its
				// payload and answered with a (throttled) ACK. MAX.SND.WND is
				// bounded by the per-session TX buffer.
				bool ackInvalid = !pawsReject
						&& (tcpState == ESTABLISHED || tcpState == FIN_WAIT_1
							|| tcpState == FIN_WAIT_2 || tcpState == CLOSE_WAIT
							|| tcpState == CLOSING || tcpState == LAST_ACK)
						&& (seq_gt(fsm_meta.meta.ackNumb, txSar.nextByte)
							|| seq_lt(fsm_meta.meta.ackNumb, txSar.prevAck - (ap_uint<32>)BUFFER_SIZE));
				if (pawsReject)
				{
					if (fsm_meta.meta.length != 0)
					{
						dropDataFifoOut.write(true);
					}
					rxEng2eventEng_setEvent.write(event(ACK_NODELAY, fsm_meta.sessionID));
					rxEng2stateTable_upd_req.write(stateQuery(fsm_meta.sessionID, tcpState, 1));
				}
				else if (ackInvalid)
				{
					if (fsm_meta.meta.length != 0)
					{
						dropDataFifoOut.write(true);
					}
					if (challengeOk)
					{
						rxEng2eventEng_setEvent.write(event(ACK_NODELAY, fsm_meta.sessionID));
						challengeSent = true;
					}
					rxEng2stateTable_upd_req.write(stateQuery(fsm_meta.sessionID, tcpState, 1));
				}
				else
				{
				// (indentation of the original block below is left unchanged)
				rxEng2timer_clearRetransmitTimer.write(rxRetransmitTimerUpdate(fsm_meta.sessionID, (fsm_meta.meta.ackNumb == txSar.nextByte)));
				rxEng2timer_resetKeepalive.write(fsm_meta.sessionID);
				if (tcpState == ESTABLISHED || tcpState == SYN_RECEIVED || tcpState == FIN_WAIT_1
						|| tcpState == FIN_WAIT_2 || tcpState == CLOSE_WAIT
						|| tcpState == CLOSING || tcpState == LAST_ACK)
				{
					// per-session effective MSS for congestion arithmetic
					ap_uint<16> sessMSS = (txSar.mss == 0) ? (ap_uint<16>)MSS : txSar.mss;
					bool enterFastRecovery = false;
					// RFC 3168 6.1.2: an ECE halves cwnd (ssthresh = max(cwnd/2,
					// 2*MSS), cwnd = ssthresh) at most once per window of data,
					// and not while loss recovery already reduced it. Derived
					// from the cwnd before this ACK's growth, in parallel. The
					// hold covers ACKs up to and including snd_nxt at the
					// reduction: those were sent before our CWR reached the peer.
					bool ecnHoldNow = txSar.ecnHold && seq_leq(fsm_meta.meta.ackNumb, txSar.ecnRecover);
					bool ecnReduce = txSar.ecnEnabled && fsm_meta.meta.ece && !ecnHoldNow
							&& !txSar.fastRetransmitted && (txSar.count != 2);
					ap_uint<WINDOW_BITS> ecnHalf = txSar.cong_window >> 1;
					ap_uint<WINDOW_BITS> ecnSsthresh = (ecnHalf > (ap_uint<WINDOW_BITS>)(2*sessMSS)) ? ecnHalf : (ap_uint<WINDOW_BITS>)(2*sessMSS);
					// track the peer's latest SACK blocks (RFC 2018, up to two)
					ap_uint<32> newPeerSackStart = txSar.peerSackStart;
					ap_uint<32> newPeerSackEnd = txSar.peerSackEnd;
					ap_uint<1> newPeerSackValid = txSar.peerSackValid;
					ap_uint<32> newPeerSackStart2 = txSar.peerSackStart2;
					ap_uint<32> newPeerSackEnd2 = txSar.peerSackEnd2;
					ap_uint<1> newPeerSackValid2 = txSar.peerSackValid2;
					// RFC 2883: a first block below the cumulative ACK, or inside
					// the second block, is a D-SACK (a duplicate the peer got,
					// e.g. after a spurious retransmit); it says nothing about
					// holes, so the scoreboard uses the remaining block only
					bool peerDsack = fsm_meta.meta.sackValid
							&& (seq_leq(fsm_meta.meta.sackEnd, fsm_meta.meta.ackNumb)
								|| (fsm_meta.meta.sackValid2 && seq_geq(fsm_meta.meta.sackStart, fsm_meta.meta.sackStart2)
									&& seq_leq(fsm_meta.meta.sackEnd, fsm_meta.meta.sackEnd2)));
					if (peerDsack && fsm_meta.meta.sackValid2)
					{
						newPeerSackStart = fsm_meta.meta.sackStart2;
						newPeerSackEnd = fsm_meta.meta.sackEnd2;
						newPeerSackValid = 1;
						newPeerSackValid2 = 0;
					}
					else if (fsm_meta.meta.sackValid && !peerDsack)
					{
						newPeerSackStart = fsm_meta.meta.sackStart;
						newPeerSackEnd = fsm_meta.meta.sackEnd;
						newPeerSackValid = 1;
						newPeerSackStart2 = fsm_meta.meta.sackStart2;
						newPeerSackEnd2 = fsm_meta.meta.sackEnd2;
						newPeerSackValid2 = fsm_meta.meta.sackValid2;
					}
					else if (seq_gt(fsm_meta.meta.ackNumb, txSar.prevAck))
					{
						//cumulative progress without new SACK info: drop the stale blocks
						newPeerSackValid = 0;
						newPeerSackValid2 = 0;
					}
					// Check if new ACK arrived
					if (fsm_meta.meta.ackNumb == txSar.prevAck && txSar.prevAck != txSar.nextByte)
					{
						// RFC 5681: a duplicate ACK carries no data AND leaves the
						// advertised window unchanged - a pure window update must
						// not count towards fast retransmit
						if (fsm_meta.meta.length == 0 && fsm_meta.meta.winSize == txSar.recv_window)
						{
							txSar.count++;
							if (txSar.count == 3 && !txSar.fastRetransmitted)
							{
								// RFC 5681 fast recovery entry on the 3rd duplicate ACK:
								// ssthresh = max(FlightSize/2, 2*MSS), cwnd = ssthresh + 3*MSS
								ap_uint<32> flightSize = txSar.nextByte - txSar.prevAck;
								ap_uint<WINDOW_BITS> ssthreshNew;
								if (flightSize > (ap_uint<32>)(4*sessMSS))
								{
									ssthreshNew = (ap_uint<WINDOW_BITS>)(flightSize >> 1);
								}
								else
								{
									ssthreshNew = 2*sessMSS;
								}
								txSar.slowstart_threshold = ssthreshNew;
								txSar.cong_window = ssthreshNew + 3*sessMSS;
								enterFastRecovery = true;
							}
							else if (txSar.fastRetransmitted && txSar.cong_window <= (CONGESTION_WINDOW_MAX - sessMSS))
							{
								// window inflation for each further duplicate ACK
								txSar.cong_window += sessMSS;
							}
						}
					}
					else if (seq_gt(fsm_meta.meta.ackNumb, txSar.prevAck))
					{
						// Notify probeTimer about new ACK
						rxEng2timer_clearProbeTimer.write(fsm_meta.sessionID);
						// Report the cumulative ACK to the RTT estimator
						rxEng2rtt_ack.write(rttAck(fsm_meta.sessionID, fsm_meta.meta.ackNumb));
						if (txSar.fastRetransmitted)
						{
							// leaving fast recovery: deflate cwnd to ssthresh (RFC 5681)
							txSar.cong_window = txSar.slowstart_threshold;
							txSar.ca_acc = 0;
						}
						// Check for SlowStart & Increase Congestion Window
						else if (txSar.cong_window <= (txSar.slowstart_threshold-sessMSS))
						{
							txSar.cong_window += sessMSS;
							txSar.ca_acc = 0;
						}
						else
						{
							// congestion avoidance, byte counting (RFC 3465):
							// grow by one MSS per cwnd-worth of acked bytes,
							// i.e. ~MSS^2/cwnd per ACK, ~one MSS per RTT
							ap_uint<32> ackedBytes32 = fsm_meta.meta.ackNumb - txSar.prevAck;
							ap_uint<WINDOW_BITS+2> accSum = (ap_uint<WINDOW_BITS+2>)txSar.ca_acc
									+ (ap_uint<WINDOW_BITS+2>)ackedBytes32(WINDOW_BITS-1, 0);
							if (accSum >= (ap_uint<WINDOW_BITS+2>)txSar.cong_window)
							{
								accSum -= txSar.cong_window;
								if (txSar.cong_window <= (ap_uint<WINDOW_BITS>)(CONGESTION_WINDOW_MAX - sessMSS))
								{
									txSar.cong_window += sessMSS;
								}
							}
							txSar.ca_acc = accSum(WINDOW_BITS-1, 0);
						}
						txSar.count = 0;
						txSar.fastRetransmitted = false;
					}
					// else: stale ACK (older than snd_una) - ignore for congestion state
					if (ecnReduce)
					{
						txSar.slowstart_threshold = ecnSsthresh;
						txSar.cong_window = ecnSsthresh;
						txSar.ca_acc = 0;
					}
					// TX SAR
					// mod-2^32 form of "snd_una <= ack <= snd_nxt" (replaces the
					// hand-rolled two-case wrap check)
					if (seq_leq(txSar.prevAck, fsm_meta.meta.ackNumb) && seq_leq(fsm_meta.meta.ackNumb, txSar.nextByte))
					{
						rxTxSarQuery txSarUpd(fsm_meta.sessionID, fsm_meta.meta.ackNumb, fsm_meta.meta.winSize, txSar.cong_window, txSar.count, ((txSar.count == 3) || txSar.fastRetransmitted));
						txSarUpd.ssthresh = txSar.slowstart_threshold;
						txSarUpd.writeSsthresh = enterFastRecovery || ecnReduce;
						txSarUpd.ca_acc = enterFastRecovery ? (ap_uint<WINDOW_BITS>)0 : txSar.ca_acc;
						txSarUpd.ecnUpdate = 1;
						txSarUpd.ecnHold = ecnHoldNow || ecnReduce;
						txSarUpd.ecnReduce = ecnReduce;
						txSarUpd.ecnRecover = txSar.nextByte;
						txSarUpd.peerSackStart = newPeerSackStart;
						txSarUpd.peerSackEnd = newPeerSackEnd;
						txSarUpd.peerSackValid = newPeerSackValid;
						txSarUpd.peerSackStart2 = newPeerSackStart2;
						txSarUpd.peerSackEnd2 = newPeerSackEnd2;
						txSarUpd.peerSackValid2 = newPeerSackValid2;
						rxEng2txSar_upd_req.write(txSarUpd);
					}
					// zero-window persist (RFC 1122 4.2.2.17): arm the probe
					// timer while the peer advertises a zero window; a window
					// update on a pure ACK disarms it (and resumes TX)
					if (fsm_meta.meta.winSize == 0)
					{
						rxEng2timer_setProbeTimer.write(fsm_meta.sessionID);
					}
					else if (fsm_meta.meta.ackNumb == txSar.prevAck && fsm_meta.meta.length == 0)
					{
						rxEng2timer_clearProbeTimer.write(fsm_meta.sessionID);
					}
					// pure ACK: persist the TS.Recent advance here (segments
					// with payload carry it on their rxSar update below)
					if ((tsUpdate || eceChange) && fsm_meta.meta.length == 0)
					{
						rxSarRecvd tsOnlyUpd(fsm_meta.sessionID);
						tsOnlyUpd.tsWrite = tsUpdate;
						tsOnlyUpd.tsRecent = fsm_meta.meta.tsVal;
						tsOnlyUpd.eceWrite = eceChange;
						tsOnlyUpd.ece = eceNext;
						rxEng2rxSar_upd_req.write(tsOnlyUpd);
					}

// 					// Check if packet contains payload
// 					if (fsm_meta.meta.length != 0)
// 					{
// 						ap_uint<32> newRecvd = fsm_meta.meta.seqNumb+fsm_meta.meta.length;
// 						// Second part makes sure that app pointer is not overtaken
// #if !(RX_DDR_BYPASS)
// 						ap_uint<WINDOW_BITS> free_space = ((rxSar.appd - rxSar.recvd(WINDOW_BITS-1, 0)) - 1);
// 						// Check if segment in order and if enough free space is available
// 						if ((fsm_meta.meta.seqNumb == rxSar.recvd) && (free_space > fsm_meta.meta.length))
// #else
// 						if ((fsm_meta.meta.seqNumb == rxSar.recvd) && ((rxbuffer_max_data_count - rxbuffer_data_count) > 375))
// #endif
// 						{
// 							rxEng2rxSar_upd_req.write(rxSarRecvd(fsm_meta.sessionID, newRecvd));
// 							// Build memory address
// 							ap_uint<32> pkgAddr;
// 							pkgAddr(31, 30) = 0x0;
// 							pkgAddr(29, WINDOW_BITS) = fsm_meta.sessionID(13, 0);
// 							pkgAddr(WINDOW_BITS-1, 0) = fsm_meta.meta.seqNumb(WINDOW_BITS-1, 0);
// #if !(RX_DDR_BYPASS)
// 							rxBufferWriteCmd.write(mmCmd(pkgAddr, fsm_meta.meta.length));
// #endif
// 							// Only notify about  new data available
// 							rxEng2rxApp_notification.write(appNotification(fsm_meta.sessionID, fsm_meta.meta.length, fsm_meta.srcIpAddress, fsm_meta.dstIpPort));
// 							dropDataFifoOut.write(false);
// 						}
// 						else
// 						{
// 							dropDataFifoOut.write(true);
// 						}


// 						// Sent ACK
// 						//rxEng2eventEng_setEvent.write(event(ACK, fsm_meta.sessionID));
// 					}

#if !(RX_DDR_BYPASS) //if enable DDR, OOO is enabled
					// Check if packet contains payload
					// Second part makes sure that app pointer is not overtaken
					ap_uint<WINDOW_BITS> free_space = ((rxSar.appd - (rxSar.gap2 ? rxSar.head2(WINDOW_BITS-1, 0) : rxSar.head(WINDOW_BITS-1, 0))) - 1);
					
					if (fsm_meta.meta.length != 0)
					{

						// Build memory address
						ap_uint<32> pkgAddr;
						pkgAddr(31, 30) = 0x0;
						pkgAddr(29, WINDOW_BITS) = fsm_meta.sessionID(13, 0);
						pkgAddr(WINDOW_BITS-1, 0) = fsm_meta.meta.seqNumb(WINDOW_BITS-1, 0);

						// Two-range out-of-order bookkeeping. Invariant (seq order):
						//   recvd < r0s < r0e < r1s < r1e
						ap_uint<32> newRecvd = rxSar.recvd;
						ap_uint<32> r0s = rxSar.offset;
						ap_uint<32> r0e = rxSar.head;
						bool r0v = rxSar.gap;
						ap_uint<32> r1s = rxSar.offset2;
						ap_uint<32> r1e = rxSar.head2;
						bool r1v = rxSar.gap2;
						ap_uint<32> segStart = fsm_meta.meta.seqNumb;
						ap_uint<32> segEnd = fsm_meta.meta.seqNumb + fsm_meta.meta.length;
						// wrap-safe acceptance: the whole segment must lie inside
						// the advertised receive window (see the F2 fix)
						ap_uint<32> winRight = rxSar.recvd + (ap_uint<32>)(ap_uint<WINDOW_BITS>)(rxSar.appd - rxSar.recvd(WINDOW_BITS-1, 0) - 1);
						bool inWindow = seq_leq(segEnd, winRight);
						bool accepted = false;
						ap_uint<WINDOW_BITS> notifyLen = 0;
						// A segment that starts below rcv_nxt but carries new data
						// (e.g. a repacketized retransmission) is accepted like an
						// in-order one when its old head is still unread: the whole
						// segment is written at its seq address, rewriting the
						// unread head with the same bytes. An already read head
						// would alias onto the far end of the window, so such a
						// segment is still dropped (the peer resends from the ACK).
						ap_uint<WINDOW_BITS> unread = rxSar.recvd(WINDOW_BITS-1, 0) - rxSar.appd;
						ap_uint<32> oldHead = rxSar.recvd - segStart;
						bool overlapHead = seq_lt(segStart, rxSar.recvd) && seq_gt(segEnd, rxSar.recvd)
								&& (oldHead <= (ap_uint<32>)unread);
						bool atRecvd = (segStart == rxSar.recvd) || overlapHead;

						if (!r0v && atRecvd && (free_space > fsm_meta.meta.length))
						{
							// ### in order, no gap
							newRecvd = segEnd;
							notifyLen = (ap_uint<WINDOW_BITS>)(segEnd - rxSar.recvd);
							accepted = true;
						}
						else if (!r0v && seq_gt(segStart, rxSar.recvd) && inWindow)
						{
							// ### first out-of-order segment: open range 0
							r0s = segStart; r0e = segEnd; r0v = true;
							accepted = true;
						}
						else if (r0v && atRecvd)
						{
							// ### fill (or partly fill) the first gap; a merge may
							// cascade through both buffered ranges. This also covers
							// retransmissions that overlap the buffered data (B6).
							if (seq_geq(segEnd, r0s))
							{
								newRecvd = seq_gt(segEnd, r0e) ? segEnd : r0e;
								r0v = false;
								if (r1v && seq_geq(newRecvd, r1s))
								{
									newRecvd = seq_gt(newRecvd, r1e) ? newRecvd : r1e;
									r1v = false;
								}
								else if (r1v)
								{
									//promote range 1 to range 0
									r0s = r1s; r0e = r1e; r0v = true;
									r1v = false;
								}
							}
							else
							{
								newRecvd = segEnd;
							}
							notifyLen = (ap_uint<WINDOW_BITS>)(newRecvd - rxSar.recvd);
							accepted = true;
						}
						else if (r0v && (segStart == r0e) && (free_space > fsm_meta.meta.length))
						{
							// ### extend range 0 upward (may close the second gap)
							if (r1v && seq_geq(segEnd, r1s))
							{
								r0e = seq_gt(segEnd, r1e) ? segEnd : r1e;
								r1v = false;
							}
							else
							{
								r0e = segEnd;
							}
							accepted = true;
						}
						else if (r1v && (segStart == r1e) && (free_space > fsm_meta.meta.length))
						{
							// ### extend range 1 upward
							r1e = segEnd;
							accepted = true;
						}
						else if (r0v && !r1v && seq_gt(segStart, r0e) && inWindow)
						{
							// ### second out-of-order range
							r1s = segStart; r1e = segEnd; r1v = true;
							accepted = true;
						}

						if (accepted)
						{
							#ifndef __SYNTHESIS__
							std::cout<<"RX_ACCEPT";
							std::cout<<std::dec<<" session id:"<<fsm_meta.sessionID;
							std::cout<<" seq:"<<segStart<<" len:"<<fsm_meta.meta.length;
							std::cout<<" recvd:"<<rxSar.recvd<<"->"<<newRecvd;
							std::cout<<" r0:"<<r0v<<" ["<<r0s<<","<<r0e<<")";
							std::cout<<" r1:"<<r1v<<" ["<<r1s<<","<<r1e<<")"<<std::endl;
							#endif
							rxSarRecvd sarUpd(fsm_meta.sessionID, newRecvd, r0v ? r0e : newRecvd, r0s, r0v);
							sarUpd.head2 = r1e;
							sarUpd.offset2 = r1s;
							sarUpd.gap2 = r1v;
							sarUpd.tsWrite = tsUpdate;
							sarUpd.tsRecent = fsm_meta.meta.tsVal;
							sarUpd.eceWrite = eceChange;
							sarUpd.ece = eceNext;
							// RFC 2883: report the duplicate head of an overlapping segment
							sarUpd.dsackWrite = overlapHead && rxSar.sackEnabled;
							sarUpd.dsackStart = segStart;
							sarUpd.dsackLen = oldHead(15, 0);
							rxEng2rxSar_upd_req.write(sarUpd);
							//write to memory
							rxBufferWriteCmd.write(mmCmd(pkgAddr, fsm_meta.meta.length));
							//notify the app about newly contiguous bytes; a zero-length
							//notification just matches the mem write status in the
							//notification delayer
							rxEng2rxApp_notification.write(appNotification(fsm_meta.sessionID, notifyLen, fsm_meta.srcIpAddress, fsm_meta.dstIpPort));
							dropDataFifoOut.write(false);
						}
						// ### drop the packet in all other cases
						else
						{
							dropDataFifoOut.write(true);
							// RFC 2883: a segment starting below rcv_nxt is a
							// duplicate (fully, or its already received head);
							// report it as a D-SACK block in the next ACK
							bool dsackHit = rxSar.sackEnabled && seq_lt(segStart, rxSar.recvd)
									&& ((ap_uint<32>)(rxSar.recvd - segStart) < (ap_uint<32>)BUFFER_SIZE);
							ap_uint<32> dsackEnd = seq_lt(segEnd, rxSar.recvd) ? segEnd : rxSar.recvd;
							if (eceChange || dsackHit)
							{
								rxSarRecvd eceOnlyUpd(fsm_meta.sessionID);
								eceOnlyUpd.eceWrite = eceChange;
								eceOnlyUpd.ece = eceNext;
								eceOnlyUpd.dsackWrite = dsackHit;
								eceOnlyUpd.dsackStart = segStart;
								eceOnlyUpd.dsackLen = dsackEnd - segStart;
								rxEng2rxSar_upd_req.write(eceOnlyUpd);
							}
							#ifndef __SYNTHESIS__
							std::cout<<"RX_DROP";
							std::cout<<std::dec<<" session id:"<<fsm_meta.sessionID;
							std::cout<<" seqNum:"<<segStart;
							std::cout<<" recvd:"<<rxSar.recvd;
							std::cout<<" r0:"<<r0v<<" ["<<r0s<<","<<r0e<<")";
							std::cout<<" r1:"<<r1v<<" ["<<r1s<<","<<r1e<<")";
							std::cout<<" length:"<<fsm_meta.meta.length;
							std::cout<<" free_space:"<<free_space<<std::endl;
							#endif
						}

					}


#else //if DDR is not used, OOO is disabled
					// Check if packet contains payload
					if (fsm_meta.meta.length != 0)
					{
						ap_uint<32> newRecvd = fsm_meta.meta.seqNumb+fsm_meta.meta.length;
						ap_uint<32> newHead = fsm_meta.meta.seqNumb+fsm_meta.meta.length;
						// A segment that starts below rcv_nxt but carries new data
						// (e.g. a repacketized retransmission) is accepted with its
						// already received head trimmed off by the package dropper
						ap_uint<32> oldHead = rxSar.recvd - fsm_meta.meta.seqNumb;
						bool overlapHead = seq_lt(fsm_meta.meta.seqNumb, rxSar.recvd) && seq_gt(newRecvd, rxSar.recvd);
						// Second part makes sure that app pointer is not overtaken
						// Check if segment in order and if enough free space is available
						if (((fsm_meta.meta.seqNumb == rxSar.recvd) || overlapHead) && ((rxbuffer_max_data_count - rxbuffer_data_count) > 375))
						{
							rxSarRecvd sarUpd3(fsm_meta.sessionID, newRecvd, newHead, rxSar.offset, rxSar.gap);
							sarUpd3.tsWrite = tsUpdate;
							sarUpd3.tsRecent = fsm_meta.meta.tsVal;
							sarUpd3.eceWrite = eceChange;
							sarUpd3.ece = eceNext;
							// RFC 2883: report the trimmed duplicate head
							sarUpd3.dsackWrite = overlapHead && rxSar.sackEnabled;
							sarUpd3.dsackStart = fsm_meta.meta.seqNumb;
							sarUpd3.dsackLen = oldHead(15, 0);
							rxEng2rxSar_upd_req.write(sarUpd3);
							// Build memory address
							ap_uint<32> pkgAddr;
							pkgAddr(31, 30) = 0x0;
							pkgAddr(29, WINDOW_BITS) = fsm_meta.sessionID(13, 0);
							pkgAddr(WINDOW_BITS-1, 0) = fsm_meta.meta.seqNumb(WINDOW_BITS-1, 0);
							// Only notify about new data available
							ap_uint<16> trim = overlapHead ? oldHead(15, 0) : (ap_uint<16>)0;
							rxEng2rxApp_notification.write(appNotification(fsm_meta.sessionID, fsm_meta.meta.length - trim, fsm_meta.srcIpAddress, fsm_meta.dstIpPort));
							dropDataFifoOut.write((ap_uint<17>)trim << 1);
						}
						else
						{
							dropDataFifoOut.write(true);
							// RFC 2883 D-SACK for a duplicate (see the OOO path)
							ap_uint<32> segStart = fsm_meta.meta.seqNumb;
							bool dsackHit = rxSar.sackEnabled && seq_lt(segStart, rxSar.recvd)
									&& ((ap_uint<32>)(rxSar.recvd - segStart) < (ap_uint<32>)BUFFER_SIZE);
							ap_uint<32> dsackEnd = seq_lt(newRecvd, rxSar.recvd) ? newRecvd : rxSar.recvd;
							if (eceChange || dsackHit)
							{
								rxSarRecvd eceOnlyUpd(fsm_meta.sessionID);
								eceOnlyUpd.eceWrite = eceChange;
								eceOnlyUpd.ece = eceNext;
								eceOnlyUpd.dsackWrite = dsackHit;
								eceOnlyUpd.dsackStart = segStart;
								eceOnlyUpd.dsackLen = dsackEnd - segStart;
								rxEng2rxSar_upd_req.write(eceOnlyUpd);
							}
						}
					}
#endif


#if FAST_RETRANSMIT
					if (txSar.count == 3 && !txSar.fastRetransmitted)
					{
						rxEng2eventEng_setEvent.write(event(RT, fsm_meta.sessionID));
					}
					else if (fsm_meta.meta.length != 0)
#else
					if (fsm_meta.meta.length != 0)
#endif
					{
						//send ack only when packet in-order
#if !(RX_DDR_BYPASS)					
						if (!rxSar.gap && (fsm_meta.meta.seqNumb == rxSar.recvd) && (free_space > fsm_meta.meta.length))
#else
						if ((fsm_meta.meta.seqNumb == rxSar.recvd) && ((rxbuffer_max_data_count - rxbuffer_data_count) > 375))
#endif
						{
							rxEng2eventEng_setEvent.write(event(ceQuickAck ? ACK_NODELAY : ACK, fsm_meta.sessionID));
						}
						//send duplicate ack with ack_nodelay
						else 
						{
							rxEng2eventEng_setEvent.write(event(ACK_NODELAY, fsm_meta.sessionID));	
						}
						
					}


					// Reset Retransmit Timer
					//rxEng2timer_clearRetransmitTimer.write(rxRetransmitTimerUpdate(fsm_meta.sessionID, (mh_meta.ackNumb == txSarNextByte)));
					if (fsm_meta.meta.ackNumb == txSar.nextByte)
					{
						switch (tcpState)
						{
						case SYN_RECEIVED:
							rxEng2stateTable_upd_req.write(stateQuery(fsm_meta.sessionID, ESTABLISHED, 1)); //TODO MAYBE REARRANGE
							// a locally opened session reaches SYN_RECEIVED only
							// through a simultaneous open: report the open now
							if (fsm_meta.dstIpPort[15] == 1)
							{
								openConStatusOut.write(openStatus(fsm_meta.sessionID, 1, fsm_meta.srcIpAddress, fsm_meta.srcIpPort));
							}
							break;
						case FIN_WAIT_1: //our FIN got ACKed
							rxEng2stateTable_upd_req.write(stateQuery(fsm_meta.sessionID, FIN_WAIT_2, 1));
							break;
						case CLOSING:
							rxEng2stateTable_upd_req.write(stateQuery(fsm_meta.sessionID, TIME_WAIT, 1));
							rxEng2timer_setCloseTimer.write(fsm_meta.sessionID);
							break;
						case LAST_ACK:
							rxEng2stateTable_upd_req.write(stateQuery(fsm_meta.sessionID, CLOSED, 1));
							break;
						default:
							rxEng2stateTable_upd_req.write(stateQuery(fsm_meta.sessionID, tcpState, 1));
							break;
						}
					}
					else //we have to release the lock
					{
						//reset rtTimer
						//rtTimer.write(rxRetransmitTimerUpdate(fsm_meta.sessionID));
						rxEng2stateTable_upd_req.write(stateQuery(fsm_meta.sessionID, tcpState, 1)); // or ESTABLISHED
					}
				} //end state if
				else if (tcpState == TIME_WAIT)
				{
					// RFC 793: a stray in-window segment in TIME_WAIT is
					// answered with an ACK, never a RST (the RST would hit the
					// peer's LAST_ACK/retransmitted FIN exchange)
					rxEng2eventEng_setEvent.write(event(ACK_NODELAY, fsm_meta.sessionID));
					if (fsm_meta.meta.length != 0)
					{
						dropDataFifoOut.write(true);
					}
					rxEng2stateTable_upd_req.write(stateQuery(fsm_meta.sessionID, tcpState, 1));
				}
				else // state == (CLOSED || SYN_SENT)
				{
					// SENT RST, RFC 793: fig.11
					rxEng2eventEng_setEvent.write(rstEvent(fsm_meta.sessionID, fsm_meta.meta.seqNumb+fsm_meta.meta.length)); // noACK ?
					// if data is in the pipe it needs to be droppped
					if (fsm_meta.meta.length != 0)
					{
						dropDataFifoOut.write(true);
					}
					rxEng2stateTable_upd_req.write(stateQuery(fsm_meta.sessionID, tcpState, 1));
				}
				} //end PAWS else
				//fsm_state = LOAD;
			}
			break;
		case 2: //SYN
			//if (!stateTable2rxEng_upd_rsp.empty())
			if (fsm_state == LOAD)
			{
				stateTable2rxEng_upd_rsp.read(tcpState);
				rxSar2rxEng_upd_rsp.read(rxSar);
				txSar2rxEng_upd_rsp.read(txSar);
				#ifndef __SYNTHESIS__
				std::cout<<"RX_SYN: ";
				std::cout<<std::dec<<" session id:"<<fsm_meta.sessionID;
				std::cout<<" seqNum:"<<fsm_meta.meta.seqNumb;
				std::cout<<" recvd:"<<rxSar.recvd;
				std::cout<<" head:"<<rxSar.head;
				std::cout<<" offset:"<<rxSar.offset;
				std::cout<<" length:"<<fsm_meta.meta.length;		
				std::cout<<" gap:"<<rxSar.gap<<std::endl;
				#endif
				// std::cout<<" free_space"<<free_space<<std::endl;
				// TIME_WAIT reuse (RFC 1122 4.2.2.13, RFC 6191): a new SYN for a
				// listening port is accepted if it is provably newer than the
				// old connection - by timestamp when the old connection used
				// them, by sequence number otherwise
				bool twNewer = (rxSar.tsEnabled && fsm_meta.meta.tsValid)
								? (ap_int<32>)(fsm_meta.meta.tsVal - rxSar.tsRecent) > 0
								: seq_gt(fsm_meta.meta.seqNumb, rxSar.recvd);
				bool twReuse = (tcpState == TIME_WAIT) && (fsm_meta.dstIpPort[15] == 0)
								&& !fsm_synBacklogFull && twNewer;
				if (tcpState == CLOSED && fsm_synBacklogFull)
				{
					// SYN backlog exhausted: drop the SYN and release the
					// session that the lookup just created for it
					rxEng2stateTable_upd_req.write(stateQuery(fsm_meta.sessionID, CLOSED, 1));
				}
				else if (tcpState == CLOSED || tcpState == SYN_SENT || twReuse) // Actually this is LISTEN || SYN_SENT
				{
					// simultaneous open (RFC 793 fig. 8): our SYN is still
					// unacknowledged; answer with a SYN-ACK that keeps its ISS
					bool simOpen = (tcpState == SYN_SENT);
					if (twReuse)
					{
						// the old connection's TIME_WAIT timer must not close the new one
						rxEng2timer_setCloseTimer.write((ap_uint<16>)(fsm_meta.sessionID | CLOSE_TIMER_CANCEL));
					}
#if (WINDOW_SCALE)
					ap_uint<4> rx_win_shift = (fsm_meta.meta.winScale == 0) ? 0 : WINDOW_SCALE_BITS;
					ap_uint<4> tx_win_shift = fsm_meta.meta.winScale;
					// effective segment size for this session (RFC 879)
					ap_uint<16> session_mss = (fsm_meta.meta.mss < MSS) ? fsm_meta.meta.mss : (ap_uint<16>)MSS;
					// Initialize rxSar, SEQ + phantom byte for recvd and head pointer, offset to 0, gap set to false
					rxSarRecvd sarInit(fsm_meta.sessionID, fsm_meta.meta.seqNumb+1, fsm_meta.meta.seqNumb+1, 0, false, rx_win_shift, (bool)fsm_meta.meta.sackPermitted);
					// RFC 7323: timestamps are on iff both sides sent the option
					sarInit.tsEnabled = TCP_TS_OPTION && fsm_meta.meta.tsValid;
					sarInit.tsWrite = sarInit.tsEnabled;
					sarInit.tsRecent = fsm_meta.meta.tsVal;
					// RFC 3168 6.1.1: an ECN-setup SYN carries ECE and CWR; a
					// simultaneous open would need our SYN to have asked too
					bool ecnOk = TCP_ECN_ACCEPT && fsm_meta.meta.ece && fsm_meta.meta.cwr && !simOpen;
					sarInit.ecnEnabled = ecnOk;
					rxEng2rxSar_upd_req.write(sarInit);
					rxEng2timer_resetKeepalive.write(fsm_meta.sessionID);
					// Initialize receive window
					rxTxSarQuery txSarInit(fsm_meta.sessionID, simOpen ? txSar.prevAck : (ap_uint<32>)0, fsm_meta.meta.winSize, txSar.cong_window, 0, false, tx_win_shift, session_mss);
					txSarInit.ecnEnabled = ecnOk;
					rxEng2txSar_upd_req.write(txSarInit); //TODO maybe include count check
#else
					// Initialize rxSar, SEQ + phantom byte, last '1' for makes sure appd is initialized
					rxEng2rxSar_upd_req.write(rxSarRecvd(fsm_meta.sessionID, fsm_meta.meta.seqNumb+1, fsm_meta.meta.seqNumb+1, 0, false, 1));
					// Initialize receive window
					rxEng2txSar_upd_req.write((rxTxSarQuery(fsm_meta.sessionID, simOpen ? txSar.prevAck : (ap_uint<32>)0, fsm_meta.meta.winSize, txSar.cong_window, 0, false))); //TODO maybe include count check
#endif
					// Set SYN_ACK event (rt_count != 0: reuse the ISS in txSar.ackd)
					rxEng2eventEng_setEvent.write(event(SYN_ACK, fsm_meta.sessionID, simOpen ? 1 : 0));
					// Change State to SYN_RECEIVED
					rxEng2stateTable_upd_req.write(stateQuery(fsm_meta.sessionID, SYN_RECEIVED, 1));
				}
				else if (tcpState == SYN_RECEIVED)// && mh_meta.seqNumb+1 == rxSar.recvd) // Maybe Check for seq
				{
					// If it is the same SYN, we resent SYN-ACK, almost like quick RT, we could also wait for RT timer
					if (fsm_meta.meta.seqNumb+1 == rxSar.recvd)
					{
						// Retransmit SYN_ACK
						rxEng2eventEng_setEvent.write(event(SYN_ACK, fsm_meta.sessionID, 1));
						rxEng2stateTable_upd_req.write(stateQuery(fsm_meta.sessionID, tcpState, 1));
					}
					else // Sent RST, RFC 793: fig.9 (old) duplicate SYN(+ACK)
					{
						rxEng2eventEng_setEvent.write(rstEvent(fsm_meta.sessionID, fsm_meta.meta.seqNumb+1)); //length == 0
						rxEng2stateTable_upd_req.write(stateQuery(fsm_meta.sessionID, CLOSED, 1));
					}
				}
				else // Any synchronized state
				{
					// Unexpected SYN arrived: RFC 5961 4.2 challenge ACK regardless
					// of the sequence number (throttled)
					if (challengeOk)
					{
						rxEng2eventEng_setEvent.write(event(ACK_NODELAY, fsm_meta.sessionID));
						challengeSent = true;
					}
					// TODo send RST, has no ACK??
					// Respond with RST, no ACK, seq ==
					//eventEngine.write(rstEvent(mh_meta.seqNumb, mh_meta.length, true));
					rxEng2stateTable_upd_req.write(stateQuery(fsm_meta.sessionID, tcpState, 1));
				}
			}
			break;
		case 3: //SYN_ACK
			//if (!stateTable2rxEng_upd_rsp.empty() && !txSar2rxEng_upd_rsp.empty())
			if (fsm_state == LOAD)
			{
				stateTable2rxEng_upd_rsp.read(tcpState);
				rxSar2rxEng_upd_rsp.read(rxSar);
				txSar2rxEng_upd_rsp.read(txSar);
				rxEng2timer_clearRetransmitTimer.write(rxRetransmitTimerUpdate(fsm_meta.sessionID, (fsm_meta.meta.ackNumb == txSar.nextByte)));
				#ifndef __SYNTHESIS__
				std::cout<<"RX_SYN_ACK: ";
				std::cout<<std::dec<<" session id:"<<fsm_meta.sessionID;
				std::cout<<" seqNum:"<<fsm_meta.meta.seqNumb;
				std::cout<<" recvd:"<<rxSar.recvd;
				std::cout<<" head:"<<rxSar.head;
				std::cout<<" offset:"<<rxSar.offset;
				std::cout<<" length:"<<fsm_meta.meta.length;		
				std::cout<<" gap:"<<rxSar.gap<<std::endl;
				#endif
				// std::cout<<" free_space"<<free_space<<std::endl;
				if ((tcpState == SYN_SENT) && (fsm_meta.meta.ackNumb == txSar.nextByte))// && !mh_lup.created)
				{
#if (WINDOW_SCALE)
					ap_uint<4> rx_win_shift = (fsm_meta.meta.winScale == 0) ? 0 : WINDOW_SCALE_BITS;
					ap_uint<4> tx_win_shift = fsm_meta.meta.winScale;
					// effective segment size for this session (RFC 879)
					ap_uint<16> session_mss = (fsm_meta.meta.mss < MSS) ? fsm_meta.meta.mss : (ap_uint<16>)MSS;
					rxSarRecvd sarInit(fsm_meta.sessionID, fsm_meta.meta.seqNumb+1, fsm_meta.meta.seqNumb+1, 0, false, rx_win_shift, (bool)fsm_meta.meta.sackPermitted);
					// RFC 7323: timestamps are on iff both sides sent the option
					sarInit.tsEnabled = TCP_TS_OPTION && fsm_meta.meta.tsValid;
					sarInit.tsWrite = sarInit.tsEnabled;
					sarInit.tsRecent = fsm_meta.meta.tsVal;
					rxEng2rxSar_upd_req.write(sarInit);
					rxEng2timer_resetKeepalive.write(fsm_meta.sessionID);
					rxEng2txSar_upd_req.write((rxTxSarQuery(fsm_meta.sessionID, fsm_meta.meta.ackNumb, fsm_meta.meta.winSize, txSar.cong_window, 0, false, tx_win_shift, session_mss))); //TODO maybe include count check
#else
					//initialize rx_sar, SEQ + phantom byte, last '1' for appd init
					rxEng2rxSar_upd_req.write(rxSarRecvd(fsm_meta.sessionID, fsm_meta.meta.seqNumb+1, fsm_meta.meta.seqNumb+1, 0, false, 1));

					rxEng2txSar_upd_req.write((rxTxSarQuery(fsm_meta.sessionID, fsm_meta.meta.ackNumb, fsm_meta.meta.winSize, txSar.cong_window, 0, false))); //TODO maybe include count check
#endif
					// set ACK event
					rxEng2eventEng_setEvent.write(event(ACK_NODELAY, fsm_meta.sessionID));

					rxEng2stateTable_upd_req.write(stateQuery(fsm_meta.sessionID, ESTABLISHED, 1));
					openConStatusOut.write(openStatus(fsm_meta.sessionID, 1, fsm_meta.srcIpAddress, fsm_meta.srcIpPort));
				}
				else if ((tcpState == SYN_RECEIVED) && (fsm_meta.meta.ackNumb == txSar.nextByte)
						&& (fsm_meta.meta.seqNumb+1 == rxSar.recvd))
				{
					// simultaneous open: the peer's SYN-ACK acknowledges our SYN
					rxEng2txSar_upd_req.write((rxTxSarQuery(fsm_meta.sessionID, fsm_meta.meta.ackNumb, fsm_meta.meta.winSize, txSar.cong_window, 0, false)));
					rxEng2eventEng_setEvent.write(event(ACK_NODELAY, fsm_meta.sessionID));
					rxEng2stateTable_upd_req.write(stateQuery(fsm_meta.sessionID, ESTABLISHED, 1));
					if (fsm_meta.dstIpPort[15] == 1) // locally opened (ephemeral port)
					{
						openConStatusOut.write(openStatus(fsm_meta.sessionID, 1, fsm_meta.srcIpAddress, fsm_meta.srcIpPort));
					}
				}
				else if (tcpState == SYN_SENT) //TODO correct answer?
				{
					// Sent RST, RFC 793: fig.9 (old) duplicate SYN(+ACK)
					rxEng2eventEng_setEvent.write(rstEvent(fsm_meta.sessionID, fsm_meta.meta.seqNumb+fsm_meta.meta.length+1));
					rxEng2stateTable_upd_req.write(stateQuery(fsm_meta.sessionID, CLOSED, 1));
				}
				else
				{
					// Unexpected SYN arrived: RFC 5961 4.2 challenge ACK regardless
					// of the sequence number (throttled)
					if (challengeOk)
					{
						rxEng2eventEng_setEvent.write(event(ACK_NODELAY, fsm_meta.sessionID));
						challengeSent = true;
					}
					rxEng2stateTable_upd_req.write(stateQuery(fsm_meta.sessionID, tcpState, 1));
				}
			}
			break;
		case 4: //pure FIN without ACK (rare but legal; was silently ignored)
		case 5: //FIN (_ACK)
			//if (!rxSar2rxEng_upd_rsp.empty() && !stateTable2rxEng_upd_rsp.empty() && !txSar2rxEng_upd_rsp.empty())
			if (fsm_state == LOAD)
			{
				stateTable2rxEng_upd_rsp.read(tcpState);
				rxSar2rxEng_upd_rsp.read(rxSar);
				txSar2rxEng_upd_rsp.read(txSar);
				#ifndef __SYNTHESIS__
				std::cout<<"RX_FIN: ";
				std::cout<<std::dec<<" session id:"<<fsm_meta.sessionID;
				std::cout<<" seqNum:"<<fsm_meta.meta.seqNumb;
				std::cout<<" recvd:"<<rxSar.recvd;
				std::cout<<" head:"<<rxSar.head;
				std::cout<<" offset:"<<rxSar.offset;
				std::cout<<" length:"<<fsm_meta.meta.length;		
				std::cout<<" gap:"<<rxSar.gap<<std::endl;
				#endif
				// std::cout<<" free_space"<<free_space<<std::endl;
				// ack-number-derived decisions are only valid if the ACK flag is set
				rxEng2timer_clearRetransmitTimer.write(rxRetransmitTimerUpdate(fsm_meta.sessionID,
						(fsm_meta.meta.ack && fsm_meta.meta.ackNumb == txSar.nextByte)));
				rxEng2timer_resetKeepalive.write(fsm_meta.sessionID);
				// Check state and if FIN in order, Current out of order FINs are not accepted
				if ((tcpState == ESTABLISHED || tcpState == FIN_WAIT_1 || tcpState == FIN_WAIT_2) && (rxSar.recvd == fsm_meta.meta.seqNumb))
				{
					if (fsm_meta.meta.ack)
					{
						rxEng2txSar_upd_req.write((rxTxSarQuery(fsm_meta.sessionID, fsm_meta.meta.ackNumb, fsm_meta.meta.winSize, txSar.cong_window, txSar.count, txSar.fastRetransmitted))); //TODO include count check
					}

					// +1 for phantom byte, there might be data too
					rxSarRecvd finUpd(fsm_meta.sessionID, fsm_meta.meta.seqNumb+fsm_meta.meta.length+1, fsm_meta.meta.seqNumb+fsm_meta.meta.length+1, rxSar.offset, rxSar.gap); //diff to ACK
					finUpd.head2 = rxSar.head2;
					finUpd.offset2 = rxSar.offset2;
					finUpd.gap2 = rxSar.gap2;
					finUpd.tsWrite = rxSar.tsEnabled && fsm_meta.meta.tsValid && seq_geq(fsm_meta.meta.tsVal, rxSar.tsRecent);
					finUpd.tsRecent = fsm_meta.meta.tsVal;
					rxEng2rxSar_upd_req.write(finUpd);

					// Clear the probe timer
					rxEng2timer_clearProbeTimer.write(fsm_meta.sessionID);

					// Check if there is payload
					if (fsm_meta.meta.length != 0)
					{
						ap_uint<32> pkgAddr;
						pkgAddr(31, 30) = 0x0;
						pkgAddr(29, WINDOW_BITS) = fsm_meta.sessionID(13, 0);
						pkgAddr(WINDOW_BITS-1, 0) = fsm_meta.meta.seqNumb(WINDOW_BITS-1, 0);
#if !(RX_DDR_BYPASS)
						rxBufferWriteCmd.write(mmCmd(pkgAddr, fsm_meta.meta.length));
#endif
						// Tell Application new data is available and connection got closed
						rxEng2rxApp_notification.write(appNotification(fsm_meta.sessionID, fsm_meta.meta.length, fsm_meta.srcIpAddress, fsm_meta.dstIpPort, true));
						dropDataFifoOut.write(false);
					}
					else if (tcpState == ESTABLISHED)
					{
						// Tell Application connection got closed
						rxEng2rxApp_notification.write(appNotification(fsm_meta.sessionID, fsm_meta.srcIpAddress, fsm_meta.dstIpPort, true)); //CLOSE
					}

					// Update state
					if (tcpState == ESTABLISHED)
					{
						// half-close: ACK the peer's FIN and enter CLOSE_WAIT;
						// our own FIN is sent when the application closes
						rxEng2eventEng_setEvent.write(event(ACK, fsm_meta.sessionID));
						rxEng2stateTable_upd_req.write(stateQuery(fsm_meta.sessionID, CLOSE_WAIT, 1));
					}
					else //FIN_WAIT_1 || FIN_WAIT_2
					{
						if (fsm_meta.meta.ack && fsm_meta.meta.ackNumb == txSar.nextByte) //check if final FIN is ACK'd -> LAST_ACK
						{
							rxEng2stateTable_upd_req.write(stateQuery(fsm_meta.sessionID, TIME_WAIT, 1));
							rxEng2timer_setCloseTimer.write(fsm_meta.sessionID);
						}
						else
						{
							rxEng2stateTable_upd_req.write(stateQuery(fsm_meta.sessionID, CLOSING, 1));
						}
						rxEng2eventEng_setEvent.write(event(ACK, fsm_meta.sessionID));
					}
				}
				else // NOT (ESTABLISHED || FIN_WAIT_1 || FIN_WAIT_2)
				{
					rxEng2eventEng_setEvent.write(event(ACK, fsm_meta.sessionID));
					rxEng2stateTable_upd_req.write(stateQuery(fsm_meta.sessionID, tcpState, 1));
					// If there is payload we need to drop it
					if (fsm_meta.meta.length != 0)
					{
						dropDataFifoOut.write(true);
					}
				}
			}
			break;
		default: //TODO MAYBE load everthing all the time
			// stateTable is locked, make sure it is released in at the end
			// If there is an ACK we read txSar
			// We always read rxSar
			if (fsm_state == LOAD)
			{
				stateTable2rxEng_upd_rsp.read(tcpState);
				rxSar2rxEng_upd_rsp.read(rxSar); //TODO not sure nb works
				txSar2rxEng_upd_rsp.read_nb(txSar);
			}
			if (fsm_state == LOAD)
			{
				// Handle if RST
				if (fsm_meta.meta.rst)
				{
					if (tcpState == SYN_SENT) //TODO this would be a RST,ACK i think
					{
						if (fsm_meta.meta.ackNumb == txSar.nextByte) // Check if matching SYN
						{
							//tell application, could not open connection
							openConStatusOut.write(openStatus(fsm_meta.sessionID, 0, fsm_meta.srcIpAddress, fsm_meta.srcIpPort));
							rxEng2stateTable_upd_req.write(stateQuery(fsm_meta.sessionID, CLOSED, 1));
							rxEng2timer_clearRetransmitTimer.write(rxRetransmitTimerUpdate(fsm_meta.sessionID, true));
						}
						else
						{
							// Ignore since not matching
							rxEng2stateTable_upd_req.write(stateQuery(fsm_meta.sessionID, tcpState, 1));
						}
					}
					else
					{
						// Check if in window
						if (fsm_meta.meta.seqNumb == rxSar.recvd)
						{
							//tell application, RST occurred, abort
							if (tcpState != SYN_RECEIVED)
							{
								rxEng2rxApp_notification.write(appNotification(fsm_meta.sessionID, fsm_meta.srcIpAddress, fsm_meta.dstIpPort, true)); //RESET
							}
							else if (fsm_meta.dstIpPort[15] == 1)
							{
								// simultaneous open refused
								openConStatusOut.write(openStatus(fsm_meta.sessionID, 0, fsm_meta.srcIpAddress, fsm_meta.srcIpPort));
							}
							rxEng2stateTable_upd_req.write(stateQuery(fsm_meta.sessionID, CLOSED, 1)); //TODO maybe some TIME_WAIT state
							rxEng2timer_clearRetransmitTimer.write(rxRetransmitTimerUpdate(fsm_meta.sessionID, true));
						}
						else
						{
							// RFC 5961 3.2: a RST inside the receive window but not
							// exactly at RCV.NXT may be a blind injection - answer
							// with a (throttled) challenge ACK; the real peer then
							// resends its RST with the exact sequence number.
							// Outside the window it is silently dropped.
							if (seq_gt(fsm_meta.meta.seqNumb, rxSar.recvd) && seq_lt(fsm_meta.meta.seqNumb, rxSar.advEdge)
								&& tcpState != CLOSED && tcpState != SYN_RECEIVED && challengeOk)
							{
								rxEng2eventEng_setEvent.write(event(ACK_NODELAY, fsm_meta.sessionID));
								challengeSent = true;
							}
							rxEng2stateTable_upd_req.write(stateQuery(fsm_meta.sessionID, tcpState, 1));
						}
					}
				}
				else // Handle non RST bogus packages
				{
					//TODO maybe sent RST ourselves, or simply ignore
					// For now ignore, sent ACK??
					//eventsOut.write(rstEvent(mh_meta.seqNumb, 0, true));
					rxEng2stateTable_upd_req.write(stateQuery(fsm_meta.sessionID, tcpState, 1));
				} // if rst
			} // if fsm_stat
			break;
		} //switch control_bits
		break;
	} //switch state

	// challenge ACK budget: refilled once per window
	if (fsm_challengeTimer == 0)
	{
		fsm_challengeTimer = CHALLENGE_ACK_WINDOW;
		fsm_challengeCount = 0;
	}
	else
	{
		fsm_challengeTimer--;
		if (challengeSent)
		{
			fsm_challengeCount++;
		}
	}
}

/** @ingroup rx_engine
 *	Drops packets if their metadata did not match / are invalid, as indicated by @param dropBuffer
 *	@param[in]		dataIn, incoming data stream
 *	@param[in]		dropFifoIn, Drop-FIFO indicating if packet needs to be dropped
 *	@param[out]		rxBufferDataOut, outgoing data stream
 */
template <int WIDTH>
void rxPackageDropper(stream<net_axis<WIDTH> >&		dataIn,
					  stream<bool>&			dropFifoIn1,
					  stream<ap_uint<17> >&	dropFifoIn2, //[0] drop, [16:1] head bytes to trim
					  stream<net_axis<WIDTH> >&		rxBufferDataOut) {
#pragma HLS INLINE off
#pragma HLS pipeline II=1

	enum tpfStateType {READ_DROP1, READ_DROP2, FWD, DROP, SKIP, SHIFT, REMAINDER};
	static tpfStateType tpf_state = READ_DROP1;
	// head trimming: SKIP discards whole words, SHIFT realigns the rest by
	// tpf_off bytes (tpf_first: the first realigned word is still pending)
	static ap_uint<16> tpf_trim = 0;
	static ap_uint<8> tpf_off = 0;
	static bool tpf_first = true;
	static net_axis<WIDTH> tpf_prev;

	bool drop;
	ap_uint<17> drop2;

	switch (tpf_state) {
	case READ_DROP1: //Drop1
		if (!dropFifoIn1.empty())
		{
			dropFifoIn1.read(drop);
			if (drop)
			{
				tpf_state = DROP;
			}
			else
			{
				tpf_state = READ_DROP2;
			}
		}
		break;
	case READ_DROP2:
		if (!dropFifoIn2.empty())
		{
			dropFifoIn2.read(drop2);
			tpf_trim = drop2(16, 1);
			tpf_off = drop2(16, 1) % (WIDTH/8);
			tpf_first = true;
			if (drop2[0])
			{
				tpf_state = DROP;
			}
			else if (drop2(16, 1) >= (WIDTH/8))
			{
				tpf_state = SKIP;
			}
			else if (drop2(16, 1) != 0)
			{
				tpf_state = SHIFT;
			}
			else
			{
				tpf_state = FWD;
			}
		}
		break;
	case SKIP:
		// the trimmed head is shorter than the payload: never the last word
		if (!dataIn.empty())
		{
			dataIn.read();
			tpf_trim -= (WIDTH/8);
			if (tpf_trim < (WIDTH/8))
			{
				tpf_state = (tpf_off == 0) ? FWD : SHIFT;
			}
		}
		break;
	case SHIFT:
		if (!dataIn.empty())
		{
			net_axis<WIDTH> currWord = dataIn.read();
			net_axis<WIDTH> sendWord;
			if (tpf_first)
			{
				if (currWord.last)
				{
					sendWord.data = currWord.data >> (8*tpf_off);
					sendWord.keep = currWord.keep >> tpf_off;
					sendWord.last = 1;
					rxBufferDataOut.write(sendWord);
					tpf_state = READ_DROP1;
				}
				tpf_first = false;
			}
			else
			{
				sendWord.data = (tpf_prev.data >> (8*tpf_off)) | (currWord.data << (WIDTH - 8*tpf_off));
				sendWord.keep = (tpf_prev.keep >> tpf_off) | (currWord.keep << (WIDTH/8 - tpf_off));
				sendWord.last = currWord.last && ((currWord.keep >> tpf_off) == 0);
				rxBufferDataOut.write(sendWord);
				if (currWord.last)
				{
					tpf_state = sendWord.last ? READ_DROP1 : REMAINDER;
				}
			}
			tpf_prev = currWord;
		}
		break;
	case REMAINDER:
		{
			net_axis<WIDTH> sendWord;
			sendWord.data = tpf_prev.data >> (8*tpf_off);
			sendWord.keep = tpf_prev.keep >> tpf_off;
			sendWord.last = 1;
			rxBufferDataOut.write(sendWord);
			tpf_state = READ_DROP1;
		}
		break;
	case FWD:
		if(!dataIn.empty())
  		{
			net_axis<WIDTH> currWord = dataIn.read();
			if (currWord.last)
			{
				tpf_state = READ_DROP1;
			}
			rxBufferDataOut.write(currWord);
		}
		break;
	case DROP:
		if(!dataIn.empty())
 		{
			net_axis<WIDTH> currWord = dataIn.read();
			if (currWord.last)
			{
				tpf_state = READ_DROP1;
			}
		}
		break;
	} // switch
}

/** @ingroup rx_engine
 *  Delays the notifications to the application until the data is actually is written to memory
 *  @param[in]		rxWriteStatusIn, the status which we get back from the DATA MOVER it indicates if the write was successful
 *  @param[in]		internalNotificationFifoIn, incoming notifications
 *  @param[out]		notificationOut, outgoing notifications
 *  @TODO Handle unsuccessful write to memory
 */
void rxAppNotificationDelayer(	stream<mmStatus>&				rxWriteStatusIn, stream<appNotification>&		internalNotificationFifoIn,
								stream<appNotification>&		notificationOut, stream<ap_uint<1> > &doubleAccess) {
#pragma HLS INLINE off
#pragma HLS pipeline II=1

	static stream<appNotification> rand_notificationBuffer("rand_notificationBuffer");
	#pragma HLS STREAM variable=rand_notificationBuffer depth=32 //depends on memory delay
	#pragma HLS aggregate  variable=rand_notificationBuffer compact=bit

	static ap_uint<1>		rxAppNotificationDoubleAccessFlag = false;
	static ap_uint<5>		rand_fifoCount = 0;
	static mmStatus			rxAppNotificationStatus1, rxAppNotificationStatus2;
	static appNotification	rxAppNotification;

	if (rxAppNotificationDoubleAccessFlag == true) {
		if(!rxWriteStatusIn.empty()) {
			rxWriteStatusIn.read(rxAppNotificationStatus2);
			rand_fifoCount--;
			if (rxAppNotificationStatus1.okay && rxAppNotificationStatus2.okay)
				if (rxAppNotification.length != 0)
				{
					notificationOut.write(rxAppNotification);
				}
			rxAppNotificationDoubleAccessFlag = false;
		}
	}
	else if (rxAppNotificationDoubleAccessFlag == false) {
		if(!rxWriteStatusIn.empty() && !rand_notificationBuffer.empty() && !doubleAccess.empty()) {
			rxWriteStatusIn.read(rxAppNotificationStatus1);
			rand_notificationBuffer.read(rxAppNotification);
			rxAppNotificationDoubleAccessFlag = doubleAccess.read(); 	// Read the double notification flag. If one then go and w8 for the second status
			if (rxAppNotificationDoubleAccessFlag == 0) {				// if the memory access was not broken down in two for this segment
				rand_fifoCount--;
				if (rxAppNotificationStatus1.okay)
					if (rxAppNotification.length!=0)
					{
						notificationOut.write(rxAppNotification);				// Output the notification
					}
			}
			//TODO else, we are screwed since the ACK is already sent
		}
		else if (!internalNotificationFifoIn.empty() && (rand_fifoCount < 31)) {
			internalNotificationFifoIn.read(rxAppNotification);
			//if (rxAppNotification.length != 0) {
			//	rand_notificationBuffer.write(rxAppNotification);
			//	rand_fifoCount++;
			//}
			//else
			//	notificationOut.write(rxAppNotification);

			if (rxAppNotification.closed & rxAppNotification.length == 0)
			{
				notificationOut.write(rxAppNotification);
			}
			else
			{
				rand_notificationBuffer.write(rxAppNotification);
				rand_fifoCount++;
			}

		}
	}
}

void rxEventMerger(stream<extendedEvent>& in1, stream<event>& in2, stream<extendedEvent>& out)
{
	#pragma HLS PIPELINE II=1
	#pragma HLS INLINE OFF

	if (!in1.empty())
	{
		out.write(in1.read());
	}
	else if (!in2.empty())
	{
		out.write(in2.read());
	}
}

template <int WIDTH>
void rxEngMemWrite(	hls::stream<net_axis<WIDTH> >& 	dataIn,
					hls::stream<mmCmd>&				cmdIn,
					hls::stream<mmCmd>&				cmdOut,
					hls::stream<net_axis<WIDTH> >&	dataOut,
					hls::stream<ap_uint<1> >&		doubleAccess)
{
#pragma HLS pipeline II=1
#pragma HLS INLINE off

	enum fsmStateType {IDLE, IDLE_REG, CUT_FIRST, ALIGN_SECOND, FWD_ALIGNED, RESIDUE};
	static fsmStateType rxMemWrState = IDLE;
	static mmCmd cmd;
	static ap_uint<WINDOW_BITS> remainingLength = 0;
	static ap_uint<WINDOW_BITS> lengthFirstPkg;
	static ap_uint<8> offset = 0;
	static net_axis<WIDTH> prevWord;


	switch (rxMemWrState)
	{
	case IDLE:
		if (!cmdIn.empty())
		{
			cmdIn.read(cmd);
			rxMemWrState = IDLE_REG;
		}
		break;
	case IDLE_REG:
		if ((cmd.saddr(WINDOW_BITS-1, 0) + cmd.bbt) > BUFFER_SIZE)
		{
			lengthFirstPkg = BUFFER_SIZE - cmd.saddr;
			remainingLength = lengthFirstPkg;
			offset = lengthFirstPkg(DATA_KEEP_BITS - 1, 0);

			doubleAccess.write(true);
			cmdOut.write(mmCmd(cmd.saddr, lengthFirstPkg));
			rxMemWrState = CUT_FIRST;
		}
		else
		{
			doubleAccess.write(false);

			cmdOut.write(cmd);
			rxMemWrState = FWD_ALIGNED;
		}
		break;
	case CUT_FIRST:
		if (!dataIn.empty())
		{
			dataIn.read(prevWord);
			net_axis<WIDTH> sendWord = prevWord;

			if (remainingLength > (WIDTH/8))
			{
				remainingLength -= (WIDTH/8);
			}
			//This means that the second packet is aligned
			else if (remainingLength == (WIDTH/8))
			{
				sendWord.last = 1;

				cmd.saddr(WINDOW_BITS-1, 0) = 0;
				cmd.bbt -= lengthFirstPkg;
				cmdOut.write(cmd);
				rxMemWrState = FWD_ALIGNED;
			}
			else
			{
				sendWord.keep = lenToKeep(remainingLength);
				sendWord.last = 1;

				cmd.saddr(WINDOW_BITS-1, 0) = 0;
				cmd.bbt -= lengthFirstPkg;
				cmdOut.write(cmd);
				rxMemWrState = ALIGN_SECOND;
				//If only part of a word is left
				if (prevWord.last)
				{
					rxMemWrState = RESIDUE;
				}
			}

			dataOut.write(sendWord);
		}
		break;
	case FWD_ALIGNED:	// This is the non-realignment state
		if (!dataIn.empty())
		{
			net_axis<WIDTH> currWord = dataIn.read();
         // std::cout << "HELP: ";
         // printLE(std::cout, currWord);
         // std::cout << std::endl;
			dataOut.write(currWord);
			if (currWord.last)
			{
				rxMemWrState = IDLE;
			}
		}
		break;
	case ALIGN_SECOND: // We go into this state when we need to realign things
		if (!dataIn.empty())
		{
			net_axis<WIDTH> currWord = dataIn.read();
			net_axis<WIDTH> sendWord;
			sendWord.data(WIDTH-1, WIDTH - (offset*8)) = currWord.data(offset*8-1, 0);
			sendWord.data(WIDTH - (offset*8) -1, 0) = prevWord.data(WIDTH-1, offset*8);
			sendWord.keep(WIDTH/8-1, WIDTH/8 - (offset)) = currWord.keep(offset-1, 0);
			sendWord.keep(WIDTH/8 - (offset) -1, 0) = prevWord.keep(WIDTH/8-1, offset);
			sendWord.last = (currWord.keep[offset] == 0);

			dataOut.write(sendWord);
			prevWord = currWord;
			if (currWord.last)
			{
				rxMemWrState = IDLE;
				if (!sendWord.last)
				{
					rxMemWrState = RESIDUE;
				}
			}

		}
		break;
	case RESIDUE: //last word
		net_axis<WIDTH> sendWord;
#ifndef __SYNTHESIS__
		sendWord.data(WIDTH-1, WIDTH - (offset*8)) = 0;
#endif
		sendWord.data(WIDTH - (offset*8) -1, 0) = prevWord.data(WIDTH-1, offset*8);
		sendWord.keep(WIDTH/8-1, WIDTH/8 - (offset)) = 0;
		sendWord.keep(WIDTH/8 - (offset) -1, 0) = prevWord.keep(WIDTH/8-1, offset);
		sendWord.last = 1;
		dataOut.write(sendWord);
		rxMemWrState = IDLE;
		break;
	} //switch
}

/** @ingroup rx_engine
 *  The @ref rx_engine is processing the data packets on the receiving path.
 *  When a new packet enters the engine its TCP checksum is tested, afterwards the header is parsed
 *  and some more checks are done. Before it is evaluated by the main TCP state machine which triggers Events
 *  and updates the data structures depending on the packet. If the packet contains valid payload it is stored
 *  in memory and the application is notified about the new data.
 *  @param[in]		ipRxData
 *  @param[in]		sLookup2rxEng_rsp
 *  @param[in]		stateTable2rxEng_upd_rsp
 *  @param[in]		portTable2rxEng_rsp
 *  @param[in]		rxSar2rxEng_upd_rsp
 *  @param[in]		txSar2rxEng_upd_rsp
 *  @param[in]		rxBufferWriteStatus
 *
 *  @param[out]		rxBufferWriteData
 *  @param[out]		rxEng2sLookup_req
 *  @param[out]		rxEng2stateTable_upd_req
 *  @param[out]		rxEng2portTable_req
 *  @param[out]		rxEng2rxSar_upd_req
 *  @param[out]		rxEng2txSar_upd_req
 *  @param[out]		rxEng2timer_clearRetransmitTimer
 *  @param[out]		rxEng2timer_setCloseTimer
 *  @param[out]		openConStatusOut
 *  @param[out]		rxEng2eventEng_setEvent
 *  @param[out]		rxBufferWriteCmd
 *  @param[out]		rxEng2rxApp_notification
 */
template <int WIDTH>
void rx_engine(	stream<net_axis<WIDTH> >&					ipRxData,
				stream<sessionLookupReply>&			sLookup2rxEng_rsp,
				stream<sessionState>&				stateTable2rxEng_upd_rsp,
				stream<bool>&						portTable2rxEng_rsp,
				stream<rxSarEntry>&					rxSar2rxEng_upd_rsp,
				stream<rxTxSarReply>&				txSar2rxEng_upd_rsp,
#if !(RX_DDR_BYPASS)
				stream<mmStatus>&					rxBufferWriteStatus,
#endif
				stream<net_axis<WIDTH> >&					rxBufferWriteData,
				stream<sessionLookupQuery>&			rxEng2sLookup_req,
				stream<stateQuery>&					rxEng2stateTable_upd_req,
				stream<ap_uint<16> >&				rxEng2portTable_req,
				stream<rxSarRecvd>&					rxEng2rxSar_upd_req,
				stream<rxTxSarQuery>&				rxEng2txSar_upd_req,
				stream<rxRetransmitTimerUpdate>&	rxEng2timer_clearRetransmitTimer,
				stream<ap_uint<16> >&				rxEng2timer_clearProbeTimer,
				stream<ap_uint<16> >&				rxEng2timer_setProbeTimer,
				stream<ap_uint<16> >&				rxEng2timer_setCloseTimer,
				stream<openStatus>&					openConStatusOut,
				stream<extendedEvent>&				rxEng2eventEng_setEvent,
				stream<rttAck>&						rxEng2rtt_ack,
				stream<ap_uint<16> >&				rxEng2timer_resetKeepalive,
				stream<bool>&						stateTable2rxEng_synBacklogFull,
#if !(RX_DDR_BYPASS)
				stream<mmCmd>&						rxBufferWriteCmd,
				stream<appNotification>&			rxEng2rxApp_notification)
#else
				stream<appNotification>&			rxEng2rxApp_notification,
				ap_uint<16>					rxbuffer_data_count,
				ap_uint<16>					rxbuffer_max_data_count)
#endif
{
//#pragma HLS DATAFLOW
//#pragma HLS INTERFACE ap_ctrl_none port=return
#pragma HLS INLINE

	// Axi Streams
	static stream<net_axis<WIDTH> >		rxEng_dataBuffer0("rxEng_dataBuffer0");
	static stream<net_axis<WIDTH> >		rxEng_dataBuffer1("rxEng_dataBuffer1");
	static stream<net_axis<WIDTH> >		rxEng_dataBuffer2("rxEng_dataBuffer2");
	static stream<net_axis<WIDTH> >		rxEng_dataBuffer3("rxEng_dataBuffer3");
	static stream<net_axis<WIDTH> >		rxEng_dataBuffer3a("rxEng_dataBuffer3a");
	static stream<net_axis<WIDTH> >		rxEng_dataBuffer3b("rxEng_dataBuffer3b");
	#pragma HLS stream variable=rxEng_dataBuffer0 depth=8
	#pragma HLS stream variable=rxEng_dataBuffer1 depth=8
	#pragma HLS stream variable=rxEng_dataBuffer2 depth=256 //critical, tcp checksum computation
	#pragma HLS stream variable=rxEng_dataBuffer3 depth=32
	#pragma HLS stream variable=rxEng_dataBuffer3a depth=8
	#pragma HLS stream variable=rxEng_dataBuffer3b depth=8
	#pragma HLS aggregate  variable=rxEng_dataBuffer0 compact=bit
	#pragma HLS aggregate  variable=rxEng_dataBuffer1 compact=bit
	#pragma HLS aggregate  variable=rxEng_dataBuffer2 compact=bit
	#pragma HLS aggregate  variable=rxEng_dataBuffer3 compact=bit
	#pragma HLS aggregate  variable=rxEng_dataBuffer3a compact=bit
	#pragma HLS aggregate  variable=rxEng_dataBuffer3b compact=bit

	// Meta Streams/FIFOs
	//static stream<bool>					rxEng_tcpValidFifo("rx_tcpValidFifo");
	static stream<rxEngineMetaData>		rxEng_metaDataFifo("rx_metaDataFifo");
	static stream<rxFsmMetaData>		rxEng_fsmMetaDataFifo("rxEng_fsmMetaDataFifo");
	static stream<fourTuple>			rxEng_tupleBuffer("rx_tupleBuffer");
	static stream<pseudoMeta>			rxEng_ipMetaFifo("rxEng_ipMetaFifo");
	//#pragma HLS stream variable=rxEng_tcpValidFifo depth=2
	#pragma HLS stream variable=rxEng_metaDataFifo depth=2
	#pragma HLS stream variable=rxEng_fsmMetaDataFifo depth=2
	#pragma HLS stream variable=rxEng_tupleBuffer depth=2
	#pragma HLS stream variable=rxEng_ipMetaFifo depth=2
	#pragma HLS aggregate  variable=rxEng_metaDataFifo compact=bit
	#pragma HLS aggregate  variable=rxEng_fsmMetaDataFifo compact=bit
	#pragma HLS aggregate  variable=rxEng_tupleBuffer compact=bit

	static stream<extendedEvent>		rxEng_metaHandlerEventFifo("rxEng_metaHandlerEventFifo");
	static stream<event>				rxEng_fsmEventFifo("rxEng_fsmEventFifo");
	#pragma HLS stream variable=rxEng_metaHandlerEventFifo depth=2
	#pragma HLS stream variable=rxEng_fsmEventFifo depth=2
	#pragma HLS aggregate  variable=rxEng_metaHandlerEventFifo compact=bit
	#pragma HLS aggregate  variable=rxEng_fsmEventFifo compact=bit

	static stream<bool>					rxEng_metaHandlerDropFifo("rxEng_metaHandlerDropFifo");
	static stream<ap_uint<17> >			rxEng_fsmDropFifo("rxEng_fsmDropFifo");
	#pragma HLS stream variable=rxEng_metaHandlerDropFifo depth=2
	#pragma HLS stream variable=rxEng_fsmDropFifo depth=2
	#pragma HLS aggregate  variable=rxEng_metaHandlerDropFifo compact=bit
	#pragma HLS aggregate  variable=rxEng_fsmDropFifo compact=bit

	static stream<appNotification> rx_internalNotificationFifo("rx_internalNotificationFifo");
	#pragma HLS stream variable=rx_internalNotificationFifo depth=8 //This depends on the memory delay
	#pragma HLS aggregate  variable=rx_internalNotificationFifo compact=bit

	static stream<mmCmd> 					rxTcpFsm2wrAccessBreakdown("rxTcpFsm2wrAccessBreakdown");
	#pragma HLS stream variable=rxTcpFsm2wrAccessBreakdown depth=8
	#pragma HLS aggregate  variable=rxTcpFsm2wrAccessBreakdown compact=bit

	static stream<net_axis<WIDTH> > 					rxPkgDrop2rxMemWriter("rxPkgDrop2rxMemWriter");
	#pragma HLS stream variable=rxPkgDrop2rxMemWriter depth=16
	#pragma HLS aggregate  variable=rxPkgDrop2rxMemWriter compact=bit

	static stream<ap_uint<1> >				rxEngDoubleAccess("rxEngDoubleAccess");
	#pragma HLS stream variable=rxEngDoubleAccess depth=8


	//TODO move
	static hls::stream<net_axis<WIDTH> > rxEng_pseudoHeaderFifo("rxEng_pseudoHeaderFifo");
	#pragma HLS stream variable=rxEng_pseudoHeaderFifo depth=2
	static stream<net_axis<WIDTH> >		rxEng_dataBuffer4("rxEng_dataBuffer4");
	static stream<net_axis<WIDTH> >		rxEng_dataBuffer5("rxEng_dataBuffer5");
	#pragma HLS stream variable=rxEng_dataBuffer4 depth=8
	#pragma HLS stream variable=rxEng_dataBuffer5 depth=8
	static stream<ap_uint<4> > rx_process2dropLengthFifo("rx_process2dropLengthFifo");
	#pragma HLS STREAM depth=2 variable=rx_process2dropLengthFifo
	// per-packet CE bit, bypasses the checksum path (covers the packets in
	// flight up to processPseudoHeader)
	static stream<ap_uint<1> > rxEng_ipCeFifo("rxEng_ipCeFifo");
	#pragma HLS STREAM depth=32 variable=rxEng_ipCeFifo


	toe_process_ipv4<WIDTH>(ipRxData, rx_process2dropLengthFifo, rxEng_ipMetaFifo, rxEng_ipCeFifo, rxEng_dataBuffer0);
	//Assumes for WIDTH > 64 no optional fields
	drop_optional_ip_header<WIDTH>(rx_process2dropLengthFifo, rxEng_dataBuffer0, rxEng_dataBuffer4);
	//align
	lshiftWordByOctet<WIDTH, 2>(((TCP_PSEUDO_HEADER_SIZE%WIDTH)/8), rxEng_dataBuffer4, rxEng_dataBuffer5);
	//rxTcpLengthExtract(ipRxData, rxEng_dataBuffer0, rxEng_tcpLenFifo);

	constructPseudoHeader<WIDTH>(rxEng_ipMetaFifo, rxEng_pseudoHeaderFifo);
	prependPseudoHeader<WIDTH>(rxEng_pseudoHeaderFifo, rxEng_dataBuffer5, rxEng_dataBuffer1);
	//rxInsertPseudoHeader(rxEng_dataBuffer0, rxEng_tcpLenFifo, rxEng_dataBuffer1);

	/*rxCheckTCPchecksum(rxEng_dataBuffer1, rxEng_dataBuffer2, rxEng_tcpValidFifo, rxEng_metaDataFifo,
						rxEng_tupleBuffer, rxEng2portTable_req);*/

	static hls::stream<subSums<WIDTH/16> >	subSumFifo("subSumFifo");
	static hls::stream<bool> rxEng_checksumValidFifo("rxEng_checksumValidFifo");
	//static hls::stream<ap_uint<4> >			dataOffsetFifo("dataOffsetFifo");
	static hls::stream<optionalFieldsMeta>	rxEng_optionalFieldsMetaFifo("rxEng_optionalFieldsMetaFifo");
	#pragma HLS stream variable=subSumFifo depth=2
	#pragma HLS stream variable=rxEng_checksumValidFifo depth=2
	//#pragma HLS stream variable=dataOffsetFifo depth=8
	#pragma HLS stream variable=rxEng_optionalFieldsMetaFifo depth=8
	//TODO data pack
#if (WINDOW_SCALE)
	static hls::stream<rxEngineMetaData>		rxEng_headerMetaFifo("rxEng_headerMetaFifo");
	static hls::stream<ap_uint<4> >		rxEng_dataOffsetFifo("rxEng_dataOffsetFifo");
	static hls::stream<ap_uint<320> >		rxEng_optionalFieldsFifo("rxEng_optionalFieldsFifo");
	static hls::stream<ap_uint<216> >		rxEng_winScaleFifo("rxEng_winScaleFifo");
	#pragma HLS stream variable=rxEng_headerMetaFifo depth=16
	#pragma HLS stream variable=rxEng_dataOffsetFifo depth=2
	#pragma HLS stream variable=rxEng_optionalFieldsFifo depth=2
	#pragma HLS stream variable=rxEng_winScaleFifo depth=2
	//TODO data pack
#endif

	two_complement_subchecksums<WIDTH, 11>(rxEng_dataBuffer1, rxEng_dataBuffer2, subSumFifo);
	toe_check_ipv4_checksum(subSumFifo, rxEng_checksumValidFifo);
	processPseudoHeader<WIDTH>(rxEng_dataBuffer2,
								rxEng_dataBuffer3a,
								rxEng_checksumValidFifo,
								rxEng_ipCeFifo,
#if !(WINDOW_SCALE)
								rxEng_metaDataFifo,
#else
								rxEng_headerMetaFifo,
#endif
								rxEng_tupleBuffer,
								rxEng2portTable_req,
								rxEng_optionalFieldsMetaFifo);

	rshiftWordByOctet<net_axis<WIDTH>, WIDTH, 3>(((TCP_FULL_PSEUDO_HEADER_SIZE%WIDTH)/8), rxEng_dataBuffer3a, rxEng_dataBuffer3b);

	drop_optional_header_fields<WIDTH>(rxEng_optionalFieldsMetaFifo,
													rxEng_dataBuffer3b,
#if (WINDOW_SCALE)
													rxEng_dataOffsetFifo,
													rxEng_optionalFieldsFifo,
#endif
													rxEng_dataBuffer3);
	//rxTcpInvalidDropper<WIDTH>(rxEng_dataBuffer3b, rxEng_tcpValidFifo, rxEng_dataBuffer3);

#if (WINDOW_SCALE)
	parse_optional_header_fields(rxEng_dataOffsetFifo, rxEng_optionalFieldsFifo, rxEng_winScaleFifo);
	merge_header_meta(rxEng_winScaleFifo, rxEng_headerMetaFifo, rxEng_metaDataFifo);
#endif

	rxMetadataHandler(	rxEng_metaDataFifo,
						sLookup2rxEng_rsp,
						portTable2rxEng_rsp,
						rxEng_tupleBuffer,
						rxEng2sLookup_req,
						rxEng_metaHandlerEventFifo,
						rxEng_metaHandlerDropFifo,
						rxEng_fsmMetaDataFifo);

	rxTcpFSM(			rxEng_fsmMetaDataFifo,
							stateTable2rxEng_upd_rsp,
							rxSar2rxEng_upd_rsp,
							txSar2rxEng_upd_rsp,
							rxEng2stateTable_upd_req,
							rxEng2rxSar_upd_req,
							rxEng2txSar_upd_req,
							rxEng2timer_clearRetransmitTimer,
							rxEng2timer_clearProbeTimer,
							rxEng2timer_setProbeTimer,
							rxEng2timer_setCloseTimer,
							openConStatusOut,
							rxEng_fsmEventFifo,
							rxEng_fsmDropFifo,
							rxEng2rtt_ack,
							rxEng2timer_resetKeepalive,
							stateTable2rxEng_synBacklogFull,
#if !(RX_DDR_BYPASS)
							rxTcpFsm2wrAccessBreakdown,
							rx_internalNotificationFifo);
#else
							rxEng2rxApp_notification,
							rxbuffer_data_count,
							rxbuffer_max_data_count);
#endif

#if !(RX_DDR_BYPASS)
	rxPackageDropper<WIDTH>(rxEng_dataBuffer3, rxEng_metaHandlerDropFifo, rxEng_fsmDropFifo, rxPkgDrop2rxMemWriter);

	//rxEngMemWrite(rxPkgDrop2rxMemWriter, rxTcpFsm2wrAccessBreakdown, rxBufferWriteCmd, rxBufferWriteData,rxEngDoubleAccess);
	rxEngMemWrite<WIDTH>(rxPkgDrop2rxMemWriter, rxTcpFsm2wrAccessBreakdown, rxBufferWriteCmd, rxBufferWriteData,rxEngDoubleAccess);

	rxAppNotificationDelayer(rxBufferWriteStatus, rx_internalNotificationFifo, rxEng2rxApp_notification, rxEngDoubleAccess);
#else
	rxPackageDropper(rxEng_dataBuffer3, rxEng_metaHandlerDropFifo, rxEng_fsmDropFifo, rxBufferWriteData);
#endif
	rxEventMerger(rxEng_metaHandlerEventFifo, rxEng_fsmEventFifo, rxEng2eventEng_setEvent);

}

template void rx_engine<DATA_WIDTH>(	stream<net_axis<DATA_WIDTH> >&					ipRxData,
				stream<sessionLookupReply>&			sLookup2rxEng_rsp,
				stream<sessionState>&				stateTable2rxEng_upd_rsp,
				stream<bool>&						portTable2rxEng_rsp,
				stream<rxSarEntry>&					rxSar2rxEng_upd_rsp,
				stream<rxTxSarReply>&				txSar2rxEng_upd_rsp,
#if !(RX_DDR_BYPASS)
				stream<mmStatus>&					rxBufferWriteStatus,
#endif
				stream<net_axis<DATA_WIDTH> >&					rxBufferWriteData,
				stream<sessionLookupQuery>&			rxEng2sLookup_req,
				stream<stateQuery>&					rxEng2stateTable_upd_req,
				stream<ap_uint<16> >&				rxEng2portTable_req,
				stream<rxSarRecvd>&					rxEng2rxSar_upd_req,
				stream<rxTxSarQuery>&				rxEng2txSar_upd_req,
				stream<rxRetransmitTimerUpdate>&	rxEng2timer_clearRetransmitTimer,
				stream<ap_uint<16> >&				rxEng2timer_clearProbeTimer,
				stream<ap_uint<16> >&				rxEng2timer_setProbeTimer,
				stream<ap_uint<16> >&				rxEng2timer_setCloseTimer,
				stream<openStatus>&					openConStatusOut,
				stream<extendedEvent>&				rxEng2eventEng_setEvent,
				stream<rttAck>&						rxEng2rtt_ack,
				stream<ap_uint<16> >&				rxEng2timer_resetKeepalive,
				stream<bool>&						stateTable2rxEng_synBacklogFull,
#if !(RX_DDR_BYPASS)
				stream<mmCmd>&						rxBufferWriteCmd,
				stream<appNotification>&			rxEng2rxApp_notification);
#else
				stream<appNotification>&			rxEng2rxApp_notification,
				ap_uint<16>					rxbuffer_data_count,
				ap_uint<16>					rxbuffer_max_data_count);
#endif
