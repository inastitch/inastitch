// Copyright (C) 2020 Inatech srl
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

// Local includes:
#include "inastitch/jpeg/include/RtpJpegEncoder.hpp"
#include "libavcodec/mjpeg.h"

// C includes:
#include <netinet/in.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>

// Std includes:
#include <cstring>
#include <tuple>
#include <iostream>


uint16_t swap16(const uint16_t x)
{
    const uint16_t h = (0x00FF & x) << 8;
    const uint16_t l = (0xFF00 & x) >> 8;

    return h | l;
}

// Helper functions to process network packets
template<class T>
void putType(uint8_t* &data, T x)
{
    reinterpret_cast<T*>(data)[0] = x;
    data += sizeof(T);
}

void put8(uint8_t* &data, uint8_t x)
{
    putType<uint8_t>(data, x);
}

void put16(uint8_t* &data, uint16_t x)
{
    //  htons: host to network short
    putType<uint16_t>(data, htons(x));
}

void put24(uint8_t* &data, uint32_t x)
{
    const uint16_t low16 = (0x0000FFFF & x);
    const uint8_t  high8 = (0x00FF0000 & x) >> 16;

    putType<uint8_t>(data, high8);
    putType<uint16_t>(data, htons(low16));
}

void put32(uint8_t* &data, uint32_t x)
{
    putType<uint32_t>(data, htonl(x));
}


inastitch::jpeg::RtpJpegEncoder::RtpJpegEncoder(std::string socketAddrStr, uint16_t socketPort, bool isMjpegType1)
        : m_isMjpegType1(isMjpegType1)
        , m_socketBuffer( new uint8_t[socketBufferSize])
{
    // Create UDP socket file descriptor
    if( (m_socketFd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 )
    {
        perror("socket creation failed");
        std::abort();
    }

    memset(&m_socketClientAddr, 0, sizeof(m_socketClientAddr));
    m_socketClientAddr.sin_family      = AF_INET; // IPv4
    m_socketClientAddr.sin_addr.s_addr = inet_addr(socketAddrStr.c_str());
    m_socketClientAddr.sin_port        = htons(socketPort);
}

inastitch::jpeg::RtpJpegEncoder::~RtpJpegEncoder()
{
    close(m_socketFd);
    delete[] m_socketBuffer;
}

void inastitch::jpeg::RtpJpegEncoder::sendFrame(
        uint8_t* jpegData, uint32_t jpegDataSize,
        uint16_t jpegWidth, uint16_t jpegHeight,
        uint32_t timestamp, bool isAvtpMode)
{
    // width/height is in block (8x8), not in pixel
    // => divide by 8
    const auto width8  = (jpegWidth + 7) >> 3;
    const auto height8 = (jpegHeight + 7) >> 3;
    //std::cout << "width8: " << std::dec << width8 << ", height8: " << height8 << std::endl;

    // read the JPEG header to find the quantization table
    // See: https://en.wikipedia.org/wiki/JPEG_File_Interchange_Format#File_format_structure
    uint32_t qtableIdx = 0;
    uint32_t qtableCount[2] = { 0, 0 };
    uint8_t* qtablePtr[2] = { nullptr, nullptr };
    uint32_t i = 0;
    for(i=0; i<jpegDataSize; i++)
    {
        // start of a marker
        if(jpegData[i] != 0xFF) {
            continue;
        }
        //std::cout << std::hex << static_cast<int>(jpegData[i]) << static_cast<int>(jpegData[i+1]) << std::endl;
        if(jpegData[i + 1] == APP0)
        {
            const uint16_t segmentLen = swap16(*reinterpret_cast<uint16_t*>(&jpegData[i + 2]));
            //std::cout << "APP0" << segmentLen << std::endl;

            const auto thumbWidth = jpegData[i + 16];
            const auto thumbHeigth = jpegData[i + 17];

            //std::cout << "thumbnail:" << static_cast<int>(thumbWidth)
            //         << "x" << static_cast<int>(thumbHeigth) << std::endl;
        }

        if(jpegData[i + 1] == DQT)
        {
            const auto precision = jpegData[i + 4];
            if(precision != 0x00) {
                //std::cerr << "Only 8-bit precision is supported"
                //          << ", byte=" << std::hex << static_cast<int>(precision) << std::endl;
                //std::abort();
            }

            const uint16_t segmentLen = swap16(*reinterpret_cast<uint16_t*>(&jpegData[i + 2]));
            //std::cout << "DQT" << segmentLen << std::endl;

            // a quantization table is 64 bytes long + precision byte
            qtableCount[qtableIdx] = (segmentLen-2) / 65;
            if(i + 5 + qtableCount[qtableIdx] * 65 > jpegDataSize) {
                std::cerr << "Too short JPEG header" << std::endl;
                return;
            }
            // keep pointer to qtables
            qtablePtr[qtableIdx] = &jpegData[i + 4]; // start after the precision byte
            // Note: there is no need to convert to "network byte order"
            //       because each value is a single byte.
            qtableIdx++;
        }
        else if(jpegData[i + 1] == SOF0)
        {
            if(jpegData[i + 14] != 17 || jpegData[i + 17] != 17) {
                std::cerr << "Only 1x1 chroma blocks are supported" << std::endl;
                return;
            }
        }
        else if(jpegData[i + 1] == SOS)
        {
            // SOS is last marker in the header
            const uint16_t sosLen = swap16(*reinterpret_cast<uint16_t*>(&jpegData[i + 2]));
            i += sosLen + 2;
            break;
        }
    }
    //std::cout << "qtableIdx:" << qtableIdx << std::endl;

    // skip JPEG header
    auto *jpegDataPtr = jpegData;
    uint64_t jpegBufferRemainingSize = jpegDataSize;
    jpegDataPtr += i;
    jpegBufferRemainingSize -= i;

    for(uint32_t i = jpegBufferRemainingSize - 2; i >= 0; i--)
    {
        if(jpegDataPtr[i] == 0xff && jpegDataPtr[i + 1] == EOI)
        {
            // Remove the EOI marker
            jpegBufferRemainingSize = i;
            break;
        }
    }

    //std::cout << "jpegDataSize: " << jpegBufferRemainingSize << "/" << jpegDataSize << " (" << i << ")" << std::endl;

    uint32_t offset = 0;
    while(jpegBufferRemainingSize > 0)
    {
        uint8_t *packetPtr = m_socketBuffer;

        uint32_t rtpHdrSize;
        if(!isAvtpMode)
        {
            rtpHdrSize = 3 * sizeof(uint32_t); // RTP header
        }
        else
        {
            rtpHdrSize = 7 * sizeof(uint32_t); // AVTP header
        }

        uint32_t jpegHdrSize = 2 * sizeof(uint32_t); // MJPEG header
        if( (offset == 0) && (qtableCount[0] + qtableCount[1] > 0) )
        {
            jpegHdrSize += (1 * sizeof(uint32_t)) + (64 * (qtableCount[0] + qtableCount[1]));
        }

        // payload max in one packet
        const uint64_t maxPayloadSize = socketBufferSize;
        const auto len = std::min(jpegBufferRemainingSize, maxPayloadSize - (rtpHdrSize+jpegHdrSize));
        const bool isLastPacketInFrame = (len == jpegBufferRemainingSize);

        const auto lenWithJpegHdr = len + jpegHdrSize; // only for AVTP "stream_data_length"

        if(!isAvtpMode)
        {
            // RTP (RFC3550) https://tools.ietf.org/html/rfc3550
            //  0                   1                   2                   3
            //  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
            // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
            // |V=2|P|X|  CC   |M|     PT      |       sequence number         |
            // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
            // |                           timestamp                           |
            // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
            // |           synchronization source (SSRC) identifier            |
            // +=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+
            // |            contributing source (CSRC) identifiers             |
            // |                        (not used here)                        |
            // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
            put8(packetPtr, 0x80); // RTP v2

            const auto rtpPayloadType = (isLastPacketInFrame ? 0x80 : 0x00 ) | 0x1a;
            //std::cout << std::hex << static_cast<int>(rtpPayloadType) << std::endl;
            put8(packetPtr, rtpPayloadType);
            // JPEG = 26 (0x1a)
            // See: https://www.iana.org/assignments/rtp-parameters/rtp-parameters.xhtml
            // Note: don't forget the marker for last packet in a frame

            put16(packetPtr, m_rtpSequenceNum++);
            // Note: should not start at zero

            put32(packetPtr, timestamp);
            // Note: see video sampling rate. Usually 90kHz (~=0.01ms)
            // Note: "Several consecutive RTP packets will have equal
            //        timestamps if they are (logically) generated at once, e.g., belong
            //        to the same video frame.""

            put32(packetPtr, 0x12345678);
            // TODO: synchronization source
        }
        else
        {
            // AVTP (IEEE Std 1722-2016)
            // + AVTP/UDP ("Annex J")
            // + AVTP/CVF ("Compressed Video Format")
            // + AVTP/MJPEG
            //  0                   1                   2                   3
            //  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
            // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ ---
            // |                  encapsulation_sequence_num                   | AVTP/UDP specific
            // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ ---
            // |    subtype    |S| vers|N|rsv|V|   seq_number  | reserved    |U| AVTP Subtype data
            // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ ---
            // |                           stream_id                           | AVTP Stream ID
            // |                                                               |
            // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ ---
            // |                         avtp_timestamp                        | AVTP Time
            // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ ---
            // |     format    | format_subtype|           reserved            | AVTP Format info
            // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ ---
            // |      stream_data_length       | rsv |M|  evt  |   reserved    | AVTP Packet info
            // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ ---

            // AVTP/UDP
            const auto avtpEncapsulationSeqNum = 0x00000000;
            // TODO: remove dummy encapsulation sequence

            put32(packetPtr, avtpEncapsulationSeqNum);

            // AVTP Subtype data
            const auto avtpSubtype = 0x03; // CVF = Compressed Video Format
            const auto avtpS = 0x01; // Stream Valid (SV)
            // 0x01 = stream_id is filled with a valid id
            // 0x00 = otherwise

            const auto avtpVersion = 0x00;
            const auto avtpN = 0x00; // Media Clock Restart (MR)

            const auto avtpV = 0x01; // Timestamp Valid (TV)
            // 0x01 = avtp_timestamp is filled with a valid number
            // 0x00 = otherwise (avtp_timestamp has an undefined value to be discarded)

            // Note: avtp_timestamp is only filled for the final MJPEG AVTP PDU of a video frame.
            // All other fragment packets have tv=0 (=invalid timestamp)

            const auto avtpSeqNumber = m_rtpSequenceNum++;
            // Note: should not start at zero

            const auto avtpU = 0x00; // Timestamp Uncertain (TU)
            // 0x00 = the timestamp is sync'd with gPTP
            // 0x01 = otherwise

            put8(packetPtr, avtpSubtype);
            const uint8_t avtpSVersionNV = (avtpS << 7) | (avtpVersion << 4) | (avtpN << 3) | (avtpV);
            put8(packetPtr, avtpSVersionNV);
            put8(packetPtr, avtpSeqNumber);
            put8(packetPtr, avtpU);

            // AVTP Stream id
            const auto avtpStreamId32_high = 0xaabbccdd;
            const auto avtpStreamId32_low  = 0xeeff0000;
            // First 6 bytes = MAC address in EUI 48 format
            // Last  2 bytes = stream unique id
            // TODO: remove dummy stream id

            put32(packetPtr, avtpStreamId32_high);
            put32(packetPtr, avtpStreamId32_low);
            // TODO: add put64()?

            // AVTP timestamp
            const auto avtpTimestamp = timestamp;
            // TODO: this is wrong!
            // AVTP timestamp should be given by a specific formula derived from gPTP in nanoseconds.

            put32(packetPtr, avtpTimestamp);

            // AVTP Format info
            const auto avtpFormat = 0x02; // RFC payload type
            const auto avtpFormatSubtype = 0x00; // MJPEG (RFC 2435)
            const auto avtpFormatReserved = 0x0000;

            put8(packetPtr, avtpFormat);
            put8(packetPtr, avtpFormatSubtype);
            put16(packetPtr, avtpFormatReserved);

            // AVTP Packet info
            const auto avtpStreamDataLen = lenWithJpegHdr;
            const auto avtpM = (isLastPacketInFrame ? 0x01 : 0x00 );
            const auto avtpEvt = 0x00;
            const auto avtpPacketReserved = 0x00;

            put16(packetPtr, avtpStreamDataLen);
            const uint8_t avtpMEvt = (avtpM << 4) | avtpEvt;
            put8(packetPtr, avtpMEvt);
            put8(packetPtr, avtpPacketReserved);
        }

        // RTP/JPEG (RFC2435) https://tools.ietf.org/html/rfc2435
        //  0                   1                   2                   3
        //  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
        // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
        // | Type-specific |              Fragment Offset                  |
        // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
        // |      Type     |       Q       |     Width     |     Height    |
        // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
        put8(packetPtr, 0x00);
        put24(packetPtr, offset);
        put8(packetPtr, m_isMjpegType1 ? 0x01 : 0x0); // type 0 or 1, see JPEG compression settings (TJSAMP_422 or TJSAMP_420)
        put8(packetPtr, 0xFF); // embedded quantization table
        put8(packetPtr, width8);
        put8(packetPtr, height8);

        // No restart marker header

        if( (offset == 0) && (qtableCount[0] + qtableCount[1] > 0) )
        {
            // Quantization Table header
            //  0                   1                   2                   3
            //  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
            // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
            // |      MBZ      |   Precision   |             Length            |
            // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
            // |                    Quantization Table Data                    |
            // |                              ...                              |
            // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
            put8(packetPtr, 0x00);
            put8(packetPtr, 0x00); // 8-bit precision
            const auto totalQtableCount = qtableCount[0] + qtableCount[1];
            put16(packetPtr, totalQtableCount * 64);

            // each DQT segment
            for(uint32_t k=0; k<2; k++) {
                // each qtable
                for (uint32_t j=0; j<qtableCount[k]; j++) {
                    // each coefficient (skip precision byte)
                    for(uint32_t i=1; i < 65; i++) {
                        put8(packetPtr, qtablePtr[k][i+(j*65)]);
                    }
                }
            }
        }

        // copy payload data
        for(uint32_t i=0; i<len; i++)
        {
            put8(packetPtr, jpegDataPtr[i]);
        }

        const uint32_t socketDataLen = reinterpret_cast<const char*>(packetPtr) - reinterpret_cast<const char*>(m_socketBuffer);
        //std::cout << "Packet: " << socketDataLen << ", Payload: " << len << std::endl;
        int addrLen = sizeof(m_socketClientAddr);
        sendto(m_socketFd, (const char *)m_socketBuffer, socketDataLen,
               0, (const struct sockaddr *) &m_socketClientAddr, addrLen);

        jpegDataPtr             += len;
        jpegBufferRemainingSize -= len;
        offset                  += len;
    }
}
