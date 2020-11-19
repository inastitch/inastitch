// Copyright (C) 2020 Inatech srl
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

// Local includes:
#include "inastitch/jpeg/include/RtpJpegParser.hpp"

// C includes:
#include <assert.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

// Std includes:
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <fstream>

// Ffmpeg source
#include "libav/libavcodec/jpegtables.c"
#include "libav/libavformat/rtpdec_jpeg.c"

// Helper functions to process network packets
template<class T>
T getType(const uint8_t* &data)
{
    const T d = reinterpret_cast<const T*>(data)[0];
    data += sizeof(T);
    return d;
}

uint8_t get8(const uint8_t* &data)
{
    return getType<uint8_t>(data);
}

uint16_t get16(const uint8_t* &data)
{
    const uint16_t d = getType<uint16_t>(data);
    // network to host short
    return ntohs(d);
}

uint32_t get24(const uint8_t* &data)
{
    const uint32_t d1 = getType<uint8_t>(data);
    const uint32_t d2 = getType<uint16_t>(data);

    const uint32_t d = (d1 << 16) | static_cast<uint32_t>(ntohs(d2));
    return d;
}

uint32_t get32(const uint8_t* &data)
{
    const uint32_t d = getType<uint32_t>(data);
    // network to host long
    return ntohl(d);
}

void printValue(const char * name, uint32_t value)
{
    std::cout << name << ": 0x" << std::hex << value << std::endl;
}
// End of helper functions

inastitch::jpeg::RtpJpegParser::RtpJpegParser(std::string socketBindStr, uint32_t maxJpegBufferSize)
    : m_maxJpegBufferSize(maxJpegBufferSize)
    , m_socketBuffer1( new uint8_t[socketBufferSize] )
    , m_socketBuffer2( new uint8_t[socketBufferSize] )
    , m_quantBuffer( new uint8_t[quantBufferSize] )
    , m_jpegSizeArray( new uint32_t[jpegBufferCount] )
    , m_timestampArray( new uint64_t[jpegBufferCount] )
{
    // TODO: add support for "hostname:port"
    const uint16_t socketPort = std::stoi(socketBindStr);

    // SOCK_DGRAM = UDP
    if( (m_socketFd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 )
    {
        perror("Error: socket creation failed");
        std::abort();
    }

    memset(&m_socketServAddr, 0, sizeof(m_socketServAddr));
    m_socketServAddr.sin_family = AF_INET;
    m_socketServAddr.sin_port = htons(socketPort);
    m_socketServAddr.sin_addr.s_addr = INADDR_ANY;

    if(bind(m_socketFd, (struct sockaddr *)&m_socketServAddr, sizeof(m_socketServAddr)) < 0 ) {
        perror( "Error: socket bind failed" );
        std::abort();
    }
    std::cout << "Bound socket to port " << socketPort << std::endl;

    // init array
    m_jpegBufferArray = new uint8_t*[jpegBufferCount];
    for(int i = 0; i < jpegBufferCount; i++) {
        m_jpegBufferArray[i] = new uint8_t[m_maxJpegBufferSize];
        m_jpegSizeArray[i] = 0;
        m_timestampArray[i] = 0;
    }

    m_socketThread = std::thread(&RtpJpegParser::socketThreadFunc, this);
}

void inastitch::jpeg::RtpJpegParser::socketThreadFunc()
{
    for(;;)
    {
        nextFrame();
    }
};

inastitch::jpeg::RtpJpegParser::~RtpJpegParser()
{
    delete[] m_socketBuffer1;
    delete[] m_socketBuffer2;
    delete[] m_quantBuffer;

    delete[] m_jpegSizeArray;
    delete[] m_timestampArray;

    for(int i = 0; i < jpegBufferCount; i++) {
        delete[] m_jpegBufferArray[i];
    }
    delete[] m_jpegBufferArray;
}

std::tuple<uint8_t*, uint32_t, uint64_t> inastitch::jpeg::RtpJpegParser::getFrame(uint32_t index)
{
    const auto bufferArrayIndex = (m_currentJpegBufferIndex + index) % jpegBufferCount;
    return { m_jpegBufferArray[bufferArrayIndex], m_jpegSizeArray[bufferArrayIndex], m_timestampArray[bufferArrayIndex] };
}

bool inastitch::jpeg::RtpJpegParser::decodePayload(const uint8_t *socketBuffer, uint32_t dataSize, uint8_t *jpegBuffer)
{
    if(dataSize > socketBufferSize) {
        std::cerr << "UDP: pre-allocated 'socketBuffer' is too small for "
                  << std::dec << dataSize << " bytes." << std::endl;
        std::abort();
    }

    const bool isPacketPending = !m_isFirstLoop && !m_isFirstNullFragOffsetFound;
    if(!isPacketPending)
    {
        std::memcpy(m_socketBuffer2, socketBuffer, dataSize);
        m_currentSocketBufferSize = dataSize;
    }

    const uint8_t *packetPtr = m_socketBuffer2;

    // Guessing RTP or AVTP
    // get the two first 32-bit words
    const auto packetHeader1 = get32(packetPtr);
    const auto packetHeader2 = get32(packetPtr);

    // decode according to RTP
    const auto rtpCcAndOthers    = (packetHeader1 & 0xFF000000) >> 24;
    const auto rtpPtAndM         = (packetHeader1 & 0x00FF0000) >> 16;
    const auto rtpSequenceNumber = (packetHeader1 & 0x0000FFFF);
    const auto rtpTimestamp      = packetHeader2;

    const uint8_t rtpVersion = rtpCcAndOthers >> 6;

    // decode according to AVTP/UDP
    const auto avtpEncapsulationSeqNum = packetHeader1;
    const auto avtpSubtype       = (packetHeader2 & 0xFF000000) >> 24;
    const auto avtpSVersionNV    = (packetHeader2 & 0x00FF0000) >> 16;
    const auto avtpSeqNumber     = (packetHeader2 & 0x0000FF00) >> 8;
    const auto avtpReservedU     = (packetHeader2 & 0x000000FF);

    const uint8_t avtpVersion = (0x70 & avtpSVersionNV) >> 4;

    // choose
    bool useAvtp = false;
    if( (rtpVersion != 0x02))// && (avtpSubtype == 0x03) )
    {
        useAvtp = true;
    }
    //std::cout << "useAvtp=" << useAvtp << std::endl;
    // TODO: make this more robust

    if(!useAvtp)
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
        // |                             ....                              |
        // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
        //const auto rtpCcAndOthers    = get8(packetPtr);
        //const auto rtpPtAndM         = get8(packetPtr);
        //const auto rtpSequenceNumber = get16(packetPtr);
        //const auto rtpTimestamp      = get32(packetPtr);
        const auto rtpSyncSourceId   = get32(packetPtr);

        //printValue("rtpCcAndOthers", rtpCcAndOthers);
        //printValue("rtpPtAndM", rtpPtAndM);
        //printValue("rtpSequenceNumber", rtpSequenceNumber);
        //printValue("rtpTimestamp", rtpTimestamp);
        //printValue("rtpSyncSourceId", rtpSyncSourceId);

        m_currentRtpTimestamp = rtpTimestamp;

        const uint8_t rtpCsrcCount = (0x0F & rtpCcAndOthers);
        const uint8_t rtpPayloadType = (0x7F & rtpPtAndM);
        if(rtpCsrcCount != 0) {
            std::cerr << "RTP: Only CC=0 is supported. "
                      << "Value: 0x" << std::hex << static_cast<int>(rtpCsrcCount) << std::endl;
            std::abort();
        }
        if(rtpPayloadType != 26) {
            std::cerr << "RTP: Only payloadType = 26 (JPEG) is supported. "
                      << "Value: " << std::dec << static_cast<int>(rtpPayloadType) << std::endl;
            std::abort();
        }
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
        //const auto avtpEncapsulationSeqNum = get32(packetPtr);

        //const auto avtpSubtype             = get8(packetPtr);
        //const auto avtpSVersionNV          = get8(packetPtr);
        //const auto avtpSeqNumber           = get8(packetPtr);
        //const auto avtpReservedU           = get8(packetPtr);

        //printValue("avtpSubtype", avtpSubtype);
        //printValue("avtpSVersionNV", avtpSVersionNV);
        //printValue("avtpSeqNumber", avtpSeqNumber);
        //printValue("avtpReservedU", avtpReservedU);

        const auto avtpStreamIdHigh = get32(packetPtr);
        const auto avtpStreamIdLow  = get32(packetPtr);
        // TODO: decode stream id

        const auto avtpTimestamp = get32(packetPtr);

        const auto avtpFormat         = get8(packetPtr);
        const auto avtpFormatSubtype  = get8(packetPtr);
        const auto avtpFormatReserved = get16(packetPtr);

        const auto avtpStreamDataLen  = get16(packetPtr);
        const auto avtpMEvt           = get8(packetPtr);
        const auto avtpPacketReserved = get8(packetPtr);

        m_currentRtpTimestamp = avtpTimestamp;

        if(avtpSubtype != 0x03) {
            std::cerr << "AVTP: Only Subtype=0x03 is supported (CVF, Compressed Video Format). "
                      << "Value: 0x" << std::hex << static_cast<int>(avtpSubtype) << std::endl;
            std::abort();
        }
        if(avtpFormat != 0x02) {
            std::cerr << "AVTP: Only format=0x02 is supported (RFC payload type). "
                      << "Value: 0x" << std::hex << static_cast<int>(avtpFormat) << std::endl;
            std::abort();
        }
        if(avtpFormatSubtype != 0x00) {
            std::cerr << "AVTP: Only format=0x00 is supported (MJPEG). "
                      << "Value: 0x" << std::hex << static_cast<int>(avtpFormatSubtype) << std::endl;
            std::abort();
        }
    }

    // RTP/JPEG is type 26 (0x)
    // https://www.iana.org/assignments/rtp-parameters/rtp-parameters.xhtml

    // RTP/JPEG (RFC2435) https://tools.ietf.org/html/rfc2435
    //  0                   1                   2                   3
    //  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
    // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    // | Type-specific |              Fragment Offset                  |
    // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    // |      Type     |       Q       |     Width     |     Height    |
    // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    const auto rtpJpegTypeSpecific   = get8(packetPtr);
    const auto rtpJpegFragmentOffset = get24(packetPtr);
    const auto rtpJpegType           = get8(packetPtr);
    const auto rtpJpegQ              = get8(packetPtr);
    const auto rtpJpegWidth8         = get8(packetPtr);
    const auto rtpJpegHeight8        = get8(packetPtr);

    //printValue("rtpJpegTypeSpecific", rtpJpegTypeSpecific);
    //printValue("rtpJpegFragmentOffset", rtpJpegFragmentOffset);
    //printValue("rtpJpegType", rtpJpegType);
    //printValue("rtpJpegQ", rtpJpegQ);
    //printValue("rtpJpegWidth8", rtpJpegWidth8);
    //printValue("rtpJpegHeight8", rtpJpegHeight8);

    // Restart Marker header
    //  0                   1                   2                   3
    //  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
    // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    // |       Restart Interval        |F|L|       Restart Count       |
    // +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

    uint16_t rtpJpegRestartInterval   = 0;
    uint16_t rtpJpegRestartCountAndFL = 0;
    if(rtpJpegType >= 64 && rtpJpegType < 128)
    {
        rtpJpegRestartInterval   = get16(packetPtr);
        rtpJpegRestartCountAndFL = get16(packetPtr);

        //printValue("rtpJpegRestartInterval", rtpJpegRestartInterval);
        //printValue("rtpJpegRestartCountAndFL", rtpJpegRestartCountAndFL);
    }

    // TODO: state-machine
    if(m_isFirstNullFragOffsetFound && (rtpJpegFragmentOffset == 0))
    {
        // append EOI marker
        jpegBuffer[m_currentJpegBufferOffset] = 0xFF;
        jpegBuffer[m_currentJpegBufferOffset+1] = EOI;
        m_currentJpegBufferOffset += 2;

        // Found payload of the next frame
        return true;
    }
    // Note: frame header of the next frame was found and will stay in the buffer
    //       to be processed in next method call.
    if(!m_isFirstNullFragOffsetFound && (rtpJpegFragmentOffset != 0))
    {
        // skip payload
        return false;
    }
    // Note: skip this payload, waiting for a frame header payload
    if(!m_isFirstNullFragOffsetFound && (rtpJpegFragmentOffset == 0))
    {
        m_isFirstNullFragOffsetFound = true;
    }
    // Note: frame header found

    // Special case of the first frame
    if(rtpJpegFragmentOffset == 0)
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
        const auto rtpJpegQtMbz       = get8(packetPtr);
        const auto rtpJpegQtPrecision = get8(packetPtr);
        const auto rtpJpegQtLength    = get16(packetPtr);

        //printValue("rtpJpegQtMbz", rtpJpegQtMbz);
        //printValue("rtpJpegQtPrecision", rtpJpegQtPrecision);
        //printValue("rtpJpegQtLength", rtpJpegQtLength);

        if(rtpJpegQ != 255) {
            std::cerr << "RTP/JPEG: Only Q=0xFF (i.e., embdded quantization table) is supported. "
                      << "Value: " << std::hex << static_cast<int>(rtpJpegQ) << std::endl;
            std::abort();
        }

        if(rtpJpegQtPrecision != 0) {
            std::cerr << "RTP/JPEG: Only precision=0 (i.e., 8-bit precisison) is supported. "
                      << "Value: " << std::dec << static_cast<int>(rtpJpegQtPrecision) << std::endl;
            std::abort();
        }

        for(uint32_t rtpJpegQtIdx=0; rtpJpegQtIdx<rtpJpegQtLength; rtpJpegQtIdx++)
        {
            const auto rtpJpegQt = get8(packetPtr);
            m_quantBuffer[rtpJpegQtIdx] = rtpJpegQt;
            //std::cout << "rtpJpegQt[" << std::dec << rtpJpegQtIdx << "] = 0x" << std::hex << static_cast<uint32_t>(rtpJpegQt) << std::endl;
        }

        const uint8_t  jpegType    = (0x3F & rtpJpegType); // remove the restart marker flag
        const uint8_t  jpegQtCount = rtpJpegQtLength/64;
        const uint32_t jpegWidth   = rtpJpegWidth8 * 8;
        const uint32_t jpegHeight  = rtpJpegHeight8 * 8;

        //printValue("jpegType", jpegType);
        //printValue("jpegQtCount", jpegQtCount);
        //std::cout << "jpegSize: " << std::dec << jpegWidth << "x" << jpegHeight << std::endl;

        if(jpegQtCount > maxQuantTableCount) {
            std::cerr << "RTP/JPEG: pre-allocated 'quantBuffer' is too small for " << jpegQtCount << "quantization tables" << std::endl;
            std::abort();
        }

        //if(jpegWidth * jpegHeight * 3 > rgbBufferSize) {
        //    std::cerr << "RTP/JPEG: pre-allocated 'rgbBuffer' is too small for " << jpegWidth << "x" << jpegHeight << "frame" << std::endl;
        //    std::abort();
        //}

        // from FFMPEG
        const uint32_t jpegHdrLen = jpeg_create_header(
            jpegBuffer, m_maxJpegBufferSize,
            jpegType,
            rtpJpegWidth8, rtpJpegHeight8,
            m_quantBuffer, jpegQtCount,
            rtpJpegRestartInterval
        );
        //std::cout << "Rebuild JPEG header: " << std::endl;
        //printValue("jpegType", jpegType);
        //printValue("rtpJpegWidth8", rtpJpegWidth8);
        //printValue("rtpJpegHeight8", rtpJpegHeight8);
        //printValue("jpegQtCount", jpegQtCount);
        //printValue("rtpJpegRestartInterval", rtpJpegRestartInterval);

        m_currentJpegBufferOffset += jpegHdrLen;
    }

    const uint32_t headerLength = reinterpret_cast<const char*>(packetPtr) - reinterpret_cast<const char*>(m_socketBuffer2);
    const uint32_t dataLength = m_currentSocketBufferSize - headerLength;
    //std::cout << "dataCopy: " << std::dec << dataLength << " bytes of " << dataSize << std::endl;
    for(uint32_t i=0; i<dataLength; i++)
    {
        const auto dataOffset = m_currentJpegBufferOffset + i;
        assert(dataOffset < m_maxJpegBufferSize);
        jpegBuffer[m_currentJpegBufferOffset + i] = get8(packetPtr);
    }
    m_currentJpegBufferOffset += dataLength;

    if(isPacketPending)
    {
        return decodePayload(socketBuffer, dataSize, jpegBuffer);
    }

    return false;
}

void inastitch::jpeg::RtpJpegParser::nextFrame()
{
    const auto nextJpegBufferIdx = (m_currentJpegBufferIndex == 0) ? jpegBufferCount - 1 : m_currentJpegBufferIndex - 1;

    for(;;)
    {
        socklen_t len;
        int recvLen = recvfrom(m_socketFd, (char *)m_socketBuffer1, socketBufferSize,
                               MSG_WAITALL, (struct sockaddr *) &m_socketServAddr, &len);

        const bool isJpegDataComplete = decodePayload(m_socketBuffer1, recvLen, m_jpegBufferArray[nextJpegBufferIdx]);
        if(isJpegDataComplete) {
            break;
        }
    }
    m_jpegSizeArray[nextJpegBufferIdx] = m_currentJpegBufferOffset;
    m_timestampArray[nextJpegBufferIdx] = m_currentRtpTimestamp;

    // reset
    m_isFirstLoop = false;
    m_currentJpegBufferOffset = 0;
    m_isFirstNullFragOffsetFound = false;

    // at last
    m_currentJpegBufferIndex = nextJpegBufferIdx;
}
