// Copyright (C) 2020 Inatech srl
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

// C includes:
#include <netinet/in.h>

// Std includes:
#include <string>
#include <thread>

namespace inastitch {
namespace jpeg {

/// @brief Hybrid RTP/AVTP parser for JPEG frames
///
/// RTP/JPEG and AVTP/JPEG over UDP are two very similar protocols. They can be processed by the
/// same function. Please note that standard AVTP is usually transmitted over raw Ethernet (instead
/// of well-known IP). AVTP over UDP is officially specified in the standard, but less common.
///
/// @note This class uses standard sockets for network access. Raw Ethernet would require a
/// completely different implementation (e.g., libpcap).
///
/// @see Have a look at the implementation of @ref decodePayload for references about the
/// specification.
class RtpJpegParser
{  
public:
    RtpJpegParser(std::string socketBindStr, uint32_t maxJpegBufferSize);
    ~RtpJpegParser();

public:
    std::tuple<uint8_t*, uint32_t, uint64_t> getFrame(uint32_t index);

private:
    void nextFrame();
    void socketThreadFunc();
    bool decodePayload(const uint8_t *socketBuffer, uint32_t dataSize, uint8_t *jpegBuffer);

private:
    static const auto jpegBufferCount = 10;
    static const auto maxQuantTableCount = 2;
    static const auto quantBufferSize = 64 * maxQuantTableCount;
    static const auto socketBufferSize = 65535;
    // TODO: remove hardcoded socket value,
    //       and set socket buffer size from current system value

private:
    const uint32_t m_maxJpegBufferSize;
    
private:
    bool m_isFirstLoop = true;
    bool m_isFirstNullFragOffsetFound = false;
    uint32_t m_currentSocketBufferSize = 0;
    uint32_t m_currentJpegBufferOffset = 0;
    uint32_t m_currentJpegBufferIndex = 0;
    uint32_t m_currentRtpTimestamp = 0;
    
private:
    uint8_t* const m_socketBuffer1;
    uint8_t* const m_socketBuffer2;
    uint8_t* const m_quantBuffer;

private:
    int m_socketFd;
    struct sockaddr_in m_socketServAddr;
    std::thread m_socketThread;
    
private:
    uint8_t** m_jpegBufferArray;
    uint32_t* const m_jpegSizeArray;
    uint64_t* const m_timestampArray;
};


} // namespace jpeg
} // namespace inastitch
