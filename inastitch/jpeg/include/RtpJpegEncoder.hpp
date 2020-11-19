// Copyright (C) 2020 Inatech srl
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.


// C includes:
#include <netinet/in.h>

// Std includes:
#include <string>

namespace inastitch {
namespace jpeg {

/// @brief Hybrid RTP/AVTP encoder for JPEG frames
///
/// RTP/JPEG and AVTP/JPEG over UDP are two very similar protocols. They can be encoded by the
/// same function. Please note that standard AVTP is usually transmitted over raw Ethernet (instead
/// of well-known IP). AVTP over UDP is officially specified in the standard, but less common.
///
/// @note This class uses standard sockets for network access. Raw Ethernet would require a
/// completely different implementation (e.g., libpcap).
///
/// @see Have a look at the implementation of @ref sendFrame for references about the specification.
class RtpJpegEncoder
{
public:
    RtpJpegEncoder(std::string socketAddrStr, uint16_t socketPort, bool isMjpegType1);
    ~RtpJpegEncoder();

public:
    void sendFrame(uint8_t* jpegData, uint32_t jpegDataSize,
                   uint16_t jpegWidth, uint16_t jpegHeight,
                   uint32_t timestamp, bool isAvtpMode = false);

private:
     static const uint32_t socketBufferSize = 1400;
     // TODO: remove hardcoded socket value,
     //       and set socket buffer size from current system value

private:
    const bool m_isMjpegType1;

private:
    uint16_t m_rtpSequenceNum = 0;
    uint8_t* const m_socketBuffer;

private:
    int m_socketFd;
    struct sockaddr_in m_socketClientAddr;
};

} // namespace jpeg
} // namespace inastitch
