// Copyright (C) 2020 Inatech srl
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

// Local includes:
#include "inastitch/jpeg/include/MjpegParser.hpp"

// Boost includes:
#include <boost/asio/post.hpp>

// Std includes:
#include <iostream>

inastitch::jpeg::MjpegParser::MjpegParser(std::string filename, uint32_t maxJpegBufferSize)
    : m_maxJpegBufferSize(maxJpegBufferSize)
    , m_threadPool(1)
    , m_jpegSizeArray( new uint32_t[jpegBufferCount] )
{
    m_mjpegFile = std::ifstream(filename, std::ios::binary);
    std::cout << "Opened MJPEG at " << filename << std::endl;

    // init array
    m_jpegBufferArray = new uint8_t*[jpegBufferCount];
    for(uint32_t i=0; i < jpegBufferCount; i++) {
        m_jpegBufferArray[i] = new uint8_t[m_maxJpegBufferSize];
        m_jpegSizeArray[i] = 0;
    }

    // fill array
    for(uint32_t i=0; i < jpegBufferCount; i++) {
        nextFrame();
    }
}

inastitch::jpeg::MjpegParser::~MjpegParser()
{
    delete[] m_jpegSizeArray;

    for(uint32_t i = 0; i < jpegBufferCount; i++) {
        delete[] m_jpegBufferArray[i];
    }

    m_mjpegFile.close();
}

bool inastitch::jpeg::MjpegParser::nextByte(uint8_t &byte)
{
    if( m_mjpegFile.read(reinterpret_cast<char*>(&byte), 1) )
    {
        return true;
    }
    return false;
}

uint32_t inastitch::jpeg::MjpegParser::parseJpeg(uint8_t* const jpegBuffer)
{
    // MJPEG mini parser
    // - 0xFFD8: start of image
    // - 0xFFD9: end of image
    // See: https://stackoverflow.com/a/4614629

    uint32_t jpegBufferOffset = 0;
    uint32_t pendingDataByteCount = 0;
    uint8_t byte1 = 0x00, byte2 = 0x00;

    // parser loop, byte by byte
    // TODO: add check on jpegBufferSize
    while( nextByte(byte2) )
    {
        // start marker
        if( (pendingDataByteCount == 0) && (byte1 == 0xFF) && (byte2 == 0xD8) )
        {
            jpegBufferOffset = 0;
            jpegBuffer[jpegBufferOffset++] = byte1;
            jpegBuffer[jpegBufferOffset++] = byte2;

            m_startMarkerCount++;
        }
        else
        // end marker
        if( (pendingDataByteCount == 0) && (byte1 == 0xFF) && (byte2 == 0xD9) )
        {
            jpegBuffer[jpegBufferOffset++] = byte2;

            // presentation timestamp (PTS)
            //ptsFile >> absTime >> relTime >> offTime;

            // End of JPEG frame, break the parser loop
            break;
        }
        else
        // bit stuffing
        if( (pendingDataByteCount == 0) && (byte1 == 0xFF) && (byte2 == 0x00) )
        {
            jpegBuffer[jpegBufferOffset++] = byte2;
        }
        else
        // marker with length
        if( (pendingDataByteCount == 0) &&
            (byte1 == 0xFF) ) //&& (b2 != 0x00) && (b2 != 0x01) && !( (b2 >= 0xD0) && (b2 <= 0xD9) ) )
        {
            //const uint32_t m1 = byte1, m2 = byte2;

            if(!(byte2 == 0xC0 ||
                 byte2 == 0xC2 ||
                 byte2 == 0xC4 ||
                 byte2 == 0xDB ||
                 byte2 == 0xDA ||
                 byte2  & 0xE0 ||
                 byte2 == 0xFE   ))
            {
                // Unexpected marker
                std::abort();
            }
            jpegBuffer[jpegBufferOffset++] = byte2;

            // get 16-bit length
            nextByte(byte1);
            nextByte(byte2);

            // big endian length
            pendingDataByteCount += static_cast<uint16_t>(byte1) << 8;
            pendingDataByteCount += byte2;
            //std::cout << std::hex << m1 << m2 << " " << std::dec << pendingData << std::endl;

            jpegBuffer[jpegBufferOffset++] = byte1;
            jpegBuffer[jpegBufferOffset++] = byte2;
            pendingDataByteCount -= 1;
        }
        // inside marker data (pendingDataByteCount > 0)
        else
        {
            jpegBuffer[jpegBufferOffset++] = byte2;
            if(pendingDataByteCount > 0)
                pendingDataByteCount--;
        }

        byte1 = byte2;
    }

    // jpegBufferOffset == jpegBufferSize
    return jpegBufferOffset;
}

void inastitch::jpeg::MjpegParser::nextFrame()
{
    const auto nextJpegBufferIdx = (m_currentJpegBufferIndex == 0) ? jpegBufferCount - 1 : m_currentJpegBufferIndex - 1;

    // read one jpeg from MJPEG file
    m_jpegSizeArray[nextJpegBufferIdx] = parseJpeg(m_jpegBufferArray[nextJpegBufferIdx]);

    // at last
    m_currentJpegBufferIndex = nextJpegBufferIdx;
}

std::tuple<uint8_t*, uint32_t> inastitch::jpeg::MjpegParser::getFrame(uint32_t index)
{
    boost::asio::post(m_threadPool,
        [&]
        {
            nextFrame();
        }
    );

    const auto bufferArrayIndex = m_currentJpegBufferIndex + index;
    return { m_jpegBufferArray[bufferArrayIndex], m_jpegSizeArray[bufferArrayIndex] };
}
