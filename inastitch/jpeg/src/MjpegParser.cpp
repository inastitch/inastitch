// Copyright (C) 2020 Inatech srl
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

// Local includes:
#include "inastitch/jpeg/include/MjpegParser.hpp"

// Std includes:
#include <iostream>

inastitch::jpeg::MjpegParser::MjpegParser(std::string filename)
{
    m_mjpegFile = std::ifstream(filename, std::ios::binary);
    std::cout << "Opened MJPEG at " << filename << std::endl;

    const auto ptsFilename = filename + ".pts";
    m_ptsFile = std::ifstream(ptsFilename);
    std::cout << "Opened PTS at" << ptsFilename << std::endl;
}

inastitch::jpeg::MjpegParser::~MjpegParser()
{
    m_mjpegFile.close();
    m_ptsFile.close();
}

bool inastitch::jpeg::MjpegParser::nextByte(uint8_t &byte)
{
    if( m_mjpegFile.read(reinterpret_cast<char*>(&byte), 1) )
    {
        return true;
    }
    return false;
}

std::tuple<uint32_t, uint64_t> inastitch::jpeg::MjpegParser::parseFrame(uint8_t* const jpegBuffer)
{
    // MJPEG mini parser
    // - 0xFFD8: start of image
    // - 0xFFD9: end of image
    // See: https://stackoverflow.com/a/4614629

    uint32_t jpegBufferOffset = 0;
    uint32_t pendingDataByteCount = 0;
    uint8_t byte1 = 0x00, byte2 = 0x00;

    // parser loop, byte by byte
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

    // read one line from PTS file
    // PTS = presentation timestamp
    uint64_t absTime, relTime, offTime;
    m_ptsFile >> absTime >> relTime >> offTime;

    // jpegBufferOffset == jpegBufferSize
    return { jpegBufferOffset, absTime };
}
