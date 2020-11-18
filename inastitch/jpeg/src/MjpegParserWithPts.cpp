// Copyright (C) 2020 Inatech srl
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

// Local includes:
#include "inastitch/jpeg/include/MjpegParserWithPts.hpp"
#include "inastitch/jpeg/include/MjpegParserAsync.hpp"

// Std includes:
#include <iostream>

inastitch::jpeg::MjpegParserWithPts::MjpegParserWithPts(std::string filename, uint32_t maxJpegBufferSize)
    : ptsCount( inastitch::jpeg::MjpegParserAsync::jpegBufferCount )
    , m_timestampArray( new uint64_t[ptsCount] )
{
    m_mjpegFile = std::ifstream(filename, std::ios::binary);
    std::cout << "Opened MJPEG at " << filename << std::endl;

    m_mjpegParser = std::make_unique<inastitch::jpeg::MjpegParserAsync>(m_mjpegFile, maxJpegBufferSize);

    const auto ptsFilename = filename + ".pts";
    m_ptsFile = std::ifstream(ptsFilename);
    std::cout << "Opened PTS at " << ptsFilename << std::endl;

    // init array
    for(int i=0; i < ptsCount; i++) {
        m_timestampArray[i] = 0;
    }
}

inastitch::jpeg::MjpegParserWithPts::~MjpegParserWithPts()
{
    delete[] m_timestampArray;

    m_ptsFile.close();
}

std::tuple<uint8_t*, uint32_t, uint64_t> inastitch::jpeg::MjpegParserWithPts::getFrame(uint32_t index)
{
    const auto nextPtsIdx = (m_currentPtsIndex == 0) ? ptsCount - 1 : m_currentPtsIndex - 1;

    // read one line from PTS file
    uint64_t absTime, relTime, offTime;
    m_ptsFile >> absTime >> relTime >> offTime;
    m_timestampArray[nextPtsIdx] = absTime;

    const auto [jpegBuffer, jpegBufferSize] = m_mjpegParser->getFrame(index);

    const auto bufferArrayIndex = m_currentPtsIndex + index;
    return { jpegBuffer, jpegBufferSize, m_timestampArray[bufferArrayIndex] };
}
