// Copyright (C) 2020 Inatech srl
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

// Local includes:
#include "inastitch/jpeg/include/MjpegParserAsync.hpp"
#include "inastitch/jpeg/include/MjpegParser.hpp"

// Boost includes:
#include <boost/asio/post.hpp>

// Std includes:
#include <iostream>

inastitch::jpeg::MjpegParserAsync::MjpegParserAsync(std::istream &mjpegStream, uint32_t maxJpegBufferSize)
    : m_maxJpegBufferSize(maxJpegBufferSize)
    , m_mjpegStream(mjpegStream)
    , m_threadPool(1)
    , m_jpegSizeArray( new uint32_t[jpegBufferCount] )
{
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

inastitch::jpeg::MjpegParserAsync::~MjpegParserAsync()
{
    delete[] m_jpegSizeArray;

    for(uint32_t i = 0; i < jpegBufferCount; i++) {
        delete[] m_jpegBufferArray[i];
    }
}

void inastitch::jpeg::MjpegParserAsync::nextFrame()
{
    const auto nextJpegBufferIdx = (m_currentJpegBufferIndex == 0) ? jpegBufferCount - 1 : m_currentJpegBufferIndex - 1;

    // read one jpeg from MJPEG file
    m_jpegSizeArray[nextJpegBufferIdx] = inastitch::jpeg::MjpegParser::parseJpeg(m_mjpegStream, m_jpegBufferArray[nextJpegBufferIdx]);

    // at last
    m_currentJpegBufferIndex = nextJpegBufferIdx;
}

std::tuple<uint8_t*, uint32_t> inastitch::jpeg::MjpegParserAsync::getFrame(uint32_t index)
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
