// Copyright (C) 2020 Inatech srl
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

// Boost includes:
#include <boost/asio/thread_pool.hpp>

// Std includes:
#include <tuple>
#include <string>
#include <fstream>

namespace inastitch {
namespace jpeg {


class MjpegParser
{
public:
    MjpegParser(std::string filename, uint32_t maxJpegBufferSize);
    ~MjpegParser();

public:
    std::tuple<uint8_t*, uint32_t, uint64_t> getFrame(uint32_t index);

private:
    bool nextByte(uint8_t &byte);
    uint32_t parseJpeg(uint8_t* const jpegBuffer);
    void nextFrame();

private:
    static const auto jpegBufferCount = 10;

private:
    const uint32_t m_maxJpegBufferSize;
    uint32_t m_currentJpegBufferIndex = 0;
    std::ifstream m_mjpegFile;
    std::ifstream m_ptsFile;
    uint32_t m_startMarkerCount = 0;
    boost::asio::thread_pool m_threadPool;

private:
    uint8_t** m_jpegBufferArray;
    uint32_t* const m_jpegSizeArray;
    uint64_t* const m_timestampArray;
};


} // namespace jpeg
} // namespace inastitch
