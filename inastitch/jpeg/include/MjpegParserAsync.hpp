// Copyright (C) 2020 Inatech srl
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

// Boost includes:
#include <boost/asio/thread_pool.hpp>

// Std includes:
#include <tuple>
#include <string>
#include <istream>

namespace inastitch {
namespace jpeg {

/// @brief Asynchronous MJPEG parser
///
/// This class uses @ref MjpegParser in a side thread to parse frames ahead of time.
class MjpegParserAsync
{
public:
    MjpegParserAsync(std::istream &mjpegStream, uint32_t maxJpegBufferSize);
    ~MjpegParserAsync();

public:
    std::tuple<uint8_t *, uint32_t> getFrame(uint32_t index);

private:
    void nextFrame();

public:
    static const uint32_t jpegBufferCount = 100;

private:
    uint32_t m_maxJpegBufferSize;
    std::istream &m_mjpegStream;
    uint32_t m_currentJpegBufferIndex = 0;
    boost::asio::thread_pool m_threadPool;

private:
    uint8_t** m_jpegBufferArray;
    uint32_t* const m_jpegSizeArray;
};


} // namespace jpeg
} // namespace inastitch
