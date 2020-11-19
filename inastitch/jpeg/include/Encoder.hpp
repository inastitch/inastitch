// Copyright (C) 2020 Inatech srl
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

// Std includes:
#include <cstdint>
#include <tuple>

namespace inastitch {
namespace jpeg {

/// @brief JPEG encoder class
///
/// This class is just a wrapper around @c libturbojpeg to compress a JPEG data buffer.
class Encoder
{
public:
    Encoder(uint32_t maxJpegBufferSize);
    ~Encoder();

public:
    std::tuple<uint8_t* const, long unsigned int> encode(const uint8_t *rgbaData, uint32_t width, uint32_t height);

private:
    const uint32_t m_jpegBufferSize;
    uint8_t* const m_jpegBuffer;

private:
    void* m_jpegCompressor;
};


} // namespace jpeg
} // namespace inastitch
