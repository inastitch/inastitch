// Copyright (C) 2020 Inatech srl
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

// Std includes:
#include <cstdint>
#include <string>

namespace inastitch {
namespace jpeg {


class Decoder
{
public:
    Decoder(uint32_t maxRgbBufferSize);
    ~Decoder();

public:
    uint8_t* decode();
    void writePpm(const std::string &filename);

public:
    uint8_t* jpegBuffer() const
    {
        return m_jpegBuffer;
    }

    uint32_t jpegBufferSize() const
    {
        return m_jpegBufferSize;
    }

    uint8_t* rgbaBuffer() const
    {
        return m_rgbaBuffer;
    }

    uint32_t rgbBufferSize() const
    {
        return m_rgbaBufferSize;
    }

private:
    unsigned int m_width = 0;
    unsigned int m_height = 0;
    unsigned int m_pixelSize = 4; // because TJPF_RGBA

private:
    const uint32_t m_rgbaBufferSize;
    const uint32_t m_jpegBufferSize;

private:
    uint8_t* const m_rgbaBuffer;
    uint8_t* const m_jpegBuffer;
    
private:
    // tjhandler
    void* m_jpegDecompressor;
};


} // namespace jpeg
} // namespace inastitch
