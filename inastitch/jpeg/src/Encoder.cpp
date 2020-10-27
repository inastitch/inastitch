// Copyright (C) 2020 Inatech srl
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

// Local includes:
#include "inastitch/jpeg/include/Encoder.hpp"

// TurboJpeg includes:
#include <turbojpeg.h>
// Note: libjpeg-turbo != libturbojpeg
// => apt install libturbojpeg0-dev


inastitch::jpeg::Encoder::Encoder(uint32_t maxJpegBufferSize)
        : m_jpegBufferSize( maxJpegBufferSize )
        , m_jpegBuffer( new uint8_t[m_jpegBufferSize] )
        , m_jpegCompressor( tjInitCompress() )
{ }

inastitch::jpeg::Encoder::~Encoder()
{
    tjDestroy(m_jpegCompressor);
    delete[] m_jpegBuffer;
}

std::tuple<uint8_t* const, long unsigned int> inastitch::jpeg::Encoder::encode(const uint8_t *rgbaData, uint32_t width, uint32_t height)
{
    const int jpegQuality = 75;

    uint8_t* jpegBuffer = m_jpegBuffer;
    long unsigned int jpegSize = 0;
    tjCompress2(m_jpegCompressor, rgbaData, width, 0, height, TJPF_RGBX,
                &jpegBuffer, &jpegSize, TJSAMP_422, jpegQuality,
                TJFLAG_BOTTOMUP | TJFLAG_FASTDCT | TJFLAG_NOREALLOC);
    // 'tjCompress2' takes a pointer to the pointer 'jpegBuffer' so as to rewrite it
    // when 'tjCompress2' is configured to allocate buffer dynamically.
    // This is not enabled here. See 'TJFLAG_NOREALLOC'.

    // Note:
    // TJSAMP_422 => MJPEG type 0
    // TJSAMP_420 => MJPEG type 1

    return std::make_tuple(m_jpegBuffer, jpegSize);
}
