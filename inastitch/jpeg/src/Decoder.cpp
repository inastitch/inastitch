// Copyright (C) 2020 Inatech srl
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

// Local includes:
#include "inastitch/jpeg/include/Decoder.hpp"

// TurboJpeg includes:
#include <turbojpeg.h>
// Note: libjpeg-turbo != libturbojpeg
// => apt install libturbojpeg0-dev

// Std includes:
#include <iostream>
#include <fstream>

inastitch::jpeg::Decoder::Decoder(uint32_t maxRgbBufferSize)
        : m_rgbaBufferSize(maxRgbBufferSize)
        // JPEG data should be at least smaller than RAW data
        , m_jpegBufferSize(maxRgbBufferSize)
        , m_rgbaBuffer( new uint8_t[m_rgbaBufferSize] )
        , m_jpegBuffer( new uint8_t[m_jpegBufferSize] )
        , m_jpegDecompressor( tjInitDecompress() )
{ }

inastitch::jpeg::Decoder::~Decoder()
{
    tjDestroy(m_jpegDecompressor);

    delete[] m_jpegBuffer;
    delete[] m_rgbaBuffer;
}

uint8_t* inastitch::jpeg::Decoder::decode()
{
    int32_t jpegWidth = 0, jpegHeight = 0, jpegSubsamp = 0;
    int tjError = 0;

    tjError = tjDecompressHeader2(m_jpegDecompressor, m_jpegBuffer, m_jpegBufferSize, &jpegWidth, &jpegHeight, &jpegSubsamp);
    if(tjError != 0) {
        std::cerr << tjGetErrorStr() << std::endl;
    }

    // check whether the RGB buffer is big enough
    const auto requiredRgbBufferSize = jpegWidth * jpegHeight * m_pixelSize;
    if(requiredRgbBufferSize > m_rgbaBufferSize) {
        std::cerr << "Error: JPEG image size " << jpegWidth << "x" << jpegHeight << " does not fit allocated buffer size." << std::endl;
        std::abort();
    }

    m_width = jpegWidth;
    m_height = jpegHeight;

    // TODO: decompress into RGBA to save processing when writing to OpenGL texture
    tjError = tjDecompress2(
        m_jpegDecompressor,
        m_jpegBuffer, m_jpegBufferSize,
        m_rgbaBuffer, 0 /*width*/, 0 /*pitch*/, 0 /*height*/,
        TJPF_RGBA, TJFLAG_FASTDCT | TJFLAG_NOREALLOC
    );
    if(tjError != 0) {
        std::cerr << tjGetErrorStr() << std::endl;
    }

    return m_rgbaBuffer;
}

void inastitch::jpeg::Decoder::writePpm(const std::string &filename)
{
    if(m_pixelSize != 4) {
        std::cerr << "writePpm: 'pixelSize' not supported" << std::endl;
        std::abort();
    }

    std::ofstream ppmFile;
    ppmFile.open(filename);

    // PPM header
    ppmFile << "P6 " << m_width << " " << m_height << " 255" << std::endl;

    // PPM data
    for(uint32_t i=0; i<m_height; i++)
    {
        for(uint32_t j=0; j<m_width; j++)
        {
            ppmFile << m_rgbaBuffer[(i*m_width+j)*m_pixelSize + 0];
            ppmFile << m_rgbaBuffer[(i*m_width+j)*m_pixelSize + 1];
            ppmFile << m_rgbaBuffer[(i*m_width+j)*m_pixelSize + 2];
            // skip alpha channel
        }
    }

    ppmFile.close();
}
