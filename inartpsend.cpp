// Copyright (C) 2020 Inatech srl
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

// inartpsend, Inatech's RTP/AVTP sender
// Created on 20.06.2020
// by Vincent Jordan

// Local includes:
#include "version.h"
#include "inastitch/jpeg/include/RtpJpegEncoder.hpp"

// C includes:
#include <netinet/in.h>

// Boost includes:
#include <boost/program_options.hpp>
namespace po = boost::program_options;

// Std includes:
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>

int main(int argc, char** argv)
{
    bool mjpegIsType1 = false;
    uint16_t jpegWidth;
    uint16_t jpegHeight;

    uint16_t outSocketPort;
    std::string outSocketAddr;
    std::string frameDumpPath;

    bool useAvtp = false;
    bool isNotTimestamp = false;
    bool isVerbose = false;
    {
        po::options_description desc(
            std::string("inartpsend ") + inastitch::version::GIT_COMMIT_TAG + 
            " (" + inastitch::version::GIT_COMMIT_DATE + ")" +
            "\n\nAllowed options"
        );
        
        desc.add_options()
            ("isMode1", "Input JPEG is Mode 1 (YUV420) instead of Mode 0 (YUV422)")
            ("width", po::value<uint16_t>(&jpegWidth)->default_value(640),
             "Input JPEG frame WIDTH")
            ("height", po::value<uint16_t>(&jpegHeight)->default_value(480),
             "Input JPEG frame HEIGHT")

            ("out-port", po::value<uint16_t>(&outSocketPort)->default_value(5000),
             "Send output RTP/JPEG stream on PORT")
            ("out-addr", po::value<std::string>(&outSocketAddr)->default_value("127.0.0.1"),
             "Send output RTP/JPEG stream to ADDR")

            ("frame-dump-path", po::value<std::string>(&frameDumpPath)->default_value(""),
             "Dump frame to PATH")

            ("use-avtp", "Use AVTP over UDP instead of RTP format")
            ("no-ts", "Do not expect timestamp after JPEG data")
            ("verbose,v", "Verbose log mode")
            ("help,h", "Show this help message")
        ;

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);

        if(vm.count("help")) {
            std::cout << desc << std::endl;
            return 0;
        }

        if(vm.count("use-avtp"))
            useAvtp = true;
        
        if(vm.count("isMode1"))
            mjpegIsType1 = true;

        if(vm.count("no-ts"))
            isNotTimestamp = true;
        
        if(vm.count("verbose"))
            isVerbose = true;
    }

    std::cout << "inartpsend " << inastitch::version::GIT_COMMIT_TAG
              << " (" << inastitch::version::GIT_COMMIT_DATE << ")" << std::endl;
    
    std::cout << "Streaming " << (useAvtp ? "AVTP" : "RTP") << std::endl;

    inastitch::jpeg::RtpJpegEncoder rtpJpegEncoder(outSocketAddr, outSocketPort, mjpegIsType1);
    // Note: JPEG data should be smaller than RAW data

    // read JPEG from pipe
    std::fstream pipeIn = std::fstream("/dev/stdin", std::ios::in | std::ios::binary);

    const auto outStreamMaxRgbBufferSize = jpegWidth * jpegHeight * 3;
    uint8_t *jpegBuffer = new uint8_t[outStreamMaxRgbBufferSize];

    // MJPEG mini parser
    // - 0xFFD8: start of image
    // - 0xFFD9: end of image
    // See: https://stackoverflow.com/a/4614629

    uint32_t frameIdx = 0;
    while(true)
    {
        int64_t timestamp = 0;
        uint32_t jpegBufferOffset = 0;
        uint32_t pendingData = 0;
        char b = 0x00;
        uint8_t b1 = 0x00, b2 = 0x00;
        while(pipeIn.read(&b, 1))
        {
            b2 = b;
            //std::cout << std::hex << static_cast<int>(b1) << " " << std::flush;

            // start marker
            if( (pendingData == 0) && (b1 == 0xFF) && (b2 == 0xD8) )
            {
                jpegBufferOffset = 0;
                jpegBuffer[jpegBufferOffset++] = b1;
                jpegBuffer[jpegBufferOffset++] = b2;
            }
            else
            // end marker
            if( (pendingData == 0) && (b1 == 0xFF) && (b2 == 0xD9) )
            {
                jpegBuffer[jpegBufferOffset++] = b2;

                if(!isNotTimestamp)
                {
                    // reached the end of the JPEG data
                    // => read timestamp appended to it
                    pipeIn.read((char*)&timestamp, sizeof(int64_t));
                }

                break;
            }
            else
            // bit stuffing
            if( (pendingData == 0) && (b1 == 0xFF) && (b2 == 0x00) )
            {
                jpegBuffer[jpegBufferOffset++] = b2;
            }
            else
            // marker with length
            if( (pendingData == 0) &&
                (b1 == 0xFF) ) //&& (b2 != 0x00) && (b2 != 0x01) && !( (b2 >= 0xD0) && (b2 <= 0xD9) ) )
            {
                //const uint32_t m1 = b1, m2 = b2;

                if(!(b2 == 0xC0 ||
                     b2 == 0xC2 ||
                     b2 == 0xC4 ||
                     b2 == 0xDB ||
                     b2 == 0xDA ||
                     b2  & 0xE0 ||
                     b2 == 0xFE   ))
                {
                    std::cerr << "Unexpected JPEG marker, abort..." << std::endl;
                    std::abort();
                }
                jpegBuffer[jpegBufferOffset++] = b2;

                pipeIn.read(&b, 1);
                b1 = b;
                pipeIn.read(&b, 1);
                b2 = b;
                // big endian length
                pendingData += static_cast<uint16_t>(b1) << 8;
                pendingData += b2;
                //std::cout << std::hex << m1 << m2 << " " << std::dec << pendingData << std::endl;

                jpegBuffer[jpegBufferOffset++] = b1;
                jpegBuffer[jpegBufferOffset++] = b2;
                pendingData -= 1;
            }
            else
            {
                jpegBuffer[jpegBufferOffset++] = b2;
                if(pendingData > 0)
                    pendingData--;
            }

            b1 = b2;
        }
        const auto jpegBufferSize = jpegBufferOffset;
        if(isVerbose) {
            std::cout << "Frame " << jpegBufferSize << " bytes with t=" << timestamp << std::endl;
        }

        if(!frameDumpPath.empty()) {
            auto jpegFile = std::fstream(frameDumpPath + "out.jpg", std::ios::out | std::ios::binary);
            jpegFile.write((char*)&jpegBuffer[0], jpegBufferSize);
            jpegFile.close();
        }

        if(isVerbose)
        {
            std::cout << "Send frame, timestamp=" << std::dec << timestamp << std::endl;
            // Note: int64_t -> uint32_t = lost of some most significant bits
            //       timestamp is expected in us
        }
        rtpJpegEncoder.sendFrame(jpegBuffer, jpegBufferSize, jpegWidth, jpegHeight, static_cast<uint32_t>(timestamp), useAvtp);
        
        if(frameIdx % 100 == 0)
        {
            std::cout << "Sent frame " << std::dec << frameIdx << " t=" << timestamp <<std::endl;
        }
        
        frameIdx++;
    }

    return 0;
}
