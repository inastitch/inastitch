// Copyright (C) 2020 Inatech srl
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

// inartpsend, Inatech's RTP/AVTP sender
// Created on 20.06.2020
// by Vincent Jordan

// Local includes:
#include "version.h"
#include "inastitch/jpeg/include/MjpegParser.hpp"
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
    std::ifstream pipeIn = std::ifstream("/dev/stdin", std::ios::binary);

    const auto outStreamMaxRgbBufferSize = jpegWidth * jpegHeight * 3;
    uint8_t *jpegBuffer = new uint8_t[outStreamMaxRgbBufferSize];

    uint32_t frameIdx = 0;
    while(true)
    {
        const auto jpegBufferSize = inastitch::jpeg::MjpegParser::parseJpeg(pipeIn, jpegBuffer);

        int64_t timestamp = 0;
        if(!isNotTimestamp)
        {
            // reached the end of the JPEG data
            // => read timestamp appended to it
            pipeIn.read((char*)&timestamp, sizeof(int64_t));
        }

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
