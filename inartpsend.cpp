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
#include <chrono>

/// @brief inartpsend
///
/// This tool takes an MJPEG stream from stdin and sends it over UDP Ethernet with either:
///  - RTP/JPEG
///  - AVTP/JPEG (UDP variant only)
///
/// It is designed primarly to work with the Raspberry Pi camera tool:
/// @verbatim raspivid -fl -t 0 -w 640 -h 480 -fps 100 -md 6 -cd MJPEG -o - | inartpsend --jpeg-mode1 --out-addr ${RTP_ADDR} --out-port ${RTP_PORT} @endverbatim
///
/// You can also use @c gstreamer with @c fdsink:
/// @verbatim gst-launch-1.0 videotestsrc ! jpegenc ! fdsink | inartpsend --jpeg-mode1 --width 320 --height 240 @endverbatim
/// @verbatim gst-launch-1.0 -e v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480,framerate=30/1 ! jpegenc ! fdsink | inartpsend --local-ts --width 640 --height 480 @verbatim
///
/// Or any other tool which outputs MJPEG on stdout.
///
/// @note In addition to standard MJPEG, this tool also supports a non-standard MJPEG stream in
/// which the @e presentation @e timestamp of each frame is appended as a 64-bit integer after
/// each JPEG data.
/// To be used with a modified @c raspivid with camera sync called @c raspivid-inatech.
/// See: https://github.com/inastitch/raspivid-inatech
///
/// @see Details about RTP or AVTP protocol are found in @ref inastitch::jpeg::RtpJpegEncoder.
int main(int argc, char** argv)
{
    uint16_t jpegWidth;
    uint16_t jpegHeight;

    uint16_t outSocketPort;
    std::string outSocketAddr;

    std::string frameDumpPath;

    bool isJpegMode1 = false;
    bool isAvtpOutput = false;
    bool isJpegTimestampAvailable = false;
    bool isLocalTimestamp = false;
    bool isVerbose = false;
    {
        po::options_description desc(
            std::string("inartpsend ") + inastitch::version::GIT_COMMIT_TAG + 
            " (" + inastitch::version::GIT_COMMIT_DATE + ")" +
            "\n\nAllowed options"
        );
        
        desc.add_options()
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

            ("jpeg-mode1", "Input JPEG is Mode 1 (YUV420) instead of Mode 0 (YUV422)")
            ("avtp", "Use AVTP over UDP instead of RTP format for output stream")
            ("jpeg-ts", "Expect timestamp after JPEG data (not standard MJPEG)")
            ("local-ts", "Overwrite timestamp with current CPU sysclk")
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

        if(vm.count("avtp"))
            isAvtpOutput = true;
        
        if(vm.count("jpeg-mode1"))
            isJpegMode1 = true;

        if(vm.count("jpeg-ts"))
            isJpegTimestampAvailable = true;

        if(vm.count("local-ts"))
            isLocalTimestamp = true;
        
        if(vm.count("verbose"))
            isVerbose = true;
    }

    std::cout << "inartpsend " << inastitch::version::GIT_COMMIT_TAG
              << " (" << inastitch::version::GIT_COMMIT_DATE << ")" << std::endl;
    
    std::cout << "Streaming " << (isAvtpOutput ? "AVTP" : "RTP") << std::endl;

    inastitch::jpeg::RtpJpegEncoder rtpJpegEncoder(outSocketAddr, outSocketPort, isJpegMode1);
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
        if(!isJpegTimestampAvailable)
        {
            // reached the end of the JPEG data
            // => read timestamp appended to it
            pipeIn.read((char*)&timestamp, sizeof(int64_t));
        }

        if(isLocalTimestamp)
        {
            using namespace std::chrono;

            // overwrite timestamp with local clock
            const auto localTimestamp = system_clock::now();
            timestamp = duration_cast<microseconds>(time_point_cast<microseconds>(localTimestamp).time_since_epoch()).count();
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
        }

        // Note: int64_t -> uint32_t = lost of some most significant bits
        //       timestamp is expected in us
        rtpJpegEncoder.sendFrame(jpegBuffer, jpegBufferSize, jpegWidth, jpegHeight, static_cast<uint32_t>(timestamp), isAvtpOutput);
        
        if(frameIdx % 100 == 0)
        {
            std::cout << "Sent frame " << std::dec << frameIdx << " t=" << timestamp <<std::endl;
        }
        
        frameIdx++;
    }

    return 0;
}
