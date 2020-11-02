// Copyright (C) 2020 Inatech srl
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

// Std includes:
#include <tuple>
#include <string>
#include <fstream>

namespace inastitch {
namespace jpeg {


class MjpegParser
{
public:
    MjpegParser(std::string filename);
    ~MjpegParser();

public:
    std::tuple<uint32_t, uint64_t> parseFrame(uint8_t* const jpegBuffer);

private:
    bool nextByte(uint8_t &byte);

private:
    std::ifstream m_mjpegFile;
    std::ifstream m_ptsFile;
    uint32_t m_startMarkerCount = 0;
};

} // namespace jpeg
} // namespace inastitch
