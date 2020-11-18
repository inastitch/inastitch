// Copyright (C) 2020 Inatech srl
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

// Std includes:
#include <istream>

namespace inastitch {
namespace jpeg {


class MjpegParser
{
public:
    static uint32_t parseJpeg(std::istream &mjpegStream, uint8_t* const jpegBuffer);
};


} // namespace jpeg
} // namespace inastitch
