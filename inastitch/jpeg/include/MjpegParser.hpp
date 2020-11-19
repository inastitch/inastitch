// Copyright (C) 2020 Inatech srl
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

// Std includes:
#include <istream>

namespace inastitch {
namespace jpeg {

/// @brief MJPEG parser class
///
/// An MJPEG video file is simply the concatenation of JPEG images without any additional data.
/// There is no sound of information about the framerate or timestamps.
///
/// To make a new MJPEG video, you can just use the @c cat command:
/// @verbatim cat *.jpg > video.mjpeg @endverbatim
///
/// Since each JPEG file has a different size, the parser decodes just enough from the JPEG header
/// to find the end of the file, and jumps there. Frames can be decompressed with a standard JPEG
/// library.
class MjpegParser
{
public:
    static uint32_t parseJpeg(std::istream &mjpegStream, uint8_t* const jpegBuffer);
};


} // namespace jpeg
} // namespace inastitch
