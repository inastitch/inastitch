// Copyright (C) 2020 Inatech srl
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

// Std includes:
#include <string>
#include <iostream>

namespace inastitch {
namespace opencv {

/// @brief Homograghy Matrix
///
/// An homography matrix is a matrix transformation to warp an image into a different point of view.
///
/// Function based on OpenCV library.
class HomographyMatrix
{
private:
    // "static const float" does not work
    static constexpr float ratio = 0.75;
    static constexpr float reprojThresh = 4.0;
    
public:
    static bool find(
        unsigned char * const centerRgbaBuffer,
        uint32_t centerRgbaBufferWidth, uint32_t centerRgbaBufferHeight,
        unsigned char * const rightRgbaBuffer,
        uint32_t rightRgbaBufferWidth, uint32_t rightRgbaBufferHeight,
        bool isFlipped, uint32_t minMatchCount, bool isDebugImageDumped,
        float matrix[3][3]
#ifdef HAVE_OPENCV
    );
#else
    ) {
        std::cout << "Feature is disabled. OpenCV support is not built in." << std::endl;
        return false;
    }
#endif
};

} // namespace opencv
} // namesapce inastitch
