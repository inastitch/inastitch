# Copyright (C) 2020 Inatech srl
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.

# C++17 required for tao JSON
# https://cmake.org/cmake/help/latest/prop_tgt/CXX_STANDARD.html
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories("${OPENCV_STATIC_LIB_PATH}/include/opencv4/")

file(GLOB OCV_STATIC_LIBS ${OPENCV_STATIC_LIB_PATH}/lib/*.a)
file(GLOB OCV_3RD_PARTY_STATIC_LIBS ${OPENCV_STATIC_LIB_PATH}/lib/opencv4/3rdparty/*.a)

project(inastitch_cal
    VERSION 0.1
    DESCRIPTION "Inatech stitcher calibration tool"
#    HOMEPAGE_URL "https://github.com/inastitch/inastitch/tool/calibration"
    LANGUAGES CXX
)

add_executable(inastitch_cal
    ../../inastitch/opencv/src/HomographyMatrix.cpp
    main.cpp
    ${CMAKE_BINARY_DIR}/version.cpp
)

find_library(BOOST_PROG_OPTS_STATIC_LIB libboost_program_options.a)
find_library(TURBO_JPEG_STATIC_LIB libturbojpeg.a)
find_library(PNG_STATIC_LIB libpng.a)

# Reminder: static dependencies need to be linked *after*.
#           For example, JPEG is after OpenCV.
target_link_libraries(inastitch_cal
    -lboost_program_options
    #${BOOST_PROG_OPTS_STATIC_LIB}

    # OpenCV static libraries
    # Note: multiples times because of dependencies
    ${OCV_STATIC_LIBS}
    ${OCV_STATIC_LIBS}
    ${OCV_3RD_PARTY_STATIC_LIBS}
    ${OCV_3RD_PARTY_STATIC_LIBS}

    #-lturbojpeg
    ${TURBO_JPEG_STATIC_LIB}

    #-lpng
    ${PNG_STATIC_LIB}

    -lz -pthread -ldl
)

install(TARGETS inastitch_cal
    RUNTIME DESTINATION "/usr/bin/"
)
