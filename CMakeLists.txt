# Copyright (C) 2020 Inatech srl
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.

# Note: Cannot build both packages at once?
# To package inastitch:
# cmake -DCMAKE_BUILD_TYPE=Release -DNO_INARTPSEND=ON ../inastitch/
# cpack
#
# To package inartpsend:
# cmake -DCMAKE_BUILD_TYPE=Release -DNO_INASTITCH=ON ../inastitch/
# cpack

cmake_minimum_required(VERSION 3.10)

option(NO_INASTITCH "Disable inastitch")
option(NO_INARTPSEND "Disable inartpsend")

# C++17 required for tao JSON
# https://cmake.org/cmake/help/latest/prop_tgt/CXX_STANDARD.html
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# https://cmake.org/cmake/help/v3.6/module/CPackDeb.html
set(CPACK_GENERATOR "DEB")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "vjp")
set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
set(CPACK_STRIP_FILES TRUE)

### Generate git version include file
find_package(Git)
execute_process(COMMAND
  "${GIT_EXECUTABLE}" describe --tags
  WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
  OUTPUT_VARIABLE GIT_COMMIT_TAG
  ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)

execute_process(COMMAND
  "${GIT_EXECUTABLE}" log -1 --format=%aD
  WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
  OUTPUT_VARIABLE GIT_COMMIT_DATE
  ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)

configure_file("${CMAKE_SOURCE_DIR}/cmake/version.cpp.in" "${CMAKE_BINARY_DIR}/version.cpp" @ONLY)
list(APPEND SOURCES "${CMAKE_BINARY_DIR}/version.cpp" include/version.h)

###
if(NO_INARTPSEND)
    message(STATUS "Skip inartpsend")
else()
    project(inartpsend
        VERSION 0.1
        DESCRIPTION "Inatech RTP/JPEG sender"
#        HOMEPAGE_URL "https://github.com/inastitch/inastitch"
        LANGUAGES CXX
    )

    add_executable(inartpsend
        inastitch/jpeg/src/RtpJpegEncoder.cpp
        inartpsend.cpp
        ${CMAKE_CURRENT_BINARY_DIR}/version.cpp
    )

    find_library(BOOST_PROG_OPTS_STATIC_LIB libboost_program_options.a)

    target_link_libraries(inartpsend
        # Boost
        -lboost_program_options
        #${BOOST_PROG_OPTS_STATIC_LIB}

        -lrt -lz -pthread
    )
endif()

###
if(NO_INASTITCH)
    message(STATUS "Skip inastitch")
else()

    project(inastitch
        VERSION 0.1
        DESCRIPTION "Inatech stitcher"
    #    HOMEPAGE_URL "https://github.com/inastitch/inastitch"
        LANGUAGES CXX
    )

    # Required for libav C code
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive")

    include_directories("${CMAKE_SOURCE_DIR}")
    include_directories("${CMAKE_SOURCE_DIR}/include/")
    include_directories("${CMAKE_SOURCE_DIR}/include/libav/")

    add_executable(inastitch
        inastitch/opengl/src/OpenGlHelper.cpp
        inastitch/opengl/src/OpenGlTextHelper.cpp
        inastitch/jpeg/src/Decoder.cpp
        inastitch/jpeg/src/Encoder.cpp
        inastitch/jpeg/src/MjpegParser.cpp
        inastitch/jpeg/src/RtpJpegParser.cpp
        inastitch/json/src/Matrix.cpp
        inastitch.cpp
        ${CMAKE_BINARY_DIR}/version.cpp
    )

    find_library(BOOST_PROG_OPTS_STATIC_LIB libboost_program_options.a)
    find_library(TURBO_JPEG_STATIC_LIB libturbojpeg.a)

    target_link_libraries(inastitch
        -lboost_program_options
        #${BOOST_PROG_OPTS_STATIC_LIB}
        -lturbojpeg
        #${TURBO_JPEG_STATIC_LIB}

        -lGLESv2 -lglfw
        -pthread
    )

    install(TARGETS inastitch
        RUNTIME DESTINATION "/usr/bin/"
    )
endif()

###
if(EXISTS ${OPENCV_STATIC_LIB_PATH})
    add_subdirectory(tools/calibration/)
else()
    message("Not building 'inastitch_cal' because missing OPENCV_STATIC_LIB_PATH")
endif()

include(CPack)
