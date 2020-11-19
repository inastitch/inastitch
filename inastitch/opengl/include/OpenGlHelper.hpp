// Copyright (C) 2020 Inatech srl
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once

// Std includes:
#include <stdint.h>
#include <vector>
#include <array>

namespace inastitch
{
namespace opengl
{
namespace helper
{

void checkError(const char*, const char*, int);

// Define DEBUG to check every OpenGL call for error (will reduce performances)
//#define DEBUG
#ifdef DEBUG
    #define GL_CHECK(statement) do                                                   \
        {                                                                            \
            statement;                                                               \
            inastitch::opengl::helper::checkError(#statement, __FILE__, __LINE__);   \
        } while (0)
#else
    #define GL_CHECK(stmt) stmt
#endif

int getShaderProgram(const char*, const char*);

class Overlay
{
public:
    Overlay(uint32_t width, uint32_t height);
    ~Overlay();

public:
    void clear();
    void putChar(uint32_t xOffset, uint32_t yOffset, uint8_t asciiCode);
    void putDigit(uint32_t xOffset, uint32_t yOffset, uint8_t digit);
    void putNumber(uint32_t xOffset, uint32_t yOffset, uint32_t number, uint8_t digitCount);
    void putString(uint32_t xOffset, uint32_t yOffset, const char * const string, uint8_t charCount);

public:
    uint8_t* rgbaBuffer() const
    {
        return m_rgbaBuffer;
    }

private:
    uint32_t m_width;
    uint32_t m_height;
    uint8_t* m_rgbaBuffer;

private:
    static const std::vector<std::array<uint8_t, 25>> digitBits;
};

} // helper
} // namespace opengl
} // namespace inastitch
