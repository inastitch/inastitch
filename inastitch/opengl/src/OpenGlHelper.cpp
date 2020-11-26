// Copyright (C) 2020 Inatech srl
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

// Local includes:
#include "inastitch/opengl/include/OpenGlHelper.hpp"

// Glfw includes:
// Use OpenGL ES 2.x
#define GLFW_INCLUDE_ES2
// Use OpenGL ES 3.x
//#define GLFW_INCLUDE_ES3
#include <GLFW/glfw3.h>

// Std includes:
#include <iostream>

using namespace inastitch::opengl::helper;

void inastitch::opengl::helper::checkError(const char *statement, const char* filename, int lineNumber)
{
    const GLenum glErrorNum = glGetError();
    if(glErrorNum != GL_NO_ERROR)
    {
        printf("OpenGL error %08x, at %s:%i - for %s\n", glErrorNum, filename, lineNumber, statement);
        abort();
    }
}

int inastitch::opengl::helper::getShaderProgram(const char *vertexShaderSource, const char *fragmentShaderSource)
{
    enum Consts {INFOLOG_LEN = 512};
    GLchar infoLog[INFOLOG_LEN];

    GLint fragmentShader;
    GLint shaderProgram;
    GLint vertexShader;
    GLint isSuccess;

    // Vertex shader
    vertexShader = glCreateShader(GL_VERTEX_SHADER);
    GL_CHECK( glShaderSource(vertexShader, 1, &vertexShaderSource, NULL) );
    GL_CHECK( glCompileShader(vertexShader) );
    GL_CHECK( glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &isSuccess) );
    if(!isSuccess) {
        GL_CHECK( glGetShaderInfoLog(vertexShader, INFOLOG_LEN, NULL, infoLog) );
        std::cerr << "OGL/SHADER/VERTEX: compilation failed." << std::endl
                  << infoLog << std::endl;
        std::abort();
    }

    // Fragment shader
    GL_CHECK( fragmentShader = glCreateShader(GL_FRAGMENT_SHADER) );
    GL_CHECK( glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL) );
    GL_CHECK( glCompileShader(fragmentShader) );
    GL_CHECK( glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &isSuccess) );
    if (!isSuccess) {
        GL_CHECK( glGetShaderInfoLog(fragmentShader, INFOLOG_LEN, NULL, infoLog) );
        std::cerr << "OGL/SHADER/FRAGMENT: compilation failed." << std::endl
                  << infoLog << std::endl;
        std::abort();
    }

    // Link shaders
    GL_CHECK( shaderProgram = glCreateProgram() );
    GL_CHECK( glAttachShader(shaderProgram, vertexShader) );
    GL_CHECK( glAttachShader(shaderProgram, fragmentShader) );
    GL_CHECK( glLinkProgram(shaderProgram) );
    GL_CHECK( glGetProgramiv(shaderProgram, GL_LINK_STATUS, &isSuccess) );
    if (!isSuccess) {
        GL_CHECK( glGetProgramInfoLog(shaderProgram, INFOLOG_LEN, NULL, infoLog) );
        std::cerr << "OGL/SHADER/PROGRAM: linking failed." << std::endl
                  << infoLog << std::endl;
        std::abort();
    }

    GL_CHECK( glDeleteShader(vertexShader) );
    GL_CHECK( glDeleteShader(fragmentShader) );

    return shaderProgram;
}

inastitch::opengl::helper::Overlay::Overlay(uint32_t width, uint32_t height)
    : m_width( width )
    , m_height( height )
    , m_rgbaBuffer( new uint8_t[m_width * m_height * 4] )
{ }

inastitch::opengl::helper::Overlay::~Overlay()
{
    delete[] m_rgbaBuffer;
}

void inastitch::opengl::helper::Overlay::clear()
{
    for(uint32_t r=0; r<m_height; r++) {
        for(uint32_t c=0; c<m_width; c++) {
            m_rgbaBuffer[(r*m_width + c)*4    ] = 0xFF; // R
            m_rgbaBuffer[(r*m_width + c)*4 + 1] = 0xFF; // G
            m_rgbaBuffer[(r*m_width + c)*4 + 2] = 0xFF; // B
            m_rgbaBuffer[(r*m_width + c)*4 + 3] = 0x00; // A
        }
    }
};

void inastitch::opengl::helper::Overlay::putChar(uint32_t xOffset, uint32_t yOffset, uint8_t asciiCode)
{
    for(uint32_t r=0; r<5; r++) {
        for(uint32_t c=0; c<5; c++) {
            m_rgbaBuffer[((r+yOffset)*m_width + (c+xOffset))*4 + 3] = digitBits.at(asciiCode).at(r*5+c);
        }
    }
};

void inastitch::opengl::helper::Overlay::putDigit(uint32_t xOffset, uint32_t yOffset, uint8_t digit)
{
    putChar(xOffset, yOffset, digit+48);
};

void inastitch::opengl::helper::Overlay::putNumber(uint32_t xOffset, uint32_t yOffset, uint32_t number, uint8_t digitCount)
{
    for(uint32_t p=0; p<digitCount; p++)
    {
        const auto n = number % 10;
        putDigit(xOffset + (digitCount*(5+1) - p*(5+1)), yOffset, n);
        number /= 10;
    }
};

void inastitch::opengl::helper::Overlay::putString(uint32_t xOffset, uint32_t yOffset, const char * const string, uint8_t charCount)
{
    for(uint32_t p=0; p<charCount; p++)
    {
        const auto c = string[p];
        putChar(xOffset + p*(5+1), yOffset, c);
    }
};
