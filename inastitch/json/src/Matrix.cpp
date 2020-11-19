// Copyright (C) 2020 Inatech srl
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

// Local includes:
#include "inastitch/json/include/Matrix.hpp"

// GLM includes:
#include <glm/glm.hpp>

// Taocpp/json includes:
#include <tao/json.hpp>
#include <tao/json/contrib/traits.hpp>
// Note: see taocpp documentation at
// https://github.com/taocpp/json/blob/master/doc/Common-Use-Cases.md

void inastitch::json::GlmJson::glmMat4ToJson(const glm::mat4 &mat, tao::json::basic_value<tao::json::traits>::array_t &array)
{
    for(uint32_t rowIdx=0; rowIdx<4; rowIdx++)
    {
        for(uint32_t colIdx=0; colIdx<4; colIdx++)
        {
            // Note: column first
            array.push_back(mat[colIdx][rowIdx]);
        }
    }
};

void inastitch::json::GlmJson::glmMat3ToJson(const glm::mat3 &mat, tao::json::basic_value<tao::json::traits>::array_t &array)
{
    for(uint32_t rowIdx=0; rowIdx<3; rowIdx++)
    {
        for(uint32_t colIdx=0; colIdx<3; colIdx++)
        {
            // Note: column first
            array.push_back(mat[colIdx][rowIdx]);
        }
    }
};

void inastitch::json::GlmJson::jsonToGlmMat4(const std::vector<float> &array, glm::mat4 &mat)
{
    for(uint32_t rowIdx=0; rowIdx<4; rowIdx++)
    {
        for(uint32_t colIdx=0; colIdx<4; colIdx++)
        {
            // Note: column first
            mat[colIdx][rowIdx] = array.at(rowIdx*4 + colIdx);
        }
    }
};

void inastitch::json::GlmJson::jsonToGlmMat3(const std::vector<float> &array, glm::mat3x3 &mat)
{
    for(uint32_t rowIdx=0; rowIdx<3; rowIdx++)
    {
        for(uint32_t colIdx=0; colIdx<3; colIdx++)
        {
            // Note: column first
            mat[colIdx][rowIdx] = array.at(rowIdx*3 + colIdx);
        }
    }
};
