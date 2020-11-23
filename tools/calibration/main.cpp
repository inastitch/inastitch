// Copyright (C) 2020 Inatech srl
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

// Stitching calibration using OpenCV
// Created on 06.08.2020
// by Vincent Jordan
// inspired by:
// https://www.pyimagesearch.com/2016/01/11/opencv-panorama-stitching/

// Local includes:
#include "inastitch/opencv/include/HomographyMatrix.hpp"
#include "version.h"

// Boost includes:
#include <boost/program_options.hpp>
namespace po = boost::program_options;

// Taocpp/json includes:
#include <tao/json.hpp>
#include <tao/json/contrib/traits.hpp>
// Note: see taocpp documentation at
// https://github.com/taocpp/json/blob/master/doc/Common-Use-Cases.md

// OpenCv includes:
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>

// Std includes:
#include <iostream>
#include <fstream>
#include <chrono>

/// @brief Standalone calibration tool
///
/// Take two input images (center and right) and generate the homography matrix for the right image
/// to stitch on the center image. The matrix is normalized for processing with OpenGL pixel shader.
int main(int argc, char** argv)
{
    std::string centerImagePath, rightImagePath;
	float ratio, reprojThresh;
    std::string jsonPath;
    bool isFlipped = false;

    std::cout << "Inatech calibration tool "
              << inastitch::version::GIT_COMMIT_TAG
              << " (" << inastitch::version::GIT_COMMIT_DATE << ")"
              << std::endl;

	{
        po::options_description desc("Allowed options");
	    desc.add_options()
	        ("center,c", po::value<std::string>(&centerImagePath)->default_value("center.jpg"),
	         "Path to the center image")
	        ("right,r", po::value<std::string>(&rightImagePath)->default_value("right.jpg"),
	         "Path to the right image")
		
            ("ratio", po::value<float>(&ratio)->default_value(0.75),
	         "Ratio")
            ("reproj-thresh", po::value<float>(&reprojThresh)->default_value(4.0),
	         "Reprojection threshold")

            ("json", po::value<std::string>(&jsonPath),
             "Path to JSON matrix file")
	    
            ("flip", "Flip input vertically")
            ("help,h", "Show this help message")
	    ;
	
        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);
	    po::notify(vm);
	
	    if(vm.count("help")) {
	        std::cout << desc << std::endl;
	        return 0;
	    }
        
        if(vm.count("flip")) {
	        isFlipped = true;
	    }
	}

    cv::Mat centerImage = cv::imread(centerImagePath);
    cv::Mat rightImage = cv::imread(rightImagePath);

    // HomographyMatrix::find expects RGBA buffers
    cv::Mat centerImageRgba, rightImageRgba;
    {
        cv::cvtColor(centerImage, centerImageRgba, cv::COLOR_BGRA2RGBA);
        cv::cvtColor(rightImage, rightImageRgba, cv::COLOR_BGRA2RGBA);
    }

    float homographyMatrix[3][3];
    const bool isMatrixValid = inastitch::opencv::HomographyMatrix::find(
        centerImageRgba.data, centerImage.size().width, centerImage.size().height,
        rightImageRgba.data, rightImage.size().width, rightImage.size().height,
        isFlipped,
        0,
        true, /* isDebugImageDumped */
        homographyMatrix
    );

    if(!isMatrixValid)
    {
        std::cout << "Homography matrix not found" << std::endl;
        return -1;
    }

    // Write homography matrix to JSON file
    if(!jsonPath.empty())
    {
        tao::json::value json = tao::json::from_file(jsonPath);

        auto cvMat3ToJson =
        [](const float matrix[3][3], tao::json::basic_value<tao::json::traits>::array_t &array)
        {
            // clear previous matrix data
            array.clear();

            for(uint32_t rowIdx=0; rowIdx<3; rowIdx++)
            {
                for(uint32_t colIdx=0; colIdx<3; colIdx++)
                {
                    array.push_back(matrix[rowIdx][colIdx]);
                }
            }
        };

        if(!isFlipped)
        {
            cvMat3ToJson(homographyMatrix, json.at("texture2").at("warp").get_array());
        }
        else
        {
            cvMat3ToJson(homographyMatrix, json.at("texture1").at("warp").get_array());
        }

        const std::string jsonStr = tao::json::to_string(json, 4);

        std::ofstream jsonFile;
        jsonFile.open(jsonPath);
        jsonFile << jsonStr << std::endl;
        jsonFile.close();
    }
    
	return 0;
}
