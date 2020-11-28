// Copyright (C) 2020 Vincent Jordan
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

// Stitching calibration using OpenCV
// Created on 27.11.2020
// inspired by:
// "stitching_detailed" sample code
// https://github.com/opencv/opencv/blob/master/samples/cpp/stitching_detailed.cpp

// Local includes:
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

#include "opencv2/stitching/detail/matchers.hpp"
#include "opencv2/stitching/detail/motion_estimators.hpp"
#include "opencv2/stitching/detail/warpers.hpp"
#include "opencv2/stitching/warpers.hpp"

// Std includes:
#include <iostream>
#include <fstream>
#include <chrono>

/// @brief Standalone calibration tool
///
/// Take two input images (center and right) and generate the homography matrix for the right image
/// to stitch on the center image. The result can be rotated by a given angle.
int main(int argc, char** argv)
{
    std::string centerImagePath, rightImagePath;
	float ratio, reprojThresh;
    float rotationAngle;

    std::cout << "Calibration tool "
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

            ("angle,a", po::value<float>(&rotationAngle)->default_value(0.0),
             "Rotation angle")
	    
            ("help,h", "Show this help message")
	    ;
	
        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);
	    po::notify(vm);
	
	    if(vm.count("help")) {
	        std::cout << desc << std::endl;
	        return 0;
	    }
	}

    cv::Mat centerImage = cv::imread(centerImagePath);
    cv::Mat rightImage = cv::imread(rightImagePath);

    // Part1: Find features
    std::vector<cv::detail::ImageFeatures> features(2);
    {
        auto finder = cv::SIFT::create();
        // TODO: SIFT might not be the best way to do this

        computeImageFeatures(finder, centerImage, features[0]);
        computeImageFeatures(finder, rightImage, features[1]);
    }
    std::cout << "Found " << features[0].keypoints.size() << " features" << std::endl;
    std::cout << "Found " << features[1].keypoints.size() << " features" << std::endl;

    // Part2: Pairwise matching
    std::vector<cv::detail::MatchesInfo> pairwiseMatches(2);
    {
        auto matcher = cv::detail::BestOf2NearestMatcher(false /* tryCuda */, ratio);
        matcher(features, pairwiseMatches);
        matcher.collectGarbage();
    }
    std::cout << "Found " << pairwiseMatches[0].matches.size() << " matches" << std::endl;
    std::cout << "Found " << pairwiseMatches[1].matches.size() << " matches" << std::endl;

    // Part3: Homography estimation
    std::vector<cv::detail::CameraParams> cameraParams(2);
    {
        auto estimator = cv::detail::HomographyBasedEstimator();
        const bool isSuccess = estimator(features, pairwiseMatches, cameraParams);

        if(!isSuccess)
        {
            std::cout << "Homography estimation failed" << std::endl;
        }
    }
    std::cout << "K0 = " << cameraParams[0].K() << std::endl;
    std::cout << "K1 = " << cameraParams[1].K() << std::endl;
    std::cout << "R0 = " << cameraParams[0].R << std::endl;
    std::cout << "R1 = " << cameraParams[1].R << std::endl;

    // Convert R matrices from integer into float
    {
        cv::Mat_<float> R0, R1;
        cameraParams[0].R.convertTo(R0, CV_32F);
        cameraParams[1].R.convertTo(R1, CV_32F);

        cameraParams[0].R = R0;
        cameraParams[1].R = R1;
    }

    // Bundle adjustement
    {
        auto adjuster = cv::detail::BundleAdjusterRay();
        adjuster.setConfThresh(1.0f);
        adjuster(features, pairwiseMatches, cameraParams);
    }

    // Rotate
    {
        const auto angleRad = rotationAngle * 3.141592653589793 / 180.0;
        cv::Mat R_y = (cv::Mat_<float>(3,3) <<
           std::cos(angleRad),  0.0, std::sin(angleRad),
           0.0,                 1.0, 0.0,
           -std::sin(angleRad), 0.0, std::cos(angleRad)
        );
        auto R0_rot = R_y * cameraParams[0].R;
        auto R1_rot = R_y * cameraParams[1].R;

        cameraParams[0].R = R0_rot;
        cameraParams[1].R = R1_rot;
    }

    // Part4: Warp images
    std::vector<cv::Mat> warpedImages(2);
    std::vector<cv::Rect> warpedImageRects(2);
    {
        auto warperCreator = cv::PlaneWarper();
        auto warper = warperCreator.create(1000.0f);

        // Make float K
        cv::Mat_<float> K0, K1;
        cameraParams[0].K().convertTo(K0, CV_32F);
        cameraParams[1].K().convertTo(K1, CV_32F);

        warper->warp(centerImage, K0, cameraParams[0].R,
                     cv::INTER_LINEAR, cv::BORDER_CONSTANT,
                     warpedImages[0]);
        warper->warp(rightImage, K1, cameraParams[1].R,
                     cv::INTER_LINEAR, cv::BORDER_CONSTANT,
                     warpedImages[1]);

        // warp corners
        warpedImageRects[0] = warper->warpRoi(centerImage.size(), K0, cameraParams[0].R);
        warpedImageRects[1] = warper->warpRoi(rightImage.size(), K1, cameraParams[1].R);
    }

    std::cout << "roi0: " << warpedImageRects[0] << std::endl;
    std::cout << "roi1: " << warpedImageRects[1] << std::endl;

    cv::imwrite("inastitch_warped0.jpg", warpedImages[0]);
    cv::imwrite("inastitch_warped1.jpg", warpedImages[1]);

    {
        // find min and max pixel coord
        const auto &roi0 = warpedImageRects[0];
        const auto &roi1 = warpedImageRects[1];

        const auto minX = std::min(roi0.x, roi1.x);
        const auto minY = std::min(roi0.y, roi1.y);
        const auto maxX = std::max(roi0.x + roi0.width, roi1.x + roi1.width);
        const auto maxY = std::max(roi0.y + roi0.height, roi1.y + roi1.width);
        std::cout << "Min(" << minX << "," << minY << ") "
                  << "Max(" << maxX << "," << maxY << ")" << std::endl;

        const auto offsetX = minX;
        const auto offsetY = minY;
        const auto sizeX = maxX - minX;
        const auto sizeY = maxY - minY;

        // Build panorama
        cv::Mat pano(sizeY, sizeX, centerImage.type());
        pano.setTo(0);

        warpedImages[1].copyTo(pano(
            cv::Rect(roi1.x - offsetX, roi1.y - offsetY, roi1.width, roi1.height)));

        warpedImages[0].copyTo(pano(
            cv::Rect(roi0.x - offsetX, roi0.y - offsetY, roi0.width, roi0.height)));

        cv::imwrite("inastitch_pano.jpg", pano);
    }
    
	return 0;
}
