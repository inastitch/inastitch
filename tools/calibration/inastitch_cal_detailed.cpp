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
#include <limits>

/// @brief Standalone calibration tool
///
/// Take a set of images and generate the camera (K) and rotation (R) matrices for each images.
/// Combined, the K and the R matrices make a homography matrix (H), like
/// @code inastitch_cal_simple. Having the homography matrix split up into K and R is more
/// convenient because it allows to change the rotation matrix before stitching, hence the
/// possibility to rotate the result by a given angle.
///
/// @note The first image will have no rotation, i.e., the R matrix will be an identity matrix.
/// Other R matrices are rotations relative to the first image.
int main(int argc, char** argv)
{
    std::vector<std::string> inputImagePaths;
	float ratio, reprojThresh;
    float rotationAngle;

    uint16_t cropWidth;
    uint16_t cropHeight;

    std::string outputPath;

    std::cout << "Calibration tool "
              << inastitch::version::GIT_COMMIT_TAG
              << " (" << inastitch::version::GIT_COMMIT_DATE << ")"
              << std::endl;

	{
        po::options_description desc("Allowed options");
        desc.add_options()
            ("input-image", po::value<std::vector<std::string>>(&inputImagePaths),
             "Input images (a list of space-separated image paths)")

            ("ratio", po::value<float>(&ratio)->default_value(0.75),
	         "Ratio")
            ("reproj-thresh", po::value<float>(&reprojThresh)->default_value(4.0),
	         "Reprojection threshold")
            ("angle,a", po::value<float>(&rotationAngle)->default_value(0.0),
             "Rotation angle")

            ("crop-width", po::value<uint16_t>(&cropWidth)->default_value(640),
             "Crop width")
            ("crop-height", po::value<uint16_t>(&cropHeight)->default_value(480),
             "Crop width")

            ("output,o", po::value<std::string>(&outputPath)->default_value("output.jpg"),
             "Output path")

            ("help,h", "Show this help message")
	    ;
	
        po::positional_options_description p;
        p.add("input-image", -1);

        po::variables_map vm;
        po::store(po::command_line_parser(argc, argv).
                  options(desc).positional(p).run(), vm);
        po::notify(vm);
	
	    if(vm.count("help")) {
	        std::cout << desc << std::endl;
	        return 0;
	    }

        if(!vm.count("input-image")) {
            std::cout << "No image to process" << std::endl;
            return 0;
        }
	}

    const auto inputImageCount = inputImagePaths.size();
    std::vector<cv::Mat> inputImages(inputImageCount);

    // Part0: Load input images
    for(uint32_t pathIdx=0; pathIdx<inputImageCount; pathIdx++)
    {
        inputImages.at(pathIdx) = cv::imread(inputImagePaths.at(pathIdx));
    }
    std::cout << "Loaded " << inputImages.size() << " images" << std::endl;

    // Part1: Find features
    std::vector<cv::detail::ImageFeatures> features(inputImageCount);
    {
        auto finder = cv::SIFT::create();
        // TODO: SIFT might not be the best way to do this
        //       Add support for other feature detectors (ORB, SURF, AKAZE)

        for(uint32_t imgIdx=0; imgIdx<inputImageCount; imgIdx++)
        {
            computeImageFeatures(finder, inputImages.at(imgIdx), features.at(imgIdx));
            std::cout << "Image" << imgIdx << ": found "
                      << features.at(imgIdx).keypoints.size() << " features" << std::endl;
        }
    }

    // Part2: Pairwise matching
    std::vector<cv::detail::MatchesInfo> pairwiseMatches(inputImageCount);
    {
        auto matcher = cv::detail::BestOf2NearestMatcher(false /* tryCuda */, ratio);
        matcher(features, pairwiseMatches);
        matcher.collectGarbage();
    }
    for(uint32_t imgIdx=0; imgIdx<inputImageCount; imgIdx++)
    {
        std::cout << "Image" << imgIdx << ": found "
                  << pairwiseMatches.at(imgIdx).matches.size() << " matches" << std::endl;
    }

    // Part3: Homography estimation
    std::vector<cv::detail::CameraParams> cameraParams(inputImageCount);
    {
        auto estimator = cv::detail::HomographyBasedEstimator();
        const bool isSuccess = estimator(features, pairwiseMatches, cameraParams);

        if(!isSuccess)
        {
            std::cout << "Homography estimation failed" << std::endl;
        }
    }
    for(uint32_t imgIdx=0; imgIdx<inputImageCount; imgIdx++)
    {
        std::cout << "Image" << imgIdx << ": K="
                  << cameraParams.at(imgIdx).K() << std::endl;
        std::cout << "Image" << imgIdx << ": R="
                  << cameraParams.at(imgIdx).R << std::endl;
    }

    // Convert R matrices from integer into float
    {
        for(uint32_t imgIdx=0; imgIdx<inputImageCount; imgIdx++)
        {
            cv::Mat_<float> Rf;
            cameraParams.at(imgIdx).R.convertTo(Rf, CV_32F);
            cameraParams.at(imgIdx).R = Rf;
        }
    }

    // Bundle adjustement
    {
        auto adjuster = cv::detail::BundleAdjusterRay();
        adjuster.setConfThresh(1.0f);
        adjuster(features, pairwiseMatches, cameraParams);
    }

    // Apply additional rotation
    {
        const auto angleRad = rotationAngle * 3.141592653589793 / 180.0;
        cv::Mat R_y = (cv::Mat_<float>(3,3) <<
           std::cos(angleRad),  0.0, std::sin(angleRad),
           0.0,                 1.0, 0.0,
           -std::sin(angleRad), 0.0, std::cos(angleRad)
        );
        // TODO: replace with cv::getRotationMatrix3D (?)

        for(uint32_t imgIdx=0; imgIdx<inputImageCount; imgIdx++)
        {
            auto R_rot = R_y * cameraParams.at(imgIdx).R;
            cameraParams.at(imgIdx).R = R_rot;
        }
    }

    // Part4: Warp images
    std::vector<cv::Mat> warpedImages(inputImageCount);
    std::vector<cv::Rect> warpedImageRects(inputImageCount);
    cv::Rect warpedCropRect;
    {
        auto warperCreator = cv::PlaneWarper();
        // Note: this warper has a GPU version

        auto warper = warperCreator.create(cameraParams.at(0).focal);
        // TODO: this assume a similar focal for all pictures
        // OpenCV sample code calcultates a median focal here.

        for(uint32_t imgIdx=0; imgIdx<inputImageCount; imgIdx++)
        {
            // Make float K
            cv::Mat_<float> Kf;
            cameraParams.at(imgIdx).K().convertTo(Kf, CV_32F);

            warper->warp(inputImages.at(imgIdx), Kf, cameraParams.at(imgIdx).R,
                         cv::INTER_LINEAR, cv::BORDER_CONSTANT,
                         warpedImages.at(imgIdx));

            // warp corners
            warpedImageRects.at(imgIdx) = warper->warpRoi(
                inputImages.at(imgIdx).size(), Kf, cameraParams.at(imgIdx).R
            );

            std::cout << "Image" << imgIdx << ": ROI " << warpedImageRects.at(imgIdx) << std::endl;
            //cv::imwrite("warped" + std::to_string(imgIdx) + ".jpg", warpedImages.at(imgIdx));
        }

        {
            // Warp cropped image corners with no rotation
            cv::Size cropSize{cropWidth, cropHeight};

            cv::Mat_<float> K;
            cameraParams.at(0).K().convertTo(K, CV_32F);

            cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
            // No rotation

            warpedCropRect = warper->warpRoi(cropSize, K, R);
            std::cout << "WarpedCropRect: " << warpedCropRect << std::endl;
        }
    }

    // Part5: Make panorama and crop
    {
        // Find min and max wrapped pixel coordinates
        int32_t minX = std::numeric_limits<int32_t>::max();
        int32_t minY = std::numeric_limits<int32_t>::max();
        int32_t maxX = std::numeric_limits<int32_t>::min();
        int32_t maxY = std::numeric_limits<int32_t>::min();
        auto updateMinMax = [&](const cv::Rect &roi)
        {
            // min top-left
            minX = std::min(minX, roi.x);
            minY = std::min(minY, roi.y);
            // max bottom-right
            maxX = std::max(maxX, roi.x + roi.width);
            maxY = std::max(maxY, roi.y + roi.height);
        };

        for(uint32_t imgIdx=0; imgIdx<inputImageCount; imgIdx++)
        {
            const auto &roi = warpedImageRects.at(imgIdx);

            updateMinMax(roi);
        }
        updateMinMax(warpedCropRect);

        std::cout << "Min(" << minX << "," << minY << ") "
                  << "Max(" << maxX << "," << maxY << ")" << std::endl;

        const auto offsetX = minX;
        const auto offsetY = minY;
        const auto sizeX = maxX - minX;
        const auto sizeY = maxY - minY;
        std::cout << "Pano: x=" << offsetX << ",y=" << offsetY << " "
                  << sizeX << "x" << sizeY << std::endl;

        // Build panorama
        cv::Mat pano(sizeY, sizeX, inputImages.at(0).type());
        {
            pano.setTo(0);

            for(uint32_t imgIdx=0; imgIdx<inputImageCount; imgIdx++)
            {
                const auto &roi = warpedImageRects.at(imgIdx);

                warpedImages.at(imgIdx).copyTo(pano(
                    cv::Rect(roi.x - offsetX, roi.y - offsetY, roi.width, roi.height)));
            }
            cv::imwrite("full_" + outputPath, pano);
        }
        // Note: depending on the additional rotation angle, the panoramic result may be very large.
        // Two suggestions:
        // - use spherical transformation instead of planar
        // - if only the cropped result is needed, it is not required to warp all inputs, espcially
        //   the ones with angles resulting in very strong perspective.

        // Crop
        {
            auto warpedCropRectWithOffset = warpedCropRect;
            warpedCropRectWithOffset.x -= offsetX;
            warpedCropRectWithOffset.y -= offsetY;
            std::cout << "CropRect: " << warpedCropRectWithOffset << std::endl;

            cv::Mat crop(pano, warpedCropRectWithOffset);
            cv::imwrite(outputPath, crop);
        }
    }

    // Additional information
    // Rotation matrix decomposition
    {
        for(uint32_t imgIdx=0; imgIdx<inputImageCount; imgIdx++)
        {
            cv::Mat rotationVect;
            // See: "cv::Rodrigues" in https://docs.opencv.org/4.4.0/d9/d0c/group__calib3d.html
            cv::Rodrigues(cameraParams.at(imgIdx).R, rotationVect);

            // transform to degree
            const auto rotationVectDeg = rotationVect * (180.0/3.141592653589793238463);

            std::cout << "Image" << imgIdx << ": RotationVect=" << rotationVectDeg << std::endl;
        }
    }
    
	return 0;
}
