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

// Dlib
#include <dlib/opencv.h>
#include <dlib/image_processing/frontal_face_detector.h>

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
    float rotationAngleDeg = 0.0;
    float maxWarpAngleDeg;

    uint16_t cropWidth;
    uint16_t cropHeight;

    std::string outputPath;

    bool isAngleOverwritten = false;

    std::cout << "Calibration tool "
              << inastitch::version::GIT_COMMIT_TAG
              << " (" << inastitch::version::GIT_COMMIT_DATE << ")"
              << std::endl;

	{
        po::options_description desc("Allowed options");
        desc.add_options()
            ("input-image", po::value<std::vector<std::string>>(&inputImagePaths),
             "Input images (a list of space-separated image paths)")
            ("angle,a", po::value<float>(&rotationAngleDeg),
             "Force rotation angle (disable face detection), in degrees")

            ("max-warp-angle", po::value<float>(&maxWarpAngleDeg)->default_value(45.0),
             "Max angle for warping input, in degrees")
            ("ratio", po::value<float>(&ratio)->default_value(0.75),
	         "Ratio")
            ("reproj-thresh", po::value<float>(&reprojThresh)->default_value(4.0),
	         "Reprojection threshold")

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

        if(vm.count("angle")) {
            isAngleOverwritten = true;
        }
	}
    const float maxWarpAngleRad = maxWarpAngleDeg * 3.141592653589793 / 180.0;

    // Part0: Load input images
    const auto inputImageCount = inputImagePaths.size();
    std::vector<cv::Mat> inputImages(inputImageCount);
    for(uint32_t pathIdx=0; pathIdx<inputImageCount; pathIdx++)
    {
        inputImages.at(pathIdx) = cv::imread(inputImagePaths.at(pathIdx));
    }
    std::cout << "Loaded " << inputImages.size() << " images" << std::endl;

    // Part1: Find image features
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

    // Part2: Pairwise matching between features in different images
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
    // Find camera K and R matrices
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

    // Part4: Rotation matrix decomposition
    // From rotation matrix to Euler angles
    std::vector<float> yAngles(inputImageCount);
    {
        for(uint32_t imgIdx=0; imgIdx<inputImageCount; imgIdx++)
        {
            cv::Mat rotationVect;
            // See: "cv::Rodrigues" in https://docs.opencv.org/4.4.0/d9/d0c/group__calib3d.html
            cv::Rodrigues(cameraParams.at(imgIdx).R, rotationVect);

            // transform to degree for printing
            const auto rotationVectDeg = rotationVect * (180.0/3.141592653589793238463);
            std::cout << "Image" << imgIdx << ": RotationVect=" << rotationVectDeg << std::endl;

            // keep rotation on Y-axis only
            yAngles.at(imgIdx) = rotationVect.at<float>(1);
        }
    }

    float additionalRotationAngleRad;
    if(isAngleOverwritten)
    {
        additionalRotationAngleRad = rotationAngleDeg * 3.141592653589793 / 180.0;
    }
    else
    {
        // Part5: Detect faces
        uint32_t faceImgIdx;
        cv::Point2f faceCenterPoint;
        {
            dlib::frontal_face_detector detector = dlib::get_frontal_face_detector();
            for(uint32_t imgIdx=0; imgIdx<inputImageCount; imgIdx++)
            {
                // Convert OpenCV image to Dlib image
                // See: http://dlib.net/imaging.html#cv_image
                dlib::cv_image<dlib::bgr_pixel> dlibImage(inputImages.at(imgIdx));

                const std::vector<dlib::rectangle> faceRects = detector(dlibImage);
                for(const auto &rect : faceRects)
                {
                    std::cout << "Image" << imgIdx << ": FaceRect " << rect << std::endl;

                    faceImgIdx = imgIdx;
                    // center point
                    const auto faceCenterX = rect.tl_corner().x() + (rect.width()/2);
                    const auto faceCenterY = rect.tl_corner().y() + (rect.height()/2);

                    faceCenterPoint = cv::Point2f(faceCenterX, faceCenterY);
                }
            }
        }
        // TODO: this code only selects the last face

        // Part6: Find face angle
        float faceAngleRad;
        {
            auto warperCreator = cv::PlaneWarper();
            auto warper = warperCreator.create(cameraParams.at(0).focal);

            // Image center point
            const auto imageSize = inputImages.at(faceImgIdx).size();
            cv::Point2f imageCenterPoint = cv::Point2f(imageSize.width/2.0f, imageSize.height/2.0f);

            // warp the face center point
            // Make float K
            cv::Mat_<float> Kf;
            cameraParams.at(faceImgIdx).K().convertTo(Kf, CV_32F);
            const auto warpImageCenterPoint = warper->warpPoint(
                        imageCenterPoint, Kf, cameraParams.at(faceImgIdx).R);
            const auto warpFaceCenterPoint  = warper->warpPoint(
                        faceCenterPoint, Kf, cameraParams.at(faceImgIdx).R);

            // face angle
            const auto xRatio = warpImageCenterPoint.x / warpFaceCenterPoint.x;
            faceAngleRad = yAngles.at(faceImgIdx) * xRatio;

            const auto faceAngleDeg = faceAngleRad * (180.0/3.141592653589793238463);
            std::cout << "Face ratio: " << xRatio << " angle: " << faceAngleDeg << std::endl;
        }
        additionalRotationAngleRad = -faceAngleRad;
    }


    // Part7: Apply additional rotation angle
    {
        const auto rotationAngleRad = additionalRotationAngleRad;

        // make rotation matrix
        cv::Mat R_y = (cv::Mat_<float>(3,3) <<
           std::cos(rotationAngleRad),  0.0, std::sin(rotationAngleRad),
           0.0,                 1.0, 0.0,
           -std::sin(rotationAngleRad), 0.0, std::cos(rotationAngleRad)
        );
        // TODO: replace with cv::getRotationMatrix3D (?)

        // apply rotation matrix to each camera
        for(uint32_t imgIdx=0; imgIdx<inputImageCount; imgIdx++)
        {
            auto R_rot = R_y * cameraParams.at(imgIdx).R;
            cameraParams.at(imgIdx).R = R_rot;
        }
    }

    // Part8: Warp images
    std::vector<cv::Mat> warpedImages;
    std::vector<cv::Rect> warpedImageRects;
    cv::Rect warpedCropRect;
    {
        auto warperCreator = cv::PlaneWarper();
        // Note: this warper has a GPU version

        auto warper = warperCreator.create(cameraParams.at(0).focal);
        // TODO: this assume a similar focal for all pictures
        // OpenCV sample code calcultates a median focal here.

        for(uint32_t imgIdx=0; imgIdx<inputImageCount; imgIdx++)
        {
            if(std::abs(yAngles.at(imgIdx)+additionalRotationAngleRad) < maxWarpAngleRad)
            {
                // Make float K
                cv::Mat_<float> Kf;
                cameraParams.at(imgIdx).K().convertTo(Kf, CV_32F);

                // warp image
                auto &warpedImage = warpedImages.emplace_back();
                warper->warp(inputImages.at(imgIdx), Kf, cameraParams.at(imgIdx).R,
                             cv::INTER_LINEAR, cv::BORDER_CONSTANT,
                             warpedImage);

                // warp corners
                auto &warpedImageRect = warpedImageRects.emplace_back();
                warpedImageRect = warper->warpRoi(
                    inputImages.at(imgIdx).size(), Kf, cameraParams.at(imgIdx).R
                );

                std::cout << "Image" << imgIdx << ": ROI " << warpedImageRect << std::endl;
                //cv::imwrite("warped" + std::to_string(imgIdx) + ".jpg", warpedImage);
            }
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
    const auto warpedImageCount = warpedImages.size();

    // Part9: Make panorama and crop
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

        for(uint32_t imgIdx=0; imgIdx<warpedImageCount; imgIdx++)
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

            for(uint32_t imgIdx=0; imgIdx<warpedImageCount; imgIdx++)
            {
                const auto &roi = warpedImageRects.at(imgIdx);

                warpedImages.at(imgIdx).copyTo(pano(
                    cv::Rect(roi.x - offsetX, roi.y - offsetY, roi.width, roi.height)));
            }
            cv::imwrite("full_" + outputPath, pano);
        }
        // Note: depending on the additional rotation angle, the panoramic result may be very large.
        // Solution:
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
    
	return 0;
}
