// Copyright (C) 2020 Inatech srl
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

// Local includes:
#include "inastitch/opencv/include/HomographyMatrix.hpp"

// OpenCv includes:
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>

// Std includes:
#include <iostream>
#include <chrono>


bool inastitch::opencv::HomographyMatrix::find(
    const std::string &centerImagePath, const std::string &rightImagePath,
    bool isFlipped, uint32_t minMatchCount,
    float matrix[3][3])
{
    cv::Mat centerImage = cv::imread(centerImagePath);
    cv::Mat rightImage = cv::imread(rightImagePath);

    if(isFlipped) {
        cv::flip(centerImage, centerImage, 1);
        cv::flip(rightImage, rightImage, 1);
    }

    // Part1: detect keypoints and extract
    // Detect and describe features
    std::function detectAndDescribe = [](const cv::Mat &image, const char *imgDesc)
    {
        // convert the image to grayscale
        cv::Mat grayImage;
        cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);

        // detect and extract features from the image
        // https://docs.opencv.org/4.4.0/d7/d60/classcv_1_1SIFT.html
        auto desc = cv::SIFT::create();

        const auto t1 = std::chrono::high_resolution_clock::now();
        // https://docs.opencv.org/4.4.0/d0/d13/classcv_1_1Feature2D.html
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat features;
        desc->detectAndCompute(grayImage, cv::noArray(), keypoints, features);
        const auto t2 = std::chrono::high_resolution_clock::now();
        std::cout << "detectAndCompute:" << std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count() << "us" << std::endl;

        // TODO: make this DLT
        {
            cv::Mat kpsImage;
            cv::drawKeypoints(image, keypoints, kpsImage);
            cv::imwrite(std::string("/tmp/inastitch_kps") + imgDesc + ".jpg", kpsImage);
        }

        return std::make_tuple(keypoints, features);
    };

    auto [ kpsC, featC ] = detectAndDescribe(centerImage, "C");
    auto [ kpsR, featR ] = detectAndDescribe(rightImage,  "R");

    std::cout << "center: keypointCount=" << kpsC.size() << std::endl;
    std::cout << "right : keypointCount=" << kpsR.size() << std::endl;

    // Part2: match features between two images
    // compute the raw matches and initialize the list of actual matches

    // https://docs.opencv.org/4.4.0/db/d39/classcv_1_1DescriptorMatcher.html
    auto matcher = cv::DescriptorMatcher::create( cv::DescriptorMatcher::MatcherType
::BRUTEFORCE);
    const auto t1 = std::chrono::high_resolution_clock::now();
    std::vector<std::vector<cv::DMatch>> rawMatchesR;
    matcher->knnMatch(featR /* query */, featC /* train */, rawMatchesR, 2 /* two best matches */ );
    const auto t2 = std::chrono::high_resolution_clock::now();
    std::cout << "knnMatch:" << std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count() << "us" << std::endl;

    std::cout << "rawMatchCount=" << rawMatchesR.size() << std::endl;

    std::function loopOverRawMatches = [&](
        const std::vector<std::vector<cv::DMatch>> &rawMatches,
        float ratio)
    {
        std::vector<cv::DMatch> matches;

        for(const auto &m : rawMatches)
        {
            // https://docs.opencv.org/4.4.0/d4/de0/classcv_1_1DMatch.html
            if( (m.size() == 2) && (m[0].distance < m[1].distance * ratio) )
            {
                matches.push_back(m[0]);
            }
        }

        // TODO: make this DLT
        {
            // draw matches
            cv::Mat matchesImage;
            cv::drawMatches(rightImage, kpsR, centerImage, kpsC, matches, matchesImage);
            cv::imwrite("/tmp/inastitch_matches.jpg", matchesImage);
        }

        return matches;
    };

    auto matchesR = loopOverRawMatches(rawMatchesR, ratio);

    std::cout << "matchCount=" << matchesR.size() << std::endl;

    // Part3: computing a homography matrix
    std::function computingOfHomographyMatrix = [minMatchCount](
        const std::vector<cv::DMatch> &matches,
        const std::vector<cv::KeyPoint> &kpsC,
        const std::vector<cv::KeyPoint> &kpsR,
        float reprojThresh)
    {
        cv::Mat homoM;
        // homography matrix requires at least 4 matches
        if(matches.size() > minMatchCount)
        {
            // construct the two sets of points
            // https://docs.opencv.org/trunk/d2/d29/classcv_1_1KeyPoint.html
            std::vector<cv::Point2f> ptsC, ptsR;
            for(const auto &m : matches)
            {
                ptsC.push_back( kpsC.at(m.queryIdx).pt );
                ptsR.push_back( kpsR.at(m.trainIdx).pt );
            }

            std::cout << "PointsC=" << ptsC.size() << std::endl;
            std::cout << "PointsR=" << ptsR.size() << std::endl;

            // compute the homography between the two sets of points
            // https://docs.opencv.org/4.4.0/d9/d0c/group__calib3d.html
            const auto t1 = std::chrono::high_resolution_clock::now();
            homoM = cv::findHomography(ptsR, ptsC, cv::RANSAC, reprojThresh);
            const auto t2 = std::chrono::high_resolution_clock::now();
            std::cout << "findHomography:" << std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count() << "us" << std::endl;
        }

        return homoM;
    };

    auto homoMatrixR = computingOfHomographyMatrix(matchesR, kpsR, kpsC, reprojThresh);

    std::cout << "Original OpenCV homography matrix:" << std::endl;
    std::cout << homoMatrixR << std::endl;

    if(homoMatrixR.empty())
    {
        std::cout << "Empty matrix, homography failed" << std::endl;
        return false;
    }

    // invert matrix
    // Note: the current matrix describes the transformation from 'source pixel' to 'destination pixel'.
    // => this is not convenient when rendering, since we start from the 'destination pixel' and we need
    //    to know its 'source pixel'.
    // Note: OpenCV warpPerspective will automatically invert the matrix with flag 'WARP_INVERSE_MAP'.
    cv::Mat homoMatrixRInv;
    cv::invert(homoMatrixR, homoMatrixRInv);

    std::cout << "Inverted OpenCV homography matrix:" << std::endl;
    std::cout << homoMatrixRInv << std::endl;

    {
        cv::Mat panoR;
        cv::warpPerspective(
            rightImage, panoR, homoMatrixRInv,
            { centerImage.cols + rightImage.cols, rightImage.rows },
            // cv::Size = ( maxCols, maxRows )
            cv::INTER_LINEAR //| cv::WARP_INVERSE_MAP
        );
        cv::imwrite("inastitch_panoR.jpg", panoR);

        // Build panorama
        const auto panoWidth  = centerImage.cols + rightImage.cols;
        const auto panoHeight = centerImage.rows;
        cv::Mat pano(panoHeight, panoWidth, centerImage.type());
        pano.setTo(0);

        panoR.copyTo(pano);

        centerImage.copyTo(pano(
            cv::Rect(0, 0, centerImage.cols, centerImage.rows)));

        cv::imwrite("inastitch_pano.jpg", pano);
    }

    // normalize matrix for OpenGL
    // Note: currently the matrix expects coordinates in pixel.
    //       If using this matrix in an OpenGL shader, coordinates will be
    //       in the range of (-1.0, 1.0), instead of pixels.

    float xFlipAndShiftMatrix_data[] = {
         -1.0, 0.0, 1.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0
    };
    cv::Mat xFlipAndShiftMatrix(3, 3, CV_32F, xFlipAndShiftMatrix_data);

    float scale1Matrix_data[] = {
         (rightImage.cols*1.0f), 0.0, 0.0,
         0.0, (rightImage.rows*1.0f), 0.0,
         0.0, 0.0, 1.0
    };
    cv::Mat scale1Matrix(3, 3, CV_32F, scale1Matrix_data);

    float descaleMatrix_data[] = {
         1.0f/(rightImage.cols*1.0f), 0.0, 0.0,
         0.0, 1.0f/(rightImage.rows*1.0f), 0.0,
         0.0, 0.0, 1.0
    };
    cv::Mat descaleMatrix(3, 3, CV_32F, descaleMatrix_data);

    // Mat needs to be of the same type
    homoMatrixR.convertTo(homoMatrixR, CV_32F);

    cv::Mat homoMatrixRInvNorm = descaleMatrix * homoMatrixR * scale1Matrix;

    if(isFlipped) {
        homoMatrixRInvNorm = xFlipAndShiftMatrix * homoMatrixRInvNorm;
    }

    std::cout << "OpenGL-ready homography matrix:" << std::endl;
    std::cout << homoMatrixRInvNorm << std::endl;

    // write back result to output parameter
    for(uint32_t row=0; row<3; row++) {
        for(uint32_t col=0; col<3; col++) {
            matrix[row][col] = homoMatrixRInvNorm.at<float>(row,col);
        }
    }

    // homography matrix is valid
    return true;
}
