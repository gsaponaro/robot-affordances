/*
 * Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */

#ifndef DESC_HELPERS_H
#define DESC_HELPERS_H

#include <algorithm>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <yarp/os/LogStream.h>
#include <yarp/sig/Image.h>

/**********************************************************/
std::vector<int> unique(const cv::Mat &input, bool shouldSort=false);

/**********************************************************/
/**
  * Convert YARP IplImage (any type) to OpenCV Mat.
  */
template <class T>
cv::Mat iplToMat(yarp::sig::ImageOf<T> &ipl)
{
    return cv::cvarrToMat(static_cast<IplImage*>(ipl.getIplImage()));
}

/**********************************************************/
bool binaryMaskFromLabel(const cv::Mat &input, const int label, cv::Mat &output);

/**********************************************************/
bool largestContour(std::vector< std::vector<cv::Point> > cnt, int largestIdx);

#endif
