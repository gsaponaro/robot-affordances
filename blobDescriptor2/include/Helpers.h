/*
 * Copyright: (C) 2012-2015 POETICON++, European Commission FP7 project ICT-288382
 * Copyright: (C) 2015 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */

#ifndef DESC_HELPERS_H
#define DESC_HELPERS_H

#include <algorithm>
#include <vector>

#include <opencv2/imgproc/imgproc.hpp>

#include <yarp/os/LogStream.h>
#include <yarp/sig/Image.h>

using namespace cv;
using namespace yarp::os;
using namespace yarp::sig;

/**********************************************************/
std::vector<int> unique(const Mat &input, bool shouldSort=false);

/**********************************************************/
/**
  * Convert YARP IplImage (any type) to OpenCV Mat.
  */
template <class T>
Mat iplToMat(ImageOf<T> &ipl)
{
    return cvarrToMat(ipl.getIplImage());
}

/**********************************************************/
bool binaryMaskFromLabel(const cv::Mat &input, const int label, cv::Mat &output);

/**********************************************************/
bool largestContour(std::vector< std::vector<Point> > cnt, int largestIdx);

#endif
