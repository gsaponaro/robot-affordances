/*
 * Copyright: (C) 2012-2015 POETICON++, European Commission FP7 project ICT-288382
 * Copyright: (C) 2015 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */

#include "Helpers.h"

/**
  * Find the unique elements of a single-channel Mat.
  * adapted from stackoverflow.com/questions/24716932
  */
std::vector<int> unique(const Mat &input, bool shouldSort)
{
    if (input.channels()>1 || input.type()!=CV_32S)
    {
        yError("unique() only works with CV_32S 1-channel Mat");
        return std::vector<int>();
    }

    std::vector<int> out;
    for (int y=0; y<input.rows; ++y)
    {
        const int *row_ptr = input.ptr<int>(y);
        for (int x=0; x<input.cols; ++x)
        {
            float value = row_ptr[x];

            if ( std::find(out.begin(),out.end(),value) == out.end() )
                out.push_back(value);
        }
    }

    if (shouldSort)
        std::sort(out.begin(), out.end());

    return out;
}

/**
  * Given a matrix and an integer number (label), return a binary matrix with
  * elements set to 1 where input==label, 0 elsewhere.
  */
bool binaryMaskFromLabel(const cv::Mat &input, const int label, cv::Mat &output)
{
    if (input.channels()>1 || input.type()!=CV_8U)
    {
        fprintf(stdout, "binaryMaskFromLabel() only works with CV_8U 1-channel Mat input");
        return false;
    }

    output = Mat::zeros(input.size(), CV_8UC1);

    int nl = input.rows;
    int nc = input.cols * input.channels();

    // from Laganière OpenCV 2 book
    if (input.isContinuous())
    {
        nc = nc * nl;
        nl = 1;
    }

    // cycle over all pixels - executed only once in case of continuous images
    for (int j=0; j<nl; j++)
    {
        // pointer to first column of line j
        uchar* data = const_cast<uchar*>( input.ptr<uchar>(j) );
        for (int i=0; i<nc; i++)
            output.at<uchar>(j,i) = ( (uint8_t)*data++ == (uint8_t)label ? 1 : 0); 
    }

    return true;

}

/**
  * Given a vector of OpenCV contours, return the index of the one with the
  * largest area.
  */
bool largestContour(std::vector< std::vector<Point> > cnt, int largestIdx)
{
    double largestArea = 0.0;
    int currLargestIdx = -1;
    
    for (int c=0; c<cnt.size(); ++c)
    {
        // default paramenter oriented=false is ok
        double currArea = contourArea(cnt[c]);
        if (currArea > largestArea)
        {
            largestArea = currArea;
            currLargestIdx = c;
        }
    }

    if (currLargestIdx == -1)
    {
        yWarning("largestContour did not find index of largest contour");
        return false;
    }

    largestIdx = currLargestIdx;
    return true;
}
