/*
 * Copyright: (C) 2012-2015 POETICON++, European Commission FP7 project ICT-288382
 * Copyright: (C) 2015 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */

#include "Obj2D.h"

/**
  * Constructor specifying validity, contour and area.
  */
Obj2D::Obj2D(bool _isValid, std::vector<cv::Point> _contour, double _area)
: valid(_isValid),
  contour(_contour),
  area(_area), convexity(0.0), eccentricity(0.0), compactness(0.0),
  circleness(0.0), squareness(0.0),
  perimeter(0.0), elongation(0.0),
  moments(cv::Moments()),
  boundingRect(cv::Rect()), enclosingRect(cv::RotatedRect()),
  circleCenter(cv::Point2f()), circleRadius(0.0),
  poly(std::vector<cv::Point>()), hull(std::vector<cv::Point>()),
  defects(std::vector<cv::Vec4i>())
{
}

/**
  * Given an existing object with validity,contour,area already set,
  * compute the remaining descriptors.
  */
bool Obj2D::computeDescriptors()
{
    perimeter = arcLength(contour, true);
    convexHull(contour, hull);
    double hull_perimeter = arcLength(hull, true);
    double convexity_temp = (perimeter>0 ? hull_perimeter/perimeter : 0);
    const int conv_exponent = 2;
    convexity = (perimeter>0 ? pow(convexity_temp,conv_exponent) : 0);
    //yDebug("hull_perimeter=%.2f perimeter=%.2f \t convexity=%.2f convexity^2=%.2f",
    //       hull_perimeter, perimeter, convexity_temp, convexity);

    double majorAxisEll, minorAxisEll;
    RotatedRect enclosingRectEll = fitEllipse(contour);
    majorAxisEll = (enclosingRectEll.size.width>enclosingRectEll.size.height ?
                    enclosingRectEll.size.width :
                    enclosingRectEll.size.height);
    minorAxisEll = (enclosingRectEll.size.width>enclosingRectEll.size.height ?
                    enclosingRectEll.size.height :
                    enclosingRectEll.size.width);
    eccentricity = (majorAxisEll>0 ? minorAxisEll/majorAxisEll : 0);
    //yDebug("minorAxisEll=%.2f majorAxisEll=%.2f \t eccentricity=%.2f",
    //       minorAxisEll, majorAxisEll, eccentricity);

    compactness = (perimeter>0 ? (4*CV_PI*area)/pow(perimeter,2) : 0);
    if (compactness > 1.0)
    {
        yWarning("compactness was >1.0 -> set to 1.0. check computation of this descriptor!");
        compactness = 1.0;
    }
    //yDebug("4*pi*area=%.2f per^2=%.2f \t compactness=%.2f",
    //       4*CV_PI*area, pow(perimeter,2), compactness);

    minEnclosingCircle(contour, circleCenter, circleRadius);
    circleness = (circleRadius>0 ? area/(CV_PI*pow(circleRadius,2)) : 0);
    //yDebug("area=%.2f pi*radius^2=%.2f \t circleness=%.2f",
    //       area, CV_PI*pow(circleRadius,2), circleness);

    enclosingRect = minAreaRect(contour);
    double majorAxisRect, minorAxisRect;
    majorAxisRect = (enclosingRect.size.width>enclosingRect.size.height ?
                     enclosingRect.size.width :
                     enclosingRect.size.height);
    minorAxisRect = (enclosingRect.size.width>enclosingRect.size.height ?
                     enclosingRect.size.height :
                     enclosingRect.size.width);
    double enclosingRectArea = majorAxisRect * minorAxisRect;
    squareness = (enclosingRectArea>0 ? area/enclosingRectArea : 0);
    //yDebug("area=%.2f enclosingRectArea=%.2f \t squareness=%.2f",
    //       area, enclosingRectArea, squareness);

    yDebug("perimeter=%.2f area=%.2f convexity=%.2f eccentricity=%.2f compactness=%.2f circleness=%.2f squareness=%.2f",
           perimeter, area, convexity, eccentricity, compactness, circleness, squareness);

    return true;
}

/**
  * Given an existing object compute the hue colour histogram.
  */
bool Obj2D::computeHueHistogram()
{
    Mat inHSV;
    cvtColor(mask, inHSV, COLOR_BGR2HSV);
    int h_bins = 16;
    int histSize[] = { h_bins };
    float h_ranges[] = { 0, 180 }; // in 8-bit images, hue varies from 0 to 179
    const float *ranges[] = { h_ranges };
    int channels[] = { 0 };

    cv::calcHist(&inHSV,   // input
                 1,        // histogram from 1 image only
                 channels, // channel used
                 Mat(),    // no mask is used
                 histH,    // output histogram
                 1,        // it is a 1D histogram
                 histSize, // number of bins
                 ranges);  // pixel value range

    cv::normalize(histH, histH);

    return true;
}

// getters

/**
  * Return whether the object is valid.
  */
bool Obj2D::isValid() const
{
    return valid;
}

/**
  * Return enclosing rectangle (rotated rectangle containing best-fit ellipse).
  */
RotatedRect Obj2D::getEnclosingRect() const
{
    return enclosingRect;
}

/**
  * Return bounding rectangle (up-right bounding box in image).
  */
Rect Obj2D::getBoundingRect() const
{
    return cv::boundingRect(contour);
}

/**
  * Return object area.
  */
double Obj2D::getArea() const
{
    return area;
}

/**
  * Return object convexity.
  */
double Obj2D::getConvexity() const
{
    return convexity;
}

/**
  * Return object eccentricity.
  */
double Obj2D::getEccentricity() const
{
    return eccentricity;
}

/**
  * Return object compactness.
  */
double Obj2D::getCompactness() const
{
    return compactness;
}

/**
  * Return object circleness.
  */
double Obj2D::getCircleness() const
{
    return circleness;
}

/**
  * Return object squareness.
  */
double Obj2D::getSquareness() const
{
    return squareness;
}

/**
  * Return image mask: nonzero on object pixels, zero elsewhere.
  */
Mat Obj2D::getMask() const
{
    return mask;
}

/**
  * Return hue histogram of object.
  */
MatND Obj2D::getHueHistogram() const
{
    return histH;
}

// setters
bool Obj2D::setMask(const Mat& m)
{
    mask = m;
}
