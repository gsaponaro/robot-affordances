/*
 * Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0
 *
 */

#include "Obj2D.h"

using namespace cv;

/**
  * Constructor specifying contour.
  */
Obj2D::Obj2D(std::vector<cv::Point> _contour)
: contour(_contour)
{
    area = contourArea(contour);
    computeDescriptors();
}

/**
  * Compute all the shape descriptors.
  */
bool Obj2D::computeDescriptors()
{
    if (area <= 0)
        return false;

    if (contour.size()<5) // not enough for fitEllipse
        return false;

    perimeter = arcLength(contour, true);
    convexHull(contour, hull);
    double hull_perimeter = arcLength(hull, true);
    double convexity_temp = (perimeter>0 ? hull_perimeter/perimeter : 0);
    const int conv_exponent = 2;
    convexity = (perimeter>0 ? pow(convexity_temp,conv_exponent) : 0);
    //yDebug("hull_perimeter=%.2f perimeter=%.2f \t convexity=%.2f convexity^2=%.2f",
    //       hull_perimeter, perimeter, convexity_temp, convexity);

    fitLine(contour,
            bestLineFit,
            DIST_L2, // distance type
            0,       // parameter for distance metric
            1e-2,    // radio accuracy parameter
            1e-2);   // angle accuracy parameter

    // convexity defects require hull as int, not Point
    std::vector<int> hull_int;
    convexHull(contour, hull_int);
    const int MIN_CONVDEF_HULL_SIZE = 3;
    const int MIN_CONVDEF_DEPTH = 8;
    std::vector<cv::Vec4i> defects_all;
    if (hull.size() > MIN_CONVDEF_HULL_SIZE)
    {
        convexityDefects(contour, hull_int, defects_all);
        for(std::vector<cv::Vec4i>::const_iterator iter = defects_all.begin();
            iter != defects_all.end();
            ++iter)
        {
            float depth = (*iter)[3]/256.0;
            //yDebug("depth is %f", depth);
            if (depth > MIN_CONVDEF_DEPTH)
            {
                defects.push_back(*iter);
            }
        }
        //yDebug("defects: %lu -> %lu", defects_all.size(), defects.size());
    }

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

    compactness = (perimeter>0 ? (4*M_PI*area)/pow(perimeter,2) : 0);
    if (compactness > 1.0)
    {
        yWarning("compactness was >1.0 -> set to 1.0. check computation of this descriptor!");
        compactness = 1.0;
    }
    //yDebug("4*pi*area=%.2f per^2=%.2f \t compactness=%.2f",
    //       4*M_PI*area, pow(perimeter,2), compactness);

    moments = cv::moments(contour);

    minEnclosingCircle(contour, circleCenter, circleRadius);
    circularity = (circleRadius>0 ? area/(M_PI*pow(circleRadius,2)) : 0);
    //yDebug("area=%.2f pi*radius^2=%.2f \t circularity=%.2f",
    //       area, M_PI*pow(circleRadius,2), circularity);

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

    elongation = majorAxisRect; // TODO: revise computation

    //yDebug("perimeter=%.2f area=%.2f convexity=%.2f eccentricity=%.2f compactness=%.2f circularity=%.2f squareness=%.2f",
    //       perimeter, area, convexity, eccentricity, compactness, circularity, squareness);

    return true;
}

/**
  * Check if the object area is within a range.
  * (Equivalent to the "valid" flag in blobDescriptor.)
  */
bool Obj2D::areaInRange(const double &minArea, const double &maxArea) const
{
    return (area>minArea && area<maxArea);
}

// getters

/**
  * Return contour.
  */
std::vector<cv::Point> Obj2D::getContour() const
{
    return contour;
}

/**
  * Return center of mass.
  */

Point Obj2D::getCenterOfMass() const
{
    return Point(moments.m10/moments.m00, moments.m01/moments.m00);
}

/**
  * Return area.
  */
double Obj2D::getArea() const
{
    return area;
}

/**
  * Return convexity.
  */
double Obj2D::getConvexity() const
{
    return convexity;
}

/**
  * Return best line fit.
  */
cv::Vec4f Obj2D::getBestLineFit() const
{
    return bestLineFit;
}

/**
  * Get the convexity defects that have depth > MIN_CONVDEF_DEPTH.
  */
std::vector<cv::Vec4i> Obj2D::getConvexityDefects() const
{
    return defects;
}

/**
  * Return eccentricity.
  */
double Obj2D::getEccentricity() const
{
    return eccentricity;
}

/**
  * Return compactness.
  */
double Obj2D::getCompactness() const
{
    return compactness;
}

/**
  * Return circularity.
  */
double Obj2D::getCircularity() const
{
    return circularity;
}

/**
  * Return squareness.
  */
double Obj2D::getSquareness() const
{
    return squareness;
}

/**
  * Return perimeter.
  */
double Obj2D::getPerimeter() const
{
    return perimeter;
}

/**
  * Return elongation.
  */
double Obj2D::getElongation() const
{
    return elongation;
}

/**
  * Return raw moments (spatial, central, central normalized).
  */
Moments Obj2D::getMoments() const
{
    return moments;
}

/**
  * Return bounding rectangle (up-right bounding box).
  */
Rect Obj2D::getBoundingRect() const
{
    return cv::boundingRect(contour);
}

/**
  * Return enclosing rectangle (rotated rectangle containing best-fit ellipse).
  */
RotatedRect Obj2D::getEnclosingRect() const
{
    return enclosingRect;
}
