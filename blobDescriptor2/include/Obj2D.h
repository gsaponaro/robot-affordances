/*
 * Copyright: (C) 2012-2015 POETICON++, European Commission FP7 project ICT-288382
 * Copyright: (C) 2015 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */

#ifndef O2D_H
#define O2D_H

#include <yarp/os/Log.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Obj.h"

using namespace cv;

class Obj2D : public Obj
{
private:
    bool valid;

    std::vector<Point> contour;
    double area;

    // shape descriptors
    double convexity;
    double eccentricity;
    double compactness;
    double circleness;
    double squareness;

    double perimeter;
    double elongation;

    // raw moments (spatial, central, central normalized)
    Moments moments;

    // minimum enclosing rectangles
    Rect boundingRect; // up-right = bounding box in image
    RotatedRect enclosingRect; // rotated rectangle containing best-fit ellipse

    // minimum enclosing circle
    Point2f circleCenter;
    float circleRadius;

    // polygonal approximation
    std::vector<Point> poly;

    // convex hull
    std::vector<Point> hull;

    // convexity defects
    std::vector<Vec4i> defects;

    // hue colour histogram
    MatND histH;
    Mat mask;

public:
    Obj2D(bool _isValid, std::vector<Point> _contour, double _area);
    bool computeDescriptors();
    bool computeHueHistogram();

    // getters
    bool isValid() const;
    RotatedRect getEnclosingRect() const;
    Rect getBoundingRect() const;
    double getArea() const;
    double getConvexity() const;
    double getEccentricity() const;
    double getCompactness() const;
    double getCircleness() const;
    double getSquareness() const;
    Mat getMask() const;
    MatND getHueHistogram() const;

    // setters
    bool setMask(const Mat& m);
};

#endif
