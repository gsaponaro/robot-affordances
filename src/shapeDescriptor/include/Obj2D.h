/*
 * Copyright: (C) 2016 VisLab, Institute for Systems and Robotics,
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

class Obj2D : public Obj
{
private:
    bool valid;

    std::vector<cv::Point> contour;
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
    cv::Moments moments;

    // minimum enclosing rectangles
    cv::Rect boundingRect; // up-right = bounding box in image
    cv::RotatedRect enclosingRect; // rotated rectangle containing best-fit ellipse

    // minimum enclosing circle
    cv::Point2f circleCenter;
    float circleRadius;

    // polygonal approximation
    std::vector<cv::Point> poly;

    // convex hull
    std::vector<cv::Point> hull;

    // convexity defects
    std::vector<cv::Vec4i> defects;

public:
    Obj2D(bool _isValid, std::vector<cv::Point> _contour, double _area);
    bool computeDescriptors();

    // getters
    bool isValid() const;
    double getArea() const;
    double getConvexity() const;
    double getEccentricity() const;
    double getCompactness() const;
    double getCircleness() const;
    double getSquareness() const;
    double getPerimeter() const;
    double getElongation() const;
    cv::Moments getMoments() const;
    cv::Rect getBoundingRect() const;
    cv::RotatedRect getEnclosingRect() const;
};

#endif
