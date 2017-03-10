/*
 * Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0
 *
 */

#ifndef DESC_DEFAULTS_H
#define DESC_DEFAULTS_H

#include <string>

const int         DefThreadPeriod = 33; // [ms]
const std::string DefModuleName   = "shapeDescriptor";

const int         DefMaxObjects = 10;
const int         DefMinArea    = 100;
const int         DefMaxArea    = 8000;

const std::string ValueOn  = "on";
const std::string ValueOff = "off";

const std::string DefUseArea                    = ValueOff;
const std::string DefUseConvexity               = ValueOn;
const std::string DefUseConvexityDefects        = ValueOn;
const std::string DefUseEccentricity            = ValueOn;
const std::string DefUseCompactness             = ValueOn;
const std::string DefUseCircularity             = ValueOn;
const std::string DefUseSquareness              = ValueOn;
const std::string DefUsePerimeter               = ValueOff;
const std::string DefUseElongation              = ValueOff;
const std::string DefSpatialMomentsCenterOfMass = ValueOff;
const std::string DefCentralMoments             = ValueOff;
const std::string DefCentralNormalizedMoments   = ValueOn;

const std::string DefBoundingRectangle          = ValueOff;
const std::string DefEnclosingRectangle         = ValueOff;

#endif
