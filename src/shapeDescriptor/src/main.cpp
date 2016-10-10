/*
 * Copyright: (C) 2016 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */

#include <yarp/os/Log.h>
#include <yarp/os/Network.h>

#include "DescriptorDefaults.h"
#include "DescriptorModule.h"

using namespace yarp::os;

int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("shapeDescriptor");    // overridden by --context
    rf.setDefaultConfigFile("shapeDescriptor.ini");  // overridden by --from
    rf.configure(argc, argv);

    #ifdef CV_MAJOR_VERSION
    yInfo("This module has been compiled with OpenCV %d.%d.%d",CV_MAJOR_VERSION,CV_MINOR_VERSION,CV_SUBMINOR_VERSION);
    #else
    yInfo("This module has been compiled with an unknown version of OpenCV (possibly < 1.0)");
    #endif    

    if(rf.check("help"))
    {
        yInfo("Options:");
        yInfo("--name <module name> (default: %s)", DefModuleName.c_str());
        yInfo(" ");
        yInfo("--maxObjects <number> (default: %d)", DefMaxObjects);
        yInfo("--minArea <minimum valid blob area> (default: %d)", DefMinArea);
        yInfo("--maxArea <maximum valid blob area> (default: %d)", DefMaxArea);
        yInfo(" ");
        yInfo("--area <on|off> (default: %s)", DefUseArea.c_str());
        yInfo("--convexity <on|off> (default: %s)", DefUseConvexity.c_str());
        yInfo("--eccentricity <on|off> (default: %s)", DefUseEccentricity.c_str());
        yInfo("--compactness <on|off> (default: %s)", DefUseCompactness.c_str());
        yInfo("--circularity <on|off> (default: %s)", DefUseCircularity.c_str());
        yInfo("--squareness <on|off> (default: %s)", DefUseSquareness.c_str());
        yInfo("--perimeter <on|off> (default: %s)", DefUsePerimeter.c_str());
        yInfo("--elongation <on|off> (default: %s)", DefUseElongation.c_str());
        yInfo("--spatialMoments <on|off> (default: %s)", DefSpatialMomentsCenterOfMass.c_str());
        yInfo("--centralMoments <on|off> (default: %s)", DefCentralMoments.c_str());
        yInfo("--centralNormalizedMoments <on|off> (default: %s)", DefCentralNormalizedMoments.c_str());
        yInfo("--boundingRectangle <on|off> (default: %s)", DefBoundingRectangle.c_str());
        yInfo("--enclosingRectangle <on|off> (default: %s)", DefEnclosingRectangle.c_str());

        return 0; // EXIT_SUCCESS
    }

    Network yarp;
    if(! yarp.checkNetwork() )
    {
        yError("YARP server not available!");
        return 1; // EXIT_FAILURE
    }

    ShapeDescriptorModule mod;

    return mod.runModule(rf);
}
