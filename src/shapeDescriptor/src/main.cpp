/*
 * Copyright: (C) 2016 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */

#include <yarp/os/Log.h>
#include <yarp/os/Network.h>

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
        yInfo("Basic options:");
        yInfo("--name <module name> (default: shapeDescriptor)");
        yInfo("--maxObjects <number> (default: 10)");

        yInfo("Advanced options:");
        yInfo("--minArea <minimum valid blob area> (default: 100)");
        yInfo("--maxArea <maximum valid blob area> (default: 3000)");
        
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
