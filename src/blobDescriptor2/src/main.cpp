/*
 * Copyright: (C) 2016 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */

/** 
@defgroup blobDescriptor2
@ingroup robot-affordances

This module extracts features from segmented images.

\section intro_sec Description

This module extracts features from segmented images.

\section lib_sec Libraries
- YARP

- OpenCV

\section parameters_sec Parameters
Basic options:
\verbatim
--name <module name> (default: objectDescriptor)
--threadPeriod <thread period in seconds> (default: 0.033)
--maxObjects <number> (default: 10)
--mode <2d or 3d> (default: 2d)
\endverbatim

Advanced options (2d mode):
\verbatim
--minArea <minimum valid blob area> (default: 100)
--maxArea <maximum valid blob area> (default: 20000)
\endverbatim

\section tested_os_sec Tested OS
Linux

\author Giovanni Saponaro
*/

#include <yarp/os/Log.h>
#include <yarp/os/Network.h>

#include "DescriptorModule.h"

using namespace yarp::os;

int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("blobDescriptor2");    // overridden by --context
    rf.setDefaultConfigFile("blobDescriptor2.ini");  // overridden by --from
    rf.configure(argc, argv);

    #ifdef CV_MAJOR_VERSION
    yInfo("This module has been compiled with OpenCV %d.%d.%d",CV_MAJOR_VERSION,CV_MINOR_VERSION,CV_SUBMINOR_VERSION);
    #else
    yInfo("This module has been compiled with an unknown version of OpenCV (probably < 1.0)");
    #endif    

    if(rf.check("help"))
    {
        yInfo("Basic options:");
        yInfo("--name <module name> (default: blobDescriptor)");
        yInfo("--maxObjects <number> (default: 10)");
        yInfo("--mode <2d or 3d> (default: 2d)");

        yInfo("Advanced options (2d mode):");
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

    BlobDescriptorModule bdm;

    return bdm.runModule(rf);
}

