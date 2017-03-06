/*
 * Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */

#include <yarp/os/Log.h>
#include <yarp/os/Network.h>

#include "DescriptorReductionDefaults.h"
#include "DescriptorReductionModule.h"

using namespace yarp::os;

int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("descriptorReduction");    // overridden by --context
    rf.setDefaultConfigFile("descriptorReduction.ini");  // overridden by --from
    rf.configure(argc, argv);

    if(rf.check("help"))
    {
        yInfo("Options:");
        yInfo("--name <module name> (default: %s)", DefModuleName.c_str());

        return 0; // EXIT_SUCCESS
    }

    Network yarp;
    if(! yarp.checkNetwork() )
    {
        yError("YARP server not available!");
        return 1; // EXIT_FAILURE
    }

    DescriptorReductionModule mod;

    return mod.runModule(rf);
}
