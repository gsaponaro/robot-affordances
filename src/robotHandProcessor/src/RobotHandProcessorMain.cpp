/*
 * Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>,
 * CopyPolicy: Released under the terms of the GNU GPL v3.0
 *
 */

#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include "RobotHandProcessorModule.h"

using namespace yarp::os;

/***************************************************/
int main(int argc, char *argv[])
{
    Network yarp;

    ResourceFinder rf;
    rf.configure(argc,argv);

    if(rf.check("help"))
    {
        yInfo("Available options:");
        yInfo("--name <name of module> (default robotHandProcessor)");

        return 0; // EXIT_SUCCESS
    }

    if (! yarp::os::Network::checkNetwork() )
    {
        yError("yarpserver not available!");
        return 1; // EXIT_FAILURE
    }

    RobotHandProcessorModule mod;
    return mod.runModule(rf);
}
