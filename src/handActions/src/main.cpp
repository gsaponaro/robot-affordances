/*
 * Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Pedro Vicente <pvicente@isr.tecnico.ulisboa.pt>,
 *         Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0
 *
 * Based on code by Ugo Pattacini <ugo.pattacini@iit.it>
 *
 */

#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include "HandActionsModule.h"

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
        yInfo("--name <name of module> (default handActions)");
        yInfo("--robot <name of robot> (default icubSim)");
        yInfo("--arm <arm to use> (default right_arm)");

        return 0; // EXIT_SUCCESS
    }

    if (! yarp::os::Network::checkNetwork() )
    {
        yError("yarpserver not available!");
        return 1; // EXIT_FAILURE
    }

    HandActionsModule mod;
    return mod.runModule(rf);
}
