/*
 * Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>,
 *         Atabak Dehban <adehban@isr.tecnico.ulisboa.pt>,
 *         Pedro Vicente <pvicente@isr.tecnico.ulisboa.pt>,
 *         Lorenzo Jamone, Afonso Gonçalves
 * CopyPolicy: Released under the terms of the GNU GPL v3.0
 *
 */

#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include "HandAffManagerModule.h"

using namespace yarp::os;

/***************************************************/
int main(int argc, char *argv[])
{
    Network yarp;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("handAffManager");    // overridden by --context
    rf.setDefaultConfigFile("handAffManager.ini");  // overridden by --from
    rf.configure(argc,argv);

    if(rf.check("help"))
    {
        yInfo("Available options:");
        yInfo("--name <name of module> (default handAffManager)");

        return 0; // EXIT_SUCCESS
    }

    if (! yarp::os::Network::checkNetwork() )
    {
        yError("yarpserver not available!");
        return 1; // EXIT_FAILURE
    }

    HandAffManagerModule mod;
    return mod.runModule(rf);
}
