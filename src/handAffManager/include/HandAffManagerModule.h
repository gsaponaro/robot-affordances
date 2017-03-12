/*
 * Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>,
 *         Atabak Dehban <adehban@isr.tecnico.ulisboa.pt>,
 *         Lorenzo Jamone, Afonso Gonçalves
 * CopyPolicy: Released under the terms of the GNU GPL v3.0
 *
 */

#ifndef HAND_AFFORDANCE_MANAGER_MODULE_H
#define HAND_AFFORDANCE_MANAGER_MODULE_H

#include <string>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>

#include "handAffManager_IDL.h"

/***************************************************/
class HandAffManagerModule : public yarp::os::RFModule,
                             public handAffManager_IDL
{
private:

    //bool closing;
    yarp::os::RpcServer rpcPort;

    std::string inHandDescPortName;
    std::string inObjDescPortName;
    yarp::os::BufferedPort<yarp::os::Bottle> inHandDescPort;
    yarp::os::BufferedPort<yarp::os::Bottle> inObjDescPort;

public:

    bool configure(yarp::os::ResourceFinder &rf);
    bool interruptModule();
    bool close();
    double getPeriod();
    bool updateModule();

    // IDL functions
    bool attach(yarp::os::RpcServer &source);
    bool quit();
};

#endif // HAND_AFFORDANCE_MANAGER_MODULE_H
