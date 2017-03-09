/*
 * Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>,
 * CopyPolicy: Released under the terms of the GNU GPL v3.0
 *
 */

#ifndef ROBOT_HAND_PROCESSOR_MODULE_H
#define ROBOT_HAND_PROCESSOR_MODULE_H

#include <string>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include "RobotHandProcessorThread.h"
#include "robotHandProcessor_IDL.h"

/***************************************************/
class RobotHandProcessorModule : public yarp::os::RFModule,
                                 public robotHandProcessor_IDL
{
private:

    // module parameters
    bool closing;
    yarp::os::RpcServer rpcPort;

    // pointer to a new thread
    RobotHandProcessorThread *thread;

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

#endif // ROBOT_HAND_PROCESSOR_MODULE_H
