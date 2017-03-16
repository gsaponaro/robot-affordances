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

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RpcServer.h>
#include <yarp/sig/Vector.h>

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
    bool look(const std::string &target);
    bool resetKinematics();
    double getArmPos(const int32_t joint);
    yarp::os::Bottle getArmPoss();
    bool setArmPos(const int32_t joint, const double value);
    bool setArmPoss(const yarp::os::Bottle &values);
    double getHeadPos(const int32_t joint);
    yarp::os::Bottle getHeadPoss();
    bool setHeadPos(const int32_t joint, const double value);
    bool setHeadPoss(const yarp::os::Bottle &values);
};

#endif // ROBOT_HAND_PROCESSOR_MODULE_H
