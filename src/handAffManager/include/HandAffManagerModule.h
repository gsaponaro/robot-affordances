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

#ifndef HAND_AFFORDANCE_MANAGER_MODULE_H
#define HAND_AFFORDANCE_MANAGER_MODULE_H

#include <string>

#include <boost/filesystem.hpp>

#include <yarp/os/all.h>
//#include <yarp/os/RpcClient.h>
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
    std::string rpcHandActionsPortName;
    std::string rpcRobotHandProcessorPortName;
    std::string rpcGazePortName;
    yarp::os::BufferedPort<yarp::os::Bottle> inHandDescPort;
    yarp::os::BufferedPort<yarp::os::Bottle> inObjDescPort;
    yarp::os::RpcClient rpcHandActionsPort;
    yarp::os::RpcClient rpcRobotHandProcessorPort;
    yarp::os::RpcClient rpcGazePort;

    yarp::os::Bottle handDesc;

    bool gotSomething;
    bool userConfirmation;

    std::string basePath;

    std::string currPosture;

public:

    bool configure(yarp::os::ResourceFinder &rf);
    bool interruptModule();
    bool close();
    double getPeriod();
    bool updateModule();

    void saveDescToFile(const std::string &label);

    // IDL functions
    bool attach(yarp::os::RpcServer &source);
    bool setHandPosture(const std::string &posture);
    bool getHandDescriptors();
    bool yes();
    bool no();
    int32_t getNumVisibleObjects();
    yarp::os::Bottle getObject3D();
    bool quit();
};

#endif // HAND_AFFORDANCE_MANAGER_MODULE_H
