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

#ifndef HAND_ACTIONS_MODULE_H
#define HAND_ACTIONS_MODULE_H

#include <string>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include "handActions_IDL.h"

const double ARM_DEF_HOME[] = {-50.0,  60.0,  0.0,    40.0,    0.0,  0.0,   0.0,     20.0,  30.0,10.0,10.0,  10.0,10.0, 10.0,10.0,  10.0};
const double TORSO_DEF_HOME[] = {0.0, 0.0, 0.0};

/***************************************************/
class HandActionsModule : public yarp::os::RFModule,
                          public handActions_IDL
{
protected:

    yarp::dev::PolyDriver drvArm,drvGaze,drvArmPos,drvTorso;
    yarp::dev::ICartesianControl *iarm;
    yarp::dev::IGazeControl      *igaze;
    yarp::dev::IPositionControl2 *posA,*posT;
    yarp::dev::IEncoders *encsA;
    yarp::dev::IControlMode2 *ctrlMA,*ctrlMT;

    yarp::os::RpcServer rpcPort;

    yarp::os::Mutex mutex;
    int *controlModesArm;
    int nAxesA;

    void fixate(const yarp::sig::Vector &x);
    yarp::sig::Vector computeHandOrientation();
    bool approachTargetWithHand(const yarp::sig::Vector &x, const yarp::sig::Vector &o, std::string side);
    void roll(const yarp::sig::Vector &targetPos, const yarp::sig::Vector &o, std::string side);
    void retrieveObjLocation(const yarp::os::Bottle &command);

public:

    bool configure(yarp::os::ResourceFinder &rf);
    bool interruptModule();
    bool close();
    double getPeriod();
    bool updateModule();

    bool safetyCheck(const yarp::sig::Vector &targetPos, const std::string &side);

    // IDL functions
    bool attach(yarp::os::RpcServer &source);
    bool look_down();
    bool home();
    bool tapFromLeft(const double x, const double y, const double z);
    bool tapFromRight(const double x, const double y, const double z);
    bool push(const double x, const double y, const double z);
    bool draw(const double x, const double y, const double z);
    bool quit();
};

#endif // HAND_ACTIONS_MODULE_H
