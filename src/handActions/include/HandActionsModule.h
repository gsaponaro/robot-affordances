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

/***************************************************/
class HandActionsModule : public yarp::os::RFModule,
                          public handActions_IDL
{
protected:

    yarp::dev::PolyDriver drvGaze;
    yarp::dev::PolyDriver drvArm;
    yarp::dev::PolyDriver drvArmPos;
    yarp::dev::PolyDriver drvArmPosOther;
    yarp::dev::PolyDriver drvTorso;

    std::string arm;
    bool useHand;

    yarp::dev::IGazeControl *igaze;
    yarp::dev::IEncoders *encsA;
    yarp::dev::ICartesianControl *iarm;
    yarp::dev::IPositionControl2 *posA;
    yarp::dev::IPositionControl2 *posAOther;
    yarp::dev::IPositionControl2 *posT;

    yarp::dev::IControlMode2 *ctrlMA;
    yarp::dev::IControlMode2 *ctrlMAOther;
    yarp::dev::IControlMode2 *ctrlMT;

    bool closing;
    bool twoArms;
    yarp::os::RpcServer rpcPort;

    std::string rpcManagerPortName;
    yarp::os::RpcClient rpcManagerPort;

    yarp::os::Mutex mutex;
    int *controlModesArm;
    int nAxesA;

    yarp::sig::Vector straightHandPoss, fortyfiveHandPoss, bentHandPoss;
    yarp::sig::Vector handVels;

    double offAppTap,offAppDraw,offAppPush,distanceMovement;

    void fixate(const yarp::sig::Vector &x);
    yarp::sig::Vector computeHandOrientation();
    bool approachTargetWithHand(const yarp::sig::Vector &x, const yarp::sig::Vector &o, std::string side);
    void roll(const yarp::sig::Vector &targetPos, const yarp::sig::Vector &o, std::string side);
    void retrieveObjLocation(const yarp::os::Bottle &command);

public:

    // indexes used for finger postures
    static const int STRAIGHT  = 0;
    static const int FORTYFIVE = 1;
    static const int BENT      = 2;

    bool configure(yarp::os::ResourceFinder &rf);
    bool interruptModule();
    bool close();
    double getPeriod();
    bool updateModule();
    bool homeOtherArm();
    bool homeTorso();
    bool homeTorsoPitch();
    bool homeArm();
    yarp::os::Bottle getBestObject3D();

    void moveHand(const int postureType);
    bool safetyCheck(const yarp::sig::Vector &targetPos, const std::string &side);

    // IDL functions
    bool attach(yarp::os::RpcServer &source);
    bool look_down();
    bool homeAll();
    bool setFingers(const std::string &posture);
    bool tapFromLeft();
    bool tapFromRight();
    bool push();
    bool draw();
    bool tapFromLeftCoords(const double x, const double y, const double z);
    bool tapFromRightCoords(const double x, const double y, const double z);
    bool pushCoords(const double x, const double y, const double z);
    bool drawCoords(const double x, const double y, const double z);
    bool quit();
};

#endif // HAND_ACTIONS_MODULE_H
