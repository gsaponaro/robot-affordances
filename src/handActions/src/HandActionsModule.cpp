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

#include "HandActionsModule.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

/***************************************************/
void HandActionsModule::fixate(const Vector &x)
{
    igaze->lookAtFixationPoint(x);
    igaze->waitMotionDone();
    //igaze->setTrackingMode(true);
}

/***************************************************/
Vector HandActionsModule::computeHandOrientation()
{
    Matrix R(3,3);
    R(0,0) = -1.0; R(1,0) = 0.0;  R(2,0) = 0.0;
    R(0,1) = 0.0;  R(1,1) = 0.0;  R(2,1) = -1.0;
    R(0,2) = 0.0;  R(1,2) = -1.0; R(2,2) = 0.0;
    return dcm2axis(R);
}

/***************************************************/
bool HandActionsModule::approachTargetWithHand(const Vector &x, const Vector &o, string side)
{
    double timeout = 5.0;
    if (!safetyCheck(x,side))
    {
        yWarning("action is dangerous, I will not do it!");
        return false;
    }

    Vector dof(10,1.0), dummy;
    // disable torso
    //dof[0]=0; // pitch?
    //dof[1]=0; // roll
    //dof[2]=0; // yaw?
    iarm->setDOF(dof,dummy);
    Vector initialApproach;
    Vector finalApproach=x;
    finalApproach[2] += 0.04; // general offset to avoid collision with table
    if(side.compare("right")==0) // tapFromRight
    {
        yDebug("tapFromRight");
        finalApproach[1] += offAppTap;
    }
    else if(side.compare("left")==0) // tapFromLeft
    {
        yDebug("tapFromLeft");
        finalApproach[1] -= offAppTap;
    }
    else if(side.compare("bottom")==0) // push
    {
        yDebug("push");
        finalApproach[0] += offAppPush;
    }
    else if(side.compare("top")==0) // draw
    {
        yDebug("draw");
        finalApproach[0] -= offAppDraw;
        // increase y when using right_arm, decrease y when using left_arm
        finalApproach[1] += 0.03*(arm=="right_arm" ? 1 : -1);

        finalApproach[2] -= 0.02; // stricter than general
    }
    initialApproach = finalApproach;
    initialApproach[2] += 0.05;              // avoid collision with objects during the approach phase
    yDebug("going to intermediate approach waypoint");
    iarm->goToPoseSync(initialApproach,o);
    bool done = false;
    done = iarm->waitMotionDone(0.1,timeout);
    if(!done)
        yWarning("Something went wrong with the initial approach, using timeout");
    // ADD a Delay here?!
    yDebug("going to final approach target");
    iarm->goToPoseSync(finalApproach,o);
    done = iarm->waitMotionDone(0.1,timeout);
    if(!done)
        yWarning("Something went wrong with the final approach, using timeout");
    yDebug("reached final approach target");
    yarp::os::Time::delay(1.0);

    return true;
}

/***************************************************/
void HandActionsModule::roll(const Vector &targetPos, const Vector &o, string side)
{
    double tempotempo;
    double timeout = 4.0;
    iarm->getTrajTime(&tempotempo);
    iarm->setTrajTime(0.7);
    Vector targetModified=targetPos;
    if(side.compare("right")==0) // tapFromRight
    {
        targetModified[1] -= distanceMovement - offAppTap;
        iarm->setTrajTime(0.7);
    }
    else if(side.compare("left")==0) // tapFromLeft
    {
        targetModified[1] += distanceMovement - offAppTap;
        iarm->setTrajTime(0.7);
    }
    else if(side.compare("bottom")==0) // push
    {
        targetModified[0] -= distanceMovement - offAppPush;
        iarm->setTrajTime(0.9);
    }
    else if(side.compare("top")==0) // draw
    {
        targetModified[0] += distanceMovement - offAppDraw;
        // increase y when using right_arm, decrease y when using left_arm
        //targetModified[1] += 0.03*(arm=="right_arm" ? 1 : -1);

        targetModified[1] += 0.03; // left_arm
        iarm->setTrajTime(1.0);
    }
    targetModified[2] += 0.04;            // Offset - avoid collision with table
    iarm->goToPoseSync(targetModified,o);
    bool done = iarm->waitMotionDone(0.1,timeout);
    if(!done)
        yWarning("Something went wrong with the roll action, using timeout");
    iarm->setTrajTime(tempotempo);
    yDebug("reached final Roll target");
}

/***************************************************/
bool HandActionsModule::configure(ResourceFinder &rf)
{
    string moduleName = rf.check("name",Value("handActions")).asString();
    string robot = rf.check("robot",Value("icub")).asString();
    arm = rf.check("arm",Value("left_arm")).asString();
    string other_arm;
    if (arm!="left_arm" && arm!="right_arm")
    {
        yWarning("invalid arm %s specified, using the default (left_arm)", arm.c_str());
        arm = "left_arm";
    }
    if(arm=="left_arm")
        other_arm = "right_arm";
    else
        other_arm = "left_arm";
    useHand = rf.check("useHand",Value("on")).asString()=="on"?true:false;
    if (useHand)
    {
        yDebug("enabled hand movements");
    }
    else
        yDebug("disabled hand movements");

    rpcManagerPortName = "/" + moduleName + "/handAffManager:rpc";
    rpcManagerPort.open(rpcManagerPortName.c_str());

    closing = false;

    rpcPort.open("/"+moduleName+"/rpc:i");
    attach(rpcPort);


    // Defining Offsets to the actions
    offAppTap        = 0.05;
    distanceMovement = 0.12;
    offAppDraw       = 0.03;
    offAppPush       = 0.08;

    straightHandPoss.resize(9, 0.0);
    straightHandPoss[0] = 50.0; // j7
    straightHandPoss[1] = 10.0;
    straightHandPoss[2] = 55.0;
    straightHandPoss[3] = 34.0;
    straightHandPoss[4] = 10.0; // j11
    straightHandPoss[5] = 10.0;
    straightHandPoss[6] = 10.0; // j13
    straightHandPoss[7] = 10.0;
    straightHandPoss[8] = 10.0; // j15

    fortyfiveHandPoss.resize(9, 0.0);
    fortyfiveHandPoss[0] =   0.0; // j7
    fortyfiveHandPoss[1] =  10.0;
    fortyfiveHandPoss[2] =  55.0;
    fortyfiveHandPoss[3] =  34.0;
    fortyfiveHandPoss[4] =  85.0; // j11
    fortyfiveHandPoss[5] =  68.0;
    fortyfiveHandPoss[6] =  85.0; // j13
    fortyfiveHandPoss[7] =  42.0;
    fortyfiveHandPoss[8] = 147.0; // j15

    bentHandPoss.resize(9, 0.0);
    bentHandPoss[0] =   0.0; // j7
    bentHandPoss[1] =  10.0;
    bentHandPoss[2] =  55.0;
    bentHandPoss[3] =  34.0;
    bentHandPoss[4] =  74.0; // j11
    bentHandPoss[5] =  10.0;
    bentHandPoss[6] =  72.0; // j13
    bentHandPoss[7] =  10.0;
    bentHandPoss[8] = 120.0; // j15

    handVels.resize(9, 0.0);
    handVels[0] = 20.0;
    handVels[1] = 40.0;
    handVels[2] = 50.0;
    handVels[3] = 50.0;
    handVels[4] = 50.0;
    handVels[5] = 50.0;
    handVels[6] = 90.0;
    handVels[7] = 50.0;
    handVels[8] = 80.0;

    /******** Gaze Control *******/
    Property optGaze;
    optGaze.put("device","gazecontrollerclient");
    optGaze.put("remote","/iKinGazeCtrl");
    optGaze.put("local","/"+moduleName+"/gaze_client");

    if (!drvGaze.open(optGaze))
    {
        yError()<<"Unable to open the Gaze Controller";
        return false;
    }

    drvGaze.view(igaze);

    /******** Cartesian Interface *******/
    Property optArm;
    optArm.put("device","cartesiancontrollerclient");
    optArm.put("remote","/"+robot+"/cartesianController/"+arm);
    optArm.put("local","/"+moduleName+"/cartesian_client/"+arm);

    // let's give the controller some time to warm up
    bool ok=false;
    double t0=Time::now();
    while (Time::now()-t0<10.0)
    {
        // this might fail if controller
        // is not connected to solver yet
        if (drvArm.open(optArm))
        {
            ok=true;
            break;
        }

        Time::delay(1.0);
    }

    if (!ok)
    {
        yError()<<"Unable to open the Cartesian Controller";
        drvGaze.close();
        return false;
    }

    drvArm.view(iarm);

    /******** Position Arm Control Interface *******/
    Property options;
    options.put("device", "remote_controlboard");
    options.put("local", "/"+moduleName+"/position_control/"+arm);
    options.put("remote", "/"+robot+"/"+arm);
    drvArmPos.open(options);

    if (!drvArmPos.isValid())
    {
        cout << moduleName << ": unable to connect to device: remote_controlboard of " << arm << endl;
        drvGaze.close();
        drvArm.close();
        return false;
    }

    if (!drvArmPos.view(posA) || !drvArmPos.view(encsA) || !drvArmPos.view(ctrlMA))
    {
        cout << moduleName << ": problems acquiring interfaces to remote_controlboard of " << arm << endl;
        drvGaze.close();
        drvArm.close();
        drvArmPos.close();
        return false;
    }

    posA->getAxes(&nAxesA);
    controlModesArm = new int[nAxesA];

    const int firstHandJoint = nAxesA-straightHandPoss.length();

    // j7-j16
    for (size_t j=0; j<straightHandPoss.length(); j++)
    {
        posA->setRefSpeed(firstHandJoint+j,handVels[j]);
    }


    /******** Position Torso Control Interface *******/
    Property optionTorso("(device remote_controlboard)");
    optionTorso.put("remote",("/"+robot+"/torso").c_str());
    optionTorso.put("local","/"+moduleName+"/position_control/torso");

    if (!drvTorso.open(optionTorso))
    {
        yError()<<"Joints Torso controller not available";
        //terminate();
        drvGaze.close();
        drvArm.close();
        drvArmPos.close();
        return false;
    }

    if (!drvTorso.view(posT) || !drvTorso.view(ctrlMT))
    {
        yError("problems acquiring interfaces to remote_controlboard of torso");
        drvGaze.close();
        drvArm.close();
        drvArmPos.close();
        drvTorso.close();
        return false;
    }
    twoArms = rf.check("twoArms",Value("on")).asString()=="on"?true:false;
    /******** Position Arm Control Interface - The Other Arm*******/
    if(!twoArms) {
        yDebug("Just opening the hand for the actions: %s", arm.c_str());
        return true; // The other arm interface will not be availabl
    }
    Property options2;
    options2.put("device", "remote_controlboard");
    options2.put("local", "/"+moduleName+"/position_control/"+other_arm);
    options2.put("remote", "/"+robot+"/"+other_arm);
    drvArmPosOther.open(options2);

    if (!drvArmPosOther.isValid())
    {
        cout << moduleName << ": unable to connect to device: remote_controlboard of " << other_arm<< endl;
        drvGaze.close();
        drvArm.close();
        drvArmPos.close();
        drvTorso.close();
        return false;
    }

    if (!drvArmPosOther.view(posAOther) || !drvArmPosOther.view(ctrlMAOther))
    {
        cout << moduleName << ": problems acquiring interfaces to remote_controlboard of " << other_arm << endl;
        drvGaze.close();
        drvArm.close();
        drvArmPos.close();
        drvTorso.close();
        drvArmPosOther.close();
        return false;
    }
    yDebug("Drivers open for both arms");

    if(!homeOtherArm())
        yWarning("%s (other arm) not in the home position", other_arm.c_str());
    yDebug("%s (other arm) in the home position", other_arm.c_str());
    return true;
}

/***************************************************/
bool HandActionsModule::interruptModule()
{
    rpcManagerPort.interrupt();
    rpcPort.interrupt();

    return true;
}

/***************************************************/
bool HandActionsModule::close()
{
    drvGaze.close();
    drvArm.close();
    drvArmPos.close();
    drvTorso.close();
    drvArmPosOther.close();
    rpcManagerPort.close();
    rpcPort.close();

    return true;
}

/***************************************************/
double HandActionsModule::getPeriod()
{
    return 0.0;
}

/***************************************************/
bool HandActionsModule::updateModule()
{
    return !closing;
}

/***************************************************/
Bottle HandActionsModule::getBestObject3D()
{
    Bottle res;
    res.clear();

    // sanity check
    if (rpcManagerPort.getOutputCount()<1)
    {
        yError("no connection to handAffManager RPC server");
        return res;
    }

    Bottle handMgrCmd;
    Bottle handMgrReply;
    handMgrCmd.addString("getBestObject3D");
    rpcManagerPort.write(handMgrCmd, handMgrReply);

    bool validReply = handMgrReply.size()>0 &&
                      handMgrReply.get(0).isList() &&
                      handMgrReply.get(0).asList()->size()==3;

    if (validReply)
    {
        //res.addDouble(handMgrReply.get(0).asList()->get(0).asDouble());
        //res.addDouble(handMgrReply.get(0).asList()->get(1).asDouble());
        //res.addDouble(handMgrReply.get(0).asList()->get(2).asDouble());
        res = *handMgrReply.get(0).asList();
    }
    else
    {
        yError("invalid reply to getBestObject3D: %s", handMgrReply.toString().c_str());
    }

    return res;
}

/***************************************************/
void HandActionsModule::moveHand(const int postureType)
{
    if (!useHand)
        return;

    Vector *poss = NULL;

    switch (postureType)
    {
    case STRAIGHT:
        poss = &straightHandPoss;
        break;

    case FORTYFIVE:
        poss = &fortyfiveHandPoss;
        break;

    case BENT:
        poss = &bentHandPoss;
        break;

    default:
        return;
    }

    const int firstHandJoint = nAxesA-straightHandPoss.length();

    // j7-j16
    for (size_t j=0; j<straightHandPoss.length(); j++)
        ctrlMA->setControlMode(firstHandJoint+j, VOCAB_CM_POSITION);

    for (size_t j=0; j<straightHandPoss.length(); j++)
    {
        //posA->setRefSpeed(firstHandJoint+j,handVels[j]);
        posA->positionMove(firstHandJoint+j,(*poss)[j]);
    }

    // wait for the last positionMove to be complete
    bool done=false;

    double elapsedTime=0.0;
    double startTime=Time::now();
    const double maxTimeout = 3.0;

    while (!done && elapsedTime<maxTimeout)
    {
        posA->checkMotionDone(&done);
        Time::delay(0.04);
        elapsedTime = Time::now()-startTime;
    }
}

/***************************************************/
bool HandActionsModule::safetyCheck(const Vector &targetPos, const std::string &side)
{
    // all actions
    const double xMinThresh = -0.23;
    const double xMaxThresh = -0.51;
    if (targetPos[0]>xMinThresh || targetPos[0]<xMaxThresh)
    {
        yWarning("unsafe x");
        return false;
    }

    const double zMaxThresh = -0.13;
    if (targetPos[2] < zMaxThresh)
    {
        yWarning("unsafe z");
        return false;
    }

    // tapFromLeft, tapFromRight with left arm
    if (arm == "left_arm")
    {
        const double yThresh = 0.10;
        if (side=="left" || side=="right")
        {
            if (targetPos[1] > yThresh)
            {
                yWarning("unsafe y");
                return false;
            }
        }
    }

    // tapFromLeft, tapFromRight with right arm
    if (arm == "right_arm")
    {
        const double yThresh = -0.10;
        if (side=="left" || side=="right")
        {
            if (targetPos[1] < yThresh)
            {
                yWarning("unsafe y");
                return false;
            }
        }
    }

    // push, draw with any arm
    const double xMaxThreshStrict = xMaxThresh + 0.05;
    const double yMinThreshStrict = 0.12*(arm=="right_arm" ? 1 : -1);;
    if (side=="top" || side=="bottom")
    {
        if (targetPos[0] < xMaxThreshStrict)
        {
            yWarning("unsafe x");
            return false;
        }
        if (arm == "left_arm" && targetPos[1] > yMinThreshStrict)
        {
            yWarning("unsafe y");
            return false;
        }
        if (arm == "right_arm" && targetPos[1] < yMinThreshStrict)
        {
            yWarning("unsafe y");
            return false;
        }

    }

    yDebug("safety check ok");

    return true;
}

// IDL functions

/***************************************************/
bool HandActionsModule::attach(RpcServer &source)
{
    return this->yarp().attachAsServer(source);
}

/***************************************************/
bool HandActionsModule::look_down()
{
    Vector ang(3,0.0);
    ang[1] = -45.0;
    igaze->lookAtAbsAngles(ang);
    igaze->waitMotionDone();

    return true;
}

/***************************************************/
bool HandActionsModule::homeOtherArm() {

    const double ARM_DEF_HOME[] = {-80.0,  80.0,  0.0,    55.0,    0.0,  0.0,   0.0};
    for(int i=0; i<7; i++)
    {
        ctrlMAOther->setControlMode(i,VOCAB_CM_POSITION);
    }
    for (int i=0; i<7; i++) // arm joints, no hand joints
    {
        posAOther->positionMove(i, ARM_DEF_HOME[i]);
    }
    bool done=false;
    double elapsedTime=0.0;
    double startTime=Time::now();

    const double maxTimeout = 3.0;

    while(!done && elapsedTime<maxTimeout)
    {
        posAOther->checkMotionDone(&done);
        Time::delay(0.04);
        elapsedTime = Time::now()-startTime;
    }
    return true;

}
/***************************************************/
bool HandActionsModule::homeAll()
{
    if(twoArms)
        homeOtherArm();
    if(!homeTorsoPitch())
        yWarning("Problems sending torso pitch to home position");
    if(!homeArm())
        yWarning("Problems sending %s Arm to home position", arm.c_str());
    if(!homeTorso())
        yWarning("Problems sending torso to home position");
    if(!look_down())
        yWarning("Problems sending head to home position");
    return true;
}
/***************************************************/
bool HandActionsModule::homeTorso()
{

    for(int i=0; i<3; i++)
    {
        ctrlMT->setControlMode(i,VOCAB_CM_POSITION);
    }
    const double TORSO_DEF_HOME[] = {0.0, 0.0, 0.0};
    bool done=false;
    double elapsedTime=0.0;
    double startTime=Time::now();
    const double maxTimeout = 3.0;

    posT->positionMove(TORSO_DEF_HOME);
    while(!done && elapsedTime<maxTimeout)
    {
        posT->checkMotionDone(&done);
        Time::delay(0.04);
        elapsedTime = Time::now()-startTime;
    }

    return true;
}

/***************************************************/
bool HandActionsModule::homeTorsoPitch()
{

    // move torso pitch
    const double torsoPitchHome = -15.0;
    const int torsoPitchIdx = 2;
    ctrlMT->setControlMode(torsoPitchIdx ,VOCAB_CM_POSITION);
    posT->positionMove(torsoPitchIdx, torsoPitchHome);

    bool done=false;
    double elapsedTime=0.0;
    double startTime=Time::now();

    const double maxTimeout = 3.0;

    while(!done && elapsedTime<maxTimeout)
    {
        posT->checkMotionDone(&done);
        Time::delay(0.04);
        elapsedTime = Time::now()-startTime;
    }
    return true;
}
/***************************************************/
bool HandActionsModule::homeArm()
{
    // set control modes
    ctrlMA->getControlModes(controlModesArm);
    const int lastArmJoint = 7;
    for(int i=0; i<lastArmJoint; i++)
    {
        if (controlModesArm[i]!=VOCAB_CM_POSITION)
        {
            ctrlMA->setControlMode(i,VOCAB_CM_POSITION);
        }
    }

   // move arm
    const double ARM_DEF_HOME[] = {-50.0,  60.0,  0.0,    40.0,    0.0,  0.0,   0.0};

    //posA->setRefSpeeds(handVels.data()); // 9 or 16 values?

    //posA->positionMove(ARM_DEF_HOME);
    for (int i=0; i<lastArmJoint; i++) // arm joints, no hand joints
    {
        posA->positionMove(i, ARM_DEF_HOME[i]);
    }

    bool done=false;
    double elapsedTime=0.0;
    double startTime=Time::now();
    const double maxTimeout = 3.0;

    while(!done && elapsedTime<maxTimeout)
    {
        posA->checkMotionDone(&done);
        Time::delay(0.04);
        elapsedTime = Time::now()-startTime;
    }
    return true;
}

/***************************************************/
bool HandActionsModule::setFingers(const std::string &posture)
{
    if (posture=="straight")
    {
        yDebug("setting fingers to straight posture");
        moveHand(STRAIGHT);
    }
    else if (posture=="fortyfive")
    {
        yDebug("setting fingers to fortyfive posture");
        moveHand(FORTYFIVE);
    }
    else if (posture=="bent")
    {
        yDebug("setting fingers to bent posture");
        moveHand(BENT);
    }
    else
    {
        yError("valid finger postures are: straight, fortyfive, bent");
        return false;
    }

    return true;
}

/***************************************************/
bool HandActionsModule::tapFromLeft()
{
    Bottle best3D = getBestObject3D();
    if (best3D.size() != 3)
    {
        //yError("problem with getBestObject3D");
        return false;
    }

    double x, y, z;
    x = best3D.get(0).asDouble();
    y = best3D.get(1).asDouble();
    z = best3D.get(2).asDouble();

    return tapFromLeftCoords(x,y,z);
}

/***************************************************/
bool HandActionsModule::tapFromRight()
{
    Bottle best3D = getBestObject3D();
    if (best3D.size() != 3)
    {
        //yError("problem with getBestObject3D");
        return false;
    }

    double x, y, z;
    x = best3D.get(0).asDouble();
    y = best3D.get(1).asDouble();
    z = best3D.get(2).asDouble();

    return tapFromRightCoords(x,y,z);
}

/***************************************************/
bool HandActionsModule::push()
{
    Bottle best3D = getBestObject3D();
    if (best3D.size() != 3)
    {
        //yError("problem with getBestObject3D");
        return false;
    }

    double x, y, z;
    x = best3D.get(0).asDouble();
    y = best3D.get(1).asDouble();
    z = best3D.get(2).asDouble();

    return pushCoords(x,y,z);
}

/***************************************************/
bool HandActionsModule::draw()
{
    Bottle best3D = getBestObject3D();
    if (best3D.size() != 3)
    {
        //yError("problem with getBestObject3D");
        return false;
    }

    double x, y, z;
    x = best3D.get(0).asDouble();
    y = best3D.get(1).asDouble();
    z = best3D.get(2).asDouble();

    return drawCoords(x,y,z);}

/***************************************************/
bool HandActionsModule::tapFromLeftCoords(const double x, const double y, const double z)
{
    Vector targetPos(3);
    targetPos[0] = x;
    targetPos[1] = y;
    targetPos[2] = z;

    fixate(targetPos);
    yInfo()<<"fixating at ("<<targetPos.toString(3,3)<<")";

    Vector o=computeHandOrientation();
    yInfo()<<"computed orientation = ("<<o.toString(3,3)<<")";

    if ( approachTargetWithHand(targetPos,o,"left") )
        roll(targetPos,o,"left");

    homeAll();
    return true;
}

/***************************************************/
bool HandActionsModule::tapFromRightCoords(const double x, const double y, const double z)
{
    Vector targetPos(3);
    targetPos[0] = x;
    targetPos[1] = y;
    targetPos[2] = z;

    fixate(targetPos);
    yInfo()<<"fixating at ("<<targetPos.toString(3,3)<<")";

    Vector o=computeHandOrientation();
    yInfo()<<"computed orientation = ("<<o.toString(3,3)<<")";

    if (approachTargetWithHand(targetPos,o,"right") )
        roll(targetPos,o,"right");

    homeAll();
    return true;
}

/***************************************************/
bool HandActionsModule::pushCoords(const double x, const double y, const double z)
{
    Vector targetPos(3);
    targetPos[0] = x;
    targetPos[1] = y;
    targetPos[2] = z;

    fixate(targetPos);
    yInfo()<<"fixating at ("<<targetPos.toString(3,3)<<")";

    Vector o=computeHandOrientation();
    yInfo()<<"computed orientation = ("<<o.toString(3,3)<<")";

    if ( approachTargetWithHand(targetPos,o,"bottom") )
        roll(targetPos,o,"bottom");

    homeAll();

    return true;
}

/***************************************************/
bool HandActionsModule::drawCoords(const double x, const double y, const double z)
{
    Vector targetPos(3);
    targetPos[0] = x;
    targetPos[1] = y;
    targetPos[2] = z;

    fixate(targetPos);
    yInfo()<<"fixating at ("<<targetPos.toString(3,3)<<")";

    Vector o=computeHandOrientation();
    yInfo()<<"computed orientation = ("<<o.toString(3,3)<<")";
    double min, max;
    int axis = 2; // torso yaw
    iarm->getLimits(axis,&min,&max);
    iarm->setLimits(axis,-15.0,15.0);
    if ( approachTargetWithHand(targetPos,o,"top") )
        roll(targetPos,o,"top");
    iarm->setLimits(axis,min,max);
    homeArm();
    yDebug("Arm at home position");
    homeTorso();
    yDebug("Torso at home position");
    look_down();
    yDebug("Head at home position");
    return true;
}

/***************************************************/
bool HandActionsModule::attachTip(const std::vector<double> &offsetPos, const std::vector<double> &offsetOri)
{
    // sanity checks
    if (offsetPos.size() != 3)
    {
        yError("wrong size of offsetPos");
        return false;
    }

    if (offsetOri.size() != 4)
    {
        yError("wrong size of offsetOri");
        return false;
    }

    Vector x(3, 0.0);
    for (int i=0; i<offsetPos.size(); ++i)
        x[i] = offsetPos[i];

    Vector o(4, 0.0);
    for (int i=0; i<offsetOri.size(); ++i)
        o[i] = offsetOri[i];

    yDebug("attachTip x: %s, o: %s", x.toString().c_str(), o.toString().c_str());

    return iarm->attachTipFrame(x,o);
}

/***************************************************/
bool HandActionsModule::quit()
{
    yInfo("quitting");
    closing = true;

    return true;
}
