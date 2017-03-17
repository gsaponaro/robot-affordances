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
    if (!useHand)
        return;

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
    if (!useHand)
        return false;

    if (!safetyCheck(x,side))
    {
        yWarning("action is dangerous, I will not do it!");
        return false;
    }

    Vector dof(10,1.0),dummy;
    // disable torso
    //dof[0]=0;
    //dof[1]=0;
    //dof[2]=0;
    iarm->setDOF(dof,dummy);
    Vector initialApproach;
    Vector finalApproach=x;
    if(side.compare("right")==0) // tapFromRight
    {
        yDebug("tapFromRight");
        finalApproach[1] += 0.1;
    }
    else if(side.compare("left")==0) // tapFromLeft
    {
        yDebug("tapFromLeft");
        finalApproach[1] -= 0.1;
    }
    else if(side.compare("bottom")==0) // push
    {
        yDebug("push");
        finalApproach[0] += 0.15;
    }
    else if(side.compare("top")==0) // draw
    {
        yDebug("draw");
        finalApproach[0] -= 0.10;
        // increase y when using right_arm, decrease y when using left_arm
        finalApproach[1] += 0.03*(arm=="right_arm" ? 1 : -1);
    }
    finalApproach[2] += 0.03;               // Offset - avoid collision with table
    initialApproach = finalApproach;
    initialApproach[2] += 0.1;              // avoid collision with objects during the approach phase
    iarm->goToPoseSync(initialApproach,o);
    iarm->waitMotionDone();
    iarm->goToPoseSync(finalApproach,o);
    iarm->waitMotionDone();

    return true;
}

/***************************************************/
void HandActionsModule::roll(const Vector &targetPos, const Vector &o, string side)
{
    if (!useHand)
        return;

    double tempotempo;
    iarm->getTrajTime(&tempotempo);
    iarm->setTrajTime(0.7);
    Vector targetModified=targetPos;
    if(side.compare("right")==0) // tapFromRight
    {
        targetModified[1] -= 0.1;
        iarm->setTrajTime(0.7);
    }
    else if(side.compare("left")==0) // tapFromLeft
    {
        targetModified[1] += 0.1;
        iarm->setTrajTime(0.7);
    }
    else if(side.compare("bottom")==0) // push
    {
        targetModified[0] -= 0.1;
        iarm->setTrajTime(1.3);
    }
    else if(side.compare("top")==0) // draw
    {
        targetModified[0] += 0.1;
        // increase y when using right_arm, decrease y when using left_arm
        targetModified[1] += 0.03*(arm=="right_arm" ? 1 : -1);
        iarm->setTrajTime(1.3);
    }
    targetModified[2] += 0.03;            // Offset - avoid collision with table
    iarm->goToPoseSync(targetModified,o);
    iarm->waitMotionDone();
    iarm->setTrajTime(tempotempo);
}

/***************************************************/
bool HandActionsModule::configure(ResourceFinder &rf)
{
    string moduleName = rf.check("name",Value("handActions")).asString();
    string robot = rf.check("robot",Value("icub")).asString();
    arm = rf.check("arm",Value("left_arm")).asString();
    if (arm!="left_arm" && arm!="right_arm")
    {
        yWarning("invalid arm %s specified, using the default (left_arm)", arm.c_str());
        arm = "left_arm";
    }
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

    straightHandPoss.resize(9, 0.0);
    straightHandPoss[0] =  0.0; // j7
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
        drvArm.close(); // IMPORTANT - Because drvArm was already open
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
        return false;
    }

    if (!drvArmPos.view(posA) || !drvArmPos.view(encsA) || !drvArmPos.view(ctrlMA))
    {
        cout << moduleName << ": problems acquiring interfaces to remote_controlboard of " << arm << endl;
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
        terminate();
        return false;
    }

    if (!drvTorso.view(posT) || !drvTorso.view(ctrlMT))
    {
        yError("problems acquiring interfaces to remote_controlboard of torso");
        return false;
    }

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
    if (useHand)
    {
        drvGaze.close();
        drvArm.close();
        drvArmPos.close();
        drvTorso.close();
    }

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
bool HandActionsModule::ensureNumVisibleObjects(int desiredNum)
{
    // sanity check
    if (rpcManagerPort.getOutputCount()<1)
    {
        yError("no connection to handAffManager RPC server");
        return false;
    }

    // send rpc query, verify if returned number equals the desired one
    Bottle handMgrCmd;
    Bottle handMgrReply;
    handMgrCmd.addString("getNumVisibleObjects");
    rpcManagerPort.write(handMgrCmd, handMgrReply);
    if (handMgrReply.size()>0 &&
        handMgrReply.get(0).asInt()==desiredNum)
    {
        return true;
    }
    else
    {
        yWarning("there are %d visible objects", handMgrReply.get(0).asInt());
        return false;
    }
}

/***************************************************/
bool HandActionsModule::getObject3D(double x, double y, double z)
{
    // sanity check
    if (rpcManagerPort.getOutputCount()<1)
    {
        yError("no connection to handAffManager RPC server");
        return false;
    }

    Bottle handMgrCmd;
    Bottle handMgrReply;
    handMgrCmd.addString("getObject3D");
    rpcManagerPort.write(handMgrCmd, handMgrReply);

    bool validReply = handMgrReply.size()>0 &&
                      handMgrReply.get(0).isList() &&
                      handMgrReply.get(0).asList()->size()==3;

    if (validReply)
    {
        x = handMgrReply.get(0).asList()->get(0).asDouble();
        y = handMgrReply.get(0).asList()->get(1).asDouble();
        z = handMgrReply.get(0).asList()->get(2).asDouble();
        return true;
    }
    else
    {
        yWarning("invalid reply to getObject3D: %s", handMgrReply.toString().c_str());
        return false;
    }

    return validReply;
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
    while (!done)
    {
        yarp::os::Time::delay(0.01);
        posA->checkMotionDone(&done);
    }
}

/***************************************************/
bool HandActionsModule::safetyCheck(const Vector &targetPos, const std::string &side)
{
    // all actions
    const double xMinThresh = -0.20;
    if (targetPos[0] > xMinThresh)
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
    const double xMaxThresh = -0.35;
    if (side=="top" || side=="bottom")
    {
        if (targetPos[0] > xMaxThresh)
        {
            yWarning("unsafe x");
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
    ang[1] = -40.0;
    igaze->lookAtAbsAngles(ang);
    igaze->waitMotionDone();

    return true;
}

/***************************************************/
bool HandActionsModule::home()
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

    for(int i=0; i<3; i++)
    {
        ctrlMT->setControlMode(i,VOCAB_CM_POSITION);
    }

    // move torso pitch
    const double TORSO_DEF_HOME[] = {0.0, 0.0, 0.0};
    const int torsoPitchIdx = 2;
    posT->positionMove(torsoPitchIdx, TORSO_DEF_HOME[torsoPitchIdx]);

    bool done=false;
    double elapsedTime=0.0;
    double startTime=Time::now();

    while(!done && elapsedTime<2.0)
    {
        posT->checkMotionDone(&done);
        Time::delay(0.04);
        elapsedTime= Time::now()-startTime;
    }

    // move arm
    const double ARM_DEF_HOME[] = {-50.0,  60.0,  0.0,    40.0,    0.0,  0.0,   0.0};

    //posA->setRefSpeeds(handVels.data()); // 9 or 16 values?

    //posA->positionMove(ARM_DEF_HOME);
    for (int i=0; i<lastArmJoint; i++) // arm joints, no hand joints
    {
        posA->positionMove(i, ARM_DEF_HOME[i]);
    }

    done=false;
    elapsedTime=0.0;
    startTime=Time::now();

    while(!done && elapsedTime<2.0)
    {
        posA->checkMotionDone(&done);
        Time::delay(0.04);
        elapsedTime= Time::now()-startTime;
    }

    // move torso yaw and roll
    posT->positionMove(TORSO_DEF_HOME);

    done=false;
    elapsedTime=0.0;
    startTime=Time::now();

    while(!done && elapsedTime<2.0)
    {
        posT->checkMotionDone(&done);
        Time::delay(0.04);
        elapsedTime= Time::now()-startTime;
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
    if (!ensureNumVisibleObjects(1))
    {
        return false;
    }

    double x, y, z;
    if (!getObject3D(x,y,z))
    {
        yError("problem with getObject3D");
        return false;
    }

    return tapFromLeftCoords(x,y,z);
}

/***************************************************/
bool HandActionsModule::tapFromRight()
{
    if (!ensureNumVisibleObjects(1))
    {
        return false;
    }

    double x, y, z;
    if (!getObject3D(x,y,z))
    {
        yError("problem with getObject3D");
        return false;
    }

    return tapFromRightCoords(x,y,z);
}

/***************************************************/
bool HandActionsModule::push()
{
    if (!ensureNumVisibleObjects(1))
    {
        return false;
    }

    double x, y, z;
    if (!getObject3D(x,y,z))
    {
        yError("problem with getObject3D");
        return false;
    }

    return pushCoords(x,y,z);
}

/***************************************************/
bool HandActionsModule::draw()
{
    if (!ensureNumVisibleObjects(1))
    {
        return false;
    }

    double x, y, z;
    if (!getObject3D(x,y,z))
    {
        yError("problem with getObject3D");
        return false;
    }

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

    home();
    look_down();

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

    if ( approachTargetWithHand(targetPos,o,"right") )
        roll(targetPos,o,"right");

    home();
    look_down();

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

    home();
    look_down();

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

    if ( approachTargetWithHand(targetPos,o,"top") )
        roll(targetPos,o,"top");

    home();
    look_down();

    return true;
}

/***************************************************/
bool HandActionsModule::quit()
{
    yInfo("quitting");
    closing = true;

    return true;
}
