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
/*
Vector HandActionsModule::retrieveTarget3D(const Vector &cogL, const Vector &cogR)
{
    Vector position(3);        
    igaze->triangulate3DPoint(cogL,cogR,position);
    // Use iGaze to retrieve the 3D point
    return position;
}
*/

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
void HandActionsModule::approachTargetWithHand(const Vector &x, const Vector &o,string side)
{
    Vector dof(10,1.0),dummy;
    // disable torso
    //dof[0]=0;
    //dof[1]=0;
    //dof[2]=0;
    iarm->setDOF(dof,dummy);
    Vector approach=x;
    if(side.compare("right")==0) {          //right
        approach[1]+=0.1;// 10 cm
        yInfo() << "app right";
    }
    else if(side.compare("left")==0) {      //left
        approach[1]-=0.1;// 10 cm
        yInfo() << "app left";
    }
    else if(side.compare("bottom")==0) {    //bottom
        approach[0]+=0.15;// 10 cm
        yInfo() << "app bottom";
    }
    iarm->goToPoseSync(approach,o);
    iarm->waitMotionDone();
}

/***************************************************/
void HandActionsModule::roll(const Vector &targetPos, const Vector &o, string side)
{
    double tempotempo;
    iarm->getTrajTime(&tempotempo);
    iarm->setTrajTime(0.7); //
    Vector target=targetPos;
    if(side.compare("right")==0) {          //right
        target[1]-=0.1;// 10 cm
        yInfo() << "roll right";
        iarm->setTrajTime(0.7);
    }
    else if(side.compare("left")==0) {      //left
        target[1]+=0.1;// 10 cm
        yInfo() << "roll right";
        iarm->setTrajTime(0.7);
    }
    else if(side.compare("bottom")==0) {    //bottom
        target[0]-=0.1;// 10 cm
        yInfo() << "roll bottom";
        iarm->setTrajTime(1.3);
    }

    iarm->goToPoseSync(target,o);
    iarm->waitMotionDone();
    iarm->setTrajTime(tempotempo);
}

/***************************************************/
void HandActionsModule::make_it_roll(const Vector &targetPos)
{
    fixate(targetPos);
    yInfo()<<"fixating at ("<<targetPos.toString(3,3)<<")";

    Vector o=computeHandOrientation();
    yInfo()<<"computed orientation = ("<<o.toString(3,3)<<")";

    approachTargetWithHand(targetPos,o,"right");
    yInfo()<<"approached";

    roll(targetPos,o,"right");
    yInfo()<<"roll!";
}

/***************************************************/
/*
void HandActionsModule::retrieveObjLocation(const Bottle &command)
{
    objLocation[0] = command.get(1).asDouble();
    objLocation[1] = command.get(2).asDouble();
    objLocation[2] = command.get(3).asDouble();
    yInfo() << "Object Location" << objLocation.toString();
}
*/

/***************************************************/
bool HandActionsModule::configure(ResourceFinder &rf)
{
    string moduleName = rf.check("name",Value("handActions")).asString();
    string robot = rf.check("robot",Value("icubSim")).asString();
    string arm = rf.check("arm",Value("right_arm")).asString();
    if(arm.compare("left")==0 ||arm.compare("leftarm")==0 || arm.compare("armleft")==0 || arm.compare("Left")==0)
        arm = "left_arm";
    else
        arm = "right_arm";

    /******** Cartesian Interface *******/

    Property optArm;
    optArm.put("device","cartesiancontrollerclient");
    optArm.put("remote","/"+robot+"/cartesianController/"+arm);
    optArm.put("local","/"+moduleName+"/cartesian_client/"+arm);

    //objLocation.resize(3);

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

    /******** Position Arm Control Interface *******/
    /* connect to remote device  */
    Property options;
    options.put("device", "remote_controlboard");             // device to open
    options.put("local", "/"+moduleName+"/position_control/"+arm);           // local port name
    options.put("remote", "/" + robot + "/"+arm);         // where we connect to
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
    Property optGaze;
    optGaze.put("device","gazecontrollerclient");
    optGaze.put("remote","/iKinGazeCtrl");
    optGaze.put("local","/"+moduleName+"/gaze_client");

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
       cout << moduleName << ": problems acquiring interfaces to remote_controlboard of Torso"<< endl;
        return false;
    }

    /******** Gaze Control *******/
    if (!drvGaze.open(optGaze))
    {
        yError()<<"Unable to open the Gaze Controller";
        drvArm.close(); // IMPORTANT - Because drvArm was already open
        return false;
    }
    drvGaze.view(igaze);
    drvArm.view(iarm);

    //imgLPortIn.open("/imgL:i");
    //imgRPortIn.open("/imgR:i");
    //imgLPortOut.open("/imgL:o");
    //imgRPortOut.open("/imgR:o");

    rpcPort.open("/"+moduleName+"/rpc:i");
    attach(rpcPort);

    return true;
}

/***************************************************/
bool HandActionsModule::interruptModule()
{
    //imgLPortIn.interrupt();
    //imgRPortIn.interrupt();
    //imgLPortOut.interrupt();
    //imgRPortOut.interrupt();
    rpcPort.interrupt();

    return true;
}

/***************************************************/
bool HandActionsModule::close()
{
    drvArm.close();
    drvGaze.close();
    drvArmPos.close();
    drvTorso.close();

    //imgLPortIn.close();
    //imgRPortIn.close();
    //imgLPortOut.close();
    //imgRPortOut.close();
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
/*
    Vector xd(3);
    xd[0] = -0.3;
    xd[1] = +0.2;
    xd[2] = 0.0;
    iarm->goToPositionSync(xd);
    iarm->waitMotionDone();
*/
    ctrlMA->getControlModes(controlModesArm);

    //fprintf(stderr,"\ngoHomeArm - step_1\n");

    for(int i=0; i<nAxesA; i++)
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

    posA->positionMove(ARM_DEF_HOME);

    bool done=false;
    double elapsedTime=0.0;
    double startTime=Time::now();

    while(!done && elapsedTime<2.0)
    {
        posA->checkMotionDone(&done);
        Time::delay(0.04);
        elapsedTime= Time::now()-startTime;
    }

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
bool HandActionsModule::tapFromLeft(const double x, const double y, const double z)
{
    Vector targetPos(3);
    targetPos[0] = x;
    targetPos[1] = y;
    targetPos[2] = z;

    fixate(targetPos);
    yInfo()<<"fixating at ("<<targetPos.toString(3,3)<<")";

    Vector o=computeHandOrientation();
    yInfo()<<"computed orientation = ("<<o.toString(3,3)<<")";

    approachTargetWithHand(targetPos,o,"left");
    yInfo()<<"approached";

    roll(targetPos,o,"left");
    yInfo()<<"roll!";

    return true;
}

/***************************************************/
bool HandActionsModule::tapFromRight(const double x, const double y, const double z)
{
    Vector targetPos(3);
    targetPos[0] = x;
    targetPos[1] = y;
    targetPos[2] = z;

    fixate(targetPos);
    yInfo()<<"fixating at ("<<targetPos.toString(3,3)<<")";

    Vector o=computeHandOrientation();
    yInfo()<<"computed orientation = ("<<o.toString(3,3)<<")";

    approachTargetWithHand(targetPos,o,"right");
    yInfo()<<"approached";

    roll(targetPos,o,"right");
    yInfo()<<"roll!";

    return true;
}

/***************************************************/
bool HandActionsModule::push(const double x, const double y, const double z)
{
    Vector targetPos(3);
    targetPos[0] = x;
    targetPos[1] = y;
    targetPos[2] = z;

    fixate(targetPos);
    yInfo()<<"fixating at ("<<targetPos.toString(3,3)<<")";

    Vector o=computeHandOrientation();
    yInfo()<<"computed orientation = ("<<o.toString(3,3)<<")";

    approachTargetWithHand(targetPos,o,"bottom");
    yInfo()<<"approached";

    roll(targetPos,o,"bottom");
    yInfo()<<"roll!";

    return true;
}

/***************************************************/
bool HandActionsModule::draw(const double x, const double y, const double z)
{
    Vector targetPos(3);
    targetPos[0] = x;
    targetPos[1] = y;
    targetPos[2] = z;

    yDebug("draw not implemented yet");
    return false;

    /*
    fixate(targetPos);
    yInfo()<<"fixating at ("<<targetPos.toString(3,3)<<")";

    Vector o=computeHandOrientation();
    yInfo()<<"computed orientation = ("<<o.toString(3,3)<<")";

    approachTargetWithHand(targetPos,o,"top");
    yInfo()<<"approached";

    roll(targetPos,o,"top");
    yInfo()<<"roll!";

    return true;
    */
}

bool HandActionsModule::quit()
{
    yInfo("quitting");
    interruptModule();
    close();

    return true;
}
