/*
 * Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>,
 * CopyPolicy: Released under the terms of the GNU GPL v3.0
 *
 */

#include "RobotHandProcessorThread.h"

using namespace cv;
using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

/**********************************************************/
RobotHandProcessorThread::RobotHandProcessorThread(
    const string &_moduleName,
    ResourceFinder &_rf)
    : RateThread(33), // [ms]
      moduleName(_moduleName),
      rf(_rf)
{
}

/**********************************************************/
bool RobotHandProcessorThread::threadInit()
{
    if ( !openPorts() )
    {
        yError("problem opening ports");
        return false;
    }

    closing = false;

    numArmJoints = 16;
    armJoints.resize(numArmJoints, 0.0);

    numHeadJoints = 6;
    headJoints.resize(numHeadJoints, 0.0);

    armHasChanged = false;
    timeSinceArmUpdate = Time::now();

    return true;
}

/**********************************************************/
void RobotHandProcessorThread::run()
{
    while (!closing)
        mainProcessing();
}


/**********************************************************/
void RobotHandProcessorThread::interrupt()
{
    closing = true;

    inImgPort.interrupt();
    inArmJointsPort.interrupt();
    inHeadJointsPort.interrupt();
    outImgPort.interrupt();
    outArmJointsPort.interrupt();
    outHeadJointsPort.interrupt();
}

/**********************************************************/
void RobotHandProcessorThread::close()
{
    inImgPort.close();
    inArmJointsPort.close();
    inHeadJointsPort.close();
    outImgPort.close();
    outArmJointsPort.close();
    outHeadJointsPort.close();
}

/**********************************************************/
bool RobotHandProcessorThread::openPorts()
{
    bool ret = true;

    inImgPortName = "/" + moduleName + "/image:i";
    ret = ret && inImgPort.open(inImgPortName.c_str());

    inArmJointsPortName = "/" + moduleName + "/armJoints:i";
    ret = ret && inArmJointsPort.open(inArmJointsPortName.c_str());

    inHeadJointsPortName = "/" + moduleName + "/headJoints:i";
    ret = ret && inHeadJointsPort.open(inHeadJointsPortName.c_str());

    outImgPortName = "/" + moduleName + "/image:o";
    ret = ret && outImgPort.open(outImgPortName.c_str());

    outArmJointsPortName = "/" + moduleName + "/armJoints:o";
    ret = ret && outArmJointsPort.open(outArmJointsPortName.c_str());

    outHeadJointsPortName = "/" + moduleName + "/headJoints:o";
    ret = ret && outHeadJointsPort.open(outHeadJointsPortName.c_str());

    return ret;
}

/**********************************************************/
void RobotHandProcessorThread::mainProcessing()
{
    if (closing)
        return;

    // send updated arm joint values periodically
    if (armHasChanged &&
        outArmJointsPort.getOutputCount()>0 &&
        timeSinceArmUpdate>5.0)
    {
        Bottle &outJoints = outArmJointsPort.prepare();
        outJoints.clear();
        for (int j=0; j<numArmJoints; ++j)
        {
            outJoints.addDouble(armJoints[j]);
        }
        outArmJointsPort.write();
        timeSinceArmUpdate = Time::now();
    }

    ImageOf<PixelBgr> *inImg;
    inImg = inImgPort.read(true);

    if (inImg != NULL)
    {
        // create OpenCV output image, for now identical to input image
        // (foreground 0 black, background 255 white)
        Mat outMat;
        outMat = cvarrToMat(static_cast<IplImage*>(inImg->getIplImage()));

        // invert foreground and background
        outMat = Scalar::all(255) - outMat;

        // apply morphological transformation to fill holes
        Mat element = getStructuringElement(MORPH_ELLIPSE, Size(10,10));
        morphologyEx(outMat, outMat, MORPH_CLOSE, element);

        // send image on yarp port
        ImageOf<PixelBgr> &outYarp = outImgPort.prepare();
        IplImage outIpl = outMat;
        outYarp.resize(outIpl.width,
                       outIpl.height);
        cvCopy( &outIpl,
                static_cast<IplImage*>(outYarp.getIplImage()) );
        outImgPort.write();
    }
}

// IDL functions
/***************************************************/
bool RobotHandProcessorThread::look(const string &target)
{
    if (target != "left_hand")
    {
        yError("for now, the only valid target is left_hand");
        return false;
    }

    if (outHeadJointsPort.getOutputCount()<1)
    {
        yWarning("not connected to iCubUnitySim head:i");
        return false;
    }

    // head joints corresponding to target position
    headJoints[0] = -30.0;
    headJoints[1] =   0.0;
    headJoints[2] =  30.0;
    headJoints[3] =   0.0;
    headJoints[4] =   0.0;
    headJoints[5] =   0.0;

    // move head
    Bottle &outHeadJoints = outHeadJointsPort.prepare();
    for (int j=0; j<numHeadJoints; ++j)
    {
        outHeadJoints.addDouble(headJoints[j]);
    }
    outHeadJointsPort.write();
    yInfo("successfully looked at target position");

    return true;
}

/***************************************************/
bool RobotHandProcessorThread::resetKinematics()
{
    yarp::os::Network::connect("/icub/right_arm/state:o", "/iCubUnitySim/rightArm:i");
    yInfo("connected /icub/right_arm/state:o /iCubUnitySim/rightArm:i/iCubUnitySim/rightArm:i");

    yarp::os::Time::delay(1.0);

    yarp::os::Network::disconnect("/icub/right_arm/state:o", "/iCubUnitySim/rightArm:i");
    yInfo("disconnected /icub/right_arm/state:o /iCubUnitySim/rightArm:i/iCubUnitySim/rightArm:i");

    return true;
}

/***************************************************/
double RobotHandProcessorThread::getPos(int32_t joint)
{
    if (joint<0 || joint>numArmJoints)
    {
        yError("getPos: joint argument must be between 0 and %d", numArmJoints);
        return 0.0; // error value
    }

    return armJoints(joint);
}

/***************************************************/
bool RobotHandProcessorThread::setPos(int32_t joint, double value)
{
    if (joint<0 || joint>numArmJoints)
    {
        yError("setPos: joint argument must be between 0 and %d", numArmJoints);
        return false;
    }

    armJoints[joint] = value;
    armHasChanged = true;

    // create string with joint values truncated to a desired decimal precision
    // http://stackoverflow.com/a/5113241/1638888
    stringstream armJointsTrunc;
    armJointsTrunc.precision(2);
    armJointsTrunc << fixed;
    bool lastIteration = false;
    for (int j=0; j<numArmJoints; ++j)
    {
        armJointsTrunc << armJoints[j];
        if (!lastIteration)
            armJointsTrunc << " ";
    }

    yInfo("arm joints: %s", armJointsTrunc.str().c_str());

    return true;
}
