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
    numHeadJoints = 6;

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

    if (inImgPort.getInputCount()<1)
    {
        yarp::os::Time::delay(0.1);
        return;
    }

    //LockGuard lg(mutex);

    ImageOf<PixelBgr> *inImg;
    inImg = inImgPort.read(true);

    if (inImg != NULL)
    {
        //yDebug("processing image");

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
    if (outHeadJointsPort.getOutputCount()<1)
    {
        yError("missing connection: %s /iCubUnitySim/head:i", outHeadJointsPortName.c_str());
        return false;
    }

    if (target != "left_hand")
    {
        yError("for now, the only valid target is left_hand");
        return false;
    }

    // head joints corresponding to target position
    Vector headPoss;
    headPoss.resize(6, 0.0);
    headPoss[0] = -30.0;
    headPoss[1] =   0.0;
    headPoss[2] =  30.0;
    headPoss[3] =   0.0;
    headPoss[4] =   0.0;
    headPoss[5] =   0.0;

    // send head joints to Unity simulator
    Bottle bHeadPoss;
    bHeadPoss.clear();
    for (int j=0; j<numHeadJoints; ++j)
        bHeadPoss.addDouble(headPoss[j]);

    return setHeadPoss(bHeadPoss);
}

/***************************************************/
bool RobotHandProcessorThread::resetKinematics()
{
    yarp::os::Network::connect("/icub/left_arm/state:o", "/iCubUnitySim/leftArm:i");
    yInfo("connected /icub/left_arm/state:o /iCubUnitySim/leftArm:i");

    yarp::os::Time::delay(1.0);

    yarp::os::Network::disconnect("/icub/left_arm/state:o", "/iCubUnitySim/leftArm:i");
    yInfo("disconnected /icub/left_arm/state:o /iCubUnitySim/leftArm:i");

    yarp::os::Time::delay(1.0);

    yarp::os::Network::connect("/icub/head/state:o", "/iCubUnitySim/head:i");
    yInfo("connected /icub/head/state:o /iCubUnitySim/head:i");

    yarp::os::Time::delay(1.0);

    yarp::os::Network::disconnect("/icub/head/state:o", "/iCubUnitySim/head:i");
    yInfo("disconnected /icub/head/state:o /iCubUnitySim/head:i");

    return true;
}

/***************************************************/
double RobotHandProcessorThread::getArmPos(const int32_t joint)
{
    double errorValue = -1.0;

    if (inArmJointsPort.getInputCount()<1)
    {
        yError("missing connection: /iCubUnitySim/leftArm:o %s", inArmJointsPortName.c_str());
        return errorValue;
    }

    if (joint<0 || joint>numArmJoints)
    {
        yError("getArmPos: joint argument must be between 0 and %d", numArmJoints-1);
        return errorValue;
    }

    Bottle *inArmJoints;
    inArmJoints = inArmJointsPort.read(true);

    if (inArmJoints != NULL)
    {
        if (inArmJoints->size() == numArmJoints)
        {
            return inArmJoints->get(joint).asDouble();
        }
    }

    return errorValue;
}

/***************************************************/
Bottle RobotHandProcessorThread::getArmPoss()
{
    if (inArmJointsPort.getInputCount()<1)
    {
        yError("missing connection: /iCubUnitySim/leftArm:o %s", inArmJointsPortName.c_str());
        return Bottle();
    }

    //LockGuard lg(mutex);

    Bottle *inArmJoints;
    inArmJoints = inArmJointsPort.read(true);

    if (inArmJoints != NULL)
        return *inArmJoints;
    else
    {
        yError("getArmPoss problem");
        return Bottle();
    }
}

/***************************************************/
bool RobotHandProcessorThread::setArmPos(const int32_t joint, const double value)
{
    // sanity checks
    if (outArmJointsPort.getOutputCount()<1)
    {
        yError("missing connection: %s /iCubUnitySim/leftArm:i", outArmJointsPortName.c_str());
        return false;
    }

    if (joint<0 || joint>numArmJoints)
    {
        yError("setPos: joint argument must be between 0 and %d", numArmJoints-1);
        return false;
    }

    // get current joints
    Bottle currentArmPoss = getArmPoss();

    // construct output Bottle
    Bottle &outArmJoints = outArmJointsPort.prepare();
    outArmJoints.clear();
    if (currentArmPoss.size() == numArmJoints)
    {
        // copy current joint values, except for the entry to overwrite
        for (int j=0; j<numArmJoints; ++j)
        {
            if (j != joint)
                outArmJoints.addDouble(currentArmPoss.get(j).asDouble());
            else
                outArmJoints.addDouble(value);
        }
    }

    // send output Bottle
    outArmJointsPort.write();

    return true;
}

/***************************************************/
bool RobotHandProcessorThread::setArmPoss(const Bottle &values)
{
    if (outArmJointsPort.getOutputCount()<1)
    {
        yError("missing connection: %s /iCubUnitySim/leftArm:i", outArmJointsPortName.c_str());
        return false;
    }

    if (values.size() != numArmJoints)
    {
        yError("setArmPoss argument must be a Bottle with %d elements enclosed within ( )", numArmJoints);
        return false;
    }

    Bottle &valuesCopy = outArmJointsPort.prepare();
    valuesCopy.clear();

    for (int j=0; j<numArmJoints; ++j)
        valuesCopy.addDouble(values.get(j).asDouble());

    outArmJointsPort.write();

    return true;
}

/***************************************************/
double RobotHandProcessorThread::getHeadPos(const int32_t joint)
{
    double errorValue = -1.0;

    if (inHeadJointsPort.getInputCount()<1)
    {
        yError("missing connection: /iCubUnitySim/head:o %s", inHeadJointsPortName.c_str());
        return errorValue;
    }

    if (joint<0 || joint>numHeadJoints)
    {
        yError("getHeadPos: joint argument must be between 0 and %d", numHeadJoints-1);
        return errorValue;
    }

    Bottle *inHeadJoints;
    inHeadJoints = inHeadJointsPort.read(true);

    if (inHeadJoints != NULL)
    {
        if (inHeadJoints->size() == numHeadJoints)
        {
            return inHeadJoints->get(joint).asDouble();
        }
    }

    return errorValue;
}

/***************************************************/
Bottle RobotHandProcessorThread::getHeadPoss()
{
    if (inHeadJointsPort.getInputCount()<1)
    {
        yError("missing connection: /iCubUnitySim/head:o %s", inHeadJointsPortName.c_str());
        return Bottle();
    }

    //LockGuard lg(mutex);

    Bottle *inHeadJoints;
    inHeadJoints = inHeadJointsPort.read(true);

    if (inHeadJoints != NULL)
        return *inHeadJoints;
}

/***************************************************/
bool RobotHandProcessorThread::setHeadPos(const int32_t joint, const double value)
{
    // sanity checks
    if (outHeadJointsPort.getOutputCount()<1)
    {
        yError("missing connection: %s /iCubUnitySim/head:i", outHeadJointsPortName.c_str());
        return false;
    }

    if (joint<0 || joint>numHeadJoints)
    {
        yError("setHeadPos: joint argument must be between 0 and %d", numHeadJoints-1);
        return false;
    }

    // get current joints
    Bottle currentHeadPoss = getHeadPoss();

    // construct output Bottle
    Bottle &outHeadJoints = outHeadJointsPort.prepare();
    outHeadJoints.clear();
    if (currentHeadPoss.size() == numHeadJoints)
    {
        // copy current joint values, except for the entry to overwrite
        for (int j=0; j<numHeadJoints; ++j)
        {
            if (j != joint)
                outHeadJoints.addDouble(currentHeadPoss.get(j).asDouble());
            else
                outHeadJoints.addDouble(value);
        }
    }

    // send output Bottle
    outHeadJointsPort.write();

    return true;
}

/***************************************************/
bool RobotHandProcessorThread::setHeadPoss(const Bottle &values)
{
    if (outHeadJointsPort.getOutputCount()<1)
    {
        yError("missing connection: %s /iCubUnitySim/head:i", outHeadJointsPortName.c_str());
        return false;
    }

    if (values.size() != numHeadJoints)
    {
        yError("setHeadPoss argument must be a Bottle with %d elements enclosed within ( )", numHeadJoints);
        return false;
    }

    //LockGuard lg(mutex);

    Bottle &valuesCopy = outHeadJointsPort.prepare();
    valuesCopy.clear();

    for (int j=0; j<numHeadJoints; ++j)
        valuesCopy.addDouble(values.get(j).asDouble());

    outHeadJointsPort.write();

    return true;
}
