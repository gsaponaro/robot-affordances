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
    outImgPort.interrupt();
}

/**********************************************************/
void RobotHandProcessorThread::close()
{
    inImgPort.close();
    outImgPort.close();
}

/**********************************************************/
bool RobotHandProcessorThread::openPorts()
{
    bool ret = true;

    inImgPortName = "/" + moduleName + "/image:i";
    ret = ret && inImgPort.open(inImgPortName.c_str());

    outImgPortName = "/" + moduleName + "/image:o";
    ret = ret && outImgPort.open(outImgPortName.c_str());

    return ret;
}

/**********************************************************/
void RobotHandProcessorThread::mainProcessing()
{
    if (closing)
        return;

    ImageOf<PixelBgr> *inImg;
    inImg = inImgPort.read(true);

    if (inImg != NULL)
    {
        // create OpenCV output image, for now identical to raw input image
        Mat outMat;
        outMat = cvarrToMat(static_cast<IplImage*>(inImg->getIplImage()));

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
