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

}

/**********************************************************/
void RobotHandProcessorThread::close()
{

}

/**********************************************************/
void RobotHandProcessorThread::mainProcessing()
{
    if (closing)
        return;


}

// IDL functions
