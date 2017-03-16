/*
 * Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>,
 * CopyPolicy: Released under the terms of the GNU GPL v3.0
 *
 */

#include "RobotHandProcessorModule.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

/***************************************************/
bool RobotHandProcessorModule::configure(ResourceFinder &rf)
{
    string moduleName = rf.check("name",Value("robotHandProcessor")).asString();

    closing = false;

    rpcPort.open("/"+moduleName+"/rpc:i");
    attach(rpcPort);

    // create new thread, pass parameters with ResourceFinder
    thread = new RobotHandProcessorThread(moduleName,
                                          rf);

    // start the thread to do the work
    if (!thread->start())
    {
        delete thread;
        return false;
    }

    return true;
}

/***************************************************/
bool RobotHandProcessorModule::interruptModule()
{
    rpcPort.interrupt();

    return true;
}

/***************************************************/
bool RobotHandProcessorModule::close()
{
    yInfo("closing RPC port");
    rpcPort.close();

    yInfo("starting shutdown procedure");
    thread->interrupt();
    thread->close();
    thread->stop();
    yInfo("deleting thread");
    if (thread) delete thread;
    yInfo("done deleting thread");

    return true;
}

/***************************************************/
double RobotHandProcessorModule::getPeriod()
{
    return 0.0;
}

/***************************************************/
bool RobotHandProcessorModule::updateModule()
{
    return !closing;
}

// IDL functions

/***************************************************/
bool RobotHandProcessorModule::attach(RpcServer &source)
{
    return this->yarp().attachAsServer(source);
}

/***************************************************/
bool RobotHandProcessorModule::quit()
{
    yInfo("quitting");
    closing = true;

    return true;
}

/***************************************************/
bool RobotHandProcessorModule::look(const string &target)
{
    return thread->look(target);
}

/***************************************************/
bool RobotHandProcessorModule::resetKinematics()
{
    return thread->resetKinematics();
}

/***************************************************/
double RobotHandProcessorModule::getArmPos(int32_t joint)
{
    return thread->getArmPos(joint);
}

/***************************************************/
Bottle RobotHandProcessorModule::getArmPoss()
{
    return thread->getArmPoss();
}

/***************************************************/
bool RobotHandProcessorModule::setArmPos(const int32_t joint, const double value)
{
    return thread->setArmPos(joint,value);
}

/***************************************************/
bool RobotHandProcessorModule::setArmPoss(const Bottle &values)
{
    return thread->setArmPoss(values);
}

/***************************************************/
double RobotHandProcessorModule::getHeadPos(const int32_t joint)
{
    return thread->getHeadPos(joint);
}

/***************************************************/
Bottle RobotHandProcessorModule::getHeadPoss()
{
    return thread->getHeadPoss();
}

/***************************************************/
bool RobotHandProcessorModule::setHeadPos(const int32_t joint, const double value)
{
    return thread->setHeadPos(joint,value);
}

/***************************************************/
bool RobotHandProcessorModule::setHeadPoss(const Bottle &values)
{
    return thread->setHeadPoss(values);
}
