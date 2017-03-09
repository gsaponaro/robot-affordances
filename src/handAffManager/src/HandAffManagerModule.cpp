/*
 * Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>,
 *         Atabak Dehban <adehban@isr.tecnico.ulisboa.pt>,
 *         Lorenzo Jamone, Afonso Gonçalves
 * CopyPolicy: Released under the terms of the GNU GPL v3.0
 *
 */

#include "HandAffManagerModule.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

/***************************************************/
bool HandAffManagerModule::configure(ResourceFinder &rf)
{
    string moduleName = rf.check("name",Value("handAffManager")).asString();

    closing = false;

    rpcPort.open("/"+moduleName+"/rpc:i");
    attach(rpcPort);

    return true;
}

/***************************************************/
bool HandAffManagerModule::interruptModule()
{
    rpcPort.interrupt();

    return true;
}

/***************************************************/
bool HandAffManagerModule::close()
{
    rpcPort.close();

    return true;
}

/***************************************************/
double HandAffManagerModule::getPeriod()
{
    return 0.0;
}

/***************************************************/
bool HandAffManagerModule::updateModule()
{
    return !closing;
}

// IDL functions

/***************************************************/
bool HandAffManagerModule::attach(RpcServer &source)
{
    return this->yarp().attachAsServer(source);
}

/***************************************************/
bool HandAffManagerModule::quit()
{
    yInfo("quitting");
    closing = true;

    return true;
}