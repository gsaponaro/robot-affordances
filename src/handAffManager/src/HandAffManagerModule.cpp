/*
 * Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>,
 *         Atabak Dehban <adehban@isr.tecnico.ulisboa.pt>,
 *         Lorenzo Jamone, Afonso Gonçalves
 * CopyPolicy: Released under the terms of the GNU GPL v3.0
 *
 */

#include "CSVFile.h"
#include "HandAffManagerModule.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

/***************************************************/
bool HandAffManagerModule::configure(ResourceFinder &rf)
{
    string moduleName = rf.check("name",Value("handAffManager")).asString();

    inHandDescPortName = "/" + moduleName + "/handDesc:i";
    inHandDescPort.open(inHandDescPortName.c_str());

    inObjDescPortName = "/" + moduleName + "/objDesc:i";
    inObjDescPort.open(inObjDescPortName.c_str());

    rpcHandActionsPortName = "/" + moduleName + "/handActions:rpc";
    rpcHandActionsPort.open(rpcHandActionsPortName.c_str());

    rpcRobotHandProcessorPortName = "/" + moduleName + "/robotHandProcessor:rpc";
    rpcRobotHandProcessorPort.open(rpcRobotHandProcessorPortName.c_str());

    //closing = false;

    rpcPort.open("/"+moduleName+"/rpc:i");
    attach(rpcPort);

    return true;
}

/***************************************************/
bool HandAffManagerModule::interruptModule()
{
    inHandDescPort.interrupt();
    inObjDescPort.interrupt();
    rpcHandActionsPort.interrupt();
    rpcRobotHandProcessorPort.interrupt();
    rpcPort.interrupt();

    return true;
}

/***************************************************/
bool HandAffManagerModule::close()
{
    inHandDescPort.close();
    inObjDescPort.close();
    rpcHandActionsPort.close();
    rpcRobotHandProcessorPort.close();
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
    /*
    if (inHandDescPort.getInputCount()>0 && inObjDescPort.getInputCount()>0)
    {
        Bottle *inHandDesc = inHandDescPort.read(true);
        Bottle *inObjDesc = inObjDescPort.read(true);

        if (inHandDesc!=NULL && inObjDesc!=NULL)
        {

        }
    }
    */

    //return !closing;
    return true;
}

// IDL functions

/***************************************************/
bool HandAffManagerModule::attach(RpcServer &source)
{
    return this->yarp().attachAsServer(source);
}

/***************************************************/
bool HandAffManagerModule::handPosture(const string &posture)
{
    // sanity checks
    if (rpcHandActionsPort.getOutputCount()<1)
    {
        yError("no connection to handActions RPC server");
        return false;
    }

    if (inHandDescPort.getInputCount()<1)
    {
        yError("no connection to hand descriptors");
        return false;
    }

    if (rpcRobotHandProcessorPort.getOutputCount()<1)
    {
        yError("no connection to robotHandProcessor RPC server");
        return false;
    }

    // set robot hand *on the real robot*
    if (posture=="straight" || posture=="fortyfive" || posture=="bent")
    {
        Bottle handActionsCmd;
        Bottle handActionsReply;
        handActionsCmd.addString("setFingers");
        handActionsCmd.addString(posture);
        rpcHandActionsPort.write(handActionsCmd, handActionsReply);
        if (handActionsReply.size()>0 &&
            handActionsReply.get(0).asVocab()==Vocab::encode("ok"))
        {
            yInfo("successfully set real hand posture to %s", posture.c_str());
        }
        else
        {
            yError("problem when setting real hand posture to %s", posture.c_str());
            return false;
        }
    }
    else
    {
        yError("valid finger postures are: straight, fortyfive, bent");
        return false;
    }

    // move head and arm *in the simulator* so that hand is fully visible
    Bottle simCmd;
    Bottle simReply;
    simCmd.addString("resetKinematics"); // arm
    rpcRobotHandProcessorPort.write(simCmd, simReply);
    if (simReply.size()>0 &&
        simReply.get(0).asVocab()==Vocab::encode("ok"))
    {
        yInfo("successfully simulated reset of arm positions");
    }
    else
    {
        yError("problem when simulating reset of arm positions");
        return false;
    }
    simCmd.clear();
    simReply.clear();
    simCmd.addString("look");
    string desiredHand = "right_hand";
    simCmd.addString(desiredHand);
    rpcRobotHandProcessorPort.write(simCmd, simReply);
    if (simReply.size()>0 &&
        simReply.get(0).asVocab()==Vocab::encode("ok"))
    {
        yInfo("successfully simulated looking at %s", desiredHand.c_str());
        yDebug("please visually check the simulated hand image!");
    }
    else
    {
        yError("problem when simulating looking at %s", desiredHand.c_str());
        return false;
    }

    // acquire hand top descriptors
    handTopDesc.clear();
    Bottle *inHandDesc = inHandDescPort.read(true);
    if (inHandDesc != NULL)
    {
        const int expectedDescSize = 39;
        if (inHandDesc->size() != expectedDescSize)
            yWarning("got %d hand descriptors instead of %d", inHandDesc->size(), expectedDescSize);

        const int firstTopIdx = 13;
        const int lastTopIdx = 25;
        for (int d=firstTopIdx; d<=25; ++d)
            handTopDesc.addDouble(inHandDesc->get(d).asDouble());
    }

    yInfo("successfully acquired hand top descriptors");

    return true;
}

/***************************************************/
bool HandAffManagerModule::quit()
{
    yInfo("quitting");
    //closing = true;

    return true;
}
