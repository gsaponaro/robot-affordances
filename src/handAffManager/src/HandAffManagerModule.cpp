/*
 * Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>,
 *         Atabak Dehban <adehban@isr.tecnico.ulisboa.pt>,
 *         Pedro Vicente <pvicente@isr.tecnico.ulisboa.pt>,
 *         Lorenzo Jamone, Afonso Gonçalves
 * CopyPolicy: Released under the terms of the GNU GPL v3.0
 *
 */

#include "CSVFile.h"
#include "HandAffManagerModule.h"

namespace fs = boost::filesystem;

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

    rpcGazePortName = "/" + moduleName + "/gaze:rpc";
    rpcGazePort.open(rpcGazePortName.c_str());

    gotSomething = false;
    userConfirmation = false;

    basePath = rf.getHomeContextPath().c_str();
    yDebug("basePath: %s", basePath.c_str());

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
    // if we have made a call to getHandDescriptors()...
    if (handDesc.size()>0)
    {
        // ask user to confirm whether simulated hand & descriptors are good
        yWarning("if the simulated hand is properly visible and the descriptors make sense, type \"yes\" (in the RPC terminal), otherwise type \"no\"");
        while(!gotSomething)
            yarp::os::Time::delay(0.1);
        if (!userConfirmation)
        {
            yWarning("no -> will reset hand descriptors, please restart the acquisition");
            handDesc.clear();
        }
        yInfo("yes -> will save hand descriptors to file");
        gotSomething = false; // reset variable

        // make sure that currPosture is up to date
        if (currPosture == "")
        {
            yWarning("I don't know the current hand posture, please set it with setHandPosture");
            return true;
        }
        else
        {
            yDebug("current hand posture is %s", currPosture.c_str());
            saveDescToFile(currPosture);
        }
    }

    //return !closing;
    return true;
}

/***************************************************/
void HandAffManagerModule::saveDescToFile(const std::string &label)
{
    // sanity checks
    const int numDesc = 13;
    if (handDesc.size() != numDesc)
    {
        yError("got %d descriptors, was expecting %d", handDesc.size(), numDesc);
        return;
    }

    // path and file where we will save
    string path = basePath+"/"+label;
    yDebug("path: %s", path.c_str());
    string filename = "test.csv";
    csvfile csv(filename);

    // write header row
    csv << "handOrObjectName" << "convexity" << "eccentricity"
        << "compactness" << "circularity" << "squareness" << "convexityDefects"
        // central normalized moments
        << "nu00" << "nu11" << "nu02" << "nu30" << "nu21" << "nu12" << "nu03"
        << endrow;

    // write data row
    csv << label;
    for (int d=0; d<numDesc; ++d)
        csv << handDesc.get(d).asDouble();

    csv << endrow;
}

// IDL functions

/***************************************************/
bool HandAffManagerModule::attach(RpcServer &source)
{
    return this->yarp().attachAsServer(source);
}

/***************************************************/
bool HandAffManagerModule::setHandPosture(const string &posture)
{
    // sanity checks
    if (rpcHandActionsPort.getOutputCount()<1)
    {
        yError("no connection to handActions RPC server");
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
        yDebug("requesting %s hand posture on the real robot", posture.c_str());
        Bottle handActionsCmd;
        Bottle handActionsReply;
        handActionsCmd.addString("setFingers");
        handActionsCmd.addString(posture);
        rpcHandActionsPort.write(handActionsCmd, handActionsReply);
        if (handActionsReply.size()>0 &&
            handActionsReply.get(0).asVocab()==Vocab::encode("ok"))
        {
            yInfo("successfully set real hand posture to %s", posture.c_str());
            currPosture = posture; // save it for later
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
    yDebug("setting simulated arm kinematics to be like the real robot...");
    rpcRobotHandProcessorPort.write(simCmd, simReply);
    yDebug("...done");

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
    string desiredHand = "left_hand";
    simCmd.addString(desiredHand);
    rpcRobotHandProcessorPort.write(simCmd, simReply);
    if (simReply.size()>0 &&
        simReply.get(0).asVocab()==Vocab::encode("ok"))
    {
        yInfo("successfully simulated looking at %s", desiredHand.c_str());
    }
    else
    {
        yError("problem when simulating looking at %s", desiredHand.c_str());
        return false;
    }

    return true;
}

/***************************************************/
bool HandAffManagerModule::getHandDescriptors()
{
    // sanity checks
    if (inHandDescPort.getInputCount()<1)
    {
        yError("no connection to hand descriptors");
        return false;
    }

    // acquire hand descriptors
    handDesc.clear();
    Bottle *inHandDesc = inHandDescPort.read(true);
    if (inHandDesc != NULL)
    {
        const int expectedDescSize = 41;
        if (inHandDesc->size() != expectedDescSize)
            yWarning("got %d hand descriptors instead of %d", inHandDesc->size(), expectedDescSize);

        // whole
        const int firstIdx = 2;
        const int lastIdx = 14;
        // top
        //const int firstIdx = 15;
        //const int lastIdx = 27;
        for (int d=firstIdx; d<=lastIdx; ++d)
            handDesc.addDouble(inHandDesc->get(d).asDouble());
    }

    yInfo("successfully acquired hand descriptors");
    yInfo("%s", handDesc.toString().c_str());

    return true;
}

/***************************************************/
bool HandAffManagerModule::yes()
{
    gotSomething = true;
    userConfirmation = true;
    return true;
}

/***************************************************/
bool HandAffManagerModule::no()
{
    gotSomething = true;
    userConfirmation = false;
    return true;
}

/***************************************************/
int32_t HandAffManagerModule::getNumVisibleObjects()
{
    if (inObjDescPort.getInputCount()>0)
    {
        Bottle *inObjDesc = inObjDescPort.read(true);

        if (inObjDesc!=NULL)
        {
            // descriptorReduction already ensures that there is only 1 blob
            return 1;
        }
    }

    yError("something wrong with getNumVisibleObjects");
    const int errorCode = -1;
    return errorCode;
}

/***************************************************/
Bottle HandAffManagerModule::getObject3D()
{
    Bottle res;
    res.clear();

    if (inObjDescPort.getInputCount()>0 && rpcGazePort.getOutputCount()>0)
    {
        Bottle *inObjDesc = inObjDescPort.read(true);

        if (inObjDesc!=NULL)
        {
            // descriptorReduction already ensures that there is only 1 blob
            const double u = inObjDesc->get(0).asDouble();
            const double v = inObjDesc->get(1).asDouble();

            const string eye = "left"; // we always use left camera so far

            // heigh of the table in z direction [m]
            const double tableOffset = -0.10; // TODO parameterize

            const double planeA =  0.0;
            const double planeB =  0.0;
            const double planeC = -1.0;
            const double planeD = tableOffset;

            // convert coords from 2D to 3D via RPC queries to gaze control
            Bottle cmd;
            Bottle reply;
            // use homography, equivalent to IGaze client get3DPointOnPlane()
            cmd.addVocab(Vocab::encode("get"));
            cmd.addVocab(Vocab::encode("3D"));
            cmd.addVocab(Vocab::encode("proj"));
            cmd.addVocab(Vocab::encode(eye));
            cmd.addDouble(u);
            cmd.addDouble(v);
            cmd.addDouble(planeA);
            cmd.addDouble(planeB);
            cmd.addDouble(planeC);
            cmd.addDouble(planeD);

            rpcGazePort.write(cmd, reply);
            yDebug("reply from gaze: %s", reply.toString().c_str());
            if (reply.get(0).asVocab()==Vocab::encode("ack") &&
                reply.size()==4) // ack x y z
            {
                res.addDouble(reply.get(1).asDouble());
                res.addDouble(reply.get(2).asDouble());
                res.addDouble(reply.get(3).asDouble());
                res.addDouble(reply.get(4).asDouble());
            }
            else
                yWarning("problem with iKinGazeCtrl RPC");
        }
    }

    return res;
}

/***************************************************/
bool HandAffManagerModule::quit()
{
    yInfo("quitting");
    //closing = true;

    return true;
}
