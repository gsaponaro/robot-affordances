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

#include "HandAffManagerModule.h"

namespace fs = boost::filesystem;

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

/***************************************************/
bool HandAffManagerModule::configure(ResourceFinder &rf)
{
    string moduleName = rf.check("name",Value("handAffManager")).asString();

    inHandImgPortName = "/" + moduleName + "/handImage:i";
    inHandImgPort.open(inHandImgPortName.c_str());

    inHandDescPortName = "/" + moduleName + "/handDesc:i";
    inHandDescPort.open(inHandDescPortName.c_str());

    inObjImgPortName = "/" + moduleName + "/objImage:i";
    inObjImgPort.open(inObjImgPortName.c_str());

    inObjDescPortName = "/" + moduleName + "/objDesc:i";
    inObjDescPort.open(inObjDescPortName.c_str());

    rpcHandActionsPortName = "/" + moduleName + "/handActions:rpc";
    rpcHandActionsPort.open(rpcHandActionsPortName.c_str());

    rpcRobotHandProcessorPortName = "/" + moduleName + "/robotHandProcessor:rpc";
    rpcRobotHandProcessorPort.open(rpcRobotHandProcessorPortName.c_str());

    rpcGazePortName = "/" + moduleName + "/gaze:rpc";
    rpcGazePort.open(rpcGazePortName.c_str());

    needUserConfirmation = false;
    userResponse = false;

    // create hands and objects descriptors file with current date and time
    string timestr = getDateAndTime();
    string filenameHandsObjects;
    filenameHandsObjects = "handsAndObjectsDescriptors_" + timestr + ".csv";
    yDebug("handsObjects filename is %s", filenameHandsObjects.c_str());
    csvHandsObjects.setFilename(filenameHandsObjects);

    // write header row
    csvHandsObjects << "handOrObjectName" << "convexity" << "eccentricity"
        << "compactness" << "circularity" << "squareness" << "convexityDefects"
        // central normalized moments
        << "nu00" << "nu11" << "nu02" << "nu30" << "nu21" << "nu12" << "nu03"
        << endrow;

    // effects files
    basePath = rf.getHomeContextPath().c_str();
    //yDebug("basePath: %s", basePath.c_str());

    //closing = false;

    rpcPort.open("/"+moduleName+"/rpc:i");
    attach(rpcPort);

    return true;
}

/***************************************************/
bool HandAffManagerModule::interruptModule()
{
    inHandImgPort.interrupt();
    inHandDescPort.interrupt();
    inObjImgPort.interrupt();
    inObjDescPort.interrupt();
    rpcHandActionsPort.interrupt();
    rpcRobotHandProcessorPort.interrupt();
    rpcPort.interrupt();

    return true;
}

/***************************************************/
bool HandAffManagerModule::close()
{
    inHandImgPort.close();
    inHandDescPort.close();
    inObjImgPort.close();
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
    // enter here after the provisional handDesc Bottle has been filled
    if (needUserConfirmation && handDesc.size()>0)
    {
        // in RPC, we asked user to confirm whether simulated hand descriptors
        // and image are good (yes or no)
        while(needUserConfirmation)
            yarp::os::Time::delay(0.1);

        if (!userResponse)
        {
            yWarning("no -> will reset hand descriptors and image, please restart the acquisition");
            handDesc.clear();
            //handImage = cv::Mat::zeros(inHandImg->height(),inHandImg->width(),CV_8UC3);
            //objImage = cv::Mat::zeros(inObjImg->height(),inObjImg->width(),CV_8UC3);
            return true;
        }
        yInfo("yes -> will save hand descriptors and image to file");

        // make sure that currPosture is up to date
        if (currPosture == "")
        {
            yWarning("I don't know the current hand posture, please set it with setHandPosture");
            return true;
        }
        else
        {
            // all good so far
            yDebug("current hand posture is %s", currPosture.c_str());
            if (saveDescAndImage(currPosture))
                currPosture = ""; // reset variable
        }
    }

    // enter here after the provisional objDesc Bottle has been filled
    if (needUserConfirmation && objDesc.size()>0)
    {
        // in RPC, we asked user to confirm whether object descriptors and image are good (yes or no)
        while(needUserConfirmation)
            yarp::os::Time::delay(0.1);

        if (!userResponse)
        {
            yWarning("no -> will reset object descriptors and image, please restart the acquisition");
            handDesc.clear();
            //handImage = cv::Mat::zeros(inHandImg->height(),inHandImg->width(),CV_8UC3);
            //objImage = cv::Mat::zeros(inObjImg->height(),inObjImg->width(),CV_8UC3);
            return true;
        }
        yInfo("yes -> will save object descriptors and image to file");

        // make sure that currObj is up to date
        if (currObj == "")
        {
            yWarning("I don't know the current object label, please set it with setObject");
            return true;
        }
        else
        {
            // all good so far
            yDebug("current object is %s", currObj.c_str());
            if (saveDescAndImage(currObj))
                currObj = ""; // reset variable
        }
    }

    //return !closing;
    return true;
}

/***************************************************/
bool HandAffManagerModule::getHandDesc()
{
    // acquire provisional hand descriptors; if successful put them
    // in the handDesc Bottle

    if (inHandDescPort.getInputCount()<1)
    {
        yError("no connection to hand descriptors");
        return false;
    }

    handDesc.clear();

    // try for a few seconds
    const double waitTime = 5.0;
    Bottle *inHandDesc = inHandDescPort.read(false);
    double t1 = yarp::os::Time::now();
    while ((yarp::os::Time::now()-t1 < waitTime) && (inHandDesc == NULL))
    {
        inHandDesc = inHandDescPort.read(false);
        yarp::os::Time::delay(0.1);
    }

    if (inHandDesc != NULL)
    {
        const int expectedDescSize = 41;
        if (inHandDesc->size() != expectedDescSize)
            yError("got %d hand descriptors instead of %d", inHandDesc->size(), expectedDescSize);

        handDesc.clear();

        // whole
        const int firstIdx = 2;
        const int lastIdx = 14;
        // top
        //const int firstIdx = 15;
        //const int lastIdx = 27;
        for (int d=firstIdx; d<=lastIdx; ++d)
            handDesc.addDouble(inHandDesc->get(d).asDouble());
    }
    else
    {
        yError("did not receive hand descriptors after trying for %f seconds", waitTime);
        return false;
    }

    yInfo("provisional hand descriptors acquired successfully:");
    yInfo("%s", handDesc.toString().c_str());

    return true;
}

/***************************************************/
bool HandAffManagerModule::getHandImage()
{
    // acquire provisional hand image; if successful put it
    // in the handImage Mat

    if (inHandImgPort.getInputCount()<1)
    {
        yError("no connection to hand image");
        return false;
    }

    // try for a few seconds
    const double waitTime = 5.0;
    ImageOf<PixelBgr> *inHandImg = inHandImgPort.read(false);
    double t1 = yarp::os::Time::now();
    while ((yarp::os::Time::now()-t1 < waitTime) && (inHandImg == NULL))
    {
        inHandImg = inHandImgPort.read(false);
        yarp::os::Time::delay(0.1);
    }

    if (inHandImg != NULL)
    {
        //handImageTimeStr.clear();
        handImage = cv::Mat::zeros(inHandImg->height(),inHandImg->width(),CV_8UC3);

        handImage = iplToMat(*inHandImg);
        //handImageTimeStr = getDateAndTime();
    }
    else
    {
        yError("did not receive hand image after trying for %f seconds", waitTime);
        return false;
    }

    yInfo("provisional hand image acquired successfully");

    return true;
}

/***************************************************/
bool HandAffManagerModule::getObjDesc()
{
    // acquire provisional object descriptors; if successful put them
    // in the objDesc Bottle

    if (inObjDescPort.getInputCount()<1)
    {
        yError("no connection to object descriptors");
        return false;
    }

    objDesc.clear();

    // try for a few seconds
    const double waitTime = 5.0;
    Bottle *inObjDesc = inObjDescPort.read(false);
    double t1 = yarp::os::Time::now();
    while ((yarp::os::Time::now()-t1 < waitTime) && (inObjDesc == NULL))
    {
        inObjDesc = inObjDescPort.read(false);
        yarp::os::Time::delay(0.1);
    }

    if (inObjDesc != NULL)
    {
        const int expectedDescSize = 41;
        if (inObjDesc->size() != expectedDescSize)
            yError("got %d object descriptors instead of %d", inObjDesc->size(), expectedDescSize);

        objDesc.clear();

        // whole
        const int firstIdx = 2;
        const int lastIdx = 14;
        // top
        //const int firstIdx = 15;
        //const int lastIdx = 27;
        for (int d=firstIdx; d<=lastIdx; ++d)
            objDesc.addDouble(inObjDesc->get(d).asDouble());
    }
    else
    {
        yError("did not receive object descriptors after trying for %f seconds", waitTime);
        return false;
    }

    yInfo("provisional object descriptors acquired successfully:");
    yInfo("%s", objDesc.toString().c_str());

    return true;
}

/***************************************************/
bool HandAffManagerModule::getObjImage()
{
    // acquire provisional object image; if successful put it
    // in the objImage Mat

    if (inObjImgPort.getInputCount()<1)
    {
        yError("no connection to object image");
        return false;
    }

    // try for a few seconds
    const double waitTime = 5.0;
    ImageOf<PixelBgr> *inObjImg = inObjImgPort.read(false);
    double t1 = yarp::os::Time::now();
    while ((yarp::os::Time::now()-t1 < waitTime) && (inObjImg == NULL))
    {
        inObjImg = inObjImgPort.read(false);
        yarp::os::Time::delay(0.1);
    }

    if (inObjImg != NULL)
    {
        //objImageTimeStr.clear();
        objImage = cv::Mat::zeros(inObjImg->height(),inObjImg->width(),CV_8UC3);

        objImage = iplToMat(*inObjImg);
        //objImageTimeStr = getDateAndTime();
    }
    else
    {
        yError("did not receive object image after trying for %f seconds", waitTime);
        return false;
    }

    yInfo("provisional object image acquired successfully");

    return true;
}

/***************************************************/
bool HandAffManagerModule::saveDescAndImage(const string &label)
{
    return saveDesc(label) && saveImage(label);
}

/***************************************************/
bool HandAffManagerModule::saveDesc(const string &label)
{
    // TODO adapt function to both hands and objects

    // sanity checks
    const int numDesc = 13;
    if (handDesc.size() != numDesc)
    {
        yError("got %d descriptors, was expecting %d", handDesc.size(), numDesc);
        return false;
    }

    // write data row
    csvHandsObjects << label;
    for (int d=0; d<numDesc; ++d)
        csvHandsObjects << handDesc.get(d).asDouble();

    csvHandsObjects << endrow;

    yInfo("sucessfully saved descriptors of %s to file", label.c_str());

    handDesc.clear();
    return true;
}

/***************************************************/
bool HandAffManagerModule::saveImage(const string &label)
{
    bool isHand = (label=="straight") ||
                  (label=="fortyfive") ||
                  (label=="bent");

    string filename;

    if (isHand)
    {
        // hand
        if (handImage.empty())
        {
            yError("handImage is empty, cannot save it");
            return false;
        }
        filename += "hand_";
    }
    else
    {
        // object
        if (objImage.empty())
        {
            yError("objImage is empty, cannot save it");
            return false;
        }
        filename += "object_";
    }

    // both hand and object
    filename += label;
    filename += "_";
    string imageTimeStr = getDateAndTime(); // TODO do it at acquisition time instead
    filename += imageTimeStr;
    filename += ".jpg";

    cv::imwrite(filename.c_str(), (isHand ? handImage : objImage));

    yInfo("sucessfully saved image of %s to file", label.c_str());

    return true;
}

/***************************************************/
string HandAffManagerModule::getDateAndTime()
{
    // http://stackoverflow.com/a/16358264/1638888
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];
    time (&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(buffer,sizeof(buffer),"%Y-%m-%d_%H%M%S",timeinfo);
    string timestr(buffer);

    return timestr;
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
        handActionsCmd.clear();
        handActionsReply.clear();
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
    simCmd.clear();
    simReply.clear();
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
string HandAffManagerModule::getHand()
{
    // acquire provisional hand descriptors
    if (!getHandDesc())
        return "failed acquiring hand descriptors";

    // acquire provisional hand image
    if (!getHandImage())
        return "failed acquiring hand image";

    // acquisition OK, now we need to ask the user for confirmation
    needUserConfirmation = true;

    return "successfully acquired hand descriptors and image: if they look OK please type 'yes', otherwise type 'no'";
}

/***************************************************/
bool HandAffManagerModule::setObjectName(const string &objName)
{
    if (objName=="")
    {
        yError("invalid object name");
        return false;
    }

    currObj = objName;
    yInfo("target object is now: %s", currObj.c_str());
    return true;
}

/***************************************************/
string HandAffManagerModule::getObject()
{
    // acquire provisional object descriptors
    if (!getObjDesc())
        return "failed acquiring object descriptors";

    // acquire provisional object image
    if (!getObjImage())
        return "failed acquiring object image";

    // acquisition OK, now we need to ask the user for confirmation
    needUserConfirmation = true;

    return "successfully acquired object descriptors and image: if they look OK please type 'yes', otherwise type 'no'";
}

/***************************************************/
string HandAffManagerModule::start(const string &action)
{
    // sanity checks
    if (action!="tapFromLeft" || action!="tapFromRight" || action!="push" || action!="draw")
    {
        return "invalid action! The valid ones are: tapFromLeft, tapFromRight, push, draw";
    }

    if (currPosture == "")
    {
        return "I don't know the current hand posture, please set it with setHandPosture";
    }

    if (rpcHandActionsPort.getOutputCount()<1)
    {
        return "no connection to handActions RPC server";
    }

    // target object initial position information
    currAction = action;
    Bottle init2D = getBestObject2D();
    if (init2D.size() != 2)
    {
        return "problem with init2D";
    }
    Bottle init3D = getBestObject3D();
    if (init3D.size() != 3)
    {
        return "problem with init3D";
    }

    // do the motor action
    yWarning("requesting %s motor action on the real robot", action.c_str());
    yarp::os::Time::delay(1.0);
    Bottle handActionsCmd;
    Bottle handActionsReply;
    handActionsCmd.clear();
    handActionsReply.clear();
    handActionsCmd.addString(action);
    rpcHandActionsPort.write(handActionsCmd, handActionsReply);
    if (handActionsReply.size()>0 &&
        handActionsReply.get(0).asVocab()==Vocab::encode("ok"))
    {
        yInfo("successfully performed %s", action.c_str());
    }
    else
    {
        yError("problem when requesting %s motor action", action.c_str());
        return "could not perform the motor action";
    }

    // target object final position information
    Bottle final2D = getBestObject2D();
    if (final2D.size() != 2)
    {
        return "problem with final2D";
    }
    Bottle final3D = getBestObject3D();
    if (final3D.size() != 3)
    {
        return "problem with final3D";
    }

    // effect computation and request for confirmation
    double effX = final3D.get(0).asDouble() - init3D.get(0).asDouble();
    double effY = final3D.get(1).asDouble() - init3D.get(1).asDouble();

    needUserConfirmation = true;

    stringstream sstm;
    sstm << "successfully performed the action and computed the effects in the root frame:\n" <<
          "X: " << effX << ", Y:" << effY << "\n" <<
          "if they look OK please type 'yes', otherwise type 'no'";

    return sstm.str();
}

/***************************************************/
bool HandAffManagerModule::yes()
{
    needUserConfirmation = false; // reset variable
    userResponse = true;
    return true;
}

/***************************************************/
bool HandAffManagerModule::no()
{
    needUserConfirmation = false; // reset variable
    userResponse = false;
    return true;
}

/***************************************************/
Bottle HandAffManagerModule::getBestObject2D()
{
    // the selection is done by descriptorReduction according to:
    // (i) sufficiently large area
    // (ii) closest to robot (highest image y)

    Bottle res;
    res.clear();

    if (inObjDescPort.getInputCount()>0)
    {
        Bottle *inObjDesc = inObjDescPort.read(false);
        double t1 = yarp::os::Time::now();
        while ((yarp::os::Time::now()-t1 < 5.0) && (inObjDesc == NULL))
        {
            inObjDesc = inObjDescPort.read(false);
            yarp::os::Time::delay(0.1);
        }

        if (inObjDesc!=NULL)
        {
            // descriptorReduction already ensures that there is only 1 blob
            const double u = inObjDesc->get(0).asDouble();
            const double v = inObjDesc->get(1).asDouble();
            res.addDouble(u);
            res.addDouble(v);
        }
    }

    return res;
}

/***************************************************/
Bottle HandAffManagerModule::getBestObject3D()
{
    // the selection is done by descriptorReduction according to:
    // (i) sufficiently large area
    // (ii) closest to robot (highest image y)

    Bottle res;
    res.clear();

    if (inObjDescPort.getInputCount()>0 && rpcGazePort.getOutputCount()>0)
    {
        Bottle *inObjDesc = inObjDescPort.read(false);
        double t1 = yarp::os::Time::now();
        while ((yarp::os::Time::now()-t1 < 5.0) && (inObjDesc == NULL))
        {
            inObjDesc = inObjDescPort.read(false);
            yarp::os::Time::delay(0.1);
        }

        if (inObjDesc!=NULL)
        {
            // descriptorReduction already ensures that there is only 1 blob
            const double u = inObjDesc->get(0).asDouble();
            const double v = inObjDesc->get(1).asDouble();

            const string eye = "left"; // we always use left camera so far

            // heigh of the table in z direction [m]
            const double tableOffset = -0.07; // TODO parameterize

            const double planeA =  0.0;
            const double planeB =  0.0;
            const double planeC = -1.0;
            const double planeD = tableOffset;

            // convert coords from 2D to 3D via RPC queries to gaze control
            Bottle cmd;
            Bottle reply;
            cmd.clear();
            reply.clear();
            // use homography, equivalent to IGaze client get3DPointOnPlane()
            cmd.addVocab(Vocab::encode("get"));
            cmd.addVocab(Vocab::encode("3D"));
            cmd.addVocab(Vocab::encode("proj"));
            Bottle &cmdContent = cmd.addList();
            cmdContent.clear();
            cmdContent.addString(eye);
            cmdContent.addDouble(u);
            cmdContent.addDouble(v);
            cmdContent.addDouble(planeA);
            cmdContent.addDouble(planeB);
            cmdContent.addDouble(planeC);
            cmdContent.addDouble(planeD);

            //yDebug("query to gaze: %s", cmd.toString().c_str());
            rpcGazePort.write(cmd, reply);
            //yDebug("reply from gaze: %s", reply.toString().c_str());
            if (reply.size()==2 && // ack (x y z)
                reply.get(0).asVocab()==Vocab::encode("ack") &&
                reply.get(1).isList() &&
                reply.get(1).asList()->size()==3)
            {
                res = *reply.get(1).asList();
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
