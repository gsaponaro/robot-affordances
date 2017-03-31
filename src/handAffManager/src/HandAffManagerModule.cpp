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

    inLeftObjImgPortName = "/" + moduleName + "/leftObjImage:i";
    inLeftObjImgPort.open(inLeftObjImgPortName.c_str());

    inRightObjImgPortName = "/" + moduleName + "/rightObjImage:i";
    inRightObjImgPort.open(inRightObjImgPortName.c_str());

    inObjDescPortName = "/" + moduleName + "/objDesc:i";
    inObjDescPort.open(inObjDescPortName.c_str());

    outProvisionalLeftImgPortName = "/" + moduleName + "/tempLeftImage:o";
    outProvisionalLeftImgPort.open(outProvisionalLeftImgPortName.c_str());

    outProvisionalRightImgPortName = "/" + moduleName + "/tempRightImage:o";
    outProvisionalRightImgPort.open(outProvisionalRightImgPortName.c_str());

    rpcHandActionsPortName = "/" + moduleName + "/handActions:rpc";
    rpcHandActionsPort.open(rpcHandActionsPortName.c_str());

    rpcRobotHandProcessorPortName = "/" + moduleName + "/robotHandProcessor:rpc";
    rpcRobotHandProcessorPort.open(rpcRobotHandProcessorPortName.c_str());

    rpcGazePortName = "/" + moduleName + "/gaze:rpc";
    rpcGazePort.open(rpcGazePortName.c_str());

    initMotor = false;
    initSim = false;

    needUserConfirmation = false;
    userResponse = false;

    basePath = rf.getHomeContextPath().c_str();
    yDebug("basePath: %s", basePath.c_str());

    // time string that will be attached to filenames
    string timestr = getDateAndTime();

    // create hand descriptors file (incl. kinematics) and write header row
    string filenameHands;
    filenameHands = basePath + "/hands_" + timestr + ".csv";
    yDebug("hands filename is %s", filenameHands.c_str());
    csvHands.setFilename(filenameHands);
    csvHands << "postureName" << "convexity" << "eccentricity"
        << "compactness" << "circularity" << "squareness" << "convexityDefects"
        // central normalized moments
        << "nu00" << "nu11" << "nu02" << "nu30" << "nu21" << "nu12" << "nu03"
        // arm joints configuration in Unity simulator
        << "arm0" << "arm1" << "arm2" << "arm3" << "arm4" << "arm5" << "arm6"
        << "arm7" << "arm8" << "arm9" << "arm10" << "arm11" << "arm12"
        << "arm13" << "arm14" << "arm15"
        // head joints configuration in Unity simulator
        << "head0" << "head1" << "head2" << "head3" << "head4" << "head5"
        << endrow;

    // create object descriptors file and write header row
    string filenameObjects;
    filenameObjects = basePath + "/objects_" + timestr + ".csv";
    yDebug("objects filename is %s", filenameObjects.c_str());
    csvObjects.setFilename(filenameObjects);
    csvObjects << "objectName" << "convexity" << "eccentricity"
        << "compactness" << "circularity" << "squareness" << "convexityDefects"
        // central normalized moments
        << "nu00" << "nu11" << "nu02" << "nu30" << "nu21" << "nu12" << "nu03"
        << endrow;

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
    inLeftObjImgPort.interrupt();
    inRightObjImgPort.interrupt();
    inObjDescPort.interrupt();
    outProvisionalLeftImgPort.interrupt();
    outProvisionalRightImgPort.interrupt();
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
    inLeftObjImgPort.close();
    inRightObjImgPort.close();
    inObjDescPort.close();
    outProvisionalLeftImgPort.close();
    outProvisionalRightImgPort.interrupt();
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
    // initialization of motor part (handActions)
    if (!initMotor && rpcHandActionsPort.getOutputCount()>0)
    {
        Bottle handActionsCmd;
        Bottle handActionsReply;
        handActionsCmd.clear();
        handActionsReply.clear();
        handActionsCmd.addString("homeAll");
        yInfo("moving all robot parts to home...");
        yarp::os::Time::delay(1.0);
        rpcHandActionsPort.write(handActionsCmd, handActionsReply);
        if (handActionsReply.size()>0 &&
            handActionsReply.get(0).asVocab()==Vocab::encode("ok"))
        {
            yInfo("successfully moved all robot parts to home");
        }
        else
        {
            yError("problem when moving robot parts to home");
        }

        initMotor = true;
    }

    // initialization of graphical hand simulator
    if (rpcRobotHandProcessorPort.getOutputCount()>0 && !initSim)
    {
        if (simResetAndLook())
            initSim = true;
        else
            yWarning("problem initializing the graphical hand simulator");
    }

    // enter here after the provisional hand data has been acquired
    if (needUserConfirmation && handDesc.size()>0 && !handImageSim.empty() && armJoints.size()>0 && headJoints.size()>0)
    {
        // in RPC, we asked user to confirm whether simulated hand descriptors
        // and image are good (yes or no)
        while(needUserConfirmation)
            yarp::os::Time::delay(0.1);

        if (!userResponse)
        {
            yWarning("no -> will reset hand information, please restart the acquisition");
            handDesc.clear();
            armJoints.clear();
            headJoints.clear();
            handImageSim = cv::Mat::zeros(handImageSim.rows,handImageSim.cols,CV_8UC3);
            return true;
        }

        yInfo("yes -> will save hand information to disk");

        yAssert(currPosture!="");

        yDebug("current hand posture is %s", currPosture.c_str());
        if (saveDescAndImage(currPosture))
        {
            //currPosture = ""; // reset variables
            armJoints.clear();
            headJoints.clear();
        }
    }

    // enter here after the provisional object data has been acquired
    if (needUserConfirmation && objDesc.size()>0 && !leftObjImage.empty())
    {
        // in RPC, we asked user to confirm whether object descriptors and image are good (yes or no)
        while(needUserConfirmation)
            yarp::os::Time::delay(0.1);

        if (!userResponse)
        {
            yWarning("no -> will reset object information, please restart the acquisition");
            objDesc.clear();
            leftObjImage = cv::Mat::zeros(leftObjImage.rows,leftObjImage.cols,CV_8UC3);
            return true;
        }

        yInfo("yes -> will save object information to disk, object name is %s", currObj.c_str());

        yAssert(currObj!="");

        if (saveDescAndImage(currObj))
            currObj = ""; // reset variable
    }

    // enter here after the provisional effects Bottle has been filled
    if (needUserConfirmation && effects.size()>0)
    {
        // in RPC, we asked user to confirm whether effects and images are good (yes or no)
        while(needUserConfirmation)
            yarp::os::Time::delay(0.1);

        if (!userResponse)
        {
            yWarning("no -> will reset effects and images, please restart the acquisition");
            effects.clear();
            currPosture = "";
            currObj = "";
            currAction = "";
            return true;
        }

        yInfo("yes -> will save effects and images to file");

        yAssert(currPosture!=""); // TODO avoid this
        yAssert(currObj!="");
        yAssert(currAction!="");

        yDebug("posture=%s object=%s action=%s", currPosture.c_str(), currObj.c_str(), currAction.c_str());
        if (saveEffectsAndImages(currPosture, currObj, currAction))
        {
            effects.clear(); // reset variables
            currPosture = "";
            currObj = "";
            currAction = "";
        }
    }

    //return !closing;
    return true;
}

/***************************************************/
bool HandAffManagerModule::simResetAndLook()
{
    Bottle simCmd;
    Bottle simReply;
    simCmd.clear();
    simReply.clear();
    simCmd.addString("resetKinematics");
    yInfo("resetting simulator kinematics");
    rpcRobotHandProcessorPort.write(simCmd, simReply);
    if (simReply.size()>0 &&
        simReply.get(0).asVocab()==Vocab::encode("ok"))
    {
        yInfo("successfully reset kinematics of simulator");
    }
    else
    {
        yError("problem when resetting kinematics of simulator");
        return false;
    }

    simCmd.clear();
    simReply.clear();
    simCmd.addString("look");
    const string target = "left_hand"; // TODO parameterize
    simCmd.addString(target);
    rpcRobotHandProcessorPort.write(simCmd, simReply);
    if (simReply.size()>0 &&
        simReply.get(0).asVocab()==Vocab::encode("ok"))
    {
        yInfo("successfully looked at target %s in the simulator", target.c_str());
    }
    else
    {
        yError("problem when looking at target %s in the simulator", target.c_str());
        return false;
    }

    return true;
}

/***************************************************/
bool HandAffManagerModule::setHandPosture(const string &posture)
{
    /**
     * Set robot fingers to one of the permitted postures: straight, fortyfive,
     * bent. This is done first on the real robot, then in the Unity simulator.
     * @param posture the name of the posture: straight, fortyfive, bent
     * @return true/false on success/failure
     */

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
        yInfo("requesting %s hand posture on the real robot", posture.c_str());
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
    simResetAndLook();

    yarp::os::Time::delay(1.0);

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

    yDebug("provisional hand descriptors acquired successfully - shown in RPC terminal");

    return true;
}

/***************************************************/
bool HandAffManagerModule::getHandImage()
{
    // acquire provisional hand image; if successful put it
    // in the handImageSim Mat

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
        handImageSim = cv::Mat::zeros(inHandImg->height(),inHandImg->width(),CV_8UC3);

        handImageSim = iplToMat(*inHandImg);
        //handImageTimeStr = getDateAndTime();
    }
    else
    {
        yError("did not receive hand image after trying for %f seconds", waitTime);
        return false;
    }

    //yDebug("provisional hand image acquired successfully");

    return true;
}

/***************************************************/
bool HandAffManagerModule::setObjectName(const string &objName)
{
    /**
     * Set the current target object name.
     * @param objName the name of the target object
     * @return true/false on success/failure
     */

    if (objName=="")
    {
        yError("invalid object name");
        return false;
    }

    currObj = objName;
    yDebug("target object is now: %s", currObj.c_str());
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
        //const int firstIdx = 2;
        //const int lastIdx = 14;
        // top
        const int firstIdx = 15;
        const int lastIdx = 27;

        if (firstIdx==2)
            yDebug("saving descriptors of whole object");
        else if (firstIdx==15)
            yDebug("saving descriptors of top part");

        for (int d=firstIdx; d<=lastIdx; ++d)
            objDesc.addDouble(inObjDesc->get(d).asDouble());
    }
    else
    {
        yError("did not receive object descriptors after trying for %f seconds", waitTime);
        return false;
    }

    yDebug("provisional object descriptors acquired successfully - shown in RPC terminal");

    return true;
}

/***************************************************/
bool HandAffManagerModule::getSimArmHead()
{
    // get the arm and head joints configuration in the Unity simulator,
    // write them to armJoints and headJoints

    if (rpcRobotHandProcessorPort.getOutputCount()<1)
    {
        yError("no connection to robotHandProcessor");
        return false;
    }

    Bottle simCmd;
    Bottle simReply;
    simCmd.clear();
    simReply.clear();
    simCmd.addString("getArmPoss");
    rpcRobotHandProcessorPort.write(simCmd, simReply);
    //yDebug("getArmPoss reply: %s", simReply.toString().c_str());
    const int numArmJoints = 16;
    if (simReply.size()>0 &&
        simReply.get(0).isList() &&
        simReply.get(0).asList()->size()==numArmJoints)
    {
        armJoints.clear();
        armJoints = *simReply.get(0).asList();
        yInfo("successfully acquired simulator arm joints");
    }
    else
    {
        yError("problem when acquiring simulator arm joints");
        return false;
    }

    simCmd.clear();
    simReply.clear();
    simCmd.addString("getHeadPoss");
    rpcRobotHandProcessorPort.write(simCmd, simReply);
    const int numHeadJoints = 6;
    if (simReply.size()>0 &&
        simReply.get(0).isList() &&
        simReply.get(0).asList()->size()==numHeadJoints)
    {
        headJoints.clear();
        headJoints = *simReply.get(0).asList();
        yInfo("successfully acquired simulator head joints");
    }
    else
    {
        yError("problem when acquiring simulator head joints");
        return false;
    }

    return true;
}

/***************************************************/
bool HandAffManagerModule::getObjImages()
{
    // acquire provisional object images; if successful put them
    // in the leftObjImage and rightObjImage Mat

    const double waitTime = 5.0;

    // left

    if (inLeftObjImgPort.getInputCount()<1)
    {
        yError("no connection to left object image");
        return false;
    }

    // try for a few seconds
    ImageOf<PixelBgr> *inLeftObjImg = inLeftObjImgPort.read(false);
    double t1 = yarp::os::Time::now();
    while ((yarp::os::Time::now()-t1 < waitTime) && (inLeftObjImg == NULL))
    {
        inLeftObjImg = inLeftObjImgPort.read(false);
        yarp::os::Time::delay(0.1);
    }

    if (inLeftObjImg != NULL)
    {
        leftObjImage = cv::Mat::zeros(inLeftObjImg->height(),inLeftObjImg->width(),CV_8UC3);
        leftObjImage = iplToMat(*inLeftObjImg);
    }
    else
    {
        yError("did not receive left object image after trying for %f seconds", waitTime);
        return false;
    }

    // right

    if (inRightObjImgPort.getInputCount()<1)
    {
        yError("no connection to right object image");
        return false;
    }

    // try for a few seconds
    ImageOf<PixelBgr> *inRightObjImg = inRightObjImgPort.read(false);
    t1 = yarp::os::Time::now();
    while ((yarp::os::Time::now()-t1 < waitTime) && (inRightObjImg == NULL))
    {
        inRightObjImg = inRightObjImgPort.read(false);
        yarp::os::Time::delay(0.1);
    }

    if (inRightObjImg != NULL)
    {
        rightObjImage = cv::Mat::zeros(inRightObjImg->height(),inRightObjImg->width(),CV_8UC3);
        rightObjImage = iplToMat(*inRightObjImg);
    }
    else
    {
        yError("did not receive right object image after trying for %f seconds", waitTime);
        return false;
    }

    //yDebug("provisional object images acquired successfully");

    return true;
}

/***************************************************/
bool HandAffManagerModule::showTempImage(const string &type)
{
    if (outProvisionalLeftImgPort.getOutputCount()<1)
        return false;

    ImageOf<PixelBgr> &outProvisionalLeftImg = outProvisionalLeftImgPort.prepare();
    ImageOf<PixelBgr> &outProvisionalRightImg = outProvisionalRightImgPort.prepare();

    const cv::Point org(10,50); // Bottom-left corner of the text string in the image.
    const int myFont = cv::FONT_HERSHEY_SIMPLEX;
    const double fontScale = 0.7;
    const cv::Scalar Red = cv::Scalar(0,0,255);
    const cv::Scalar Blue = cv::Scalar(255,0,0);
    const int Radius = 5;

    if (type=="hand" && handDesc.size()>0 && !handImageSim.empty())
    {
        //yDebug("showTempImage hand");
        cv::Mat handImageSim2 = handImageSim.clone();
        outProvisionalLeftImg.resize(handImageSim2.cols, handImageSim2.rows);
        cv::putText(handImageSim2,"unsaved hand image",org,myFont,fontScale,Red);
        handImageSim2.copyTo(iplToMat(outProvisionalLeftImg));
        outProvisionalRightImgPort.unprepare(); // ugly
    }
    else if (type=="object" && objDesc.size()>0 && !leftObjImage.empty())
    {
        //yDebug("showTempImage object");
        cv::Mat leftObjImage2 = leftObjImage.clone();
        outProvisionalLeftImg.resize(leftObjImage2.cols, leftObjImage2.rows);
        cv::putText(leftObjImage2,"unsaved object image",org,myFont,fontScale,Red);
        leftObjImage2.copyTo(iplToMat(outProvisionalLeftImg));
        outProvisionalRightImgPort.unprepare(); // ugly
    }
    else if (type=="effect_init" && !leftObjImage.empty() && !rightObjImage.empty())
    {
        //yDebug("showTempImage effect_init");
        // left
        cv::Mat leftObjImage2 = leftObjImage.clone();
        outProvisionalLeftImg.resize(leftObjImage2.cols, leftObjImage2.rows);
        cv::putText(leftObjImage2,"unsaved initial left",org,myFont,fontScale,Red);
        if (init2D.size()==2)
        {
            cv::Point com(init2D.get(0).asDouble(), init2D.get(1).asDouble());
            cv::circle(leftObjImage2,
                       com,
                       Radius,
                       Blue,
                       -1); // filled
        }
        leftObjImage2.copyTo(iplToMat(outProvisionalLeftImg));

        // right
        cv::Mat rightObjImage2 = rightObjImage.clone();
        outProvisionalRightImg.resize(rightObjImage2.cols, rightObjImage2.rows);
        cv::putText(rightObjImage2,"unsaved initial right",org,myFont,fontScale,Red);
        rightObjImage2.copyTo(iplToMat(outProvisionalRightImg));
    }
    else if (type=="effect_final" && !leftObjImage.empty())
    {
        //yDebug("showTempImage effect_final");
        // left
        cv::Mat leftObjImage2 = leftObjImage.clone();
        outProvisionalLeftImg.resize(leftObjImage2.cols, leftObjImage2.rows);
        cv::putText(leftObjImage2,"unsaved final left",org,myFont,fontScale,Red);
        cv::Point comInit;
        cv::Point comFinal;
        if (init2D.size()==2)
        {
            comInit = cv::Point(init2D.get(0).asDouble(), init2D.get(1).asDouble());
            cv::circle(leftObjImage2,
                       comInit,
                       Radius,
                       Blue,
                       -1); // filled
        }
        if (final2D.size()==2)
        {
            comFinal = cv::Point(final2D.get(0).asDouble(), final2D.get(1).asDouble());
            cv::circle(leftObjImage2,
                       comFinal,
                       Radius,
                       Red,
                       -1); // filled
        }
        if (init2D.size()==2 && final2D.size()==2)
        {
            cv::arrowedLine(leftObjImage2,
                            comInit,
                            comFinal,
                            Blue,
                            2); // thickness
        }
        leftObjImage2.copyTo(iplToMat(outProvisionalLeftImg));

        // right
        cv::Mat rightObjImage2 = rightObjImage.clone();
        outProvisionalRightImg.resize(rightObjImage2.cols, rightObjImage2.rows);
        cv::putText(rightObjImage2,"unsaved final right",org,myFont,fontScale,Red);
        rightObjImage2.copyTo(iplToMat(outProvisionalRightImg));
    }
    else
    {
        yWarning("showTempImage: nothing to do");
        outProvisionalLeftImgPort.unprepare();
        outProvisionalRightImgPort.unprepare();
        return false;
    }

    outProvisionalLeftImgPort.write();
    outProvisionalRightImgPort.write();

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
    bool isHand = (label=="straight") ||
                  (label=="fortyfive") ||
                  (label=="bent");

    // sanity checks
    const int numDesc = 13;

    // TODO
    //Bottle *desc;
    //    *desc = handDesc;
    if (isHand)
    {
        // hand
        if (handDesc.size() != numDesc)
        {
            yError("got %d hand descriptors, was expecting %d", handDesc.size(), numDesc);
            return false;
        }

        const int numArmJoints = 16;
        if (armJoints.size() != numArmJoints)
        {
            yError("got %d arm joints, was expecting %d", armJoints.size(), numArmJoints);
            return false;
        }

        const int numHeadJoints = 6;
        if (headJoints.size() != numHeadJoints)
        {
            yError("got %d head joints, was expecting %d", headJoints.size(), numHeadJoints);
            return false;
        }

        // write data row
        csvHands << label;
        for (int d=0; d<numDesc; ++d)
            csvHands << handDesc.get(d).asDouble();

        for (int a=0; a<numArmJoints; ++a)
            csvHands << armJoints.get(a).asDouble();

        for (int h=0; h<numHeadJoints; ++h)
            csvHands << headJoints.get(h).asDouble();

        csvHands << endrow;
    }
    else
    {
        // object
        if (objDesc.size() != numDesc)
        {
            yError("got %d object descriptors, was expecting %d", objDesc.size(), numDesc);
            return false;
        }

        // write data row
        csvObjects << label;
        for (int d=0; d<numDesc; ++d)
            csvObjects << objDesc.get(d).asDouble();

        csvObjects << endrow;
    }

    yInfo("sucessfully saved descriptors of %s to file", label.c_str());

    if (isHand)
        handDesc.clear();
    else
        objDesc.clear();

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
        if (handImageSim.empty())
        {
            yError("handImageSim is empty, cannot save it");
            return false;
        }
        filename += "hand_";
    }
    else
    {
        // object
        if (leftObjImage.empty())
        {
            yError("leftObjImage is empty, cannot save it");
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

    cv::imwrite((basePath+"/"+filename).c_str(), (isHand ? handImageSim : leftObjImage));

    yInfo("sucessfully saved image of %s to file", label.c_str());

    return true;
}

/***************************************************/
bool HandAffManagerModule::computeTipFrame(const std::string &posture,
                                           const std::string &action,
                                           yarp::os::Bottle &auxpos,
                                           yarp::os::Bottle &auxori)
{
    // sanity checks

    auxpos.clear();
    auxori.clear();

    if (posture=="straight" &&
        (action=="tapFromLeft" || action=="tapFromRight" || action=="push"))
    {
        // case 1
        auxpos.addDouble(0.07);
        auxpos.addDouble(0.0);
        auxpos.addDouble(0.0);

        auxori.addDouble(1.0); // axis
        auxori.addDouble(0.0);
        auxori.addDouble(0.0);
        auxori.addDouble(0.0); // angle
    }
    else if ((posture=="straight") && (action=="draw"))
    {
        // case 2
        auxpos.addDouble(0.07);
        auxpos.addDouble(0.0);
        auxpos.addDouble(-0.03);

        auxori.addDouble(1.0);
        auxori.addDouble(0.0);
        auxori.addDouble(0.0);
        auxori.addDouble(0.0);
    }
    else if ((posture=="fortyfive" || posture=="bent") &&
             (action=="push" || action=="draw"))
    {
        // case 3
        auxpos.addDouble(0.02);
        auxpos.addDouble(0.0);
        auxpos.addDouble(-0.03); // should be positive if right_hand, negative if left_hand

        auxori.addDouble(1.0);
        auxori.addDouble(0.0);
        auxori.addDouble(0.0);
        auxori.addDouble(0.0);
    }
    else if (posture=="bent" && action=="tapFromLeft")
    {
        // case 4
        auxpos.addDouble(0.02);
        auxpos.addDouble(0.0);
        auxpos.addDouble(-0.05); // should be positive if right_hand, negative if left_hand

        auxori.addDouble(1.0);
        auxori.addDouble(0.0);
        auxori.addDouble(0.0);
        auxori.addDouble(0.0);
    }
    else if (posture=="fortyfive" && action=="tapFromLeft")
    {
        // case 5
        auxpos.addDouble(0.01);
        auxpos.addDouble(0.0);
        auxpos.addDouble(-0.03); // should be positive if right_hand, negative if left_hand

        auxori.addDouble(1.0);
        auxori.addDouble(0.0);
        auxori.addDouble(0.0);
        auxori.addDouble(0.0);
    }
    else if ((posture=="fortyfive" || posture=="bent") &&
             action=="tapFromRight")
    {
        // case 6
        auxpos.addDouble(0.0);
        auxpos.addDouble(0.0);
        auxpos.addDouble(0.04); // should be negative if right_hand, positive if left_hand

        auxori.addDouble(1.0);
        auxori.addDouble(0.0);
        auxori.addDouble(0.0);
        auxori.addDouble(0.0);
    }
    else
    {
        yError("unknown tip frame transformation");
        return false;
    }

    return true;
}

/***************************************************/
bool HandAffManagerModule::saveEffectsAndImages(const string &posture,
                                                const string &objName,
                                                const string &action)
{
    return saveEffects(posture,objName,action) &&
           saveInitEffImage(posture,objName,action) &&
           saveFinalEffImage(posture,objName,action);
}

/***************************************************/
bool HandAffManagerModule::saveEffects(const string &posture,
                                       const string &objName,
                                       const string &action)
{
    // sanity checks
    const int numEffects = 8;
    if (effects.size() != numEffects)
    {
        yError("got %d effects, was expecting %d", effects.size(), numEffects);
        return false;
    }

    // create effects file  and write header row
    csvfile csvEffects;
    string timestr = getDateAndTime();
    string filename;
    filename = basePath+"/"+posture+"_"+objName+"_"+action+"_"+timestr+".csv";
    yDebug("effects filename is %s", filename.c_str());
    csvEffects.setFilename(filename);
    csvEffects << "postureName" << "objectName" << "action"
        << "initX" << "initY" << "initu" << "initv"
        << "finalX" << "finalY" << "finalu" << "finalv"
        << endrow;

    // write data row
    csvEffects << posture << objName << action;
    for (int e=0; e<numEffects; ++e)
        csvEffects << effects.get(e).asDouble();

    csvEffects << endrow;
}

/***************************************************/
bool HandAffManagerModule::saveInitEffImage(const string &posture,
                                            const string &objName,
                                            const string &action)
{
    string filename;

    if (leftObjImage.empty())
    {
        yError("effect_init leftObjImage is empty, cannot save it");
        return false;
    }

    if (rightObjImage.empty())
    {
        yError("effect_init rightObjImage is empty, cannot save it");
        return false;
    }

    filename += posture;
    filename += "_";
    filename += objName;
    filename += "_";
    filename += action;
    filename += "_before_";
    string imageTimeStr = getDateAndTime(); // TODO do it at acquisition time instead
    filename += imageTimeStr;

    string filenameLeft = filename;
    filenameLeft += "_left_camera";
    filenameLeft += ".jpg";

    string filenameRight = filename;
    filenameRight += "_right_camera";
    filenameRight += ".jpg";

    cv::imwrite((basePath+"/"+filenameLeft).c_str(), leftObjImageInitial);
    cv::imwrite((basePath+"/"+filenameRight).c_str(), rightObjImageInitial);

    yInfo("sucessfully saved initial images %s to files", imageTimeStr.c_str());

    return true;
}

/***************************************************/
bool HandAffManagerModule::saveFinalEffImage(const string &posture,
                                             const string &objName,
                                             const string &action)
{
    string filename;

    if (leftObjImage.empty())
    {
        yError("effect_init leftObjImage is empty, cannot save it");
        return false;
    }

    if (rightObjImage.empty())
    {
        yError("effect_init rightObjImage is empty, cannot save it");
        return false;
    }

    filename += posture;
    filename += "_";
    filename += objName;
    filename += "_";
    filename += action;
    filename += "_after_";
    string imageTimeStr = getDateAndTime(); // TODO do it at acquisition time instead
    filename += imageTimeStr;

    string filenameLeft = filename;
    filenameLeft += "_left_camera";
    filenameLeft += ".jpg";

    string filenameRight = filename;
    filenameRight += "_right_camera";
    filenameRight += ".jpg";

    cv::imwrite((basePath+"/"+filenameLeft).c_str(), leftObjImageInitial);
    cv::imwrite((basePath+"/"+filenameRight).c_str(), rightObjImageInitial);

    yInfo("sucessfully saved final images %s to file", imageTimeStr.c_str());

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
string HandAffManagerModule::getHand(const string &posture)
{
    // if necessary move real and simulated hand, set currPosture variable
    if (currPosture != posture)
    {
      if (!setHandPosture(posture))
          return "problem setting the hand posture";
    }

    // acquire provisional hand descriptors
    if (!getHandDesc())
        return "failed acquiring hand descriptors";

    // acquire provisional hand image
    if (!getHandImage())
        return "failed acquiring hand image";

    showTempImage("hand");

    // acquire arm and head joints configuration used in Unity for the desc
    if (!getSimArmHead())
        return "failed acquiring arm and head joints from the simulator";

    // acquisition OK, now we need to ask the user for confirmation
    needUserConfirmation = true;

    //return "successfully acquired hand descriptors and image: if they look OK please type 'yes', otherwise type 'no'";
    stringstream sstm;
    sstm << "successfully acquired provisional hand image, kinematics and descriptors: "
         << handDesc.toString().c_str()
         << " if they look OK please type 'yes', otherwise type 'no'";
    // TODO confirmation images

    return sstm.str();
}

/***************************************************/
string HandAffManagerModule::getObject(const string &objName)
{
    if (!setObjectName(objName))
        return "problem setting the object name";

    // acquire provisional object descriptors
    if (!getObjDesc())
        return "failed acquiring object descriptors";

    // acquire provisional object image
    if (!getObjImages())
        return "failed acquiring object image";

    showTempImage("object");

    // acquisition OK, now we need to ask the user for confirmation
    needUserConfirmation = true;

    stringstream sstm;
    sstm << "successfully acquired provisional object image and descriptors: "
         << objDesc.toString().c_str()
         << " if they look OK please type 'yes', otherwise type 'no'";
    // TODO confirmation images

    return sstm.str();
}

/***************************************************/
string HandAffManagerModule::startEffect(const string &action, const string &posture, const string &objName)
{
    // sanity checks
    if (action!="tapFromLeft" && action!="tapFromRight" && action!="push" && action!="draw")
    {
        return "invalid action! The valid ones are: tapFromLeft, tapFromRight, push, draw";
    }

    if (posture!="straight" && posture!="fortyfive" && posture!="bent")
    {
        return "invalid posture! The valid ones are: straight, fortyfive, bent";
    }

    if (rpcHandActionsPort.getOutputCount()<1)
    {
        return "no connection to handActions RPC server";
    }

    currAction = action; // save it for later
    currPosture = posture;
    currObj = objName;

    // target object initial position information
    init2D.clear();
    init2D = getBestObject2D();
    if (init2D.size() != 2)
    {
        return "problem with init2D";
    }

    init3D.clear();
    init3D = getBestObject3D();
    if (init3D.size() != 3)
    {
        return "problem with init3D";
    }

    // acquire provisional object image for effect_init visualization
    if (!getObjImages())
        return "failed acquiring effect_init object image(s)";

    // save images for later, because we will overwrite them at final instant
    leftObjImageInitial = leftObjImage.clone();
    rightObjImageInitial = rightObjImage.clone();

    showTempImage("effect_init");

    // adjust the end effector depending on hand posture and action
    Bottle auxpos;
    auxpos.clear();
    Bottle auxori;
    auxori.clear();
    if (!computeTipFrame(posture, action, auxpos, auxori))
        yError("problem with computeTipFrame");
    Bottle handActionsCmd;
    Bottle handActionsReply;
    handActionsCmd.clear();
    handActionsReply.clear();
    handActionsCmd.addString("attachTip");
    handActionsCmd.addList() = auxpos;
    handActionsCmd.addList() = auxori;
    yDebug("requesting tip frame change pos: %s ori: %s", auxpos.toString().c_str(), auxori.toString().c_str());
    rpcHandActionsPort.write(handActionsCmd, handActionsReply);
    if (handActionsReply.size()>0 &&
        handActionsReply.get(0).asVocab()==Vocab::encode("ok"))
    {
        yInfo("successfully changed tip frame");
    }
    else
    {
        yError("problem when changing tip frame");
        return "could not change tip frame";
    }

    // do the motor action
    yWarning("requesting %s motor action on the real robot", action.c_str());
    yarp::os::Time::delay(1.0);
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

    stringstream sstm;
    sstm << "started acquiring the effects of " <<
         action << " " << posture << " " << objName <<
         " - when the final position is reached, type 'stopEffect'";

    return sstm.str();
}

/***************************************************/
string HandAffManagerModule::stopEffect()
{
    yDebug("stopping data acquisition of %s %s %s",
           currAction.c_str(), currPosture.c_str(), currObj.c_str());

    // acquire provisional object image for effect_final visualization
    if (!getObjImages())
        return "failed acquiring effect_final object image";

    // target object final position information
    final2D.clear();
    final2D = getBestObject2D();
    if (final2D.size() != 2)
    {
       return "problem with final2D";
    }
    final3D.clear();
    final3D = getBestObject3D();
    if (final3D.size() != 3)
    {
       return "problem with final3D";
    }

    showTempImage("effect_final");

    // effect computation and request for confirmation
    effects.clear();
    effects.addDouble(init3D.get(0).asDouble()); // X robot frame
    effects.addDouble(init3D.get(1).asDouble()); // Y robot frame
    effects.addDouble(init2D.get(0).asDouble()); // u image frame
    effects.addDouble(init2D.get(1).asDouble()); // v image frame
    effects.addDouble(final3D.get(0).asDouble()); // X robot frame
    effects.addDouble(final3D.get(1).asDouble()); // etc.
    effects.addDouble(final2D.get(0).asDouble());
    effects.addDouble(final2D.get(1).asDouble());

    needUserConfirmation = true;

    double effX = final3D.get(0).asDouble() - init3D.get(0).asDouble();
    double effY = final3D.get(1).asDouble() - init3D.get(1).asDouble();
    stringstream sstm;
    sstm << "successfully computed the effects " <<
         "X: " << effX << ", Y:" << effY <<
         " if they look OK please type 'yes', otherwise type 'no'";

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
