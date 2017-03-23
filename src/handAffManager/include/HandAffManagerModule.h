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

#ifndef HAND_AFFORDANCE_MANAGER_MODULE_H
#define HAND_AFFORDANCE_MANAGER_MODULE_H

#include <ctime>
#include <sstream>
#include <string>

#include <boost/filesystem.hpp>

#include <opencv2/opencv.hpp>
#include "OpenCVHelpers.h"

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Image.h>

#include "CSVFile.h"
#include "handAffManager_IDL.h"

/***************************************************/
class HandAffManagerModule : public yarp::os::RFModule,
                             public handAffManager_IDL
{
private:

    //bool closing;
    yarp::os::RpcServer rpcPort;

    std::string inHandImgPortName;
    std::string inHandDescPortName;
    std::string inObjImgPortName;
    std::string inObjDescPortName;
    std::string rpcHandActionsPortName;
    std::string rpcRobotHandProcessorPortName;
    std::string rpcGazePortName;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > inHandImgPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > inObjImgPort;
    yarp::os::BufferedPort<yarp::os::Bottle> inHandDescPort;
    yarp::os::BufferedPort<yarp::os::Bottle> inObjDescPort;
    yarp::os::RpcClient rpcHandActionsPort;
    yarp::os::RpcClient rpcRobotHandProcessorPort;
    yarp::os::RpcClient rpcGazePort;

    bool initMotor;
    bool initSim;

    bool needUserConfirmation;
    bool userResponse;

    // hands and objects
    csvfile csvHandsObjects;
    yarp::os::Bottle handDesc;
    yarp::os::Bottle objDesc;
    cv::Mat handImageSim;
    cv::Mat objImage;
    //std::string handImageTimeStr;

    std::string basePath;

    std::string currPosture;
    std::string currObj;
    std::string currAction;

    yarp::os::Bottle effects;

public:

    bool configure(yarp::os::ResourceFinder &rf);
    bool interruptModule();
    bool close();
    double getPeriod();
    bool updateModule();

    bool setHandPosture(const std::string &posture);
    bool getHandDesc();
    bool getHandImage();

    bool setObjectName(const std::string &objName);
    bool getObjDesc();
    bool getObjImage();

    bool saveDescAndImage(const std::string &label);
    bool saveDesc(const std::string &label);
    bool saveImage(const std::string &label);

    // helper functions, TODO put them in an external file
    std::string getDateAndTime();

    // IDL functions
    bool attach(yarp::os::RpcServer &source);
    std::string getHand(const std::string &posture);
    std::string getObject(const std::string &objName);
    std::string getEffect(const std::string &action);
    bool yes();
    bool no();
    yarp::os::Bottle getBestObject2D();
    yarp::os::Bottle getBestObject3D();
    bool quit();
};

#endif // HAND_AFFORDANCE_MANAGER_MODULE_H
