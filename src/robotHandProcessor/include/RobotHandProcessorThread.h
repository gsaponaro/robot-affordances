/*
 * Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>,
 * CopyPolicy: Released under the terms of the GNU GPL v3.0
 *
 */

#ifndef ROBOT_HAND_PROCESSOR_THREAD_H
#define ROBOT_HAND_PROCESSOR_THREAD_H

#include <iomanip>
#include <sstream>
#include <string>

#include <opencv2/opencv.hpp>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Log.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>

/***************************************************/
class RobotHandProcessorThread : public yarp::os::RateThread
{
private:

    std::string moduleName;
    yarp::os::ResourceFinder rf;
    bool closing;

    int numArmJoints;
    int numHeadJoints;
    yarp::sig::Vector armJoints;
    yarp::sig::Vector headJoints;
    bool armHasChanged; // at least one joint changed w.r.t. initial defaults
    double timeSinceArmUpdate;

    std::string inImgPortName;
    std::string inArmJointsPortName;
    std::string inHeadJointsPortName;
    std::string outImgPortName;
    std::string outArmJointsPortName;
    std::string outHeadJointsPortName;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > inImgPort;
    yarp::os::BufferedPort<yarp::os::Bottle> inArmJointsPort;
    yarp::os::BufferedPort<yarp::os::Bottle> inHeadJointsPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > outImgPort;
    yarp::os::BufferedPort<yarp::os::Bottle> outArmJointsPort;
    yarp::os::BufferedPort<yarp::os::Bottle> outHeadJointsPort;

public:

    RobotHandProcessorThread(const std::string &_moduleName,
                             yarp::os::ResourceFinder &_rf);
    bool threadInit();
    void run();
    void interrupt();
    void close();

    bool openPorts();
    void mainProcessing();

    // IDL functions
    bool look(const std::string &target);
    bool resetKinematics();
    double getPos(int32_t joint);
    bool setPos(int32_t joint, double value);
};

#endif // ROBOT_HAND_PROCESSOR_THREAD_H
