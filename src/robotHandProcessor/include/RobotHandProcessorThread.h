/*
 * Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>,
 * CopyPolicy: Released under the terms of the GNU GPL v3.0
 *
 */

#ifndef ROBOT_HAND_PROCESSOR_THREAD_H
#define ROBOT_HAND_PROCESSOR_THREAD_H

#include <string>

#include <opencv2/opencv.hpp>

#include <yarp/os/RateThread.h>
#include <yarp/os/ResourceFinder.h>

/***************************************************/
class RobotHandProcessorThread : public yarp::os::RateThread
{
private:

    std::string moduleName;
    yarp::os::ResourceFinder rf;
    bool closing;

public:

    RobotHandProcessorThread(const std::string &_moduleName,
                             yarp::os::ResourceFinder &_rf);
    bool threadInit();
    void run();
    void interrupt();
    void close();

    void mainProcessing();

    // IDL functions
};

#endif // ROBOT_HAND_PROCESSOR_THREAD_H
