/*
 * Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0
 *
 */

#ifndef DESCRIPTOR_REDUCTION_MODULE_H
#define DESCRIPTOR_REDUCTION_MODULE_H

#include <string>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Log.h>
#include <yarp/os/Property.h>
#include <yarp/os/RFModule.h>

class DescriptorReductionModule : public yarp::os::RFModule
{
    private:
        std::string moduleName;
        std::string inWholeDescPortName;
        std::string inPartDescPortName;
        std::string outReducedDescPortName;
        yarp::os::BufferedPort<yarp::os::Bottle> inWholeDescPort;
        yarp::os::BufferedPort<yarp::os::Bottle> inPartDescPort;
        yarp::os::BufferedPort<yarp::os::Bottle> outReducedDescPort;

    public:
        virtual bool configure(yarp::os::ResourceFinder &rf);
        virtual bool interruptModule();
        virtual bool close();

        virtual bool quit();

        virtual bool updateModule();
        virtual double getPeriod();

        bool verifyProperty(const yarp::os::Property &p, const std::string &k, const int vSize);
};

#endif
