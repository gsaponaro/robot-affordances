/*
 * Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0
 *
 */

#ifndef DESC_MODULE_H
#define DESC_MODULE_H

#include <yarp/os/Log.h>
#include <yarp/os/RFModule.h>

#include "DescriptorThread.h"

class ShapeDescriptorModule : public yarp::os::RFModule
{
    private:
        // module parameters
        std::string moduleName;
        bool closing;

        // pointer to a new thread
        ShapeDescriptorThread *thread;

    public:
        virtual bool configure(yarp::os::ResourceFinder &rf);
        virtual bool interruptModule();
        virtual bool close();

        virtual bool quit();

        virtual bool updateModule();
        virtual double getPeriod();
};

#endif
