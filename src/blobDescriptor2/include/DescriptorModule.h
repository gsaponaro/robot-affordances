/*
 * Copyright: (C) 2016 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */

#ifndef DESC_MODULE_H
#define DESC_MODULE_H

#include <yarp/os/Log.h>
#include <yarp/os/RFModule.h>

#include "DescriptorThread.h"

class BlobDescriptorModule : public yarp::os::RFModule
{
    private:
        // module parameters
        std::string moduleName;
        bool closing;

        // pointer to a new thread
        BlobDescriptorThread *thread;

        // thread parameters
        double threadPeriod;
        int maxObjects;
        std::string mode;
        int minArea, maxArea;
        //bool synch;

    public:
        virtual bool configure(yarp::os::ResourceFinder &rf);
        virtual bool interruptModule();
        virtual bool close();

        virtual bool quit();

        virtual bool updateModule();
        virtual double getPeriod();
};

#endif
