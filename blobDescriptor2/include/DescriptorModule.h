/*
 * Copyright: (C) 2012-2015 POETICON++, European Commission FP7 project ICT-288382
 * Copyright: (C) 2015 VisLab, Institute for Systems and Robotics,
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

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

class BlobDescriptorModule : public RFModule
{
    private:
        // module parameters
        string moduleName;
        bool closing;

        // pointer to a new thread
        BlobDescriptorThread *thread;

        // thread parameters
        double threadPeriod;
        int maxObjects;
        string mode;
        int minArea, maxArea;
        //bool synch;

    public:
        virtual bool configure(ResourceFinder &rf);
        virtual bool interruptModule();
        virtual bool close();

        virtual bool quit();

        virtual bool updateModule();
        virtual double getPeriod();
};

#endif
