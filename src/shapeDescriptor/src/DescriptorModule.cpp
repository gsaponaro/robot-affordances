/*
 * Copyright: (C) 2016 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */

#include "DescriptorDefaults.h"
#include "DescriptorModule.h"

using namespace yarp::os;

bool ShapeDescriptorModule::configure(ResourceFinder &rf)
{
    moduleName = rf.check("name", Value(DefModuleName)).asString();
    setName(moduleName.c_str());

    closing = false;

    // create the thread, pass parameters with ResourceFinder
    thread = new ShapeDescriptorThread(moduleName, rf);

    // start the thread to do the work
    if (!thread->start()) {
        delete thread;
        return false;
    }

    return true;
}

bool ShapeDescriptorModule::interruptModule()
{
    // interrupt rpc ports, if any

    return true;
}

bool ShapeDescriptorModule::close()
{
    // close rpc ports, if any

    yInfo("starting the shutdown procedure");
    thread->interrupt();
    thread->close();
    thread->stop();
    yInfo("deleting thread");
    if (thread) delete thread;
    yInfo("done deleting thread");

    return true;
}

bool ShapeDescriptorModule::quit()
{
    closing = true;
    return true;
}

bool ShapeDescriptorModule::updateModule()
{
    return !closing;
}

double ShapeDescriptorModule::getPeriod()
{
    return 0.0;
}
