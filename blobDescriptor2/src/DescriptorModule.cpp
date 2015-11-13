/*
 * Copyright: (C) 2012-2015 POETICON++, European Commission FP7 project ICT-288382
 * Copyright: (C) 2015 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */

#include "DescriptorModule.h"

bool BlobDescriptorModule::configure(ResourceFinder &rf)
{
    // parse basic options
    moduleName = rf.check("name", Value("blobDescriptor"), "module name (string)").asString();
    setName(moduleName.c_str());

    threadPeriod = rf.check("threadPeriod", Value(0.033),
        "thread period in seconds (double)").asDouble();

    maxObjects = rf.check("maxObjects", Value(10),
        "maximum number of objects to process (int)").asInt();

    mode = rf.check("mode", Value("2d"), "2d or 3d (string)").asString();
    std::transform(mode.begin(), mode.end(), mode.begin(), ::tolower);
    if (mode!="2d" && mode!="3d")
        mode = "2d";
        
    // parse advanced options (2d mode)
    minArea = rf.check("minArea", Value(100),
        "minimum valid blob area (int)").asInt();

    maxArea = rf.check("maxArea", Value(20000),
        "maximum valid blob area (int)").asInt();

    // parse advanced options (3d mode)
    // tbc

    // end of options parsing

    closing = false;

    // create the thread and pass pointers to the module parameters
    thread = new BlobDescriptorThread(moduleName, threadPeriod, maxObjects,
                                      mode,
                                      minArea, maxArea);

    // start the thread to do the work
    if (!thread->start()) {
        delete thread;
        return false;
    }

    return true;
}

bool BlobDescriptorModule::interruptModule()
{
    // interrupt rpc ports, if any

    return true;
}

bool BlobDescriptorModule::close()
{
    // close rpc ports, if any

    yInfo("starting the shutdown procedure");
    thread->interrupt();
    thread->close();
    yInfo("deleting thread");
    delete thread;
    yInfo("done deleting thread");

    return true;
}

bool BlobDescriptorModule::quit()
{
    closing = true;
    return true;
}

bool BlobDescriptorModule::updateModule()
{
    return !closing;
}

double BlobDescriptorModule::getPeriod()
{
    return 0.0;
}
