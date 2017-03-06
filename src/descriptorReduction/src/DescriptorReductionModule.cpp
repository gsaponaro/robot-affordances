/*
 * Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */

#include "DescriptorReductionDefaults.h"
#include "DescriptorReductionModule.h"

using namespace yarp::os;

bool DescriptorReductionModule::configure(ResourceFinder &rf)
{
    moduleName = rf.check("name", Value(DefModuleName)).asString();
    setName(moduleName.c_str());

    inWholeDescPortName = "/" + moduleName + "/wholeDescriptors:i";
    inWholeDescPort.open(inWholeDescPortName.c_str());

    inPartDescPortName = "/" + moduleName + "/partDescriptors:i";
    inPartDescPort.open(inPartDescPortName.c_str());

    outReducedDescPortName = "/" + moduleName + "/desc:o";
    outReducedDescPort.open(outReducedDescPortName.c_str());

    yInfo("ready, waiting for descriptors");

    return true;
}

bool DescriptorReductionModule::interruptModule()
{
    inWholeDescPort.interrupt();
    inPartDescPort.interrupt();

    return true;
}

bool DescriptorReductionModule::close()
{
    inWholeDescPort.close();
    inPartDescPort.close();

    return true;
}

bool DescriptorReductionModule::quit()
{
    return true;
}

bool DescriptorReductionModule::updateModule()
{
    if (inWholeDescPort.getInputCount()>0 && inPartDescPort.getInputCount()>0)
    {
        yDebug("both inputs connected");
        Bottle *inWholeDesc = inWholeDescPort.read(true);
        Bottle *inPartDesc = inWholeDescPort.read(true);
        if (inWholeDesc!=NULL && inPartDesc!=NULL)
        {
            const int numBlobs = inWholeDesc->size();
            const int desiredBlobs = 1;
            if (numBlobs > desiredBlobs)
            {
                yWarning("got more than %d blobs, will not do anything", inWholeDesc->size());
                return true;
            }

            
        }
    }

    return true;
}

double DescriptorReductionModule::getPeriod()
{
    return 0.0;
}
