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
        Bottle *inWholeDesc = inWholeDescPort.read(true);
        Bottle *inPartDesc = inPartDescPort.read(true);
        if (inWholeDesc!=NULL && inPartDesc!=NULL)
        {
            const int numBlobs = inWholeDesc->size();
            const int desiredBlobs = 1;
            if (numBlobs > desiredBlobs)
            {
                yWarning("got more than %d blobs, will not do anything", inWholeDesc->size());
                return true;
            }

            // further sanity checks
            bool ok = true;
            ok = ok &&
                 inPartDesc->size()==1 &&
                 inPartDesc->get(0).asList()->size()==2;
            if (!ok)
            {
                yError("partDescriptors problem");
                return true;
            }

            // create Property objects for parsing
            yarp::os::Property pWhole;
            //yDebug("inWholeDesc->get(0).asList()->toString().c_str(): %s", inWholeDesc->get(0).asList()->toString().c_str());
            pWhole.fromString(inWholeDesc->get(0).asList()->toString().c_str());
            yarp::os::Property pTop;
            //yDebug("inPartDesc top: %s", inPartDesc->get(0).asList()->get(0).asList()->toString().c_str());
            pTop.fromString(inPartDesc->get(0).asList()->get(0).asList()->toString().c_str());
            yarp::os::Property pBottom;
            //yDebug("inPartDesc bottom: %s", inPartDesc->get(0).asList()->get(1).asList()->toString().c_str());
            pBottom.fromString(inPartDesc->get(0).asList()->get(1).asList()->toString().c_str());

            Bottle &r = outReducedDescPort.prepare();
            // we will output 3 * 13 = 39 numbers, as follows
            r.clear();
            // whole: 0 - 12
            r.addDouble(pWhole.find("convexity").asList()->get(0).asDouble());
            r.addDouble(pWhole.find("eccentricity").asList()->get(0).asDouble());
            r.addDouble(pWhole.find("compactness").asList()->get(0).asDouble());
            r.addDouble(pWhole.find("circularity").asList()->get(0).asDouble());
            r.addDouble(pWhole.find("squareness").asList()->get(0).asDouble());
            r.addDouble(pWhole.find("convexityDefects").asList()->get(0).asDouble());
            r.addDouble(pWhole.find("centralNormalizedMoments").asList()->get(0).asDouble());
            r.addDouble(pWhole.find("centralNormalizedMoments").asList()->get(1).asDouble());
            r.addDouble(pWhole.find("centralNormalizedMoments").asList()->get(2).asDouble());
            r.addDouble(pWhole.find("centralNormalizedMoments").asList()->get(3).asDouble());
            r.addDouble(pWhole.find("centralNormalizedMoments").asList()->get(4).asDouble());
            r.addDouble(pWhole.find("centralNormalizedMoments").asList()->get(5).asDouble());
            r.addDouble(pWhole.find("centralNormalizedMoments").asList()->get(6).asDouble());
            // top: 13 - 25
            r.addDouble(pTop.find("convexity").asList()->get(0).asDouble());
            r.addDouble(pTop.find("eccentricity").asList()->get(0).asDouble());
            r.addDouble(pTop.find("compactness").asList()->get(0).asDouble());
            r.addDouble(pTop.find("circularity").asList()->get(0).asDouble());
            r.addDouble(pTop.find("squareness").asList()->get(0).asDouble());
            r.addDouble(pTop.find("convexityDefects").asList()->get(0).asDouble());
            r.addDouble(pTop.find("centralNormalizedMoments").asList()->get(0).asDouble());
            r.addDouble(pTop.find("centralNormalizedMoments").asList()->get(1).asDouble());
            r.addDouble(pTop.find("centralNormalizedMoments").asList()->get(2).asDouble());
            r.addDouble(pTop.find("centralNormalizedMoments").asList()->get(3).asDouble());
            r.addDouble(pTop.find("centralNormalizedMoments").asList()->get(4).asDouble());
            r.addDouble(pTop.find("centralNormalizedMoments").asList()->get(5).asDouble());
            r.addDouble(pTop.find("centralNormalizedMoments").asList()->get(6).asDouble());
            // bottom: 26 - 38
            r.addDouble(pBottom.find("convexity").asList()->get(0).asDouble());
            r.addDouble(pBottom.find("eccentricity").asList()->get(0).asDouble());
            r.addDouble(pBottom.find("compactness").asList()->get(0).asDouble());
            r.addDouble(pBottom.find("circularity").asList()->get(0).asDouble());
            r.addDouble(pBottom.find("squareness").asList()->get(0).asDouble());
            r.addDouble(pBottom.find("convexityDefects").asList()->get(0).asDouble());
            r.addDouble(pBottom.find("centralNormalizedMoments").asList()->get(0).asDouble());
            r.addDouble(pBottom.find("centralNormalizedMoments").asList()->get(1).asDouble());
            r.addDouble(pBottom.find("centralNormalizedMoments").asList()->get(2).asDouble());
            r.addDouble(pBottom.find("centralNormalizedMoments").asList()->get(3).asDouble());
            r.addDouble(pBottom.find("centralNormalizedMoments").asList()->get(4).asDouble());
            r.addDouble(pBottom.find("centralNormalizedMoments").asList()->get(5).asDouble());
            r.addDouble(pBottom.find("centralNormalizedMoments").asList()->get(6).asDouble());
            outReducedDescPort.write();
        }
    }

    return true;
}

double DescriptorReductionModule::getPeriod()
{
    return 0.0;
}
