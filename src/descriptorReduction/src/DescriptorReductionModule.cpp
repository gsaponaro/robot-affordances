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
        if (inWholeDesc!=NULL && inPartDesc!=NULL && inWholeDesc->size()>0 && inPartDesc->size()>0)
        {
            // check that whole and part ports refer to same number of blobs
            const int numWholeBlobs = inWholeDesc->size();
            const int numPartBlobs = inPartDesc->size();
            if (numWholeBlobs != numPartBlobs)
            {
                yWarning("numWholeBlobs==%d != numPartBlobs==%d", numWholeBlobs, numPartBlobs);
                return true;
            }

            // criteria for best blob selection:
            // (i) sufficiently large area
            // (ii) closest to robot (highest image y)

            // we provisionally consider the first blob (0) as the best one
            int idx = 0;
            Property pWholeFirst;
            pWholeFirst.fromString(inWholeDesc->get(idx).asList()->toString().c_str());
            // however, we will still confirm that its area is within the
            // required range (criterion i)
            const double minAreaForSelection = 50.0;
            const int expectedAreaBottleSize = 1;
            if (numWholeBlobs > 1)
            {
                // initially index and maxY are those of the first blob
                const int expectedComBottleSize = 2;
                if (!verifyProperty(pWholeFirst,"center",expectedComBottleSize))
                {
                    yWarning("problem reading center coordinates of first blob");
                    return true;
                }
                double maxY = pWholeFirst.find("center").asList()->get(1).asDouble();

                // compare with other blobs in list, update if criteria met
                for (int o=1; o<numWholeBlobs; ++o)
                {
                    Property pWholeCurr;
                    pWholeCurr.fromString(inWholeDesc->get(o).asList()->toString().c_str());
                    // criterion (i): sufficiently large area
                    if (!verifyProperty(pWholeCurr,"area",expectedAreaBottleSize))
                    {
                        yWarning("problem reading area of blob %d", o);
                        continue; // next o
                    }
                    if (pWholeCurr.find("area").asList()->get(0).asDouble()<minAreaForSelection)
                    {
                        yWarning("blob %d area is only %f -> will not select it",
                                 o, pWholeCurr.find("area").asList()->get(0).asDouble());
                        continue; // next o
                    }
                    // criterion (ii): closest to robot (highest image y)
                    if (!verifyProperty(pWholeCurr,"center",expectedComBottleSize))
                    {
                        yWarning("problem reading center coordinates of blob %d", o);
                        continue; // next o
                    }
                    const double currY = pWholeCurr.find("center").asList()->get(1).asDouble();
                    if (currY > maxY)
                    {
                        idx = o;
                        maxY = currY;
                    }
                }
            }
            else
            {
                // only 1 blob
                // we just need to check criterion (ii): sufficiently large area
                if (!verifyProperty(pWholeFirst,"area",expectedAreaBottleSize))
                {
                    yWarning("problem reading area of blob");
                    return true; // skip
                }
                if (pWholeFirst.find("area").asList()->get(0).asDouble()<minAreaForSelection)
                {
                    yWarning("blob's area is only %f -> will not select it",
                             pWholeFirst.find("area").asList()->get(0).asDouble());
                    return true; // skip
                }
            }

            // now idx is most likely a safe blob index; just a further sanity
            // check on partDesc
            const int expectedNumberOfParts = 2;
            if (inPartDesc->get(idx).asList()->size() != expectedNumberOfParts)
            {
                yError("partDescriptors problem, size of list is not %d", expectedNumberOfParts);
                return true; // skip
            }

            // now idx is the selected best blob index; we construct the
            // relevant Property objects (of whole, top, bottom) for further
            // usage, i.e., robot control towards the object associated to
            // the selected blob
            Property pWhole;
            pWhole.fromString(inWholeDesc->get(idx).asList()->toString().c_str());

            /*
            yDebug("selected blob %d: center (%f %f), area %f",
                   idx,
                   pWhole.find("center").asList()->get(0).asDouble(),
                   pWhole.find("center").asList()->get(1).asDouble(),
                   pWhole.find("area").asList()->get(0).asDouble());
            */

            Property pTop;
            pTop.fromString(inPartDesc->get(idx).asList()->get(0).asList()->toString().c_str());

            Property pBottom;
            pBottom.fromString(inPartDesc->get(idx).asList()->get(1).asList()->toString().c_str());

            Bottle &r = outReducedDescPort.prepare();
            // we will output 2 + 3*13 numbers, as follows
            r.clear();
            // centroid of whole blob: 0 - 1
            r.addDouble(pWhole.find("center").asList()->get(0).asDouble());
            r.addDouble(pWhole.find("center").asList()->get(1).asDouble());
            // whole descriptors: 2 - 14
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
            // top part descriptors: 15 - 27
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
            // bottom part descriptors: 28 - 40
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

// TODO: move to external helper file
/**
  * Check if a shape descriptor is present and fulfills the specified
  * constraints.
  *
  * In particular:
  *
  * Return true iff the list of shape descriptors p contains a descriptor
  * called k whose corresponding value is a list of length vSize.
  */
bool DescriptorReductionModule::verifyProperty(const yarp::os::Property &p,
                                               const std::string &k,
                                               const int vSize)
{
    return p.check(k.c_str()) && p.find(k.c_str()).asList()->size()==vSize;
}
