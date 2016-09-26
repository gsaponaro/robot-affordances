/*
 * Copyright: (C) 2016 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */

#include "DescriptorThread.h"
#include "Obj2D.h"

#define THREAD_PERIOD 33 // [ms]

using namespace cv;
using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

ShapeDescriptorThread::ShapeDescriptorThread(const string &_moduleName,
    ResourceFinder &_rf)
    : RateThread(THREAD_PERIOD),
      moduleName(_moduleName),
      rf(_rf)
{
}

bool ShapeDescriptorThread::openPorts()
{
    inRawImgPortName = "/" + moduleName + "/rawImg:i";
    inRawImgPort.open(inRawImgPortName.c_str());

    inBinImgPortName = "/" + moduleName + "/binImg:i";
    inBinImgPort.open(inBinImgPortName.c_str());

    inLabImgPortName = "/" + moduleName + "/labeledImg:i";
    inLabImgPort.open(inLabImgPortName.c_str());

    //inRoiPortName = "/" + moduleName + "/blobs:i";
    //inRoiPort.open(inRoiPortName.c_str());

    //outRawImgPortName = "/" + moduleName + "/rawImg:o";
    //outRawImgPort.open(outRawImgPortName.c_str());

    outAnnotatedImgPortName = "/" + moduleName + "/annotatedImg:o";
    outAnnotatedImgPort.open(outAnnotatedImgPortName.c_str());

    outWholeDescPortName = "/" + moduleName + "/wholeDescriptors:o";
    outWholeDescPort.open(outWholeDescPortName.c_str());

    // tbc
    //outPartDescPortName = "/" + moduleName + "/partDescriptors:o";
    //outPartDescPort.open(outPartDescPortName.c_str());

    //outBothPartsImgPortName = "/" + moduleName + "/bothPartsImg:o";
    //outBothPartsImgPort.open(outBothPartsImgPortName.c_str());

    return true;
}

void ShapeDescriptorThread::close()
{
    yInfo("closing ports");

    // critical section
    mutex.wait();

    inRawImgPort.close();
    inBinImgPort.close();
    inLabImgPort.close();

    //outRawImgPort.writeStrict();
    //outRawImgPort.close();
    outAnnotatedImgPort.close();
    outWholeDescPort.writeStrict();
    outWholeDescPort.close();

    /*
    inRoiPort.close();
    // for debug
    outBothPartsImgPort.close();
    */

    mutex.post();
}

void ShapeDescriptorThread::interrupt()
{
    yInfo("interrupting ports");

    closing = true;

    inRawImgPort.interrupt();
    inBinImgPort.interrupt();
    inLabImgPort.interrupt();

    //outRawImgPort.interrupt();
    outAnnotatedImgPort.interrupt();
    outWholeDescPort.interrupt();

    /*
    inRoiPort.interrupt();
    outPartDescPort.interrupt();
    // for debug
    outBothPartsImgPort.interrupt();
    */
}

bool ShapeDescriptorThread::threadInit()
{
    maxObjects = rf.check("maxObjects", Value(10)).asInt();
    minArea = rf.check("minArea", Value(100)).asInt();
    maxArea = rf.check("maxArea", Value(3000)).asInt();

    yInfo("initialized thread with maxObjects=%d minArea=%d maxArea=%d",
          maxObjects, minArea, maxArea);

    useArea = rf.check("area",Value("on")).asString()=="on"?true:false;
    useConvexity = rf.check("convexity",Value("on")).asString()=="on"?true:false;
    useEccentricity = rf.check("eccentricity",Value("on")).asString()=="on"?true:false;
    useCompactness = rf.check("compactness",Value("on")).asString()=="on"?true:false;
    useCircleness = rf.check("circleness",Value("on")).asString()=="on"?true:false;
    useSquareness = rf.check("squareness",Value("on")).asString()=="on"?true:false;
    usePerimeter = rf.check("perimeter",Value("on")).asString()=="on"?true:false;
    useElongation = rf.check("elongation",Value("on")).asString()=="on"?true:false;
    useSpatialMoments = rf.check("spatialMoments",Value("on")).asString()=="on"?true:false;
    useCentralMoments = rf.check("centralMoments",Value("off")).asString()=="on"?true:false;
    useCentralNormalizedMoments = rf.check("centralNormalizedMoments",Value("off")).asString()=="on"?true:false;
    useBoundingRectangle = rf.check("boundingRectangle",Value("off")).asString()=="on"?true:false;
    useEnclosingRectangle = rf.check("enclosingRectangle",Value("off")).asString()=="on"?true:false;
    useColorHistogram = rf.check("colorHistogram",Value("off")).asString()=="on"?true:false;

    if( !openPorts() )
    {
        yError("problem opening ports");
    };

    closing = false;

    return true;
}

void ShapeDescriptorThread::run()
{
    while(!closing)
    {
        run2d();
        yarp::os::Time::delay(0.01);
    }
}

void ShapeDescriptorThread::run2d()
{
    // acquire new input images
    ImageOf<PixelBgr> *inRawImg = inRawImgPort.read(true);
    ImageOf<PixelBgr> *inBinImg = inBinImgPort.read(true);
    ImageOf<PixelInt> *inLabImg = inLabImgPort.read(true);

    // read timestamps
    Stamp tsRaw, tsBin, tsLab;
    if ( (inRawImgPort.getInputCount() && !inRawImgPort.getEnvelope(tsRaw)) ||
         (inBinImgPort.getInputCount() && !inBinImgPort.getEnvelope(tsBin)) ||
         (inLabImgPort.getInputCount() && !inLabImgPort.getEnvelope(tsBin)) )
    {
        yWarning("timestamp(s) missing");
    }

    if (inRawImg!=NULL && inBinImg!=NULL && inLabImg!=NULL)
    {
        // check dimensions of input images to be equal
        const int refWidth  = inRawImg->width(); 
        const int refHeight = inRawImg->height();
        if ( refWidth!=inBinImg->width() || refHeight!=inBinImg->height() ||
             refWidth!=inLabImg->width() || refHeight!=inLabImg->height() )
        {
            yWarning("image dimensions mismatch");
        }

        // initialize object instances using labelled image
        // (alternative: use bounding boxes ROIs from segmentation)
        Mat inLab = iplToMat(*inLabImg);

        // unique labels excluding 0 (background)
        std::vector<int> uniq = unique(inLab,true);
        uniq.erase( std::remove( uniq.begin(),uniq.end(),0 ), uniq.end() );
        int numObjects = uniq.size();

        // container of binary masks that are 1 where inLab==labelValue, 0 elsewhere
        std::vector<Mat> binMask(numObjects);

        // container of contours: i'th object associated to vector<vector<Point> >
        std::vector< std::vector<std::vector<Point> > > cont(numObjects);

        // container to store temporary objects/blobs with associated features
        std::vector<Obj2D> objs;

        for (std::vector<int>::iterator it=uniq.begin(); it!=uniq.end(); ++it)
        {
            // *it is the current label value: firstLabel, secondLabel, ...
            // intIdx is an auxiliary current index: 0, 1, ...
            int intIdx = std::distance(uniq.begin(),it);

            inLab.convertTo(inLab, CV_8UC1);
            binaryMaskFromLabel(inLab, *it, binMask[intIdx]);

            // extract contour of current object - requires 8UC1 or 32SC1
            findContours(binMask[intIdx], cont[intIdx], CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
            int largest;
            if (cont[intIdx].size() == 1)
                largest = 0;
            else
            {
                yDebug("findContours returned more than one contour! selecting the largest one");
                largestContour(cont[intIdx], largest);
                yDebug() << "largest contour index" << largest << "out of" << cont[intIdx].size();
            }

            double largestArea = contourArea(cont[intIdx][largest]);
            bool isValid = (largestArea>minArea && largestArea<maxArea);

            // construct temporary Obj2D with validity,contour,area
            objs.push_back( Obj2D(isValid, cont[intIdx][largest], largestArea) );
            // compute remaining shape descriptors and colour histogram
            objs[intIdx].computeDescriptors();
            Mat inRaw = iplToMat(*inRawImg);
            objs[intIdx].setMask( inRaw(objs[intIdx].getBoundingRect()) );
            if (!objs[intIdx].computeHueHistogram())
                yWarning("error computing hue histogram");
            //for (int i=0; i<16; i++) yDebug() << "value" << i << "=" << objs[intIdx].getHueHistogram().at<float>(i);
        }

        // output shape descriptors of whole blobs
        Bottle &bDesc = outWholeDescPort.prepare();
        bDesc.clear();
        double t0, t1;
        t0 = yarp::os::Time::now();
        for (std::vector<Obj2D>::iterator it=objs.begin(); it!=objs.end(); ++it)
        {
            if (it->isValid())
            {
                Bottle &bObj = bDesc.addList();

                // area info
                if (useArea)
                {
                    Bottle &areaBot = bObj.addList();
                    areaBot.addString("area");
                    Bottle &areaBotCnt = areaBot.addList();
                    areaBotCnt.addDouble(it->getArea());
                }

                // convexity info
                if (useConvexity)
                {
                    Bottle &convBot = bObj.addList();
                    convBot.addString("convexity");
                    Bottle &convBotCnt = convBot.addList();
                    convBotCnt.addDouble(it->getConvexity());
                }

                // eccentricity info
                if (useEccentricity)
                {
                    Bottle &eccBot = bObj.addList();
                    eccBot.addString("eccentricity");
                    Bottle &eccBotCnt = eccBot.addList();
                    eccBotCnt.addDouble(it->getEccentricity());
                }

                // compactness info
                if (useCompactness)
                {
                    Bottle &compBot = bObj.addList();
                    compBot.addString("compactness");
                    Bottle &compBotCnt = compBot.addList();
                    compBotCnt.addDouble(it->getCompactness());
                }

                // circleness info
                if (useCircleness)
                {
                    Bottle &circBot = bObj.addList();
                    circBot.addString("circleness");
                    Bottle &circBotCnt = circBot.addList();
                    circBotCnt.addDouble(it->getCircleness());
                }

                // squareness info
                if (useSquareness)
                {
                    Bottle &sqBot = bObj.addList();
                    sqBot.addString("squareness");
                    Bottle &sqBotCnt = sqBot.addList();
                    sqBotCnt.addDouble(it->getSquareness());
                }

                // perimeter info
                if (usePerimeter)
                {
                    Bottle &perBot = bObj.addList();
                    perBot.addString("perimeter");
                    Bottle &perBotCnt = perBot.addList();
                    perBotCnt.addDouble(it->getPerimeter());
                }

                // elongation info
                if (useElongation)
                {
                    Bottle &eloBot = bObj.addList();
                    eloBot.addString("elongation");
                    Bottle &eloBotCnt = eloBot.addList();
                    eloBotCnt.addDouble(it->getElongation());
                }

                // acquire raw moments if needed later
                Moments mom;
                if (useSpatialMoments || useCentralMoments || useCentralNormalizedMoments)
                    mom = it->getMoments();

                // spatial moments and center of mass info
                if (useSpatialMoments)
                {
                    Bottle &smBot = bObj.addList();
                    smBot.addString("spatialMoments");
                    Bottle &smBotCnt = smBot.addList();
                    smBotCnt.addDouble(mom.m00);
                    smBotCnt.addDouble(mom.m10);
                    smBotCnt.addDouble(mom.m01);
                    smBotCnt.addDouble(mom.m20);
                    smBotCnt.addDouble(mom.m11);
                    smBotCnt.addDouble(mom.m02);
                    smBotCnt.addDouble(mom.m30);
                    smBotCnt.addDouble(mom.m21);
                    smBotCnt.addDouble(mom.m12);
                    smBotCnt.addDouble(mom.m03);

                    Bottle &centerBot = bObj.addList();
                    centerBot.addString("center");
                    Bottle &centerBotCnt = centerBot.addList();
                    centerBotCnt.addDouble(mom.m10/mom.m00);
                    centerBotCnt.addDouble(mom.m01/mom.m00);
                }

                // central moments info
                if (useCentralMoments)
                {
                    Bottle &cmBot = bObj.addList();
                    cmBot.addString("centralMoments");
                    Bottle &cmBotCnt = cmBot.addList();
                    cmBotCnt.addDouble(mom.mu20);
                    cmBotCnt.addDouble(mom.mu11);
                    cmBotCnt.addDouble(mom.mu02);
                    cmBotCnt.addDouble(mom.mu30);
                    cmBotCnt.addDouble(mom.mu21);
                    cmBotCnt.addDouble(mom.mu12);
                    cmBotCnt.addDouble(mom.mu03);
                }

                // central normalized moments info
                if (useCentralNormalizedMoments)
                {
                    Bottle &cnmBot = bObj.addList();
                    cnmBot.addString("centralNormalizedMoments");
                    Bottle &cnmBotCnt = cnmBot.addList();
                    cnmBotCnt.addDouble(mom.nu20);
                    cnmBotCnt.addDouble(mom.nu11);
                    cnmBotCnt.addDouble(mom.nu02);
                    cnmBotCnt.addDouble(mom.nu30);
                    cnmBotCnt.addDouble(mom.nu21);
                    cnmBotCnt.addDouble(mom.nu12);
                    cnmBotCnt.addDouble(mom.nu03);
                }

                // (up-right) bounding rectangle info
                if (useBoundingRectangle)
                {
                    Rect br = it->getBoundingRect();
                    Bottle &brBot = bObj.addList();
                    brBot.addString("bounding_rectangle");
                    Bottle &brBotCnt = brBot.addList();
                    brBotCnt.addDouble(br.tl().x);
                    brBotCnt.addDouble(br.tl().y);
                    brBotCnt.addDouble(br.br().x);
                    brBotCnt.addDouble(br.br().y);
                    // OLD: estimate of point over the table
                    // br.x + br.width/2.;
                    // br.y + br.height;
                }

                // (rotated) enclosing rectangle info
                if (useEnclosingRectangle)
                {
                    RotatedRect er = it->getEnclosingRect();
                    Bottle &erBot = bObj.addList();
                    erBot.addString("enclosing_rectangle");
                    Bottle &erBotCnt = erBot.addList();
                    erBotCnt.addDouble(er.center.x);
                    erBotCnt.addDouble(er.center.y);
                    erBotCnt.addDouble(er.size.width);
                    erBotCnt.addDouble(er.size.height);
                    erBotCnt.addDouble(er.angle);
                }

                // OLD colour histograms info
                /*
                if (useColorHistogram)
                {
                    MatND hueHist = it->getHueHistogram();
                    for (int i=0; i<16; i++)
                        bObj.addDouble( hueHist.at<float>(i) );
                }
                */
            }
        }
        t1 = yarp::os::Time::now();
        yDebug("computed descriptors of %d objects in %f msec",
               bDesc.size(), 1000.0*(t1-t0));
        outWholeDescPort.setEnvelope(tsRaw);
        outWholeDescPort.write();

        // annotated output image
        Mat outAnnotatedMat;
        outAnnotatedMat = iplToMat(*inRawImg);

        const Scalar Blue(255,0,0);
        const Scalar Yellow(255,255,0);
        const int thickness = 2;
        for (std::vector<Obj2D>::iterator it = objs.begin();
             it != objs.end();
             ++it)
        {
            // intIdx is an auxiliary current index: 0, 1, ...
            int intIdx = std::distance(objs.begin(),it);

            if (it->isValid())
            {
                drawContours(outAnnotatedMat,
                             cont[intIdx],
                             -1, // draw all the contours of cont[intIdx]
                             Blue,
                             thickness);
            }
            else
            {
                drawContours(outAnnotatedMat,
                             cont[intIdx],
                             -1, // draw all the contours of cont[intIdx]
                             Yellow,
                             thickness);
            }
        }

        ImageOf<PixelBgr> &outAnnotatedYarp = outAnnotatedImgPort.prepare();
        outAnnotatedYarp.resize(outAnnotatedMat.cols,
                                outAnnotatedMat.rows);
        outAnnotatedMat.copyTo(iplToMat(outAnnotatedYarp));
        outAnnotatedImgPort.setEnvelope(tsRaw);
        outAnnotatedImgPort.write();
    }
}
