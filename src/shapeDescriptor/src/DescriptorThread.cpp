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

    // tbc
    //inRoiPortName = "/" + moduleName + "/blobs:i";
    //inRoiPort.open(inRoiPortName.c_str());

    outRawImgPortName = "/" + moduleName + "/rawImg:o";
    outRawImgPort.open(outRawImgPortName.c_str());

    outAnnotatedImgPortName = "/" + moduleName + "/annotatedImg:o";
    outAnnotatedImgPort.open(outAnnotatedImgPortName.c_str());

    outAffPortName = "/" + moduleName + "/affDescriptor:o";
    outAffPort.open(outAffPortName.c_str());

    // tbc
    //outToolAffPortName = "/" + moduleName + "/toolAffDescriptor:o";
    //outToolAffPort.open(outToolAffPortName.c_str());

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

    outRawImgPort.writeStrict();
    outRawImgPort.close();
    outAnnotatedImgPort.close();
    outAffPort.writeStrict();
    outAffPort.close();

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

    outRawImgPort.interrupt();
    outAnnotatedImgPort.interrupt();
    outAffPort.interrupt();

    /*
    inRoiPort.interrupt();
    outToolAffPort.interrupt();
    // for debug
    outBothPartsImgPort.interrupt();
    */
}

bool ShapeDescriptorThread::threadInit()
{
    // parse basic options
    maxObjects = rf.check("maxObjects", Value(10),
        "maximum number of objects to process (int)").asInt();

    minArea = rf.check("minArea", Value(100),
        "minimum valid blob area (int)").asInt();

    maxArea = rf.check("maxArea", Value(3000),
        "maximum valid blob area (int)").asInt();

    yInfo("initialized thread with "
          "maxObjects=%d minArea=%d maxArea=%d",
          maxObjects, minArea, maxArea);

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

        // container of objects/blobs with associated features
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

            // construct Obj2D with validity,contour,area
            objs.push_back( Obj2D(isValid, cont[intIdx][largest], largestArea) );
            // compute remaining shape descriptors and colour histogram
            objs[intIdx].computeDescriptors();
            Mat inRaw = iplToMat(*inRawImg);
            objs[intIdx].setMask( inRaw(objs[intIdx].getBoundingRect()) );
            if (!objs[intIdx].computeHueHistogram())
                yWarning("error computing hue histogram");
            //for (int i=0; i<16; i++) yDebug() << "value" << i << "=" << objs[intIdx].getHueHistogram().at<float>(i);
        }

        bool printArea = true;
        bool printConvexity = true;
        bool printEccentricity = true;
        bool printCompactness = true;
        bool printCircleness = true;
        bool printSquareness = true;

        bool printBoundingRectangle = false;
        bool printEnclosingRectangle = false;

        // output shape descriptors of whole blobs
        Bottle &bDesc = outAffPort.prepare();
        bDesc.clear();
        for (std::vector<Obj2D>::iterator it=objs.begin(); it!=objs.end(); ++it)
        {
            if (it->isValid())
            {
                Bottle &bObj = bDesc.addList();

                // area info
                if (printArea)
                {
                    Bottle &areaBot = bObj.addList();
                    areaBot.addString("area");
                    Bottle &areaBotCnt = areaBot.addList();
                    areaBotCnt.addDouble(it->getArea());
                }

                // convexity info
                if (printConvexity)
                {
                    Bottle &convBot = bObj.addList();
                    convBot.addString("convexity");
                    Bottle &convBotCnt = convBot.addList();
                    convBotCnt.addDouble(it->getConvexity());
                }

                // eccentricity info
                if (printEccentricity)
                {
                    Bottle &eccBot = bObj.addList();
                    eccBot.addString("eccentricity");
                    Bottle &eccBotCnt = eccBot.addList();
                    eccBotCnt.addDouble(it->getEccentricity());
                }

                // compactness info
                if (printCompactness)
                {
                    Bottle &compBot = bObj.addList();
                    compBot.addString("compactness");
                    Bottle &compBotCnt = compBot.addList();
                    compBotCnt.addDouble(it->getCompactness());
                }

                // circleness info
                if (printCircleness)
                {
                    Bottle &circBot = bObj.addList();
                    circBot.addString("circleness");
                    Bottle &circBotCnt = circBot.addList();
                    circBotCnt.addDouble(it->getCircleness());
                }

                // squareness info
                if (printSquareness)
                {
                    Bottle &sqBot = bObj.addList();
                    sqBot.addString("squareness");
                    Bottle &sqBotCnt = sqBot.addList();
                    sqBotCnt.addDouble(it->getSquareness());
                }

                // (up-right) bounding rectangle info
                if (printBoundingRectangle)
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
                if (printEnclosingRectangle)
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
                MatND hueHist = it->getHueHistogram();
                for (int i=0; i<16; i++)
                    bObj.addDouble( hueHist.at<float>(i) );
                */
            }
        }
        outAffPort.setEnvelope(tsRaw);
        outAffPort.write();
    }

    if (inRawImg!=NULL)
    {
        // annotated output image
        Mat outAnnotatedMat;
        outAnnotatedMat = iplToMat(*inRawImg);
        Mat channels[4];
        split(outAnnotatedMat,channels);
        Scalar minColor(0,0,0);
        Scalar maxColor(255,255,0);
        inRange(outAnnotatedMat,minColor,maxColor,channels[3]);
        Mat outAnnotatedMat2;
        // FIXME
        /*
        cv::merge(channels,4,outAnnotatedMat2);
        ImageOf<PixelBgr> &outAnnotatedYarp = outAnnotatedImgPort.prepare();
        IplImage outAnnotatedIpl = outAnnotatedMat2;
        outAnnotatedYarp.resize(outAnnotatedIpl.width,
                                outAnnotatedIpl.height);
        //outAnnotatedYarp.resize(outAnnotatedMat2.cols,
        //                        outAnnotatedMat2.rows);
        //outAnnotatedMat2.copyTo( iplToMat(outAnnotatedYarp) );
        outAnnotatedMat2.copyTo( cv::cvarrToMat(static_cast<IplImage*>(outAnnotatedYarp.getIplImage())) );
        outAnnotatedImgPort.setEnvelope(tsRaw);
        outAnnotatedImgPort.write();
        */
    }
}
