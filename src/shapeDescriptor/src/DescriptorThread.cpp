/*
 * Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0
 *
 */

#include "DescriptorDefaults.h"
#include "DescriptorThread.h"
#include "Obj2D.h"

using namespace cv;
using namespace yarp::os;
using namespace yarp::sig;

ShapeDescriptorThread::ShapeDescriptorThread(const std::string &_moduleName,
    ResourceFinder &_rf)
    : RateThread(DefThreadPeriod),
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

    outPartDescPortName = "/" + moduleName + "/partDescriptors:o";
    outPartDescPort.open(outPartDescPortName.c_str());

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

    outPartDescPort.writeStrict();
    outPartDescPort.close();

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

    //inRoiPort.interrupt();
    outPartDescPort.interrupt();
    // for debug
    //outBothPartsImgPort.interrupt();
}

bool ShapeDescriptorThread::threadInit()
{
    maxObjects = rf.check("maxObjects", Value(DefMaxObjects)).asInt();
    minArea = rf.check("minArea", Value(DefMinArea)).asInt();
    maxArea = rf.check("maxArea", Value(DefMaxArea)).asInt();

    yInfo("initialized thread with maxObjects=%d minArea=%d maxArea=%d",
          maxObjects, minArea, maxArea);

    useCenterOfMass = rf.check("center",Value(DefUseCenterOfMass)).asString()=="on"?true:false;
    useArea = rf.check("area",Value(DefUseArea)).asString()=="on"?true:false;
    useConvexity = rf.check("convexity",Value(DefUseConvexity)).asString()=="on"?true:false;
    useConvexityDefects = rf.check("convexityDefects",Value(DefUseConvexityDefects)).asString()=="on"?true:false;
    useEccentricity = rf.check("eccentricity",Value(DefUseEccentricity)).asString()=="on"?true:false;
    useCompactness = rf.check("compactness",Value(DefUseCompactness)).asString()=="on"?true:false;
    useCircularity = rf.check("circularity",Value(DefUseCircularity)).asString()=="on"?true:false;
    useSquareness = rf.check("squareness",Value(DefUseSquareness)).asString()=="on"?true:false;
    usePerimeter = rf.check("perimeter",Value(DefUsePerimeter)).asString()=="on"?true:false;
    useElongation = rf.check("elongation",Value(DefUseElongation)).asString()=="on"?true:false;
    useSpatialMoments = rf.check("spatialMoments",Value(DefSpatialMoments)).asString()=="on"?true:false;
    useCentralMoments = rf.check("centralMoments",Value(DefCentralMoments)).asString()=="on"?true:false;
    useCentralNormalizedMoments = rf.check("centralNormalizedMoments",Value(DefCentralNormalizedMoments)).asString()=="on"?true:false;
    useBoundingRectangle = rf.check("boundingRectangle",Value(DefBoundingRectangle)).asString()=="on"?true:false;
    useEnclosingRectangle = rf.check("enclosingRectangle",Value(DefEnclosingRectangle)).asString()=="on"?true:false;

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
    bool useColor = false;

    if (inRawImgPort.getInputCount())
        useColor = true;

    // acquire new input images
    ImageOf<PixelBgr> *inRawImg;
    if (useColor)
        inRawImg = inRawImgPort.read(true);

    ImageOf<PixelBgr> *inBinImg = inBinImgPort.read(true);
    ImageOf<PixelInt> *inLabImg = inLabImgPort.read(true);

    // read timestamps
    Stamp tsBin, tsLab;
    if ( (inBinImgPort.getInputCount() && !inBinImgPort.getEnvelope(tsBin)) ||
         (inLabImgPort.getInputCount() && !inLabImgPort.getEnvelope(tsBin)) )
    {
        //yWarning("timestamp(s) missing");
    }

    if (inBinImg!=NULL &&
        inLabImg!=NULL)
    {
        // check dimensions of input images to be equal
        const int refWidth  = inBinImg->width();
        const int refHeight = inBinImg->height();
        if (refWidth!=inLabImg->width() || refHeight!=inLabImg->height())
        {
            yWarning("dimension mismatch between binary and labelled images");
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

        // container of whole objects with associated features
        std::vector<Obj2D> objs;

        // container of object parts with associated features
        std::vector<std::pair<Obj2D,Obj2D> > parts;

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

            // construct whole object and extract features
            objs.push_back( Obj2D(cont[intIdx][largest]) );



            // TODO optimize
            RotatedRect enclosingRect = objs.back().getEnclosingRect();
            float cx = enclosingRect.center.x,
                  cy = enclosingRect.center.y;

            float wi = enclosingRect.size.width,
                  he = enclosingRect.size.height,
                  an = enclosingRect.angle;

            float ca = cos(an / 180.0 * CV_PI),
                  sa = sin(an / 180.0 * CV_PI);

            if (wi==1 || he==1)
                yWarning("enclosingRectangle of blob %d is one-dimensional", intIdx);

            Size2f half_size(wi,he); // initially, same size as whole object encl. rect

            // force width to be the smaller dimension, height to be the larger one
            if (wi < he)
            {
                half_size.height = half_size.height/2.;
                //yDebug("OK: height>width, thus I made splitting of height for cropping obj parts");
            }
            else
            {
                // this should never happen in the first place
                yWarning("width>height... need to swap them!");
                // to complete
            }

            Point2f top_center, bot_center; // w.r.t. original non-rotated image

            // centers of halves calculated from cx,cy, in whole image coordinates
            top_center = Point2f( cvRound(cx+sa*half_size.height/2.), cvRound(cy-ca*half_size.height/2.) );
            bot_center = Point2f( cvRound(cx-sa*half_size.height/2.), cvRound(cy+ca*half_size.height/2.) );

            // top-left corners of upright halves, in whole image coordinates
            Point2f top_tl2, bot_tl2;
            top_tl2 = Point2f( cvRound(cx-half_size.width/2.), cvRound(cy-half_size.height) );
            bot_tl2 = Point2f( cvRound(cx-half_size.width/2.), cvRound(cy) );

            // get transformation matrix provided by rotation "an" and no scaling factor
            Mat M = getRotationMatrix2D(enclosingRect.center, an, 1.0);

            Mat inBin = iplToMat(*inBinImg);

            // findContours needs 1 channel 8uC1 or 32sC1
            cvtColor(inBin, inBin, CV_BGR2GRAY, 1);

            // fast connected component analysis
            /*
            int numLab;
            Mat labMap;
            Mat ccStats;
            Mat centroids;
            double cct0 = yarp::os::Time::now();
            double cct1;
            numLab = connectedComponentsWithStats(inBin,
                                                  labMap,
                                                  ccStats,
                                                  centroids);
            cct1 = yarp::os::Time::now();
            yDebug("computed %d connected components in %f msec",
                   numLab-1, 1000.0*(cct1-cct0));
            */

            // perform affine transformation (rotation)
            Mat rotated(inBin.size(), inBin.type()); // output
            warpAffine(inBin, rotated, M, inBin.size(), INTER_CUBIC);

            // top part
            Point2f top_center_rot;
            top_center_rot.x =  M.at<double>(0,0)*(top_center.x-cx) + M.at<double>(0,1)*(top_center.y-cy) + cx;
            top_center_rot.y = -M.at<double>(1,0)*(top_center.y-cy) + M.at<double>(1,1)*(top_center.x-cx) + cy;

            // bottom part
            Point2f bot_center_rot;
            bot_center_rot.x =  M.at<double>(0,0)*(bot_center.x-cx) + M.at<double>(0,1)*(bot_center.y-cy) + cx;
            bot_center_rot.y = -M.at<double>(1,0)*(bot_center.y-cy) + M.at<double>(1,1)*(bot_center.x-cx) + cy;

            // shift crop centers vertically by halfObjHeight/2, i.e., wholeObjHeight/4
            top_center_rot.y -= half_size.height/2.; // moving up in image
            bot_center_rot.y += half_size.height/2.; // moving down in image

            // enlarge crop area to capture pixels around blob borders that could have been missed
            Size2f crop_size(half_size.width,half_size.height);
            const int HORIZ_CROPAREA_SHIFT = 8; // to slightly enlarge tool part crop area
            const int VERT_CROPAREA_SHIFT  = 5;
            crop_size.width  += HORIZ_CROPAREA_SHIFT; // along x
            crop_size.height += VERT_CROPAREA_SHIFT;  // small along y, or we'd capture too much of other part

            // crop resulting top image - Tool Top
            Mat topRectCropped;
            getRectSubPix(rotated, crop_size, top_center_rot, topRectCropped);

            // crop resulting bottom image - Tool Bottom
            Mat botRectCropped;
            getRectSubPix(rotated, crop_size, bot_center_rot, botRectCropped);

            // continue processing of top part, whose image is now safe to modify
            std::vector<std::vector<Point> > top_cnt;
            std::vector<Vec4i> top_hrch;
            topRectCropped.convertTo(topRectCropped, CV_8UC1); // redundant?
            findContours(topRectCropped, top_cnt, top_hrch, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

            if (top_cnt.size()==0)
            {
                yWarning("blob %d, top part has zero contours", intIdx);
                continue;
            }

            double top_largest_area = 0;
            int top_largest_cnt_index = 0;
            // find index of contour with largest area
            for( int c = 0; c < top_cnt.size(); c++ )
            {
                double curr_area = contourArea(top_cnt[c], false);

                if (curr_area > top_largest_area)
                {
                    top_largest_area = curr_area;
                    top_largest_cnt_index = c;
                }
            }

            // continue processing of bottom part, whose image is now safe to modify
            std::vector<std::vector<Point> > bot_cnt;
            std::vector<Vec4i> bot_hrch;
            botRectCropped.convertTo(botRectCropped, CV_8UC1); // redundant?
            findContours(botRectCropped, bot_cnt, bot_hrch, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

            if (bot_cnt.size()==0)
            {
                yWarning("blob %d, bottom part has zero contours", intIdx);
                continue;
            }

            double bot_largest_area = 0;
            int bot_largest_cnt_index = 0;
            // find index of contour with largest area
            for( int c = 0; c < bot_cnt.size(); c++ )
            {
                double curr_area = contourArea(bot_cnt[c], false);

                if (curr_area > bot_largest_area)
                {
                    bot_largest_area = curr_area;
                    bot_largest_cnt_index = c;
                }
            }



            // construct object parts and extract features
            parts.push_back( std::make_pair(Obj2D(top_cnt[top_largest_cnt_index]),
                                            Obj2D(bot_cnt[bot_largest_cnt_index])) );
        }

        // output shape descriptors of whole blobs
        Bottle &bDesc = outWholeDescPort.prepare();
        bDesc.clear();
        double t0, t1;
        t0 = yarp::os::Time::now();
        for (std::vector<Obj2D>::iterator it=objs.begin(); it!=objs.end(); ++it)
        {
            if (it->areaInRange(minArea,maxArea))
            {
                Bottle &bObj = bDesc.addList();
                addDescriptors(*it, bObj);
            }
            else
                yWarning("area %.2f not in range (%d,%d)", it->getArea(),minArea,maxArea);
        }
        outWholeDescPort.setEnvelope(tsBin);
        outWholeDescPort.write();

        // output shape descriptors of blob parts
        Bottle &bPartDesc = outPartDescPort.prepare();
        bPartDesc.clear();
        for (int o=0; o<parts.size(); ++o)
        {
            // new list for current object
            Bottle &bothPartsBot = bPartDesc.addList();

            // top part
            Bottle &topBot = bothPartsBot.addList();
            topBot.clear();
            addDescriptors(parts[o].first, topBot);

            // bottom part
            Bottle &botBot = bothPartsBot.addList();
            botBot.clear();
            addDescriptors(parts[o].second, botBot);
        }
        t1 = yarp::os::Time::now();
        yDebug("computed descriptors of %d objects (whole and parts) in %f msec",
               bDesc.size(), 1000.0*(t1-t0));
        outPartDescPort.setEnvelope(tsBin);
        outPartDescPort.write();

        // annotated output image
        Mat outAnnotatedMat;
        if (useColor)
        {
            outAnnotatedMat = iplToMat(*inRawImg);

            const Scalar Blue(255,0,0);
            const Scalar Red(0,0,255);
            const Scalar Yellow(255,255,0);
            const int thickness = 2;
            for (std::vector<Obj2D>::iterator it = objs.begin();
                 it != objs.end();
                 ++it)
            {
                // intIdx is an auxiliary current index: 0, 1, ...
                int intIdx = std::distance(objs.begin(),it);

                if (it->areaInRange(minArea,maxArea))
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

                // draw best fit line
                bool drawBestFitLine = false;
                if (drawBestFitLine)
                {
                    const int LE = it->getEnclosingRect().size.height/2;
                    const Vec4f bestLine = it->getBestLineFit();
                    // draw segment from center to first extreme
                    line(outAnnotatedMat,
                         Point(bestLine[2],bestLine[3]),
                         Point(bestLine[2]+bestLine[0]*LE,bestLine[3]+bestLine[1]*LE),
                         Red,
                         thickness);
                    // draw segment from center to second extreme
                    line(outAnnotatedMat,
                         Point(bestLine[2],bestLine[3]),
                         Point(bestLine[2]-bestLine[0]*LE,bestLine[3]-bestLine[1]*LE),
                         Red,
                         thickness);
                }

                // draw convexity defects
                bool drawConvexityDefects = true;
                if (drawConvexityDefects)
                {
                    std::vector<Vec4i> defs = it->getConvexityDefects();
                    for (std::vector<Vec4i>::iterator d = defs.begin();
                         d != defs.end();
                         ++d)
                    {
                        // http://stackoverflow.com/a/31358401/1638888
                        int startidx = (*d)[0]; Point ptStart( it->getContour()[startidx] );
                        int endidx = (*d)[1]; Point ptEnd( it->getContour()[endidx] );
                        int faridx = (*d)[2]; Point ptFar( it->getContour()[faridx] );
                        float depth = (*d)[3] / 256;

                        const int Radius = 5;

                        line( outAnnotatedMat, ptStart, ptEnd, Red, thickness );
                        //line( outAnnotatedMat, ptStart, ptFar, Red, thickness );
                        //line( outAnnotatedMat, ptEnd, ptFar, Red, thickness );
                        circle( outAnnotatedMat, ptFar, Radius, Red, thickness );

                        arrowedLine(outAnnotatedMat,
                                    ptFar,
                                    Point((ptStart.x+ptEnd.x)/2,(ptStart.y+ptEnd.y)/2),
                                    Red,
                                    thickness,
                                    8,    // lineType
                                    0,    // shift
                                    0.3); // tipLength
                    }
                }
            }

            ImageOf<PixelBgr> &outAnnotatedYarp = outAnnotatedImgPort.prepare();
            outAnnotatedYarp.resize(outAnnotatedMat.cols,
                                    outAnnotatedMat.rows);
            outAnnotatedMat.copyTo(iplToMat(outAnnotatedYarp));
            outAnnotatedImgPort.setEnvelope(tsBin);
            outAnnotatedImgPort.write();
        } // end if (useColor)
    } // end if (inBinImg!=NULL)
}

/**
  * Add a list of descriptors to a Bottle. Each descriptor is a Property-like
  * list in the format ("name" (value)), where value can be one or more
  * numbers.
  */
bool ShapeDescriptorThread::addDescriptors(Obj2D &o, Bottle &b)
{
    // acquire raw spatial moments if needed later
    Moments mom;
    if (useCenterOfMass || useSpatialMoments || useCentralMoments || useCentralNormalizedMoments)
        mom = o.getMoments();

    // center of mass info (for now computed from spatial moments)
    if (useCenterOfMass)
    {
        Bottle &centerBot = b.addList();
        centerBot.addString("center");
        Bottle &centerBotCnt = centerBot.addList();
        centerBotCnt.addDouble(mom.m10/mom.m00);
        centerBotCnt.addDouble(mom.m01/mom.m00);
    }

    // spatial moments and center of mass info
    if (useSpatialMoments)
    {
        Bottle &smBot = b.addList();
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
    }

    // area info
    if (useArea)
    {
        Bottle &areaBot = b.addList();
        areaBot.addString("area");
        Bottle &areaBotCnt = areaBot.addList();
        areaBotCnt.addDouble(o.getArea());
    }

    // convexity info
    if (useConvexity)
    {
        Bottle &convBot = b.addList();
        convBot.addString("convexity");
        Bottle &convBotCnt = convBot.addList();
        convBotCnt.addDouble(o.getConvexity());
    }

    // convexity defects info
    if (useConvexityDefects)
    {
        Bottle &convDefBot = b.addList();
        convDefBot.addString("convexityDefects");
        Bottle &convDefBotCnt = convDefBot.addList();
        std::vector<cv::Vec4i> defs = o.getConvexityDefects();
        convDefBotCnt.addDouble(defs.size());
        /*
        for(std::vector<cv::Vec4i>::const_iterator iter = defs.begin();
            iter != defs.end();
            ++iter)
        {
            Bottle &d = convDefBotCnt.addList();
            d.addDouble((*iter)[0]);
            d.addDouble((*iter)[1]);
            d.addDouble((*iter)[2]);
            d.addDouble((*iter)[3]);
        }
        */
    }

    // eccentricity info
    if (useEccentricity)
    {
        Bottle &eccBot = b.addList();
        eccBot.addString("eccentricity");
        Bottle &eccBotCnt = eccBot.addList();
        eccBotCnt.addDouble(o.getEccentricity());
    }

    // compactness info
    if (useCompactness)
    {
        Bottle &compBot = b.addList();
        compBot.addString("compactness");
        Bottle &compBotCnt = compBot.addList();
        compBotCnt.addDouble(o.getCompactness());
    }

    // circularity info
    if (useCircularity)
    {
        Bottle &circBot = b.addList();
        circBot.addString("circularity");
        Bottle &circBotCnt = circBot.addList();
        circBotCnt.addDouble(o.getCircularity());
    }

    // squareness info
    if (useSquareness)
    {
        Bottle &sqBot = b.addList();
        sqBot.addString("squareness");
        Bottle &sqBotCnt = sqBot.addList();
        sqBotCnt.addDouble(o.getSquareness());
    }

    // perimeter info
    if (usePerimeter)
    {
        Bottle &perBot = b.addList();
        perBot.addString("perimeter");
        Bottle &perBotCnt = perBot.addList();
        perBotCnt.addDouble(o.getPerimeter());
    }

    // elongation info
    if (useElongation)
    {
        Bottle &eloBot = b.addList();
        eloBot.addString("elongation");
        Bottle &eloBotCnt = eloBot.addList();
        eloBotCnt.addDouble(o.getElongation());
    }

    // central moments info
    if (useCentralMoments)
    {
        Bottle &cmBot = b.addList();
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
        Bottle &cnmBot = b.addList();
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
        Rect br = o.getBoundingRect();
        Bottle &brBot = b.addList();
        brBot.addString("boundingRectangle");
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
        RotatedRect er = o.getEnclosingRect();
        Bottle &erBot = b.addList();
        erBot.addString("enclosingRectangle");
        Bottle &erBotCnt = erBot.addList();
        erBotCnt.addDouble(er.center.x);
        erBotCnt.addDouble(er.center.y);
        erBotCnt.addDouble(er.size.width);
        erBotCnt.addDouble(er.size.height);
        erBotCnt.addDouble(er.angle);
    }

    return true;
}
