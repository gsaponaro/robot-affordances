/*
 * Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */

#ifndef DESC_THREAD_H
#define DESC_THREAD_H

#include <string>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Time.h>

#include "Helpers.h"
#include "Obj2D.h"

class ShapeDescriptorThread : public yarp::os::RateThread
{
    private:
        std::string moduleName;
        yarp::os::ResourceFinder rf;

        bool closing;
        yarp::os::Semaphore mutex;

        int maxObjects;

        std::string inRawImgPortName;
        std::string inBinImgPortName;
        std::string inLabImgPortName;
        //std::string inRoiPortName; // alternative to labelled image

        //std::string outRawImgPortName;
        std::string outAnnotatedImgPortName;
        std::string outWholeDescPortName;
        std::string outPartDescPortName;

        yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > inRawImgPort;
        yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > inBinImgPort;
        yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelInt> > inLabImgPort;
        //yarp::os::BufferedPort<yarp::os::Bottle>                         inRoiPort;

        //yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > outRawImgPort;
        yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > outAnnotatedImgPort;
        yarp::os::BufferedPort<yarp::os::Bottle>                         outWholeDescPort;
        yarp::os::BufferedPort<yarp::os::Bottle>                         outPartDescPort;

        int minArea;
        int maxArea;

        bool useArea;
        bool useConvexity;
        bool useConvexityDefects;
        bool useEccentricity;
        bool useCompactness;
        bool useCircularity;
        bool useSquareness;
        bool usePerimeter;
        bool useElongation;
        bool useSpatialMoments;
        bool useCentralMoments;
        bool useCentralNormalizedMoments;
        bool useBoundingRectangle;
        bool useEnclosingRectangle;

        // for debug
        std::string    outBothPartsImgPortName;
        yarp::os::Port outBothPartsImgPort;
        cv::Mat        bothParts;

    public:
        ShapeDescriptorThread(const std::string &_moduleName,
                              yarp::os::ResourceFinder &_rf);

        bool openPorts();
        void close();
        void interrupt();

        bool threadInit();
        void run();

        void run2d();

        bool addDescriptors(Obj2D &o, yarp::os::Bottle &b);
};

#endif
