/*
 * Copyright: (C) 2016 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */

#ifndef DESC_THREAD_H
#define DESC_THREAD_H

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Stamp.h>

#include "Helpers.h"

class BlobDescriptorThread : public yarp::os::RateThread
{
    private:
        std::string moduleName;

        bool closing;
        yarp::os::Semaphore mutex;

        int maxObjects;
        std::string mode;

        // 2d mode

        std::string inRawImgPortName;
        std::string inBinImgPortName;
        std::string inLabImgPortName;
        std::string inRoiPortName;

        std::string outRawImgPortName;
        std::string outAnnotatedImgPortName;
        std::string outAffPortName;
        std::string outToolAffPortName;

        yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > inRawImgPort;
        yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > inBinImgPort;
        yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelInt> > inLabImgPort;
        yarp::os::BufferedPort<yarp::os::Bottle>                                   inRoiPort;

        yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > outRawImgPort;
        yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > outAnnotatedImgPort;
        yarp::os::BufferedPort<yarp::os::Bottle>                                   outAffPort;
        yarp::os::BufferedPort<yarp::os::Bottle>                                   outToolAffPort;

        yarp::os::Bottle aff;
        yarp::os::Bottle toolAff;

        int minArea;
        int maxArea;

        // for debug
        std::string                           outBothPartsImgPortName;
        yarp::os::Port                        outBothPartsImgPort;
        cv::Mat                               bothParts;

        // 3d mode

    public:
        BlobDescriptorThread(const std::string &_moduleName, const double _period,
                             const int &_maxObjects,
                             const std::string &_mode,
                             const int &_minArea, const int &_maxArea);

        bool openPorts();
        void close();
        void interrupt();

        bool threadInit();
        void run();

        void run2d();
        void run3d();
};

#endif
