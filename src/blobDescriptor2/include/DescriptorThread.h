/*
 * Copyright: (C) 2012-2015 POETICON++, European Commission FP7 project ICT-288382
 * Copyright: (C) 2015 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 */

#ifndef DESC_THREAD_H
#define DESC_THREAD_H

#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Stamp.h>

#include <opencv2/legacy/compat.hpp> // cvCopyImage

#include "Helpers.h"

using namespace cv;
using namespace yarp::os;
using namespace yarp::sig;

class BlobDescriptorThread : public RateThread
{
    private:
        string moduleName;

        bool closing;
        Semaphore mutex;

        int maxObjects;
        string mode;

        // 2d mode

        string inRawImgPortName;
        string inBinImgPortName;
        string inLabImgPortName;
        string inRoiPortName;

        string outRawImgPortName;
        string outAnnotatedImgPortName;
        string outAffPortName;
        string outToolAffPortName;

        BufferedPort<ImageOf<PixelBgr> > inRawImgPort;
        BufferedPort<ImageOf<PixelBgr> > inBinImgPort;
        BufferedPort<ImageOf<PixelInt> > inLabImgPort;
        BufferedPort<Bottle>             inRoiPort;

        BufferedPort<ImageOf<PixelBgr> > outRawImgPort;
        BufferedPort<ImageOf<PixelBgr> > outAnnotatedImgPort;
        BufferedPort<Bottle>             outAffPort;
        BufferedPort<Bottle>             outToolAffPort;

        Bottle aff;
        Bottle toolAff;
        
        int minArea, maxArea;

        // for debug
        string                           outBothPartsImgPortName;
        Port                             outBothPartsImgPort;
        Mat                              bothParts;

        // 3d mode

    public:
        BlobDescriptorThread(const string &_moduleName, const double _period,
                             const int &_maxObjects,
                             const string &_mode,
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
