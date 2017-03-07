/*
 * Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
 *                Istituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Pedro Vicente <gsaponaro@isr.tecnico.ulisboa.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0
 *
 * Based on code by Ugo Pattacini <ugo.pattacini@iit.it>
 *
 */

#include <string>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>


const double ARM_DEF_HOME[] = {-50.0,  60.0,  0.0,    40.0,    0.0,  0.0,   0.0,     20.0,  30.0,10.0,10.0,  10.0,10.0, 10.0,10.0,  10.0};

const double TORSO_DEF_HOME[] = {0.0, 0.0, 0.0};

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;


/***************************************************/
class CtrlModule: public RFModule
{
protected:
    PolyDriver drvArm, drvGaze,drvArmPos,drvTorso;
    ICartesianControl *iarm;
    IGazeControl      *igaze;
    IPositionControl2 *posA,*posT;
    IEncoders *encsA;
    IControlMode2 *ctrlMA,*ctrlMT;

    BufferedPort<ImageOf<PixelRgb> > imgLPortIn,imgRPortIn;
    BufferedPort<ImageOf<PixelRgb> > imgLPortOut,imgRPortOut;
    RpcServer rpcPort;

    Mutex mutex;
    Vector cogL,cogR;
    Vector objLocation;
    bool okL,okR;
    int *controlModesArm;
    int nAxesA;
    /***************************************************/
    Vector retrieveTarget3D(const Vector &cogL, const Vector &cogR)
    {
        Vector position(3);        
        igaze->triangulate3DPoint(cogL,cogR,position);
        // Use iGaze to retrieve the 3D point
        return position;
    }

    /***************************************************/
    void fixate(const Vector &x)
    {
        igaze->lookAtFixationPoint(x);
        igaze->waitMotionDone();
        //igaze->setTrackingMode(true);
    }

    /***************************************************/
    Vector computeHandOrientation()
    {
        Matrix R(3,3);
        R(0,0) = -1.0; R(1,0) = 0.0;  R(2,0) = 0.0; 
        R(0,1) = 0.0;  R(1,1) = 0.0;  R(2,1) = -1.0; 
        R(0,2) = 0.0;  R(1,2) = -1.0; R(2,2) = 0.0; 
        return dcm2axis(R);
    }

    /***************************************************/
    void approachTargetWithHand(const Vector &x, const Vector &o,string side)
    {
        Vector dof(10,1.0),dummy;
        // disable torso
        //dof[0]=0;
        //dof[1]=0;
        //dof[2]=0;
        iarm->setDOF(dof,dummy);
        Vector approach=x;
        if(side.compare("right")==0) {          //right
            approach[1]+=0.1;// 10 cm
            yInfo() << "app right";
        }
        else if(side.compare("left")==0) {      //left
            approach[1]-=0.1;// 10 cm
            yInfo() << "app left";
        }
        else if(side.compare("bottom")==0) {    //bottom
            approach[0]+=0.15;// 10 cm
            yInfo() << "app bottom";
        }
        iarm->goToPoseSync(approach,o);
        iarm->waitMotionDone();
    }

    /***************************************************/
    void roll(const Vector &x, const Vector &o,string side)
    {
        double tempotempo;
        iarm->getTrajTime(&tempotempo);
        iarm->setTrajTime(0.7); //
        Vector target=x;
        if(side.compare("right")==0) {          //right
            target[1]-=0.1;// 10 cm
            yInfo() << "roll right";
            iarm->setTrajTime(0.7);
        }
        else if(side.compare("left")==0) {      //left
            target[1]+=0.1;// 10 cm
            yInfo() << "roll right";
            iarm->setTrajTime(0.7);
        }
        else if(side.compare("bottom")==0) {    //bottom
            target[0]-=0.1;// 10 cm
            yInfo() << "roll bottom";
            iarm->setTrajTime(1.3);
        }
            
        iarm->goToPoseSync(target,o);
        iarm->waitMotionDone();
        iarm->setTrajTime(tempotempo);
    }

    /***************************************************/
    void look_down()
    {
        Vector ang(3,0.0);
        ang[1] = -40.0;
        igaze->lookAtAbsAngles(ang);
        igaze->waitMotionDone();
    }

    /***************************************************/
    void make_it_roll(const Vector &x)
    {

        fixate(x);
        yInfo()<<"fixating at ("<<x.toString(3,3)<<")";

        Vector o=computeHandOrientation();
        yInfo()<<"computed orientation = ("<<o.toString(3,3)<<")";

        approachTargetWithHand(x,o,"right");
        yInfo()<<"approached";

        roll(x,o,"right");
        yInfo()<<"roll!";
    }
    /***************************************************/
    void tapFromRight(const Vector &x)
    {

        fixate(x);
        yInfo()<<"fixating at ("<<x.toString(3,3)<<")";

        Vector o=computeHandOrientation();
        yInfo()<<"computed orientation = ("<<o.toString(3,3)<<")";

        approachTargetWithHand(x,o,"right");
        yInfo()<<"approached";

        roll(x,o,"right");
        yInfo()<<"roll!";
    }
    /***************************************************/
    void tapFromLeft(const Vector &x)
    {

        fixate(x);
        yInfo()<<"fixating at ("<<x.toString(3,3)<<")";

        Vector o=computeHandOrientation();
        yInfo()<<"computed orientation = ("<<o.toString(3,3)<<")";

        approachTargetWithHand(x,o,"left");
        yInfo()<<"approached";

        roll(x,o,"left");
        yInfo()<<"roll!";
    }
    /***************************************************/
    void push(const Vector &x)
    {

        fixate(x);
        yInfo()<<"fixating at ("<<x.toString(3,3)<<")";

        Vector o=computeHandOrientation();
        yInfo()<<"computed orientation = ("<<o.toString(3,3)<<")";

        approachTargetWithHand(x,o,"bottom");
        yInfo()<<"approached";

        roll(x,o,"bottom");
        yInfo()<<"roll!";
    }
    /***************************************************/
    void home()
    {
/*
        Vector xd(3);
        xd[0] = -0.3;
        xd[1] = +0.2;
        xd[2] = 0.0;
        iarm->goToPositionSync(xd);
        iarm->waitMotionDone();
*/
        ctrlMA->getControlModes(controlModesArm);

        //fprintf(stderr,"\ngoHomeArm - step_1\n");

        for(int i=0; i<nAxesA; i++)
        {
            if (controlModesArm[i]!=VOCAB_CM_POSITION)
            {
                ctrlMA->setControlMode(i,VOCAB_CM_POSITION);
            }
        }
        for(int i=0; i<3; i++)
        {
            ctrlMT->setControlMode(i,VOCAB_CM_POSITION);
        }
        posT->positionMove(TORSO_DEF_HOME);

        bool done=false;
        double elapsedTime=0.0;
        double startTime=Time::now();

        while(!done && elapsedTime<2.0)
        {
            posT->checkMotionDone(&done);
            Time::delay(0.04);
            elapsedTime= Time::now()-startTime;
        }

        posA->positionMove(ARM_DEF_HOME);

        startTime=Time::now();
        while(!done && elapsedTime<2.0)
        {
            posA->checkMotionDone(&done);
            Time::delay(0.04);
            elapsedTime= Time::now()-startTime;
        }
    }
    void retrieveObjLocation(const Bottle &command)
    {
        objLocation[0] = command.get(1).asDouble();
        objLocation[1] = command.get(2).asDouble();
        objLocation[2] = command.get(3).asDouble();
        yInfo() << "Object Location" << objLocation.toString();
    }
public:
    /***************************************************/
    bool configure(ResourceFinder &rf)
    {
        string robot=rf.check("robot",Value("icubSim")).asString();
        string moduleName = rf.check("name", Value("handActions"),"module name (string)").asString();
        string arm=rf.check("arm",Value("right_arm")).asString();
        if(arm.compare("left")==0 ||arm.compare("leftarm")==0 || arm.compare("armleft")==0 || arm.compare("Left")==0)
            arm = "left_arm";
        else
            arm = "right_arm";

        /******** Cartesian Interface *******/

        Property optArm;
        optArm.put("device","cartesiancontrollerclient");
        optArm.put("remote","/"+robot+"/cartesianController/"+arm);
        optArm.put("local","/"+moduleName+"/cartesian_client/"+arm);

        objLocation.resize(3);  
        // let's give the controller some time to warm up
        bool ok=false;
        double t0=Time::now();
        while (Time::now()-t0<10.0)
        {
            // this might fail if controller
            // is not connected to solver yet
            if (drvArm.open(optArm))
            {
                ok=true;
                break;
            }

            Time::delay(1.0);
        }

        if (!ok)
        {
            yError()<<"Unable to open the Cartesian Controller";
            return false;
        }

        /******** Position Arm Control Interface *******/
        /* connect to remote device  */
        Property options;
        options.put("device", "remote_controlboard");             // device to open
        options.put("local", "/"+moduleName+"/position_control/"+arm);           // local port name
        options.put("remote", "/" + robot + "/"+arm);         // where we connect to
        drvArmPos.open(options);
        if (!drvArmPos.isValid()) 
        {
            cout << moduleName << ": unable to connect to device: remote_controlboard of " << arm << endl;
            return false;
        }
        if (!drvArmPos.view(posA) || !drvArmPos.view(encsA) || !drvArmPos.view(ctrlMA)) 
        {
            cout << moduleName << ": problems acquiring interfaces to remote_controlboard of " << arm << endl;
            return false;
        }
        posA->getAxes(&nAxesA);
        controlModesArm = new int[nAxesA];
        Property optGaze;
        optGaze.put("device","gazecontrollerclient");
        optGaze.put("remote","/iKinGazeCtrl");
        optGaze.put("local","/"+moduleName+"/gaze_client");
        /******** Position Torso Control Interface *******/

        Property optionTorso("(device remote_controlboard)");
        optionTorso.put("remote",("/"+robot+"/torso").c_str());
        optionTorso.put("local","/"+moduleName+"/position_control/torso");
        if (!drvTorso.open(optionTorso))
        {
            yError()<<"Joints Torso controller not available";
            terminate();
            return false;
        }
        if (!drvTorso.view(posT) || !drvTorso.view(ctrlMT)) 
        {
           cout << moduleName << ": problems acquiring interfaces to remote_controlboard of Torso"<< endl;
            return false;
        }
        /******** Gaze Control *******/
        if (!drvGaze.open(optGaze))
        {
            yError()<<"Unable to open the Gaze Controller";
            drvArm.close(); // IMPORTANT - Because drvArm was already open
            return false;
        }
        drvGaze.view(igaze);
        drvArm.view(iarm);

        imgLPortIn.open("/imgL:i");
        imgRPortIn.open("/imgR:i");

        imgLPortOut.open("/imgL:o");
        imgRPortOut.open("/imgR:o");

        rpcPort.open("/"+moduleName+"/rpc:i");
        attach(rpcPort);
        return true;
    }

    /***************************************************/
    bool interruptModule()
    {
        imgLPortIn.interrupt();
        imgRPortIn.interrupt();
        return true;
    }

    /***************************************************/
    bool close()
    {
        drvArm.close();
        drvGaze.close();
        imgLPortIn.close();
        imgRPortIn.close();
        imgLPortOut.close();
        imgRPortOut.close();
        rpcPort.close();
        return true;
    }

    /***************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
        string cmd=command.get(0).asString();
        
        if (cmd=="help")
        {
            reply.addVocab(Vocab::encode("many"));
            reply.addString("Available commands:");
            reply.addString("- look_down");
            reply.addString("- tapFromLeft x y z");
            reply.addString("- tapFromRight x y z");
            reply.addString("- push x y z");
            reply.addString("- draw x y z");
            reply.addString("- home");
            reply.addString("- quit");
        }
        else if (cmd=="look_down")
        {
            look_down();
            // we assume the robot is not moving now
            reply.addString("ack");
            reply.addString("Yep! I'm looking down now!");
        }
        else if (cmd=="home")
        {
                home();
                // we assume the robot is not moving now
                reply.addString("ack");
                reply.addString("I've got the hard work done! Gone home.");
        }
        else if (cmd=="tapFromRight")
        {
            if(command.size()!=4)
            {
                reply.addString("nack");
                reply.addString("Missing target 3D position");
                return false;
            }
            retrieveObjLocation(command);
            /*
            mutex.lock();
            bool go = okL && okR;
            Vector cogL = this->cogL;
            Vector cogR = this->cogR;
            mutex.unlock();
            */
            tapFromRight(objLocation);
            // we assume the robot is not moving now
            reply.addString("ack");
            reply.addString("Yeah! Tapping from Right!");
        }

        else if (cmd=="tapFromLeft")
        {
            if(command.size()!=4)
            {
                reply.addString("nack");
                reply.addString("Missing target 3D position");
                return false;
            }
            retrieveObjLocation(command);
            tapFromLeft(objLocation);
            // we assume the robot is not moving now
            reply.addString("ack");
            reply.addString("Yeah! Tapping from Left!");
        }
        else if (cmd=="push")
        {
            if(command.size()!=4)
            {
                reply.addString("nack");
                reply.addString("Missing target 3D position");
                return false;
            }
            retrieveObjLocation(command);
            push(objLocation);
            // we assume the robot is not moving now
            reply.addString("ack");
            reply.addString("Yeah! Pushing!");
        }
/*
        else if (cmd=="push")
        {

            push(x);
            // we assume the robot is not moving now
            reply.addString("ack");
            reply.addString("Yeah! Pushing!");
        }
        else if (cmd=="draw")
        {

            draw(x);
            // we assume the robot is not moving now
            reply.addString("ack");
            reply.addString("Yeah! Drawing!");
        }
*/
        else
            // the father class already handles the "quit" command
            return RFModule::respond(command,reply);

        return true;
    }

    /***************************************************/
    double getPeriod()
    {
        return 0.0;     // sync upon incoming images
    }

    /***************************************************/
    bool updateModule()
    {
        return true;
    }
};


/***************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"YARP doesn't seem to be available";
        return 1;
    }

    CtrlModule mod;
    ResourceFinder rf;
    rf.configure(argc,argv);
    return mod.runModule(rf);
}
