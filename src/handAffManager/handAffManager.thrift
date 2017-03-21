# Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
#                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
# Author: Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>,
#         Atabak Dehban <adehban@isr.tecnico.ulisboa.pt>,
#         Pedro Vicente <pvicente@isr.tecnico.ulisboa.pt>,
#         Lorenzo Jamone, Afonso Gonçalves
# CopyPolicy: Released under the terms of the GNU GPL v3.0
#
# handAffManager.thrift

struct Bottle { }
(
yarp.name = "yarp::os::Bottle"
yarp.includefile="yarp/os/Bottle.h"
)

service handAffManager_IDL
{
    /**
     * Set robot fingers to one of the permitted postures: straight, fortyfive,
     * bent. This is done first on the real robot, then in the Unity simulator.
     * @return true/false on success/failure
     */
    bool setHandPosture(1:string posture);

    /**
     * Acquire shape descriptors of the hand from the Unity simulator.
     * @return true/false on success/failure
     */
    bool getHandDescriptors();

    /**
     * Provide positive user response to a program request for information.
     * @return true/false on success/failure
     */
    bool yes();

    /**
     * Provide negative user response to a program request for information.
     * @return true/false on success/failure
     */
    bool no();

    /**
     * Get the 3D position of the "best" object currently seen by segmentation,
     * determined according these criteria: (i) sufficiently large area,
     * (ii) closest to robot.
     * @return Bottle containing 3D offset
     */
    Bottle getBestObject3D();

    /**
     * Quit the module.
     * @return true/false on success/failure
     */
    bool quit();
}
