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
     * Set fingers to one of the permitted postures: straight, fortyfive,
     * bent.
     * @return true/false on success/failure
     */
    bool setHandPosture(1:string posture);

    /**
     * Acquire shape descriptors of the hand.
     * @return true/false on success/failure
     */
    bool getHandDescriptors();

    /**
     * Positive response by user to request for information.
     * @return true/false on success/failure
     */
    bool yes();

    /**
     * Negative response by user to request for information.
     * @return true/false on success/failure
     */
    bool no();

    /**
     * Get number of objects that are currently found by segmentation.
     * @return int number of visible objects
     */
    i32 getNumVisibleObjects();

    /**
     * Get the 3D position of an object.
     * @return Bottle containing 3D offset
     */
    Bottle getObject3D();

    /**
     * Quit the module.
     * @return true/false on success/failure
     */
    bool quit();
}
