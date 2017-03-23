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
    yarp.includefile = "yarp/os/Bottle.h"
)

service handAffManager_IDL
{
    /**
     * Set the robot hand to one of the available postures (straight, fortyfive,
     * bent), then acquire provisional information about the robot hand: shape
     * descriptors and image from the Unity simulator. This information then
     * needs to be verified by the user before saving to disk.
     * @param posture the name of the posture: straight, fortyfive, bent
     * @return string containing the next interactive instruction for the user
     */
    string getHand(1:string posture);

    /**
     * Acquire provisional information about the target object (shape
     * descriptors and image) from the real robot perception. This information
     * then needs to be verified by the user before saving to disk.
     * @param objName the name of the target object
     * @return string containing the next interactive instruction for the user
     */
    string getObject(1:string objName);

    /**
     * Ask the real robot to perform a specific motor actions (tapFromLeft,
     * tapFromRight, push, draw) with one of the permitted postures (straight,
     * fortyfive, bent) on a target object (the "best" one currently seen by
     * segmentation), and record effects. These effects then need to be verified
     * by the user before saving to disk.
     * @param string containing the desired action: tapFromLeft,
     * tapFromRight, push, draw
     * @param posture the name of the posture: straight, fortyfive, bent
     * @param objName the name of the target object
     * @return string containing the next interactive instruction for the user
     */
    string startEffect(1:string action, 2:string posture, 3:string objName);

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
     * Get the 2D position of the "best" object currently seen by segmentation,
     * determined according to these criteria: (i) sufficiently large area,
     * (ii) closest to robot.
     * @return Bottle containing 2D coordinates u v
     */
    Bottle getBestObject2D();

    /**
     * Get the 3D position of the "best" object currently seen by segmentation,
     * determined according to these criteria: (i) sufficiently large area,
     * (ii) closest to robot.
     * @return Bottle containing 3D coordinates x y z
     */
    Bottle getBestObject3D();

    /**
     * Quit the module.
     * @return true/false on success/failure
     */
    bool quit();
}
