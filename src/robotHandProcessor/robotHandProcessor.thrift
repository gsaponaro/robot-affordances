# Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
#                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
# Author: Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>,
# CopyPolicy: Released under the terms of the GNU GPL v3.0
#
# robotHandProcessor.thrift

struct Bottle { }
(
    yarp.name = "yarp::os::Bottle"
    yarp.includefile = "yarp/os/Bottle.h"
)

service robotHandProcessor_IDL
{
    /**
     * Command the simulated head so that it gazes at a target.
     * @param target string containing the name of the target (e.g. left_hand)
     * @return true/false on success/failure
     */
    bool look(1:string target);

    /**
     * Reset simulated joints (arm and head) to the real robot ones.
     * @return true/false on success/failure
     */
    bool resetKinematics();

    /**
     * Get the current value of an arm joint.
     * @param joint the index of the joint (0..15)
     * @return true/false on success/failure
     */
    double getArmPos(1:i32 joint);

    /**
     * Get the current values of all the arm joints.
     * @return Bottle list of joint values
     */
    Bottle getArmPoss();

    /**
     * Set an arm joint to a value.
     * @param joint the index of the joint (0..15)
     * @param value desired value in degrees
     * @return true/false on success/failure
     */
    bool setArmPos(1:i32 joint, 2:double value);

    /**
     * Set all the arm joints to a desired vector of values.
     * @param values Bottle of desired values in degrees, enclosed within ( )
     * @return true/false on success/failure
     */
    bool setArmPoss(1:Bottle values);

    /**
     * Get the current value of a head joint.
     * @param joint the index of the joint (0..5)
     * @return true/false on success/failure
     */
    double getHeadPos(1:i32 joint);

    /**
     * Get the current values of all the head joints.
     * @return Bottle list of joint values
     */
    Bottle getHeadPoss();

    /**
     * Set a head joint to a value.
     * @param joint the index of the joint (0..5)
     * @param value desired value in degrees
     * @return true/false on success/failure
     */
    bool setHeadPos(1:i32 joint, 2:double value);

    /**
     * Set all the head joints to a desired vector of values.
     * @param values Bottle of desired values in degrees, enclosed within ( )
     * @return true/false on success/failure
     */
    bool setHeadPoss(1:Bottle values);

    /**
     * Quit the module.
     * @return true/false on success/failure
     */
    bool quit();
}
