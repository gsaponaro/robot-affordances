# Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
#                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
# Author: Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>,
# CopyPolicy: Released under the terms of the GNU GPL v3.0
#
# robotHandProcessor.thrift

service robotHandProcessor_IDL
{
    /**
     * Quit the module.
     * @return true/false on success/failure
     */
    bool quit();

    /**
     * Command the simulated head so that it gazes at a target.
     * @param target string containing the name of the target (e.g. right_hand)
     * @return true/false on success/failure
     */
    bool look(1:string target);

    /**
     * Reset arm kinematics to real joint values.
     * @return true/false on success/failure
     */
    bool resetKinematics();

    /**
     * Get the current value of an arm joint.
     * @param joint the index of the joint (0..15)
     * @return true/false on success/failure
     */
    double getPos(1:i32 joint);

    /**
     * Set an arm joint to a value.
     * @param joint the index of the joint (0..15)
     * @param value desired value in degrees
     * @return true/false on success/failure
     */
    bool setPos(1:i32 joint, 2:double value);
}
