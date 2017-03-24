# Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
#                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
# Author: Pedro Vicente <pvicente@isr.tecnico.ulisboa.pt>,
#         Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>
# CopyPolicy: Released under the terms of the GNU GPL v3.0
#
# handActions.thrift

service handActions_IDL
{
    /**
     * Make the robot gaze down.
     * @return true/false on success/failure
     */
    bool look_down();

    /**
     * Move both robot arms, the torso and the head to the home positions.
     * @return true/false on success/failure
     */
    bool homeAll();

    /**
     * Set fingers to one of the permitted postures: straight, fortyfive, bent.
     * @param posture type of fingers bending
     * @return true/false on success/failure
     */
    bool setFingers(1:string posture);

    /**
     * Tap an object from the left. Only one object must be visible.
     * @return true/false on success/failure
     */
    bool tapFromLeft();

    /**
     * Tap an object from the right. Only one object must be visible.
     * @return true/false on success/failure
     */
    bool tapFromRight();

    /**
     * Push an object away from the robot. Only one object must be visible.
     * @return true/false on success/failure
     */
    bool push();

    /**
     * Draw an object closer to the robot. Only one object must be visible.
     * @return true/false on success/failure
     */
    bool draw();

    /**
     * Tap an object from the left, specifying the object coordinates.
     * @param x object 3D x position in meters
     * @param y object 3D y position in meters
     * @param z object 3D z position in meters
     * @return true/false on success/failure
     */
    bool tapFromLeftCoords(1:double x, 2:double y, 3:double z);

    /**
     * Tap an object from the right, specifying the object coordinates.
     * @param x object 3D x position in meters
     * @param y object 3D y position in meters
     * @param z object 3D z position in meters
     * @return true/false on success/failure
     */
    bool tapFromRightCoords(1:double x, 2:double y, 3:double z);

    /**
     * Push an object away from the robot, specifying the object coordinates.
     * @param x object 3D x position in meters
     * @param y object 3D y position in meters
     * @param z object 3D z position in meters
     * @return true/false on success/failure
     */
    bool pushCoords(1:double x, 2:double y, 3:double z);

    /**
     * Draw an object closer to the robot, specifying the object coordinates.
     * @param x object 3D x position in meters
     * @param y object 3D y position in meters
     * @param z object 3D z position in meters
     * @return true/false on success/failure
     */
    bool drawCoords(1:double x, 2:double y, 3:double z);

    /**
     * Quit the module.
     * @return true/false on success/failure
     */
    bool quit();
}
