# Copyright: (C) 2017 VisLab, Institute for Systems and Robotics,
#                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
# Author: Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>,
#         Atabak Dehban <adehban@isr.tecnico.ulisboa.pt>,
#         Lorenzo Jamone, Afonso Gonçalves
# CopyPolicy: Released under the terms of the GNU GPL v3.0
#
# handAffManager.thrift

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
     * Quit the module.
     * @return true/false on success/failure
     */
    bool quit();
}
