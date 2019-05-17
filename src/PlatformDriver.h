#pragma once

#include <string>
#include <vector>
#include "CanEnumsAndStructs.h"

/**
 * Represents and Controls all Drive components on an arbitrary platform.
 * Drives shall be connected in a CAN Bus network and comply with the CANopen protocol to control
 * different types of motors.
 */
class PlatformDriver
{
  public:
    virtual ~PlatformDriver(){};
    /**
     * Initializes all CAN nodes of the platform and performs homing procedure of the steered
     * motors. Note: The homing routine is hardware-dependent. This method is now adapted to ExoTer
     * rover configuration. Re-adapt this before using in different platforms.
     * @return True if initialization is successful, false otherwise.
     */
    virtual bool initPltf(GearMotorParamType wheel_drive_params,
                          GearMotorParamType steer_drive_params,
                          GearMotorParamType walk_drive_params,
                          GearMotorParamType pan_drive_params,
                          GearMotorParamType tilt_drive_params,
                          GearMotorParamType arm_drive_params,
                          PltfCanParams can_params) = 0;

    /**
     * Sets CAN node and interface configuration and motor parameters for each drive.
     * (should be adapted to use RoCK component properties)
     */
    virtual bool readConfiguration(GearMotorParamType wheel_drive_params,
                                   GearMotorParamType steer_drive_params,
                                   GearMotorParamType walk_drive_params,
                                   GearMotorParamType pan_drive_params,
                                   GearMotorParamType tilt_drive_params,
                                   GearMotorParamType arm_drive_params,
                                   PltfCanParams can_params) = 0;

    /**
     * Shuts down the platform.
     * Disables motors, enables brake and disconnects.
     * @return True if all drives are properly shut down.
     */
    virtual bool shutdownPltf() = 0;

    /**
     * Shuts down the specific Node.
     * Disables motor, enables brake and disconnects.
     * @return True if the drive is properly shut down.
     */
    virtual bool shutdownNode(unsigned int drive_id) = 0;

    /**
     * Sends position (and velocity) command for specific can node (PTP Motion).
     * Node must be in position control mode
     * @param can_id selects the can node
     * @param pos_rad position command in radians
     * @param vel_rad_s velocity command in radian per second
     */
    virtual void nodePositionCommandRad(unsigned int can_id, double pos_rad, double vel_rad_s=0) = 0;

    /**
     * Starts the specific Node.
     * Enables the motor, check the status.
     * @return True if the drive is properly started.
     */
    virtual bool startNode(unsigned int drive_id) = 0;

    /**
     * Reinitializes the nodes on the bus.
     * The function might be necessary after an emergency stop or an hardware failure to re-init
     * drives.
     * @return True if re-initialization is successful, false otherwise.
     */
    virtual bool resetPltf() = 0;

    /**
     * Reinitializes the specific node on the bus.
     * The function might be necessary after an emergency stop or an hardware failure to re-init
     * drives.
     * @return True if re-initialization is successful, false otherwise.
     */
    virtual bool resetNode(unsigned int drive_id) = 0;

    /**
     * Sends velocity command for specific can node.
     * Node must be in velocity control mode
     * @param drive_id selects the can node
     * @param dVelGearRadS velocity command in radian per second
     */
    virtual void nodeVelocityCommandRadS(unsigned int drive_id, double dVelRadS) = 0;

    /**
     * Sends torque command for specific can node.
     * @param drive_id selects the can node
     * @param dTorqueNM motor-torque in Nm
     */
    virtual void nodeTorqueCommandNm(unsigned int drive_id, double dTorqueNm) = 0;

    /**
     * Gets the velocity of a given node.
     * @param drive_id selects the can node
     * @param pdVelGearRadS The value (in radians/s) of the current velocity of the motor is stored
     * in this pointer.
     */
    virtual void getNodeVelocityRadS(unsigned int drive_id, double* pdVelocityRadS) = 0;

    virtual bool getNodeData(unsigned int drive_id,
                             double* pdAngleGearRad,
                             double* pdVelGearRadS,
                             double* pdCurrentAmp,
                             double* pdTorqueNm) = 0;

    virtual void getNodeAnalogInput(unsigned int drive_id, double* pdAnalogInput) = 0;
};
