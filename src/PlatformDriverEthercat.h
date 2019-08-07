#pragma once

#include <string>
#include <vector>
#include "CanEnumsAndStructs.h"

namespace platform_driver_ethercat
{

class CanDeviceAtiFts;
class CanDriveTwitter;
class CanOverEthercat;

/**
 * Represents and Controls all Drive components on an arbitrary platform.
 * Drives shall be connected in a CAN Bus network and comply with the CANopen protocol to control
 * different types of motors.
 */
class PlatformDriverEthercat
{
  public:
    /**
     * Default constructor.
     */
    PlatformDriverEthercat(unsigned int num_motors,
                           unsigned int num_nodes,
                           unsigned int can_dev_type,
                           std::string can_dev_addr,
                           unsigned int watchdog);

    /**
     * Default destructor.
     */
    ~PlatformDriverEthercat();

    /**
     * Initializes all CAN nodes of the platform and performs homing procedure of the steered
     * motors. Note: The homing routine is hardware-dependent. This method is now adapted to ExoTer
     * rover configuration. Re-adapt this before using in different platforms.
     * @return True if initialization is successful, false otherwise.
     */
    bool initPltf(GearMotorParamType wheel_drive_params,
                  GearMotorParamType steer_drive_params,
                  GearMotorParamType walk_drive_params,
                  GearMotorParamType pan_drive_params,
                  GearMotorParamType tilt_drive_params,
                  GearMotorParamType arm_drive_params,
                  PltfCanParams can_params);

    /**
     * Sets CAN node and interface configuration and motor parameters for each drive.
     * (should be adapted to use RoCK component properties)
     */
    bool readConfiguration(GearMotorParamType wheel_drive_params,
                           GearMotorParamType steer_drive_params,
                           GearMotorParamType walk_drive_params,
                           GearMotorParamType pan_drive_params,
                           GearMotorParamType tilt_drive_params,
                           GearMotorParamType arm_drive_params,
                           PltfCanParams can_params);

    /**
     * Shuts down the platform.
     * Disables motors, enables brake and disconnects.
     * @return True if all drives are properly shut down.
     */
    bool shutdownPltf();

    /**
     * Shuts down the specific Node.
     * Disables motor, enables brake and disconnects.
     * @return True if the drive is properly shut down.
     */
    bool shutdownNode(unsigned int drive_id);

    /**
     * Starts the specific Node.
     * Enables the motor, check the status.
     * @return True if the drive is properly started.
     */
    bool startNode(unsigned int drive_id);

    /**
     * Reinitializes the nodes on the bus.
     * The function might be necessary after an emergency stop or an hardware failure to re-init
     * drives.
     * @return True if re-initialization is successful, false otherwise.
     */
    bool resetPltf();

    /**
     * Reinitializes the specific node on the bus.
     * The function might be necessary after an emergency stop or an hardware failure to re-init
     * drives.
     * @return True if re-initialization is successful, false otherwise.
     */
    bool resetNode(unsigned int drive_id);

    /**
     * Sends position (and velocity) command for specific can node (PTP Motion).
     * Node must be in position control mode
     * @param drive_id selects the can node
     * @param dPosGearRad position command in radians
     * @param dVelGearRadS velocity command in radian per second, not used in ethercat driver
     */
    void nodePositionCommandRad(unsigned int drive_id, double dPosRad, double dVelRadS = 0);

    /**
     * Sends velocity command for specific can node.
     * Node must be in velocity control mode
     * @param drive_id selects the can node
     * @param dVelGearRadS velocity command in radian per second
     */
    void nodeVelocityCommandRadS(unsigned int drive_id, double dVelRadS);

    /**
     * Sends torque command for specific can node.
     * @param drive_id selects the can node
     * @param dTorqueNM motor-torque in Nm
     */
    void nodeTorqueCommandNm(unsigned int drive_id, double dTorqueNm);

    /**
     * Gets the position and velocity of a given node.
     * @param drive_id selects the can node
     * @param pdAngleGearRad The value (in radians) of the current position of the motor is stored
     * in this pointer.
     */
    void getNodePositionRad(unsigned int drive_id, double* pdPositionRad);

    /**
     * Gets the velocity of a given node.
     * @param drive_id selects the can node
     * @param pdVelGearRadS The value (in radians/s) of the current velocity of the motor is stored
     * in this pointer.
     */
    void getNodeVelocityRadS(unsigned int drive_id, double* pdVelocityRadS);

    /**
     * Gets the motor torque (from active current) of a given node.
     * @param drive_id selects the can node
     * @param pdTorqueNm The value (in Nm) of the current motor torque is stored in this pointer.
     */
    void getNodeTorqueNm(unsigned int drive_id, double* pdTorqueNm);

    bool getNodeData(unsigned int drive_id,
                     double* pdAngleGearRad,
                     double* pdVelGearRadS,
                     double* pdCurrentAmp,
                     double* pdTorqueNm);

    void getNodeAnalogInputV(unsigned int drive_id, double* pdAnalogInputV);

    void getNodeFtsForceN(unsigned int fts_id, double* fx, double* fy, double* fz);

    void getNodeFtsTorqueNm(unsigned int fts_id, double* tx, double* ty, double* tz);

  protected:
    // Address of the can device interface in the system
    std::string can_address_;
    // CAN interface device class object (PeakSysUSB)
    CanOverEthercat* can_interface_;
    // Motor controllers. Pointer to each motor's CanDrive-Itf
    std::vector<CanDriveTwitter*> can_drives_;
    std::vector<CanDeviceAtiFts*> can_fts_;

    // Array of CanNodeTypes. Keeps information of all Node IDs and high level description
    PltfCanParams can_parameters_;
    unsigned int num_motors_;
    unsigned int num_nodes_;
};

}
