#pragma once

#include <string>
#include <vector>
#include "CanOverEthercat.h"
#include "PlatformDriverEthercatTypes.h"

namespace platform_driver_ethercat
{

class CanDeviceAtiFts;
class CanDriveTwitter;

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
    PlatformDriverEthercat(std::string can_address,
                           unsigned int num_slaves,
                           DriveSlaveMapping drive_mapping,
                           FtsSlaveMapping fts_mapping);

    /**
     * Default destructor.
     */
    ~PlatformDriverEthercat();

    /**
     * Initializes the ethercat interface and starts up the drives.
     * @return True if initialization is successful, false otherwise.
     */
    bool initPlatform();

    /**
     * Starts up the platform.
     * Enables the motors
     * @return True if all drive are properly started.
     */
    bool startupPlatform();

    /**
     * Starts up the specific drive.
     * Enables the motor, check the status.
     * @return True if the drive is properly started.
     */
    bool startupDrive(unsigned int drive_id);

    /**
     * Shuts down the platform.
     * Disables motors, enables brake and disconnects.
     * @return True if all drives are properly shut down.
     */
    bool shutdownPlatform();

    /**
     * Shuts down the specific Node.
     * Disables motor, enables brake and disconnects.
     * @return True if the drive is properly shut down.
     */
    bool shutdownDrive(unsigned int drive_id);

    /**
     * Reinitializes the nodes on the bus.
     * The function might be necessary after an emergency stop or an hardware failure to re-init
     * drives.
     * @return True if re-initialization is successful, false otherwise.
     */
    bool resetPlatform();

    /**
     * Reinitializes the specific node on the bus.
     * The function might be necessary after an emergency stop or an hardware failure to re-init
     * drives.
     * @return True if re-initialization is successful, false otherwise.
     */
    bool resetDrive(unsigned int drive_id);

    /**
     * Sends position command for specific can node (PTP Motion).
     */
    void commandDrivePositionRad(unsigned int drive_id, double position_rad);

    /**
     * Sends velocity command for specific drive.
     */
    void commandDriveVelocityRadSec(unsigned int drive_id, double velocity_rad_sec);

    /**
     * Sends torque command for specific drive.
     */
    void commandDriveTorqueNm(unsigned int drive_id, double torque_nm);

    /**
     * Gets the position and velocity of a given drive.
     */
    void readDrivePositionRad(unsigned int drive_id, double& position_rad);

    /**
     * Gets the velocity of a given node.
     */
    void readDriveVelocityRadSec(unsigned int drive_id, double& velocity_rad_sec);

    /**
     * Gets the motor torque (from active current) of a given node.
     */
    void readDriveTorqueNm(unsigned int drive_id, double& torque_nm);

    bool readDriveData(unsigned int drive_id,
                       double& position_rad,
                       double& velocity_rad_sec,
                       double& current_amp,
                       double& torque_nm);

    void readDriveAnalogInputV(unsigned int drive_id, double& analog_input_v);

    void readFtsForceN(unsigned int fts_id, double& fx, double& fy, double& fz);

    void readFtsTorqueNm(unsigned int fts_id, double& tx, double& ty, double& tz);

  private:
    bool applyConfiguration(DriveSlaveMapping drive_mapping, FtsSlaveMapping fts_mapping);

    CanOverEthercat can_interface_;
    std::vector<CanDriveTwitter*> can_drives_;
    std::vector<CanDeviceAtiFts*> can_fts_;
};
}
