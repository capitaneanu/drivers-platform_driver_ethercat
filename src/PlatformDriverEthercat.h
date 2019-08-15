#pragma once

#include <memory>
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
    bool startupDrive(std::string drive_name);

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
    bool shutdownDrive(std::string drive_name);

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
    bool resetDrive(std::string drive_name);

    /**
     * Sends position command for specific drive (PTP Motion).
     */
    void commandDrivePositionRad(std::string drive_name, double position_rad);

    /**
     * Sends velocity command for specific drive.
     */
    void commandDriveVelocityRadSec(std::string drive_name, double velocity_rad_sec);

    /**
     * Sends torque command for specific drive.
     */
    void commandDriveTorqueNm(std::string drive_name, double torque_nm);

    /**
     * Gets the position and velocity of a given drive.
     */
    void readDrivePositionRad(std::string drive_name, double& position_rad);

    /**
     * Gets the velocity of a given drive.
     */
    void readDriveVelocityRadSec(std::string drive_name, double& velocity_rad_sec);

    /**
     * Gets the motor torque (from active current) of a given drive.
     */
    void readDriveTorqueNm(std::string drive_name, double& torque_nm);

    bool readDriveData(std::string drive_name,
                       double& position_rad,
                       double& velocity_rad_sec,
                       double& current_amp,
                       double& torque_nm);

    void readDriveAnalogInputV(std::string drive_name, double& analog_input_v);

    void readFtsForceN(std::string fts_name, double& fx, double& fy, double& fz);

    void readFtsTorqueNm(std::string fts_name, double& tx, double& ty, double& tz);

  private:
    bool applyConfiguration(DriveSlaveMapping drive_mapping, FtsSlaveMapping fts_mapping);

    std::map<std::string, std::shared_ptr<CanDriveTwitter>> can_drives_;
    std::map<std::string, std::shared_ptr<CanDeviceAtiFts>> can_fts_;

    CanOverEthercat can_interface_;
};
}
