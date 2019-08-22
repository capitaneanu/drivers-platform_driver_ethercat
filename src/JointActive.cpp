#include <limits>

#include "CanDriveTwitter.h"
#include "JointActive.h"

using namespace platform_driver_ethercat;

JointActive::JointActive(std::string name, std::shared_ptr<CanDriveTwitter>& drive, bool enabled)
    : Joint(name, drive, enabled){};

bool JointActive::commandPositionRad(const double position_rad)
{
    if (enabled_)
    {
        drive_->commandPositionRad(position_rad);
        return true;
    }
    else
    {
        return false;
    }
}

bool JointActive::commandVelocityRadSec(const double velocity_rad_sec)
{
    if (enabled_)
    {
        drive_->commandVelocityRadSec(velocity_rad_sec);
        return true;
    }
    else
    {
        return false;
    }
}

bool JointActive::commandTorqueNm(const double torque_nm)
{
    if (enabled_)
    {
        drive_->commandTorqueNm(torque_nm);
        return true;
    }
    else
    {
        return false;
    }
}

bool JointActive::readPositionRad(double& position_rad)
{
    if (enabled_)
    {
        position_rad = drive_->readPositionRad();
        return true;
    }
    else
    {
        position_rad = std::numeric_limits<double>::quiet_NaN();
        return false;
    }
}

bool JointActive::readVelocityRadSec(double& velocity_rad_sec)
{
    if (enabled_)
    {
        velocity_rad_sec = drive_->readVelocityRadSec();
        return true;
    }
    else
    {
        velocity_rad_sec = std::numeric_limits<double>::quiet_NaN();
        return false;
    }
}

bool JointActive::readTorqueNm(double& torque_nm)
{
    if (enabled_)
    {
        torque_nm = drive_->readTorqueNm();
        return true;
    }
    else
    {
        torque_nm = std::numeric_limits<double>::quiet_NaN();
        return false;
    }
}

bool JointActive::readTempDegC(double& temp_deg_c)
{
    if (enabled_)
    {
        double Vout = drive_->readAnalogInputV();

        const double alpha = 0.00385;
        const double V0 = 3.3;
        const double R0 = 100;
        const double R1 = 3000;
        const double R2 = 2000;
        const double R3 = 200;

        temp_deg_c = 1.0 / alpha * (V0 - (R0 + R1) / R0 * R3 / (R2 + R3) * Vout)
                     / (-V0 + R3 / (R2 + R3) * Vout);

        return true;
    }
    else
    {
        temp_deg_c = std::numeric_limits<double>::quiet_NaN();
        return false;
    }
}
