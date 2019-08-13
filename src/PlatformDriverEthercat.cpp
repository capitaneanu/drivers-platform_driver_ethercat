#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <sys/time.h>
#include <Eigen/Dense>
#include <cstdlib>
#include <future>
#include <iostream>
#include <tuple>
#include <vector>

#include "CanDeviceAtiFts.h"
#include "CanDriveTwitter.h"
#include "CanOverEthercat.h"
#include "PlatformDriverEthercat.h"

#include <base-logging/Logging.hpp>

using namespace platform_driver_ethercat;

PlatformDriverEthercat::PlatformDriverEthercat(std::string dev_address,
                                               unsigned int num_slaves,
                                               DriveSlaveMapping drive_mapping,
                                               FtsSlaveMapping fts_mapping)
    : can_interface_(new CanOverEthercat(dev_address, num_slaves))
{
    applyConfiguration(drive_mapping, fts_mapping);
}

PlatformDriverEthercat::~PlatformDriverEthercat()
{
    if (can_interface_ != NULL)
    {
        delete can_interface_;
    }

    for (auto drive : can_drives_)
    {
        if (drive != NULL)
        {
            delete drive;
        }
    }

    for (auto fts : can_fts_)
    {
        if (fts != NULL)
        {
            delete fts;
        }
    }
}

bool PlatformDriverEthercat::applyConfiguration(DriveSlaveMapping drive_mapping,
                                                FtsSlaveMapping fts_mapping)
{
    for (auto drive_params : drive_mapping)
    {
        CanDriveTwitter* drive = new CanDriveTwitter(can_interface_,
                                                     drive_params.slave_id,
                                                     drive_params.name,
                                                     drive_params.config,
                                                     drive_params.enabled);
        can_drives_.push_back(drive);
        can_interface_->addDevice(drive);
    }

    for (auto fts_params : fts_mapping)
    {
        CanDeviceAtiFts* device =
            new CanDeviceAtiFts(can_interface_, fts_params.slave_id, fts_params.name);
        can_fts_.push_back(device);
        can_interface_->addDevice(device);
    }

    LOG_DEBUG_S << __PRETTY_FUNCTION__ << ": Success" << std::endl;

    return true;
}

bool PlatformDriverEthercat::initPlatform()
{
    LOG_DEBUG_S << __PRETTY_FUNCTION__ << ": Initializing EtherCAT interface";

    if (!can_interface_->init())
    {
        LOG_ERROR_S << __PRETTY_FUNCTION__ << ": Could not initialize EtherCAT interface";
        return false;
    }

    if (!startupPlatform())
    {
        LOG_ERROR_S << __PRETTY_FUNCTION__ << ": Could not start up drives";
        return false;
    }

    LOG_DEBUG_S << __PRETTY_FUNCTION__ << ": Platform init success";
    return true;
}

bool PlatformDriverEthercat::startupPlatform()
{
    // Start all drives in groups of 6
    unsigned int i = 0;

    while (i < can_drives_.size())
    {
        std::vector<std::tuple<CanDriveTwitter*, std::future<bool>>> future_tuples;

        unsigned int j = 0;

        while (j < 6)
        {
            CanDriveTwitter* drive = can_drives_[i];

            if (drive->isEnabled())
            {
                LOG_DEBUG_S << __PRETTY_FUNCTION__ << ": Starting drive " << drive->getDeviceName();

                auto future = std::async(std::launch::async, &CanDriveTwitter::startup, drive);
                auto tuple = std::make_tuple(drive, std::move(future));

                future_tuples.push_back(std::move(tuple));

                j++;
            }

            i++;

            if (i >= can_drives_.size())
            {
                break;
            }
        }

        for (auto& future_tuple : future_tuples)
        {
            CanDriveTwitter* drive = std::get<0>(future_tuple);
            std::future<bool> future = std::move(std::get<1>(future_tuple));

            if (future.get())
            {
                LOG_DEBUG_S << __PRETTY_FUNCTION__ << ": Drive " << drive->getDeviceName()
                            << " started";
            }
            else
            {
                LOG_DEBUG_S << __PRETTY_FUNCTION__ << ": Startup of drive "
                            << drive->getDeviceName() << " failed";
                return false;
            }
        }
    }

    return true;
}

bool PlatformDriverEthercat::startupDrive(unsigned int drive_id)
{
    bool bRet = true;
    // start up the motor
    bRet &= can_drives_[drive_id]->startup();
    return bRet;
}

bool PlatformDriverEthercat::shutdownPlatform()
{
    bool bRet = true;
    // shut down all motors
    for (auto drive : can_drives_)
    {
        bRet &= drive->shutdown();
    }
    return bRet;
}

bool PlatformDriverEthercat::shutdownDrive(unsigned int drive_id)
{
    bool bRet = true;
    // shut down the motor
    bRet &= can_drives_[drive_id]->shutdown();
    return bRet;
}

bool PlatformDriverEthercat::resetPlatform()
{
    bool bRetMotor = true;
    bool bRet = true;

    for (auto drive : can_drives_)
    {
        bRetMotor = drive->reset();

        if (!bRetMotor)
        {
            LOG_ERROR_S << "Resetting of Motor " << drive->getDeviceName() << " failed";
        }

        bRet &= bRetMotor;
    }

    return bRet;
}

bool PlatformDriverEthercat::resetDrive(unsigned int drive_id)
{
    auto drive = can_drives_[drive_id];
    bool bRet = drive->reset();

    if (!bRet)
    {
        LOG_ERROR_S << "Resetting of Motor " << drive->getDeviceName() << " failed";
    }

    return bRet;
}

void PlatformDriverEthercat::commandDrivePositionRad(unsigned int drive_id, double position_rad)
{
    can_drives_[drive_id]->commandPositionRad(position_rad);
}

void PlatformDriverEthercat::commandDriveVelocityRadSec(unsigned int drive_id,
                                                        double velocity_rad_sec)
{
    can_drives_[drive_id]->commandVelocityRadSec(velocity_rad_sec);
}

void PlatformDriverEthercat::commandDriveTorqueNm(unsigned int drive_id, double torque_nm)
{
    can_drives_[drive_id]->commandTorqueNm(torque_nm);
}

void PlatformDriverEthercat::readDrivePositionRad(unsigned int drive_id, double& position_rad)
{
    position_rad = can_drives_[drive_id]->readPositionRad();
}

void PlatformDriverEthercat::readDriveVelocityRadSec(unsigned int drive_id,
                                                     double& velocity_rad_sec)
{
    velocity_rad_sec = can_drives_[drive_id]->readVelocityRadSec();
}

void PlatformDriverEthercat::readDriveTorqueNm(unsigned int drive_id, double& torque_nm)
{
    torque_nm = can_drives_[drive_id]->readTorqueNm();
}

bool PlatformDriverEthercat::readDriveData(unsigned int drive_id,
                                           double& position_rad,
                                           double& velocity_rad_sec,
                                           double& current_amp,
                                           double& torque_nm)
{
    if (!can_drives_[drive_id]->isError())
    {
        position_rad = can_drives_[drive_id]->readPositionRad();
        velocity_rad_sec = can_drives_[drive_id]->readVelocityRadSec();
        // TODO: Read out current
        torque_nm = can_drives_[drive_id]->readTorqueNm();

        return true;
    }

    return false;
}

void PlatformDriverEthercat::readDriveAnalogInputV(unsigned int drive_id, double& analog_input_v)
{
    analog_input_v = can_drives_[drive_id]->readAnalogInputV();
}

void PlatformDriverEthercat::readFtsForceN(unsigned int fts_id, double& fx, double& fy, double& fz)
{
    Eigen::Vector3d force = can_fts_[fts_id]->readForceN();

    fx = force[0];
    fy = force[1];
    fz = force[2];
}

void PlatformDriverEthercat::readFtsTorqueNm(unsigned int fts_id,
                                             double& tx,
                                             double& ty,
                                             double& tz)
{
    Eigen::Vector3d torque = can_fts_[fts_id]->readTorqueNm();

    tx = torque[0];
    ty = torque[1];
    tz = torque[2];
}
