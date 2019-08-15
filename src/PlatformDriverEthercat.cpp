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
    : can_interface_(dev_address, num_slaves)
{
    applyConfiguration(drive_mapping, fts_mapping);
}

PlatformDriverEthercat::~PlatformDriverEthercat() {}

bool PlatformDriverEthercat::applyConfiguration(DriveSlaveMapping drive_mapping,
                                                FtsSlaveMapping fts_mapping)
{
    for (const auto& drive_params : drive_mapping)
    {
        auto drive = std::make_shared<CanDriveTwitter>(can_interface_,
                                                       drive_params.slave_id,
                                                       drive_params.name,
                                                       drive_params.config,
                                                       drive_params.enabled);
        can_drives_.insert(std::make_pair(drive->getDeviceName(), drive));
        can_interface_.addDevice(drive);
    }

    for (const auto& fts_params : fts_mapping)
    {
        auto device =
            std::make_shared<CanDeviceAtiFts>(can_interface_, fts_params.slave_id, fts_params.name);
        can_fts_.insert(std::make_pair(device->getDeviceName(), device));
        can_interface_.addDevice(device);
    }

    LOG_DEBUG_S << __PRETTY_FUNCTION__ << ": Success" << std::endl;

    return true;
}

bool PlatformDriverEthercat::initPlatform()
{
    LOG_DEBUG_S << __PRETTY_FUNCTION__ << ": Initializing EtherCAT interface";

    if (!can_interface_.init())
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

    auto drive_iterator = can_drives_.begin();
    while (drive_iterator != can_drives_.end())
    {
        std::vector<std::tuple<CanDriveTwitter&, std::future<bool>>> future_tuples;

        unsigned int j = 0;

        while (j < 6)
        {
            CanDriveTwitter& drive = *drive_iterator->second;

            if (drive.isEnabled())
            {
                LOG_DEBUG_S << __PRETTY_FUNCTION__ << ": Starting drive " << drive.getDeviceName();

                auto future = std::async(std::launch::async, &CanDriveTwitter::startup, drive);
                auto tuple =
                    std::tuple<CanDriveTwitter&, std::future<bool>>(drive, std::move(future));

                future_tuples.push_back(std::move(tuple));

                j++;
            }

            drive_iterator++;

            if (drive_iterator == can_drives_.end())
            {
                break;
            }
        }

        for (auto& future_tuple : future_tuples)
        {
            CanDriveTwitter& drive = std::get<0>(future_tuple);
            std::future<bool> future = std::move(std::get<1>(future_tuple));

            if (future.get())
            {
                LOG_DEBUG_S << __PRETTY_FUNCTION__ << ": Drive " << drive.getDeviceName()
                            << " started";
            }
            else
            {
                LOG_DEBUG_S << __PRETTY_FUNCTION__ << ": Startup of drive " << drive.getDeviceName()
                            << " failed";
                return false;
            }
        }
    }

    return true;
}

bool PlatformDriverEthercat::startupDrive(std::string drive_name)
{
    bool bRet = true;
    // start up the motor
    bRet &= can_drives_[drive_name]->startup();
    return bRet;
}

bool PlatformDriverEthercat::shutdownPlatform()
{
    bool bRet = true;
    // shut down all motors
    for (auto& drive : can_drives_)
    {
        bRet &= drive.second->shutdown();
    }
    return bRet;
}

bool PlatformDriverEthercat::shutdownDrive(std::string drive_name)
{
    bool bRet = true;
    // shut down the motor
    bRet &= can_drives_[drive_name]->shutdown();
    return bRet;
}

bool PlatformDriverEthercat::resetPlatform()
{
    bool bRetMotor = true;
    bool bRet = true;

    for (auto& drive : can_drives_)
    {
        bRetMotor = drive.second->reset();

        if (!bRetMotor)
        {
            LOG_ERROR_S << "Resetting of Motor " << drive.second->getDeviceName() << " failed";
        }

        bRet &= bRetMotor;
    }

    return bRet;
}

bool PlatformDriverEthercat::resetDrive(std::string drive_name)
{
    auto& drive = can_drives_[drive_name];

    bool bRet = drive->reset();

    if (!bRet)
    {
        LOG_ERROR_S << "Resetting of Motor " << drive->getDeviceName() << " failed";
    }

    return bRet;
}

void PlatformDriverEthercat::commandDrivePositionRad(std::string drive_name, double position_rad)
{
    can_drives_[drive_name]->commandPositionRad(position_rad);
}

void PlatformDriverEthercat::commandDriveVelocityRadSec(std::string drive_name,
                                                        double velocity_rad_sec)
{
    can_drives_[drive_name]->commandVelocityRadSec(velocity_rad_sec);
}

void PlatformDriverEthercat::commandDriveTorqueNm(std::string drive_name, double torque_nm)
{
    can_drives_[drive_name]->commandTorqueNm(torque_nm);
}

void PlatformDriverEthercat::readDrivePositionRad(std::string drive_name, double& position_rad)
{
    position_rad = can_drives_[drive_name]->readPositionRad();
}

void PlatformDriverEthercat::readDriveVelocityRadSec(std::string drive_name,
                                                     double& velocity_rad_sec)
{
    velocity_rad_sec = can_drives_[drive_name]->readVelocityRadSec();
}

void PlatformDriverEthercat::readDriveTorqueNm(std::string drive_name, double& torque_nm)
{
    torque_nm = can_drives_[drive_name]->readTorqueNm();
}

bool PlatformDriverEthercat::readDriveData(std::string drive_name,
                                           double& position_rad,
                                           double& velocity_rad_sec,
                                           double& current_amp,
                                           double& torque_nm)
{
    auto& drive = can_drives_[drive_name];

    if (!drive->isError())
    {
        position_rad = drive->readPositionRad();
        velocity_rad_sec = drive->readVelocityRadSec();
        // TODO: Read out current
        torque_nm = drive->readTorqueNm();

        return true;
    }

    return false;
}

void PlatformDriverEthercat::readDriveAnalogInputV(std::string drive_name, double& analog_input_v)
{
    analog_input_v = can_drives_[drive_name]->readAnalogInputV();
}

void PlatformDriverEthercat::readFtsForceN(std::string fts_name, double& fx, double& fy, double& fz)
{
    Eigen::Vector3d force = can_fts_[fts_name]->readForceN();

    fx = force[0];
    fy = force[1];
    fz = force[2];
}

void PlatformDriverEthercat::readFtsTorqueNm(std::string fts_name,
                                             double& tx,
                                             double& ty,
                                             double& tz)
{
    Eigen::Vector3d torque = can_fts_[fts_name]->readTorqueNm();

    tx = torque[0];
    ty = torque[1];
    tz = torque[2];
}
