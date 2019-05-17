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
#include "Platform_Driver.h"

#include <base-logging/Logging.hpp>

Platform_Driver::Platform_Driver(unsigned int num_motors,
                                 unsigned int num_nodes,
                                 unsigned int can_dev_type,
                                 std::string can_dev_addr,
                                 unsigned int watchdog)
{
    num_motors_ = num_motors;
    num_nodes_ = num_nodes;
    can_address_ = can_dev_addr;
    can_interface_ = new CanOverEthercat(can_address_);
}

Platform_Driver::~Platform_Driver()
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
}

bool Platform_Driver::readConfiguration(GearMotorParamType wheel_drive_params,
                                        GearMotorParamType steer_drive_params,
                                        GearMotorParamType walk_drive_params,
                                        GearMotorParamType pan_drive_params,
                                        GearMotorParamType tilt_drive_params,
                                        GearMotorParamType arm_drive_params,
                                        PltfCanParams can_params)
{
    if (can_params.CanId.size() != num_nodes_    //
        || can_params.Name.size() != num_nodes_  //
        || can_params.Type.size() != num_nodes_  //
        || can_params.Active.size() != num_nodes_)
    {
        LOG_ERROR_S << __PRETTY_FUNCTION__
                    << ": The size of the can parameter vectors (CanId, Name, Type, Active) do not "
                       "match the stated number of nodes"
                    << std::endl;
        return false;
    }

    can_parameters_ = can_params;

    for (unsigned int i = 0; i < num_nodes_; i++)
    {
        // set Motor parameters depending on the type of motor
        if (can_parameters_.Type[i] == WHEEL_DRIVE)
        {
            CanDriveTwitter* drive = new CanDriveTwitter(
                can_interface_, can_parameters_.CanId[i], can_parameters_.Name[i]);

            drive->getDriveParam()->setParam(wheel_drive_params.iEncIncrPerRevMot,
                                             wheel_drive_params.dBeltRatio,
                                             wheel_drive_params.dGearRatio,
                                             wheel_drive_params.iSign,
                                             wheel_drive_params.dPosLimitLowIncr,
                                             wheel_drive_params.dPosLimitHighIncr,
                                             wheel_drive_params.dVelMaxEncIncrS,
                                             wheel_drive_params.dPtpVelDefaultIncrS,
                                             wheel_drive_params.dAccIncrS2,
                                             wheel_drive_params.dDecIncrS2,
                                             wheel_drive_params.bIsSteer,
                                             wheel_drive_params.dCurrentToTorque,
                                             wheel_drive_params.dCurrMax,
                                             wheel_drive_params.iEncOffsetIncr,
                                             wheel_drive_params.dAnalogFactor,
                                             wheel_drive_params.dNominalCurrent);

            can_drives_.push_back(drive);
            can_interface_->addDevice(drive);
        }
        else if (can_parameters_.Type[i] == WHEEL_STEER)
        {
            CanDriveTwitter* drive = new CanDriveTwitter(
                can_interface_, can_parameters_.CanId[i], can_parameters_.Name[i]);

            drive->getDriveParam()->setParam(steer_drive_params.iEncIncrPerRevMot,
                                             steer_drive_params.dBeltRatio,
                                             steer_drive_params.dGearRatio,
                                             steer_drive_params.iSign,
                                             steer_drive_params.dPosLimitLowIncr,
                                             steer_drive_params.dPosLimitHighIncr,
                                             steer_drive_params.dVelMaxEncIncrS,
                                             steer_drive_params.dPtpVelDefaultIncrS,
                                             steer_drive_params.dAccIncrS2,
                                             steer_drive_params.dDecIncrS2,
                                             steer_drive_params.bIsSteer,
                                             steer_drive_params.dCurrentToTorque,
                                             steer_drive_params.dCurrMax,
                                             steer_drive_params.iEncOffsetIncr,
                                             steer_drive_params.dAnalogFactor,
                                             steer_drive_params.dNominalCurrent);

            can_drives_.push_back(drive);
            can_interface_->addDevice(drive);
        }
        else if (can_parameters_.Type[i] == WHEEL_WALK)
        {
            CanDriveTwitter* drive = new CanDriveTwitter(
                can_interface_, can_parameters_.CanId[i], can_parameters_.Name[i]);

            drive->getDriveParam()->setParam(walk_drive_params.iEncIncrPerRevMot,
                                             walk_drive_params.dBeltRatio,
                                             walk_drive_params.dGearRatio,
                                             walk_drive_params.iSign,
                                             walk_drive_params.dPosLimitLowIncr,
                                             walk_drive_params.dPosLimitHighIncr,
                                             walk_drive_params.dVelMaxEncIncrS,
                                             walk_drive_params.dPtpVelDefaultIncrS,
                                             walk_drive_params.dAccIncrS2,
                                             walk_drive_params.dDecIncrS2,
                                             walk_drive_params.bIsSteer,
                                             walk_drive_params.dCurrentToTorque,
                                             walk_drive_params.dCurrMax,
                                             walk_drive_params.iEncOffsetIncr,
                                             walk_drive_params.dAnalogFactor,
                                             walk_drive_params.dNominalCurrent);

            can_drives_.push_back(drive);
            can_interface_->addDevice(drive);
        }
        else if (can_parameters_.Type[i] == MANIP_JOINT)
        {
            CanDriveTwitter* drive = new CanDriveTwitter(
                can_interface_, can_parameters_.CanId[i], can_parameters_.Name[i]);

            drive->getDriveParam()->setParam(arm_drive_params.iEncIncrPerRevMot,
                                             arm_drive_params.dBeltRatio,
                                             arm_drive_params.dGearRatio,
                                             arm_drive_params.iSign,
                                             arm_drive_params.dPosLimitLowIncr,
                                             arm_drive_params.dPosLimitHighIncr,
                                             arm_drive_params.dVelMaxEncIncrS,
                                             arm_drive_params.dPtpVelDefaultIncrS,
                                             arm_drive_params.dAccIncrS2,
                                             arm_drive_params.dDecIncrS2,
                                             arm_drive_params.bIsSteer,
                                             arm_drive_params.dCurrentToTorque,
                                             arm_drive_params.dCurrMax,
                                             arm_drive_params.iEncOffsetIncr,
                                             arm_drive_params.dAnalogFactor,
                                             arm_drive_params.dNominalCurrent);

            can_drives_.push_back(drive);
            can_interface_->addDevice(drive);
        }
        else if (can_parameters_.Type[i] == MAST_PAN)
        {
            CanDriveTwitter* drive = new CanDriveTwitter(
                can_interface_, can_parameters_.CanId[i], can_parameters_.Name[i]);

            drive->getDriveParam()->setParam(pan_drive_params.iEncIncrPerRevMot,
                                             pan_drive_params.dBeltRatio,
                                             pan_drive_params.dGearRatio,
                                             pan_drive_params.iSign,
                                             pan_drive_params.dPosLimitLowIncr,
                                             pan_drive_params.dPosLimitHighIncr,
                                             pan_drive_params.dVelMaxEncIncrS,
                                             pan_drive_params.dPtpVelDefaultIncrS,
                                             pan_drive_params.dAccIncrS2,
                                             pan_drive_params.dDecIncrS2,
                                             pan_drive_params.bIsSteer,
                                             pan_drive_params.dCurrentToTorque,
                                             pan_drive_params.dCurrMax,
                                             pan_drive_params.iEncOffsetIncr,
                                             pan_drive_params.dAnalogFactor,
                                             pan_drive_params.dNominalCurrent);

            can_drives_.push_back(drive);
            can_interface_->addDevice(drive);
        }
        else if (can_parameters_.Type[i] == MAST_TILT)
        {
            CanDriveTwitter* drive = new CanDriveTwitter(
                can_interface_, can_parameters_.CanId[i], can_parameters_.Name[i]);

            drive->getDriveParam()->setParam(tilt_drive_params.iEncIncrPerRevMot,
                                             tilt_drive_params.dBeltRatio,
                                             tilt_drive_params.dGearRatio,
                                             tilt_drive_params.iSign,
                                             tilt_drive_params.dPosLimitLowIncr,
                                             tilt_drive_params.dPosLimitHighIncr,
                                             tilt_drive_params.dVelMaxEncIncrS,
                                             tilt_drive_params.dPtpVelDefaultIncrS,
                                             tilt_drive_params.dAccIncrS2,
                                             tilt_drive_params.dDecIncrS2,
                                             tilt_drive_params.bIsSteer,
                                             tilt_drive_params.dCurrentToTorque,
                                             tilt_drive_params.dCurrMax,
                                             tilt_drive_params.iEncOffsetIncr,
                                             tilt_drive_params.dAnalogFactor,
                                             tilt_drive_params.dNominalCurrent);

            can_drives_.push_back(drive);
            can_interface_->addDevice(drive);
        }
        else if (can_parameters_.Type[i] == FT_SENSOR)
        {
            CanDeviceAtiFts* device = new CanDeviceAtiFts(
                can_interface_, can_parameters_.CanId[i], can_parameters_.Name[i]);
            can_fts_.push_back(device);
            can_interface_->addDevice(device);
        }
        else
        {
            LOG_ERROR_S << __PRETTY_FUNCTION__ << ": Unknown type " << can_parameters_.Type[i]
                        << " of motor " << can_parameters_.Name[i] << std::endl;
            return false;
        }
    }

    LOG_DEBUG_S << __PRETTY_FUNCTION__ << ": Success" << std::endl;

    return true;
}

bool Platform_Driver::initPltf(GearMotorParamType wheel_drive_params,
                               GearMotorParamType steer_drive_params,
                               GearMotorParamType walk_drive_params,
                               GearMotorParamType pan_drive_params,
                               GearMotorParamType tilt_drive_params,
                               GearMotorParamType arm_drive_params,
                               PltfCanParams can_params)
{
    //* Platform configuration. CAN interface and CAN nodes setup.
    if (!readConfiguration(wheel_drive_params,
                           steer_drive_params,
                           walk_drive_params,
                           pan_drive_params,
                           tilt_drive_params,
                           arm_drive_params,
                           can_params))
    {
        LOG_ERROR_S << __PRETTY_FUNCTION__ << ": Error in readConfiguration call";
        return false;
    }

    LOG_DEBUG_S << __PRETTY_FUNCTION__ << ": Initializing EtherCAT interface";

    if (!can_interface_->init())
    {
        LOG_ERROR_S << __PRETTY_FUNCTION__ << ": Could not initialize EtherCAT interface";
        return false;
    }

    // Start all drives in groups of 6
    unsigned int i = 0;

    while (i < num_motors_)
    {
        std::vector<std::tuple<CanDriveTwitter*, std::future<bool>>> future_tuples;

        unsigned int j = 0;

        while (j < 6)
        {
            if (can_params.Active[i])
            {
                CanDriveTwitter* drive = can_drives_[i];
                LOG_DEBUG_S << __PRETTY_FUNCTION__ << ": Starting drive " << drive->getDeviceName();

                auto future = std::async(std::launch::async, &CanDriveTwitter::startup, drive);
                auto tuple = std::make_tuple(drive, std::move(future));

                future_tuples.push_back(std::move(tuple));

                j++;
            }

            i++;

            if (i >= num_motors_)
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

    LOG_DEBUG_S << __PRETTY_FUNCTION__ << ": Platform init success";
    return true;
}

bool Platform_Driver::shutdownPltf()
{
    bool bRet = true;
    // shut down all motors
    for (auto drive : can_drives_)
    {
        bRet &= drive->shutdown();
    }
    return bRet;
}

bool Platform_Driver::shutdownNode(unsigned int drive_id)
{
    bool bRet = true;
    // shut down the motor
    bRet &= can_drives_[drive_id]->shutdown();
    return bRet;
}

bool Platform_Driver::startNode(unsigned int drive_id)
{
    bool bRet = true;
    // start up the motor
    bRet &= can_drives_[drive_id]->startup();
    return bRet;
}

bool Platform_Driver::resetPltf()
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

bool Platform_Driver::resetNode(unsigned int drive_id)
{
    auto drive = can_drives_[drive_id];
    bool bRet = drive->reset();

    if (!bRet)
    {
        LOG_ERROR_S << "Resetting of Motor " << drive->getDeviceName() << " failed";
    }

    return bRet;
}

void Platform_Driver::nodePositionCommandRad(unsigned int drive_id, double dPosGearRad)
{
    can_drives_[drive_id]->commandPositionRad(dPosGearRad);
}

void Platform_Driver::nodeVelocityCommandRadS(unsigned int drive_id, double dVelGearRadS)
{
    can_drives_[drive_id]->commandVelocityRadSec(dVelGearRadS);
}

void Platform_Driver::nodeTorqueCommandNm(unsigned int drive_id, double dTorqueNm)
{
    can_drives_[drive_id]->commandTorqueNm(dTorqueNm);
}

void Platform_Driver::getNodePositionRad(unsigned int drive_id, double* pdAngleGearRad)
{
    *pdAngleGearRad = can_drives_[drive_id]->readPositionRad();
}

void Platform_Driver::getNodeVelocityRadS(unsigned int drive_id, double* pdVelocityRadS)
{
    *pdVelocityRadS = can_drives_[drive_id]->readVelocityRadSec();
}

void Platform_Driver::getNodeTorqueNm(unsigned int drive_id, double* pdTorqueNm)
{
    *pdTorqueNm = can_drives_[drive_id]->readTorqueNm();
}

bool Platform_Driver::getNodeData(unsigned int drive_id,
                                  double* pdAngleGearRad,
                                  double* pdVelGearRadS,
                                  double* pdCurrentAmp,
                                  double* pdTorqueNm)
{
    if (!can_drives_[drive_id]->isError())
    {
        *pdAngleGearRad = can_drives_[drive_id]->readPositionRad();
        *pdVelGearRadS = can_drives_[drive_id]->readVelocityRadSec();
        // TODO: add current output
        *pdTorqueNm = can_drives_[drive_id]->readTorqueNm();

        return true;
    }

    return false;
}

void Platform_Driver::getNodeAnalogInput(unsigned int drive_id, double* pdAnalogInput)
{
    *pdAnalogInput = can_drives_[drive_id]->readAnalogInput();
}

void Platform_Driver::getNodeFtsForceN(unsigned int fts_id, double* fx, double* fy, double* fz)
{
    Eigen::Vector3d force = can_fts_[fts_id]->readForceN();

    *fx = force[0];
    *fy = force[1];
    *fz = force[2];
}

void Platform_Driver::getNodeFtsTorqueNm(unsigned int fts_id, double* tx, double* ty, double* tz)
{
    Eigen::Vector3d torque = can_fts_[fts_id]->readTorqueNm();

    *tx = torque[0];
    *ty = torque[1];
    *tz = torque[2];
}
