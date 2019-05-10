//* general includes
#include <cstdlib>
#include <future>
#include <iostream>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <sys/time.h>
#include <tuple>
#include <vector>

#include "CanDeviceAtiFts.h"
#include "CanDriveTwitter.h"
#include "CanOverEthercat.h"
#include "Platform_Driver.h"

Platform_Driver::Platform_Driver(unsigned int num_motors, unsigned int num_nodes, unsigned int can_dev_type, std::string can_dev_addr, unsigned int watchdog)
{	
    _num_motors = num_motors;
    _num_nodes = num_nodes;
	_can_address = can_dev_addr;
    _can_interface = new CanOverEthercat(_can_address);
}

Platform_Driver::~Platform_Driver()
{
	if (_can_interface != NULL)
	{
		delete _can_interface;
	}

	for(auto drive : _can_drives)
	{
		if (drive != NULL)
		{
			delete drive;
		}
	}	

	std::cout << "Platform_Driver::~Platform_Driver: exiting the destructor... " << std::endl;
}

bool Platform_Driver::readConfiguration(GearMotorParamType wheel_drive_params, GearMotorParamType steer_drive_params, GearMotorParamType walk_drive_params, GearMotorParamType pan_drive_params, GearMotorParamType tilt_drive_params, GearMotorParamType arm_drive_params, PltfCanParams can_params)
{
	if (can_params.CanId.size() != _num_nodes
        || can_params.Name.size() != _num_nodes
        || can_params.Type.size() != _num_nodes
        || can_params.Active.size() != _num_nodes)
	{
		std::cout << "Platform_Driver::ReadConfiguration: The size of the can parameter vectors (CanId, Name, Type, Active) do not match the stated number of nodes" <<std::endl;
		return false;
	}

    _can_parameters = can_params;

	for (unsigned int i = 0; i < _num_nodes; i++)
	{
		//* set Motor parameters depending on the type of motor
		if (_can_parameters.Type[i] == WHEEL_DRIVE)
		{
    		//* add new Twitter
            CanDriveTwitter *drive = new CanDriveTwitter(_can_interface, _can_parameters.CanId[i], _can_parameters.Name[i]);

			drive->getDriveParam()->setParam(
					wheel_drive_params.iEncIncrPerRevMot,
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

		    _can_drives.push_back(drive);
            _can_interface->addDevice(drive);
		}
		else if (_can_parameters.Type[i] == WHEEL_STEER)
		{
    		//* add new Twitter
            CanDriveTwitter *drive = new CanDriveTwitter(_can_interface, _can_parameters.CanId[i], _can_parameters.Name[i]);

			drive->getDriveParam()->setParam(
					steer_drive_params.iEncIncrPerRevMot,
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

		    _can_drives.push_back(drive);
            _can_interface->addDevice(drive);
		}
		else if (_can_parameters.Type[i] == WHEEL_WALK)
		{
    		//* add new Twitter
            CanDriveTwitter *drive = new CanDriveTwitter(_can_interface, _can_parameters.CanId[i], _can_parameters.Name[i]);

			drive->getDriveParam()->setParam(
					walk_drive_params.iEncIncrPerRevMot,
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

		    _can_drives.push_back(drive);
            _can_interface->addDevice(drive);
		}
		else if (_can_parameters.Type[i] == MANIP_JOINT)
		{
    		//* add new Twitter
            CanDriveTwitter *drive = new CanDriveTwitter(_can_interface, _can_parameters.CanId[i], _can_parameters.Name[i]);

			drive->getDriveParam()->setParam(
					arm_drive_params.iEncIncrPerRevMot,
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

		    _can_drives.push_back(drive);
            _can_interface->addDevice(drive);
		}
		else if (_can_parameters.Type[i] == MAST_PAN)
		{
    		//* add new Twitter
            CanDriveTwitter *drive = new CanDriveTwitter(_can_interface, _can_parameters.CanId[i], _can_parameters.Name[i]);

			drive->getDriveParam()->setParam(
					pan_drive_params.iEncIncrPerRevMot,
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
            
		    _can_drives.push_back(drive);
            _can_interface->addDevice(drive);
		}
		else if (_can_parameters.Type[i] == MAST_TILT)
		{
    		//* add new Twitter
            CanDriveTwitter *drive = new CanDriveTwitter(_can_interface, _can_parameters.CanId[i], _can_parameters.Name[i]);

			drive->getDriveParam()->setParam(
					tilt_drive_params.iEncIncrPerRevMot,
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

		    _can_drives.push_back(drive);
            _can_interface->addDevice(drive);
		}
        else if (_can_parameters.Type[i] == FT_SENSOR)
        {
            CanDeviceAtiFts *device = new CanDeviceAtiFts(_can_interface, _can_parameters.CanId[i], _can_parameters.Name[i]);
            _can_fts.push_back(device);
            _can_interface->addDevice(device);
        }
		else
		{
			std::cout << "Platform_Driver::ReadConfiguration: Unknown type "<< _can_parameters.Type[i] <<" of motor "<< _can_parameters.Name[i] <<std::endl;
			return false;
		}
	}

    std::cout << "Platform_Driver::readConfiguration: Success" <<std::endl;

	return true;
}

bool Platform_Driver::initPltf(GearMotorParamType wheel_drive_params, GearMotorParamType steer_drive_params, GearMotorParamType walk_drive_params, GearMotorParamType pan_drive_params, GearMotorParamType tilt_drive_params, GearMotorParamType arm_drive_params, PltfCanParams can_params)
{	
	//* Platform configuration. CAN interface and CAN nodes setup.
	if(!readConfiguration(wheel_drive_params, steer_drive_params, walk_drive_params, pan_drive_params, tilt_drive_params, arm_drive_params, can_params))
	{
		std::cout << "Platform_Driver::initPltf: Error in readConfiguration call" << std::endl;
		return false;
	}
	
    std::cout << "Platform_Driver::initPltf: Initializing EtherCAT interface" << std::endl;

    if (!_can_interface->init())
    {
        std::cout << "Platform_Driver::initPltf: Could not initialize EtherCAT interface" << std::endl;
        return false;
    }

	//* Start all drives in groups of 6
    unsigned int i = 0;

    while (i < _num_motors)
	{
        std::vector<std::tuple<CanDriveTwitter *, std::future<bool>>> future_tuples;

        unsigned int j = 0;

        while (j < 6)
        {
            if (can_params.Active[i])
            {
                 CanDriveTwitter* drive = _can_drives[i];
                 std::cout << "Platform_Driver::initPltf: Starting drive " << drive->getDeviceName() << std::endl;

                 auto future = std::async(std::launch::async, &CanDriveTwitter::startup, drive);
                 auto tuple = std::make_tuple(drive, std::move(future));

                 future_tuples.push_back(std::move(tuple));

                 j++;
            }

            i++;

            if (i >= _num_motors)
            {
                break;
            }
        }

        for (auto & future_tuple : future_tuples)
        {
            CanDriveTwitter *drive = std::get<0>(future_tuple);
            std::future<bool> future = std::move(std::get<1>(future_tuple));

            if (future.get())
            {
                std::cout << "Platform_Driver::initPltf: Drive " << drive->getDeviceName() << " started" << std::endl;
            }
            else
            {
                std::cout << "Platform_Driver::initPltf: Startup of drive " << drive->getDeviceName() << " failed" << std::endl;
                return false;
            }
        }
	}

    std::cout << "Platform_Driver::initPltf: Platform init success" << std::endl;
    return true;
}

bool Platform_Driver::shutdownPltf()
{
	bool bRet = true;
	//* shut down all motors
	for(auto drive : _can_drives)
	{
		bRet &= drive->shutdown();
	}
	return bRet;
}

bool Platform_Driver::shutdownNode(unsigned int drive_id)
{
	bool bRet = true;
	//* shut down the motor
	bRet &= _can_drives[drive_id]->shutdown();
	return bRet;
}

bool Platform_Driver::startNode(unsigned int drive_id)
{
	bool bRet = true;
	//* start up the motor
	bRet &= _can_drives[drive_id]->startup();
	return bRet;
}

bool Platform_Driver::resetPltf()
{
	bool bRetMotor = true;
	bool bRet = true;

	for(auto drive : _can_drives)
	{
		bRetMotor = drive->reset();

		if (!bRetMotor)
		{
			std::cout << "Resetting of Motor " << drive->getDeviceName() << " failed" << std::endl;
		}

        bRet &= bRetMotor;
	}

	return bRet;
}

bool Platform_Driver::resetNode(unsigned int drive_id)
{
    auto drive = _can_drives[drive_id];
    bool bRet = drive->reset();

    if (!bRet)
    {
        std::cout << "Resetting of Motor " << drive->getDeviceName() << " failed" << std::endl;
    }

	return bRet;
}

void Platform_Driver::nodePositionCommandRad(unsigned int drive_id, double dPosGearRad)
{		
	_can_drives[drive_id]->commandPositionRad(dPosGearRad);
}

void Platform_Driver::nodeVelocityCommandRadS(unsigned int drive_id, double dVelGearRadS)
{
	_can_drives[drive_id]->commandVelocityRadSec(dVelGearRadS);
}

void Platform_Driver::nodeTorqueCommandNm(unsigned int drive_id, double dTorqueNm)
{
	_can_drives[drive_id]->commandTorqueNm(dTorqueNm);
}

void Platform_Driver::getNodePositionRad(unsigned int drive_id, double* pdAngleGearRad)
{
	*pdAngleGearRad = _can_drives[drive_id]->readPositionRad();
}

void Platform_Driver::getNodeVelocityRadS(unsigned int drive_id, double* pdVelocityRadS)
{
	*pdVelocityRadS = _can_drives[drive_id]->readVelocityRadSec();
}

void Platform_Driver::getNodeTorqueNm(unsigned int drive_id, double* pdTorqueNm)
{
	*pdTorqueNm = _can_drives[drive_id]->readTorqueNm();
}

bool Platform_Driver::getNodeData(unsigned int drive_id, double* pdAngleGearRad, double* pdVelGearRadS, double* pdCurrentAmp, double* pdTorqueNm)
{
	if (!_can_drives[drive_id]->isError())
	{
	    *pdAngleGearRad = _can_drives[drive_id]->readPositionRad();
	    *pdVelGearRadS = _can_drives[drive_id]->readVelocityRadSec();
        // TODO: add current output
	    *pdTorqueNm = _can_drives[drive_id]->readTorqueNm();

		return true;
	}

	return false;
}

void Platform_Driver::getNodeAnalogInput(unsigned int drive_id, double* pdAnalogInput)
{
	*pdAnalogInput = _can_drives[drive_id]->readAnalogInput();
}

void Platform_Driver::getNodeFtsForceN(unsigned int fts_id, double *fx, double *fy, double *fz)
{
    CanDeviceAtiFts::Force force = _can_fts[fts_id]->readForceN();

    *fx = force.fx;
    *fy = force.fy;
    *fz = force.fz;
}

void Platform_Driver::getNodeFtsTorqueNm(unsigned int fts_id, double *tx, double *ty, double *tz)
{
    CanDeviceAtiFts::Torque torque = _can_fts[fts_id]->readTorqueNm();

    *tx = torque.tx;
    *ty = torque.ty;
    *tz = torque.tz;
}
