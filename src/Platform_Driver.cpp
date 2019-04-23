//* general includes
#include <math.h>
#include <vector>
#include <signal.h>
#include <sys/time.h>
#include <pthread.h>
#include <cstdlib>
#include <iostream>

#include "Platform_Driver.h"

Platform_Driver::Platform_Driver(int num_motors, int num_nodes, int can_dev_type, std::string can_dev_addr, int watchdog)
{	

	m_iNumMotors = num_motors;
	m_iNumNodes = num_nodes;
	m_iCanItfType = can_dev_type;
	_can_address = can_dev_addr;
    m_pCanCtrl = new CanOverEthercat(_can_address);
}

Platform_Driver::~Platform_Driver()
{
	if (m_pCanCtrl != NULL)
	{
		delete m_pCanCtrl;
	}

	for(unsigned int i = 0; i < m_vpMotor.size(); i++)
	{
		if (m_vpMotor[i] != NULL)
		{
			delete m_vpMotor[i];
		}
	}	

	std::cout << "Platform_Driver::~Platform_Driver: exiting the destructor... " << std::endl;
}

bool Platform_Driver::readConfiguration(GearMotorParamType wheel_drive_params, GearMotorParamType steer_drive_params, GearMotorParamType walk_drive_params, GearMotorParamType pan_drive_params, GearMotorParamType tilt_drive_params, GearMotorParamType arm_drive_params, PltfCanParams can_params)
{
    m_vCanNodeIDs = can_params;

	if (static_cast<unsigned int>(m_iNumNodes) != m_vCanNodeIDs.CanId.size())
	{
		std::cout << "Platform_Driver::ReadConfiguration: The number of motors does not match the size of CAN Node IDs array" <<std::endl;
		return false;
	}

	for (int i = 0; i < m_iNumNodes; i++)
	{
		//* add new Twitter
        CanDriveTwitter *drive = new CanDriveTwitter(m_pCanCtrl, m_vCanNodeIDs.CanId[i], m_vCanNodeIDs.Name[i]);

		//* set Motor parameters depending on the type of motor
		if (m_vCanNodeIDs.Type[i] == WHEEL_DRIVE)
		{
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
		}
		else if (m_vCanNodeIDs.Type[i] == WHEEL_STEER)
		{
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
		}
		else if (m_vCanNodeIDs.Type[i] == WHEEL_WALK)
		{
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
		}
		else if (m_vCanNodeIDs.Type[i] == MANIP_JOINT)
		{
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
		}
		else if (m_vCanNodeIDs.Type[i] == MAST_PAN)
		{
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
		}
		else if (m_vCanNodeIDs.Type[i] == MAST_TILT)
		{
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
		}
		else
		{
			std::cout << "Platform_Driver::ReadConfiguration: Unknown type "<< m_vCanNodeIDs.Type[i] <<" of motor "<< m_vCanNodeIDs.Name[i] <<std::endl;
			return false;
		}

		m_vpMotor.push_back(drive);
        m_pCanCtrl->addDrive(drive);
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

    if (!m_pCanCtrl->init())
    {
        std::cout << "Platform_Driver::initPltf: Could not initialize EtherCAT interface" << std::endl;
        return false;
    }

	//* Start all motors
	for (int i = 0; i < m_iNumMotors; i++)
	{
		if (can_params.Active[i])
		{
		    std::cout << "Platform_Driver::initPltf: Starting drive "<< m_vCanNodeIDs.Name[i] << std::endl;

			if (!m_vpMotor[i]->startup())
			{
				std::cout << "Platform_Driver::initPltf: Startup of drive " << m_vCanNodeIDs.Name[i] << " failed" << std::endl;
				return false;
			}

            //m_vpMotor[i]->commandVelocityRadSec(0.1);
            m_vpMotor[i]->commandPositionRad(0.0);
		}
	}

    std::cout << "Platform_Driver::initPltf: Platform init success" << std::endl;
    return true;
}

bool Platform_Driver::shutdownPltf()
{
	bool bRet = true;
	//* shut down all motors
	for(int i = 0; i < m_iNumMotors; i++)
	{
		bRet &= m_vpMotor[i]->shutdown();
	}
	return bRet;
}

bool Platform_Driver::shutdownNode(int iCanIdent)
{
	bool bRet = true;
	//* shut down the motor
	bRet &= m_vpMotor[iCanIdent]->shutdown();
	return bRet;
}

bool Platform_Driver::startNode(int iCanIdent)
{
	bool bRet = true;
	//* start up the motor
	bRet &= m_vpMotor[iCanIdent]->startup();
	return bRet;
}

bool Platform_Driver::resetPltf()
{
	bool bRetMotor = true;
	bool bRet = true;

	for(int i = 0; i < m_iNumMotors; i++)
	{
		bRetMotor = m_vpMotor[i]->reset();

		if (!bRetMotor)
		{
			std::cout << "Resetting of Motor " << m_vCanNodeIDs.Name[i] << " failed" << std::endl;
		}

        bRet &= bRetMotor;
	}

	return bRet;
}

bool Platform_Driver::resetNode(int iCanIdent)
{
	bool bRet = true;
	bRet = m_vpMotor[iCanIdent]->reset();

    if (!bRet)
    {
        std::cout << "Resetting of Motor " << m_vCanNodeIDs.Name[iCanIdent] << " failed" << std::endl;
    }

	return bRet;
}

void Platform_Driver::nodePositionCommandRad(int iCanIdent, double dPosGearRad)
{		
	m_vpMotor[iCanIdent]->commandPositionRad(dPosGearRad);
}

void Platform_Driver::nodeVelocityCommandRadS(int iCanIdent, double dVelGearRadS)
{
	m_vpMotor[iCanIdent]->commandVelocityRadSec(dVelGearRadS);
}

void Platform_Driver::nodeTorqueCommandNm(int iCanIdent, double dTorqueNm)
{
	m_vpMotor[iCanIdent]->commandTorqueNm(dTorqueNm);
}

void Platform_Driver::getNodePositionRad(int iCanIdent, double* pdAngleGearRad)
{
	*pdAngleGearRad = m_vpMotor[iCanIdent]->readPositionRad();
}

void Platform_Driver::getNodeVelocityRadS(int iCanIdent, double* pdVelocityRadS)
{
	*pdVelocityRadS = m_vpMotor[iCanIdent]->readVelocityRadSec();
}

void Platform_Driver::getNodeTorque(int iCanIdent, double* pdTorqueNm)
{
	*pdTorqueNm = m_vpMotor[iCanIdent]->readTorqueNm();
}

bool Platform_Driver::getNodeData(int iCanIdent,double* pdAngleGearRad, double* pdVelGearRadS, double* pdCurrentAmp, double* pdTorqueNm)
{
	if (!m_vpMotor[iCanIdent]->isError())
	{
	    *pdAngleGearRad = m_vpMotor[iCanIdent]->readPositionRad();
	    *pdVelGearRadS = m_vpMotor[iCanIdent]->readVelocityRadSec();
        // TODO: add current output
	    *pdTorqueNm = m_vpMotor[iCanIdent]->readTorqueNm();

		return true;
	}

	return false;
}

void Platform_Driver::getNodeAnalogInput(int iCanIdent,double* pdAnalogInput)
{
	*pdAnalogInput = m_vpMotor[iCanIdent]->readAnalogInput();
}

int Platform_Driver::getNumMotors()
{
	return m_iNumMotors;
}
