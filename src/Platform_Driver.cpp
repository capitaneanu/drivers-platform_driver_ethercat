/****************************************************************
 *
 * Copyright (c) 2013
 *
 * European Space Technology and Research Center
 * ESTEC - European Space Agency
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: ROBS
 * stack name: MotionControl
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Martin Azkarate, email:martin.azkarate@esa.int
 * Supervised by: Pantelis Poulakis, email:pantelis.poulakis@esa.int
 *
 * Date of creation: Jan 2013
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering	
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_drivers
 * ROS package name: cob_base_drive_chain
 * Description: This is a sample implementation of a can-bus with several nodes. In this case it implements the drive-chain of the Care-O-bot3 mobile base. yet, this can be used as template for using the generic_can and canopen_motor packages to implement arbitrary can-setups.
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Christian Connette, email:christian.connette@ipa.fhg.de
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
 *
 * Date of creation: Feb 2010
 * ToDo: - Check whether motor status request in "setVelGearRadS" "setMotorTorque" make sense (maybe remove in "CanDriveHarmonica").
 *		 - move implementational details (can cmds) of configureElmoRecorder to CanDriveHarmonica (check whether CanDriveItf has to be adapted then)
 *		 - Check: what is the iRecordingGap, what is its unit
 *		 - Remove Mutex.h search for a Boost lib
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *	 * Redistributions of source code must retain the above copyright
 *	   notice, this list of conditions and the following disclaimer.
 *	 * Redistributions in binary form must reproduce the above copyright
 *	   notice, this list of conditions and the following disclaimer in the
 *	   documentation and/or other materials provided with the distribution.
 *	 * Neither the name of the Fraunhofer Institute for Manufacturing 
 *	   Engineering and Automation (IPA) nor the names of its
 *	   contributors may be used to endorse or promote products derived from
 *	   this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public 
 * License LGPL along with this program. 
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

//* general includes
#include <math.h>
#include <vector>
#include <signal.h>
#include <sys/time.h>
#include <pthread.h>
#include <cstdlib>

//* Headers provided by other packages inside the library
#include "CanPeakSysUSB.h"
#include "CanDriveItf.h"
#include "Platform_Driver.h"

const double PI = 4*atan(1.0);								/**< PI constant variable */

//* The following variables are defined outside the class because they are used by the Thread and Timer handler methods.

CanItf* m_pCanCtrl;											/**< CAN interface device class object (PeakSysUSB) */
std::vector<CanDriveItf*> m_vpMotor;						/**< Motor controllers. Pointer to each motor's CanDrive-Itf */
PltfCanParams m_vCanNodeIDs;								/**< Array of CanNodeTypes. Keeps information of all Node IDs and high level description */

bool m_bExit;												/**< Exit execution flag */
bool m_bWatchdogActive;										/**< Watchdog active flag */
int m_iTimerDiv;											/**< Counter used to select which motors to monitor at each timer loop */

//-----------------------------------------------

Platform_Driver::Platform_Driver(int num_motors,int num_nodes,int can_dev_type,std::string can_dev_addr, int watchdog)
{	

	m_iNumMotors = num_motors;
	m_iNumNodes = num_nodes;
	m_iCanItfType = can_dev_type;
	can_address=can_dev_addr;
	m_pCanCtrl = NULL;
	//m_DrivingMode = STOPPED;
	m_vpMotor.resize(m_iNumNodes);
	for(int i=0; i<num_nodes; i++)
	{
		m_vpMotor[i] = NULL;
	}
	m_vCanNodeIDs.CanId.resize(num_nodes);
	m_vCanNodeIDs.Name.resize(num_nodes);
	m_vCanNodeIDs.Type.resize(num_nodes);	
	m_vCanNodeIDs.Active.resize(num_nodes);
	m_bExit=false;
	m_bWatchdogActive = watchdog;
	rc = 0;
	/*
	//! Generic Rover Manoeuvre parameters
	double MyInitParam[] = {
		6, 									// number of wheels

		1,1,1,1,1,1,						// position of walking wheels (1 -> walk, 0 -> no walk)
		1,1,0,0,1,1,						// position of steering wheels (1 -> steer, 0 -> no steer)
		1,1,1,1,1,1,						// position of driving wheels (1 -> drive, 0 -> no drive)

		.075,.075,.075,.075,.075,.075,		// radius of the wheels

		0,									// indicate type of coordinates for wheel: 0 CARTESIAN, 1 POLAR, 2 BOTH
		.265,.305,							// wheel Cartesian coordinate FL
		.265,-.305,							// FR
		0,.305,								// CL
		0,-.305,							// CR
		-.265,.305,							// RL
		-.265,-.305,						// RR

		.120,.120,.120,.120,.120,.120,		// leg length

		0,									// indicate type of coordinates for walk: 0 CARTESIAN, 1 POLAR, 2 BOTH
		.265,.305,							// wheel Cartesian coordinate FL
		.265,-.305,							// FR
		0,.305,								// CL
		0,-.305,							// CR
		-.265,.305,							// RL
		-.265,-.305,						// RR

	};

	RoverInit( &MyRover, MyInitParam, sizeof(MyInitParam) / sizeof(MyInitParam[0]) );
	*/
}

//-----------------------------------------------
Platform_Driver::~Platform_Driver()
{
	void *status;
	m_bExit=true;

        struct itimerval timer;
	timer.it_interval.tv_sec = 0;
  	timer.it_interval.tv_usec = 0;
  	timer.it_value.tv_sec = 0;
  	timer.it_value.tv_usec = 0;
  	setitimer(ITIMER_REAL, &timer,0);
	signal(SIGALRM, SIG_DFL);
	//* wait until the thread will be finished
	rc = pthread_join(msg_task, &status);

    if (rc)
    {
    	std::cout << "Platform_Driver::~Platform_Driver: ERROR in thread joining. Return code is " << rc << std::endl;
    	exit(-1);
    }
    std::cout << "Platform_Driver::~Platform_Driver: Completed thread joining. Return code is " << rc << " and status is " << (long)status << std::endl;

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

void Platform_Driver::configureTimer()
{
	m_iTimerDiv=0;
	struct itimerval timer;

	timer.it_interval.tv_sec = TIMER_INTERVAL_SEC;
  	timer.it_interval.tv_usec = TIMER_INTERVAL_USEC;
  	timer.it_value.tv_sec = TIMER_INTERVAL_SEC;
  	timer.it_value.tv_usec = TIMER_INTERVAL_USEC;
  	setitimer(ITIMER_REAL, &timer,0);

	signal(SIGALRM, timer_handler);
}

//-----------------------------------------------
bool Platform_Driver::readConfiguration(GearMotorParamType wheel_drive, GearMotorParamType steer_drive, GearMotorParamType walk_drive, GearMotorParamType pan_drive, GearMotorParamType tilt_drive, GearMotorParamType arm_joint, PltfCanParams can_params)
{
	if (m_iCanItfType == CanItf::CAN_PEAK_USB)
	{
		m_pCanCtrl = new CANPeakSysUSB(can_address);
		std::cout << "Uses CAN-Peak-USB" << std::endl;
	}
	else
	{
		std::cout << "Wrong or unknown Type of CAN interface" << std::endl;
		return false;
	}

	/**
	 * Motor Parameters setup for each type of motors. Adapt to your specific HW!
	 * this configuration could (or should) be done through RoCK properties and configuration files.
	 */
	
	m_GearMotWheelDrive = wheel_drive;
	m_GearMotWheelSteer = steer_drive;
	m_GearMotWheelWalk = walk_drive;
	m_GearMotManipJoint = arm_joint;
	m_GearMotMastPan = pan_drive;
	m_GearMotMastTilt = tilt_drive;
	m_vCanNodeIDs = can_params;

	/*
	/// Motor Parameters for Wheel Drive Motors
	m_GearMotWheelDrive.iEncIncrPerRevMot=512;
	m_GearMotWheelDrive.dBeltRatio=1;
	m_GearMotWheelDrive.dGearRatio=675;
	m_GearMotWheelDrive.iSign=1;
	m_GearMotWheelDrive.dPosLimitLowIncr = -100000000;
	m_GearMotWheelDrive.dPosLimitHighIncr = 100000000;
	m_GearMotWheelDrive.dVelMaxEncIncrS=75000;
	m_GearMotWheelDrive.dPtpVelDefaultIncrS=50000;
	m_GearMotWheelDrive.dAccIncrS2=100000;
	m_GearMotWheelDrive.dDecIncrS2=100000;
	m_GearMotWheelDrive.bIsSteer=0;
	m_GearMotWheelDrive.dCurrentToTorque=0;
	m_GearMotWheelDrive.dCurrMax=2;
	m_GearMotWheelDrive.iEncOffsetIncr=0;

	/// Motor Parameters for Wheel Steer Motors
	m_GearMotWheelSteer.iEncIncrPerRevMot=512;
	m_GearMotWheelSteer.dBeltRatio=1;
	m_GearMotWheelSteer.dGearRatio=1900;
	m_GearMotWheelSteer.iSign=1;
	m_GearMotWheelSteer.dPosLimitLowIncr = -115111;
	m_GearMotWheelSteer.dPosLimitHighIncr = 115111;
	m_GearMotWheelSteer.dVelMaxEncIncrS=75000;
	m_GearMotWheelSteer.dPtpVelDefaultIncrS=50000;
	m_GearMotWheelSteer.dAccIncrS2=100000;
	m_GearMotWheelSteer.dDecIncrS2=100000;
	m_GearMotWheelSteer.bIsSteer=1;
	m_GearMotWheelSteer.dCurrentToTorque=0;
	m_GearMotWheelSteer.dCurrMax=1;
	m_GearMotWheelSteer.iEncOffsetIncr=0;

	/// Motor Parameters for Wheel Walk Motors
	m_GearMotWheelWalk.iEncIncrPerRevMot=512;
	m_GearMotWheelWalk.dBeltRatio=1;
	m_GearMotWheelWalk.dGearRatio=1900;
	m_GearMotWheelWalk.iSign=1;
	m_GearMotWheelWalk.dPosLimitLowIncr = -250222;
	m_GearMotWheelWalk.dPosLimitHighIncr = 250222;
	m_GearMotWheelWalk.dVelMaxEncIncrS=75000;
	m_GearMotWheelWalk.dPtpVelDefaultIncrS=50000;
	m_GearMotWheelWalk.dAccIncrS2=100000;
	m_GearMotWheelWalk.dDecIncrS2=100000;
	m_GearMotWheelWalk.bIsSteer=1;
	m_GearMotWheelWalk.dCurrentToTorque=0;
	m_GearMotWheelWalk.dCurrMax=2;
	m_GearMotWheelWalk.iEncOffsetIncr=0;

	/// Motor Parameters for Manipulator Joint Motors
	m_GearMotManipJoint.iEncIncrPerRevMot=2000;
	m_GearMotManipJoint.dBeltRatio=1;
	m_GearMotManipJoint.dGearRatio=100;
	m_GearMotManipJoint.iSign=1;
	m_GearMotManipJoint.dPosLimitLowIncr = 1000000000;
	m_GearMotManipJoint.dPosLimitHighIncr = -1000000000;
	m_GearMotManipJoint.dVelMaxEncIncrS=100000;
	m_GearMotManipJoint.dPtpVelDefaultIncrS=75000;
	m_GearMotManipJoint.dAccIncrS2=10000;
	m_GearMotManipJoint.dDecIncrS2=10000;
	m_GearMotManipJoint.bIsSteer=1;
	m_GearMotManipJoint.dCurrentToTorque=0;
	m_GearMotManipJoint.dCurrMax=36;
	m_GearMotManipJoint.iEncOffsetIncr=0;

	/// Motor Parameters for Pan and Tilt Unit Motors
	m_GearMotPTU.iEncIncrPerRevMot=2000;
	m_GearMotPTU.dBeltRatio=1;
	m_GearMotPTU.dGearRatio=100;
	m_GearMotPTU.iSign=1;
	m_GearMotPTU.dPosLimitLowIncr = -1000000000;
	m_GearMotPTU.dPosLimitHighIncr = 1000000000;
	m_GearMotPTU.dVelMaxEncIncrS=100000;
	m_GearMotPTU.dPtpVelDefaultIncrS=75000;
	m_GearMotPTU.dAccIncrS2=10000;
	m_GearMotPTU.dDecIncrS2=10000;
	m_GearMotPTU.bIsSteer=1;
	m_GearMotPTU.dCurrentToTorque=0;
	m_GearMotPTU.dCurrMax=36;
	m_GearMotPTU.iEncOffsetIncr=0;
	*/

	/**
	 * Manually build the CAN Node IDs array of structures. Put the data that corresponds to your specific HW!
	 * This configuration could (or should) be done through RoCK properties and configuration files.
	 */
	
	//int iMotorCounter=0;
	//int iGroupCounter=0;

	// m_vCanNodeIDs[CANNODE_DRIVE_STEER_MOTOR] = {127, "SINGLE_MOTOR_TEST", WHEEL_STEER}; iMotorCounter++;
	// create also groups of node to address several drives at the same time. Group ID has to be set at each node in advance
	// m_vCanNodeIDs[CANNODE_WHEEL_DRIVE_GROUP] = {126, "SINGLE_MOTOR_TEST", WHEEL_DRIVE}; iGroupCounter++;

	/*
	m_vCanNodeIDs.CanId[CANNODE_WHEEL_DRIVE_FL]=23;
	m_vCanNodeIDs.Type[CANNODE_WHEEL_DRIVE_FL]=WHEEL_DRIVE;
	m_vCanNodeIDs.Name[CANNODE_WHEEL_DRIVE_FL]="WHEEL_DRIVE_FL";
	m_vCanNodeIDs.CanId[CANNODE_WHEEL_DRIVE_FR]=22;
	m_vCanNodeIDs.Type[CANNODE_WHEEL_DRIVE_FR]=WHEEL_DRIVE;
	m_vCanNodeIDs.Name[CANNODE_WHEEL_DRIVE_FR]="WHEEL_DRIVE_FR";
	m_vCanNodeIDs.CanId[CANNODE_WHEEL_DRIVE_CL]=19;
	m_vCanNodeIDs.Type[CANNODE_WHEEL_DRIVE_CL]=WHEEL_DRIVE;
	m_vCanNodeIDs.Name[CANNODE_WHEEL_DRIVE_CL]="WHEEL_DRIVE_CL";
	m_vCanNodeIDs.CanId[CANNODE_WHEEL_DRIVE_CR]=18;
	m_vCanNodeIDs.Type[CANNODE_WHEEL_DRIVE_CR]=WHEEL_DRIVE;
	m_vCanNodeIDs.Name[CANNODE_WHEEL_DRIVE_CR]="WHEEL_DRIVE_CR";
	m_vCanNodeIDs.CanId[CANNODE_WHEEL_DRIVE_BL]=11;
	m_vCanNodeIDs.Type[CANNODE_WHEEL_DRIVE_BL]=WHEEL_DRIVE;
	m_vCanNodeIDs.Name[CANNODE_WHEEL_DRIVE_BL]="WHEEL_DRIVE_BL";
	m_vCanNodeIDs.CanId[CANNODE_WHEEL_DRIVE_BR]=10;
	m_vCanNodeIDs.Type[CANNODE_WHEEL_DRIVE_BR]=WHEEL_DRIVE;
	m_vCanNodeIDs.Name[CANNODE_WHEEL_DRIVE_BR]="WHEEL_DRIVE_BR";
	m_vCanNodeIDs.CanId[CANNODE_WHEEL_STEER_FL]=17;
	m_vCanNodeIDs.Type[CANNODE_WHEEL_STEER_FL]=WHEEL_STEER;
	m_vCanNodeIDs.Name[CANNODE_WHEEL_STEER_FL]="WHEEL_STEER_FL";
	m_vCanNodeIDs.CanId[CANNODE_WHEEL_STEER_FR]=15;
	m_vCanNodeIDs.Type[CANNODE_WHEEL_STEER_FR]=WHEEL_STEER;
	m_vCanNodeIDs.Name[CANNODE_WHEEL_STEER_FR]="WHEEL_STEER_FR";
	m_vCanNodeIDs.CanId[CANNODE_WHEEL_STEER_BL]=16;
	m_vCanNodeIDs.Type[CANNODE_WHEEL_STEER_BL]=WHEEL_STEER;
	m_vCanNodeIDs.Name[CANNODE_WHEEL_STEER_BL]="WHEEL_STEER_BL";
	m_vCanNodeIDs.CanId[CANNODE_WHEEL_STEER_BR]=14;
	m_vCanNodeIDs.Type[CANNODE_WHEEL_STEER_BR]=WHEEL_STEER;
	m_vCanNodeIDs.Name[CANNODE_WHEEL_STEER_BR]="WHEEL_STEER_BR";
	m_vCanNodeIDs.CanId[CANNODE_WHEEL_WALK_FL]=25;
	m_vCanNodeIDs.Type[CANNODE_WHEEL_WALK_FL]=WHEEL_WALK;
	m_vCanNodeIDs.Name[CANNODE_WHEEL_WALK_FL]="WHEEL_WALK_FL";
	m_vCanNodeIDs.CanId[CANNODE_WHEEL_WALK_FR]=24;
	m_vCanNodeIDs.Type[CANNODE_WHEEL_WALK_FR]=WHEEL_WALK;
	m_vCanNodeIDs.Name[CANNODE_WHEEL_WALK_FR]="WHEEL_WALK_FR";
	m_vCanNodeIDs.CanId[CANNODE_WHEEL_WALK_CL]=21;
	m_vCanNodeIDs.Type[CANNODE_WHEEL_WALK_CL]=WHEEL_WALK;
	m_vCanNodeIDs.Name[CANNODE_WHEEL_WALK_CL]="WHEEL_WALK_CL";
	m_vCanNodeIDs.CanId[CANNODE_WHEEL_WALK_CR]=20;
	m_vCanNodeIDs.Type[CANNODE_WHEEL_WALK_CR]=WHEEL_WALK;
	m_vCanNodeIDs.Name[CANNODE_WHEEL_WALK_CR]="WHEEL_WALK_CR";
	m_vCanNodeIDs.CanId[CANNODE_WHEEL_WALK_BL]=13;
	m_vCanNodeIDs.Type[CANNODE_WHEEL_WALK_BL]=WHEEL_WALK;
	m_vCanNodeIDs.Name[CANNODE_WHEEL_WALK_BL]="WHEEL_WALK_BL";
	m_vCanNodeIDs.CanId[CANNODE_WHEEL_WALK_BR]=12;
	m_vCanNodeIDs.Type[CANNODE_WHEEL_WALK_BR]=WHEEL_WALK;
	m_vCanNodeIDs.Name[CANNODE_WHEEL_WALK_BR]="WHEEL_WALK_BR";
	m_vCanNodeIDs.CanId[CANNODE_WHEEL_DRIVE_GROUP]=31;
	m_vCanNodeIDs.Type[CANNODE_WHEEL_DRIVE_GROUP]=WHEEL_DRIVE;
	m_vCanNodeIDs.Name[CANNODE_WHEEL_DRIVE_GROUP]="WHEEL_DRIVE_GROUP";
	m_vCanNodeIDs.CanId[CANNODE_WHEEL_STEER_GROUP]=32;
	m_vCanNodeIDs.Type[CANNODE_WHEEL_STEER_GROUP]=WHEEL_STEER;
	m_vCanNodeIDs.Name[CANNODE_WHEEL_STEER_GROUP]="WHEEL_STEER_GROUP";
	m_vCanNodeIDs.CanId[CANNODE_WHEEL_WALK_GROUP]=33;
	m_vCanNodeIDs.Type[CANNODE_WHEEL_WALK_GROUP]=WHEEL_WALK;
	m_vCanNodeIDs.Name[CANNODE_WHEEL_WALK_GROUP]="WHEEL_WALK_GROUP";
	*/

/*
	m_vCanNodeIDs[CANNODE_WHEEL_DRIVE_FL] = {23, "WHEEL_DRIVE_FL", WHEEL_DRIVE}; iMotorCounter++;
	m_vCanNodeIDs[CANNODE_WHEEL_DRIVE_FR] = {22, "WHEEL_DRIVE_FR", WHEEL_DRIVE}; iMotorCounter++;
	m_vCanNodeIDs[CANNODE_WHEEL_DRIVE_CL] = {19, "WHEEL_DRIVE_CL", WHEEL_DRIVE}; iMotorCounter++;
	m_vCanNodeIDs[CANNODE_WHEEL_DRIVE_CR] = {18, "WHEEL_DRIVE_CR", WHEEL_DRIVE}; iMotorCounter++;
	m_vCanNodeIDs[CANNODE_WHEEL_DRIVE_BL] = {11, "WHEEL_DRIVE_BL", WHEEL_DRIVE}; iMotorCounter++;
	m_vCanNodeIDs[CANNODE_WHEEL_DRIVE_BR] = {10, "WHEEL_DRIVE_BR", WHEEL_DRIVE}; iMotorCounter++;


	m_vCanNodeIDs[CANNODE_WHEEL_STEER_FL] = {17, "WHEEL_STEER_FL", WHEEL_STEER}; iMotorCounter++;
	m_vCanNodeIDs[CANNODE_WHEEL_STEER_FR] = {15, "WHEEL_STEER_FR", WHEEL_STEER}; iMotorCounter++;
	m_vCanNodeIDs[CANNODE_WHEEL_STEER_BL] = {16, "WHEEL_STEER_BL", WHEEL_STEER}; iMotorCounter++;
	m_vCanNodeIDs[CANNODE_WHEEL_STEER_BR] = {14, "WHEEL_STEER_BR", WHEEL_STEER}; iMotorCounter++;

	m_vCanNodeIDs[CANNODE_WHEEL_WALK_FL] = {25, "WHEEL_WALK_FL", WHEEL_WALK}; iMotorCounter++;
	m_vCanNodeIDs[CANNODE_WHEEL_WALK_FR] = {24, "WHEEL_WALK_FR", WHEEL_WALK}; iMotorCounter++;
	m_vCanNodeIDs[CANNODE_WHEEL_WALK_CL] = {21, "WHEEL_WALK_CL", WHEEL_WALK}; iMotorCounter++;
	m_vCanNodeIDs[CANNODE_WHEEL_WALK_CR] = {20, "WHEEL_WALK_CR", WHEEL_WALK}; iMotorCounter++;
	m_vCanNodeIDs[CANNODE_WHEEL_WALK_BL] = {13, "WHEEL_WALK_BL", WHEEL_WALK}; iMotorCounter++;
	m_vCanNodeIDs[CANNODE_WHEEL_WALK_BR] = {12, "WHEEL_WALK_BR", WHEEL_WALK}; iMotorCounter++;

	m_vCanNodeIDs[CANNODE_WHEEL_DRIVE_GROUP] = {31, "WHEEL_DRIVE_GROUP", WHEEL_DRIVE}; iGroupCounter++;
	m_vCanNodeIDs[CANNODE_WHEEL_STEER_GROUP] = {32, "WHEEL_STEER_GROUP", WHEEL_STEER}; iGroupCounter++;
	m_vCanNodeIDs[CANNODE_WHEEL_WALK_GROUP] = {33, "WHEEL_WALK_GROUP", WHEEL_WALK}; iGroupCounter++;

*/

	/*
	m_vCanNodeIDs[CANNODE_MANIP_JOINT_1] = {30, "MANIP_JOINT_1", MANIP_JOINT}; iMotorCounter++;
	m_vCanNodeIDs[CANNODE_MANIP_JOINT_2] = {31, "MANIP_JOINT_2", MANIP_JOINT}; iMotorCounter++;
	m_vCanNodeIDs[CANNODE_MANIP_JOINT_3] = {32, "MANIP_JOINT_3", MANIP_JOINT}; iMotorCounter++;
	m_vCanNodeIDs[CANNODE_MANIP_JOINT_4] = {33, "MANIP_JOINT_4", MANIP_JOINT}; iMotorCounter++;
	m_vCanNodeIDs[CANNODE_MANIP_JOINT_5] = {34, "MANIP_JOINT_5", MANIP_JOINT}; iMotorCounter++;

	m_vCanNodeIDs[CANNODE_MAST_PTU_PAN] = {35, "MAST_PTU_PAN", MAST_PTU}; iMotorCounter++;
	m_vCanNodeIDs[CANNODE_MAST_PTU_TILT] = {36, "MAST_PTU_TILT", MAST_PTU}; iMotorCounter++;

	m_vCanNodeIDs[CANNODE_WHEEL_DRIVE_GROUP] = {41, "WHEEL_DRIVE_GROUP", WHEEL_DRIVE}; iGroupCounter++;
	m_vCanNodeIDs[CANNODE_WHEEL_STEER_GROUP] = {42, "WHEEL_STEER_GROUP", WHEEL_STEER}; iGroupCounter++;
	m_vCanNodeIDs[CANNODE_WHEEL_WALK_GROUP] = {43, "WHEEL_WALK_GROUP", WHEEL_WALK}; iGroupCounter++;
	m_vCanNodeIDs[CANNODE_MANIP_JOINT_GROUP] = {44, "MANIP_JOINT_GROUP", MANIP_JOINT}; iGroupCounter++;
	m_vCanNodeIDs[CANNODE_MAST_PTU_GROUP] = {36, "MAST_PTU_GROUP", MAST_PTU}; iGroupCounter++;
	*/

	if (static_cast<unsigned int>(m_iNumNodes) != m_vCanNodeIDs.CanId.size())
	{
		std::cout << "Platform_Driver::ReadConfigutation: The number of motors does not match the size of CAN Node IDs array" <<std::endl;
		m_bExit=true;
		return false;
	}

	for (int i=0;i<m_iNumNodes;i++)
	{
		//* add new Whistle
		m_vpMotor[i] = new CanDriveWhistle();

		//* set CANopen parameters
		((CanDriveWhistle*) m_vpMotor[i])->setCanOpenParam(m_vCanNodeIDs.CanId[i]);

		//* set Drive name
		((CanDriveWhistle*) m_vpMotor[i])->setDriveName(m_vCanNodeIDs.Name[i]);

		//* set CAN interface to the Whistle, which is the same used in the platform driver: PEAK-CAN-USB
		m_vpMotor[i]->setCanItf(m_pCanCtrl);

		//* set Motor parameters depending on the type of motor
		if (m_vCanNodeIDs.Type[i] == WHEEL_DRIVE)
		{
			((CanDriveWhistle*)m_vpMotor[i])->getDriveParam()->setParam(
					m_GearMotWheelDrive.iEncIncrPerRevMot,
					m_GearMotWheelDrive.dBeltRatio,
					m_GearMotWheelDrive.dGearRatio,
					m_GearMotWheelDrive.iSign,
					m_GearMotWheelDrive.dPosLimitLowIncr,
					m_GearMotWheelDrive.dPosLimitHighIncr,
					m_GearMotWheelDrive.dVelMaxEncIncrS,
					m_GearMotWheelDrive.dPtpVelDefaultIncrS,
					m_GearMotWheelDrive.dAccIncrS2,
					m_GearMotWheelDrive.dDecIncrS2,
					m_GearMotWheelDrive.bIsSteer,
					m_GearMotWheelDrive.dCurrentToTorque,
					m_GearMotWheelDrive.dCurrMax,
					m_GearMotWheelDrive.iEncOffsetIncr,
					m_GearMotWheelDrive.dAnalogFactor,
					m_GearMotWheelDrive.dNominalCurrent);
		}
		else if (m_vCanNodeIDs.Type[i] == WHEEL_STEER)
		{
			((CanDriveWhistle*)m_vpMotor[i])->getDriveParam()->setParam(
					m_GearMotWheelSteer.iEncIncrPerRevMot,
					m_GearMotWheelSteer.dBeltRatio,
					m_GearMotWheelSteer.dGearRatio,
					m_GearMotWheelSteer.iSign,
					m_GearMotWheelSteer.dPosLimitLowIncr,
					m_GearMotWheelSteer.dPosLimitHighIncr,
					m_GearMotWheelSteer.dVelMaxEncIncrS,
					m_GearMotWheelSteer.dPtpVelDefaultIncrS,
					m_GearMotWheelSteer.dAccIncrS2,
					m_GearMotWheelSteer.dDecIncrS2,
					m_GearMotWheelSteer.bIsSteer,
					m_GearMotWheelSteer.dCurrentToTorque,
					m_GearMotWheelSteer.dCurrMax,
					m_GearMotWheelSteer.iEncOffsetIncr,
					m_GearMotWheelSteer.dAnalogFactor,
					m_GearMotWheelSteer.dNominalCurrent);
		}
		else if (m_vCanNodeIDs.Type[i] == WHEEL_WALK)
		{
			((CanDriveWhistle*)m_vpMotor[i])->getDriveParam()->setParam(
					m_GearMotWheelWalk.iEncIncrPerRevMot,
					m_GearMotWheelWalk.dBeltRatio,
					m_GearMotWheelWalk.dGearRatio,
					m_GearMotWheelWalk.iSign,
					m_GearMotWheelWalk.dPosLimitLowIncr,
					m_GearMotWheelWalk.dPosLimitHighIncr,
					m_GearMotWheelWalk.dVelMaxEncIncrS,
					m_GearMotWheelWalk.dPtpVelDefaultIncrS,
					m_GearMotWheelWalk.dAccIncrS2,
					m_GearMotWheelWalk.dDecIncrS2,
					m_GearMotWheelWalk.bIsSteer,
					m_GearMotWheelWalk.dCurrentToTorque,
					m_GearMotWheelWalk.dCurrMax,
					m_GearMotWheelWalk.iEncOffsetIncr,
					m_GearMotWheelWalk.dAnalogFactor,
					m_GearMotWheelWalk.dNominalCurrent);
		}
		else if (m_vCanNodeIDs.Type[i] == MANIP_JOINT)
		{
			((CanDriveWhistle*)m_vpMotor[i])->getDriveParam()->setParam(
					m_GearMotManipJoint.iEncIncrPerRevMot,
					m_GearMotManipJoint.dBeltRatio,
					m_GearMotManipJoint.dGearRatio,
					m_GearMotManipJoint.iSign,
					m_GearMotManipJoint.dPosLimitLowIncr,
					m_GearMotManipJoint.dPosLimitHighIncr,
					m_GearMotManipJoint.dVelMaxEncIncrS,
					m_GearMotManipJoint.dPtpVelDefaultIncrS,
					m_GearMotManipJoint.dAccIncrS2,
					m_GearMotManipJoint.dDecIncrS2,
					m_GearMotManipJoint.bIsSteer,
					m_GearMotManipJoint.dCurrentToTorque,
					m_GearMotManipJoint.dCurrMax,
					m_GearMotManipJoint.iEncOffsetIncr,
					m_GearMotManipJoint.dAnalogFactor,
					m_GearMotManipJoint.dNominalCurrent);
		}
		else if (m_vCanNodeIDs.Type[i] == MAST_PAN)
		{
			((CanDriveWhistle*)m_vpMotor[i])->getDriveParam()->setParam(
					m_GearMotMastPan.iEncIncrPerRevMot,
					m_GearMotMastPan.dBeltRatio,
					m_GearMotMastPan.dGearRatio,
					m_GearMotMastPan.iSign,
					m_GearMotMastPan.dPosLimitLowIncr,
					m_GearMotMastPan.dPosLimitHighIncr,
					m_GearMotMastPan.dVelMaxEncIncrS,
					m_GearMotMastPan.dPtpVelDefaultIncrS,
					m_GearMotMastPan.dAccIncrS2,
					m_GearMotMastPan.dDecIncrS2,
					m_GearMotMastPan.bIsSteer,
					m_GearMotMastPan.dCurrentToTorque,
					m_GearMotMastPan.dCurrMax,
					m_GearMotMastPan.iEncOffsetIncr,
					m_GearMotMastPan.dAnalogFactor,
					m_GearMotMastPan.dNominalCurrent);
		}
		else if (m_vCanNodeIDs.Type[i] == MAST_TILT)
		{
			((CanDriveWhistle*)m_vpMotor[i])->getDriveParam()->setParam(
					m_GearMotMastTilt.iEncIncrPerRevMot,
					m_GearMotMastTilt.dBeltRatio,
					m_GearMotMastTilt.dGearRatio,
					m_GearMotMastTilt.iSign,
					m_GearMotMastTilt.dPosLimitLowIncr,
					m_GearMotMastTilt.dPosLimitHighIncr,
					m_GearMotMastTilt.dVelMaxEncIncrS,
					m_GearMotMastTilt.dPtpVelDefaultIncrS,
					m_GearMotMastTilt.dAccIncrS2,
					m_GearMotMastTilt.dDecIncrS2,
					m_GearMotMastTilt.bIsSteer,
					m_GearMotMastTilt.dCurrentToTorque,
					m_GearMotMastTilt.dCurrMax,
					m_GearMotMastTilt.iEncOffsetIncr,
					m_GearMotMastTilt.dAnalogFactor,
					m_GearMotMastTilt.dNominalCurrent);
		}
		else
		{
			std::cout << "Platform_Driver::ReadConfigutation: Unknown type "<< m_vCanNodeIDs.Type[i] <<" of motor "<< m_vCanNodeIDs.Name[i] <<std::endl;
			return false;
		}

	}

	return true;
}

//-----------------------------------------------
int Platform_Driver::evalCanBuffer()
{
	bool bRet;
	int iNode=0;

	//* As long as there is something in the can buffer -> read out next message
	while((m_pCanCtrl->availableMessages() > 0)  && (m_pCanCtrl->receiveMsg(&m_CanMsgRec) == true))
	{
		bRet = false;

		iNode = getNodeFromCobId(m_CanMsgRec.getID());
		if(iNode!=-1)
		{
			bRet |= m_vpMotor[iNode]->evalReceivedMsg(m_CanMsgRec);
		}
		if (bRet == false)
		{
			std::cout << "evalCanBuffer(): Received CAN_Message with unknown identifier " << m_CanMsgRec.getID() << std::endl;
		}		
	};

	return 0;
}

//-----------------------------------------------
bool Platform_Driver::initPltf(GearMotorParamType wheel_drive, GearMotorParamType steer_drive, GearMotorParamType walk_drive, GearMotorParamType pan_drive, GearMotorParamType tilt_drive, GearMotorParamType arm_joint, PltfCanParams can_params)
{	
	bool bHomingOk=false;

	//* Platform configuration. CAN interface and CAN nodes setup.
	if(!readConfiguration(wheel_drive, steer_drive, walk_drive, pan_drive, tilt_drive, arm_joint, can_params))
	{
		std::cout << "Platform_Driver::initPltf: ERROR in readConfiguration call. Check CAN interface " << std::endl;
		return false;
	}

	//* Create thread for message handling
	rc = pthread_create(&msg_task, NULL, msg_handler, this);

	if (rc)
	{
		std::cout << "Platform_Driver::initPltf: ERROR while creating the message handler thread. Return code is " << rc << std::endl;
		return false;
	}
	
	for(int i = 0; i < m_iNumMotors; i++)
	{
		m_vpMotor[i]->reset();
	}
    /* Not necessary in HDPR because there is no AUTOEXEC code in the Whistles that needs time
	usleep(10000000); //!Wait for the AUTOEXEC script in the Whistles to finish
	*/

	//* Initialize and start all motors
	for (int i = 0; i<m_iNumMotors; i++)
	{
		if (!m_vpMotor[i]->init())
		{
			std::cout << "Initialization of Whistle " << m_vCanNodeIDs.Name[i] << " failed" << std::endl;
			return false;
		}
		usleep(10000);
	
		if (can_params.Active[i])
		{
			if (!m_vpMotor[i]->start())
			{
				std::cout << "Starting Whistle " << m_vCanNodeIDs.Name[i] << " failed" << std::endl;
				return false;
			}
			usleep(10000);
		}

		std::cout << "Motor "<< m_vCanNodeIDs.Name[i] << " started" << std::endl;
	}

	//* Start timer for sending periodic SYNC messages, watchdog error detection and error control
	configureTimer();

	if (!startWatchdog(m_bWatchdogActive))
	{
		std::cout << "Starting watchdog failed" << std::endl;
		return false;
	}

	/*
	 * Home all non driving motors. Using group IDs for efficiency.
	 */

	for (int i = 0; i<m_iNumNodes; i++)
	{
		if (m_vpMotor[i]->getDriveParam()->getIsSteer())
			((CanDriveWhistle*)m_vpMotor[i])->Homing();
		else
			((CanDriveWhistle*)m_vpMotor[i])->setTypeMotionVariable(CanDriveItf::MOTIONTYPE_VELCTRL);
	}

	//((CanDriveWhistle*)m_vpMotor[CANNODE_WHEEL_STEER_GROUP])->Homing();
	//((CanDriveWhistle*)m_vpMotor[CANNODE_WHEEL_WALK_GROUP])->Homing();
	//((CanDriveWhistle*)m_vpMotor[CANNODE_MANIP_JOINT_GROUP])->Homing();
	//((CanDriveWhistle*)m_vpMotor[CANNODE_MAST_PTU_GROUP])->Homing();
	usleep(2000000);


	//* check correct homing achieved
	while(!bHomingOk)
	{
		bHomingOk=true;
		for (int i=0; i<m_iNumMotors; i++)
		{
			if (m_vCanNodeIDs.Type[i] != WHEEL_DRIVE)
			{
                		if (can_params.Active[i]){
                    			bHomingOk &= m_vpMotor[i]->checkTargetReached();
                		}
			}
		}
	}
	std::cout << "Homing motors finished with status: " << bHomingOk << std::endl;
	return (bHomingOk);
}

//-----------------------------------------------
bool Platform_Driver::isPltfError()
{
	bool bErrMotor = true;
	//* check all motors for errors
	for(int i = 0; i < m_iNumMotors; i++)
	{
		bErrMotor &= m_vpMotor[i]->isError();
	}
	return bErrMotor;
}

//-----------------------------------------------
bool Platform_Driver::shutdownPltf()
{
	bool bRet=true;
	//* shut down all motors
	for(int i = 0; i < m_iNumMotors; i++)
	{
		bRet &= m_vpMotor[i]->shutdown();
	}
	return bRet;
}

//-----------------------------------------------
bool Platform_Driver::shutdownNode(int iCanIdent)
{
	bool bRet=true;
	//* shut down the motor
	bRet &= m_vpMotor[iCanIdent]->shutdown();
	return bRet;
}

//-----------------------------------------------
bool Platform_Driver::startNode(int iCanIdent)
{
	bool bRet=true;
	//* shut down the motor
	bRet &= m_vpMotor[iCanIdent]->start();
	return bRet;
}

//-----------------------------------------------
bool Platform_Driver::resetPltf()
{
	bool bRetMotor = true;
	bool bRet = true;

	for(int i = 0; i < m_iNumMotors; i++)
	{
		m_vpMotor[i]->reset();
	}
	usleep(10000000); //!Wait for the AUTOEXEC script in the Whistles to finish

	for(int i = 0; i < m_iNumMotors; i++)
	{
		bRetMotor = m_vpMotor[i]->init();
		if (bRetMotor == true)
		{
			m_vpMotor[i]->start();
		}
		else
		{
			std::cout << "Resetting of Motor " << m_vCanNodeIDs.Name[i] << " failed" << std::endl;
		}

		bRet &= bRetMotor;
	}
	return bRet;
}

//-----------------------------------------------
bool Platform_Driver::resetNode(int iCanIdent)
{
	bool bRet = true;
	m_vpMotor[iCanIdent]->reset();
	usleep(2000000); //! Wait for the AUTOEXEC script in the Whistle to finish
	bRet = m_vpMotor[iCanIdent]->init();
	if (bRet)
	{
		m_vpMotor[iCanIdent]->start();
	}
	else
	{
		std::cout << "Resetting of Motor " << m_vCanNodeIDs.Name[iCanIdent] << " failed" << std::endl;
	}

	return bRet;
}

//-----------------------------------------------
bool Platform_Driver::startWatchdog(bool bStart)
{

	bool bRet = true;
	std::cout << "Configuration of Watchdogs..." << std::endl;
	for(int i = 0; i < m_iNumMotors; i++)
	{
		if (!(m_vpMotor[i]->startWatchdog(bStart)))
		{
			std::cout << "Error initializing watchdog of motor " << m_vCanNodeIDs.Name[i] << std::endl;
			bRet = false;
		}
	}
	usleep(100000);
	std::cout << "...done" << std::endl;
	return (bRet);
}

/*
//-----------------------------------------------
bool Platform_Driver::setDrivingMode(PltfDrivingMode mode)
{
	PltfDrivingMode newDrivingMode = mode;
	bool bTargeted;
	int cnt=0;

	switch (newDrivingMode)
	{
		case STOPPED:
			m_vpMotor[CANNODE_WHEEL_DRIVE_GROUP]->velocityCommandRadS(0);
			break;
		case STRAIGHT_LINE:
			m_vpMotor[CANNODE_WHEEL_DRIVE_GROUP]->velocityCommandRadS(0);
			m_vpMotor[CANNODE_WHEEL_STEER_GROUP]->positionCommandRad(0,PTP_VELOCICTY_DEFAULT);
			m_vpMotor[CANNODE_WHEEL_WALK_GROUP]->positionCommandRad(0,PTP_VELOCICTY_DEFAULT);
			do
			{
				bTargeted=true;
				usleep(20000);
				bTargeted &= m_vpMotor[CANNODE_WHEEL_STEER_FL]->checkTargetReached();
				bTargeted &= m_vpMotor[CANNODE_WHEEL_STEER_FR]->checkTargetReached();
				bTargeted &= m_vpMotor[CANNODE_WHEEL_STEER_BL]->checkTargetReached();
				bTargeted &= m_vpMotor[CANNODE_WHEEL_STEER_BR]->checkTargetReached();
				bTargeted &= m_vpMotor[CANNODE_WHEEL_WALK_FL]->checkTargetReached();
				bTargeted &= m_vpMotor[CANNODE_WHEEL_WALK_FR]->checkTargetReached();
				bTargeted &= m_vpMotor[CANNODE_WHEEL_WALK_CL]->checkTargetReached();
				bTargeted &= m_vpMotor[CANNODE_WHEEL_WALK_CR]->checkTargetReached();
				bTargeted &= m_vpMotor[CANNODE_WHEEL_WALK_BL]->checkTargetReached();
				bTargeted &= m_vpMotor[CANNODE_WHEEL_WALK_BR]->checkTargetReached();
				cnt++;
			}while(!bTargeted && cnt<100);
			break;
		case ACKERMAN:
			m_vpMotor[CANNODE_WHEEL_DRIVE_GROUP]->velocityCommandRadS(0);
			m_vpMotor[CANNODE_WHEEL_WALK_GROUP]->positionCommandRad(0,PTP_VELOCICTY_DEFAULT);
			do
			{
				bTargeted=true;
				usleep(20000);
				bTargeted &= m_vpMotor[CANNODE_WHEEL_WALK_FL]->checkTargetReached();
				bTargeted &= m_vpMotor[CANNODE_WHEEL_WALK_FR]->checkTargetReached();
				bTargeted &= m_vpMotor[CANNODE_WHEEL_WALK_CL]->checkTargetReached();
				bTargeted &= m_vpMotor[CANNODE_WHEEL_WALK_CR]->checkTargetReached();
				bTargeted &= m_vpMotor[CANNODE_WHEEL_WALK_BL]->checkTargetReached();
				bTargeted &= m_vpMotor[CANNODE_WHEEL_WALK_BR]->checkTargetReached();
				cnt++;
			}while(!bTargeted && cnt<100);
			break;
		case SPOT_TURN:
			m_vpMotor[CANNODE_WHEEL_DRIVE_GROUP]->velocityCommandRadS(0);
			m_vpMotor[CANNODE_WHEEL_WALK_GROUP]->positionCommandRad(0,PTP_VELOCICTY_DEFAULT);
			do
			{
				bTargeted=true;
				usleep(20000);
				bTargeted &= m_vpMotor[CANNODE_WHEEL_WALK_FL]->checkTargetReached();
				bTargeted &= m_vpMotor[CANNODE_WHEEL_WALK_FR]->checkTargetReached();
				bTargeted &= m_vpMotor[CANNODE_WHEEL_WALK_CL]->checkTargetReached();
				bTargeted &= m_vpMotor[CANNODE_WHEEL_WALK_CR]->checkTargetReached();
				bTargeted &= m_vpMotor[CANNODE_WHEEL_WALK_BL]->checkTargetReached();
				bTargeted &= m_vpMotor[CANNODE_WHEEL_WALK_BR]->checkTargetReached();
				cnt++;
			}while(!bTargeted && cnt<100);
			break;
		case SKID_TURN:
			m_vpMotor[CANNODE_WHEEL_DRIVE_GROUP]->velocityCommandRadS(0);
			m_vpMotor[CANNODE_WHEEL_STEER_GROUP]->positionCommandRad(0,PTP_VELOCICTY_DEFAULT);
			m_vpMotor[CANNODE_WHEEL_WALK_GROUP]->positionCommandRad(0,PTP_VELOCICTY_DEFAULT);
			do
			{
				bTargeted=true;
				usleep(20000);
				bTargeted &= m_vpMotor[CANNODE_WHEEL_STEER_FL]->checkTargetReached();
				bTargeted &= m_vpMotor[CANNODE_WHEEL_STEER_FR]->checkTargetReached();
				bTargeted &= m_vpMotor[CANNODE_WHEEL_STEER_BL]->checkTargetReached();
				bTargeted &= m_vpMotor[CANNODE_WHEEL_STEER_BR]->checkTargetReached();
				bTargeted &= m_vpMotor[CANNODE_WHEEL_WALK_FL]->checkTargetReached();
				bTargeted &= m_vpMotor[CANNODE_WHEEL_WALK_FR]->checkTargetReached();
				bTargeted &= m_vpMotor[CANNODE_WHEEL_WALK_CL]->checkTargetReached();
				bTargeted &= m_vpMotor[CANNODE_WHEEL_WALK_CR]->checkTargetReached();
				bTargeted &= m_vpMotor[CANNODE_WHEEL_WALK_BL]->checkTargetReached();
				bTargeted &= m_vpMotor[CANNODE_WHEEL_WALK_BR]->checkTargetReached();
				cnt++;
			}while(!bTargeted && cnt<100);
			break;
		case WHEEL_WALKING:
			m_vpMotor[CANNODE_WHEEL_DRIVE_GROUP]->velocityCommandRadS(0);
			m_vpMotor[CANNODE_WHEEL_STEER_GROUP]->positionCommandRad(0,PTP_VELOCICTY_DEFAULT);
			do
			{
				bTargeted=true;
				usleep(20000);
				bTargeted &= m_vpMotor[CANNODE_WHEEL_STEER_FL]->checkTargetReached();
				bTargeted &= m_vpMotor[CANNODE_WHEEL_STEER_FR]->checkTargetReached();
				bTargeted &= m_vpMotor[CANNODE_WHEEL_STEER_BL]->checkTargetReached();
				bTargeted &= m_vpMotor[CANNODE_WHEEL_STEER_BR]->checkTargetReached();
				cnt++;
			}while(!bTargeted && cnt<100);
			break;
		case DIRECT_DRIVE:
			m_vpMotor[CANNODE_WHEEL_DRIVE_GROUP]->velocityCommandRadS(0);
			break;
	}

	m_DrivingMode = newDrivingMode;
	std::cout<< "Driving mode set to: " << m_DrivingMode <<std::endl;
	return true;
}


//-----------------------------------------------
// Motor Controlers
//-----------------------------------------------

//-----------------------------------------------
void Platform_Driver::pltfDriveStraightVelocityCmS(double iVelocity)
{
	if (m_DrivingMode!=STRAIGHT_LINE){
		std::cout << "Trying to drive straight without being in straight line mode. Exiting without driving..." << std::endl;
		return;
	}

	double dVelRadS = 10*iVelocity/WHEEL_RADIUS_MM;
	m_vpMotor[CANNODE_WHEEL_DRIVE_GROUP]->velocityCommandRadS(dVelRadS);
}

void Platform_Driver::pltfDriveGenericAckerman(double dVelocity, double *dRotationCenter, double *dPointToControl)
{
	if (m_DrivingMode!=ACKERMAN){
		std::cout << "Trying to drive Ackerman without being in Ackerman mode. Exiting without driving..." << std::endl;
		return;
	}

	if(GenericAckermann( &MyRover,
		dVelocity,
		dRotationCenter,
		dPointToControl,
		m_dWheelSteering,
		m_dWheelVelocity ))
	{
		std::cout << "Error in GenericAckerman function. Exiting without driving..." << std::endl;
		return;
	}

	nodePositionSetPointRad(CANNODE_WHEEL_STEER_FL,m_dWheelSteering[0],PTP_VELOCICTY_DEFAULT);
	nodePositionSetPointRad(CANNODE_WHEEL_STEER_FR,m_dWheelSteering[1],PTP_VELOCICTY_DEFAULT);
	nodePositionSetPointRad(CANNODE_WHEEL_STEER_BL,m_dWheelSteering[4],PTP_VELOCICTY_DEFAULT);
	nodePositionSetPointRad(CANNODE_WHEEL_STEER_BR,m_dWheelSteering[5],PTP_VELOCICTY_DEFAULT);
	nodeCommandSetPoint(CANNODE_WHEEL_STEER_GROUP);

	// wait to reach steering angles
	bool bTargeted;
	int cnt=0;
	do
	{
		bTargeted=true;
		usleep(20000);
		bTargeted &= m_vpMotor[CANNODE_WHEEL_STEER_FL]->checkTargetReached();
		bTargeted &= m_vpMotor[CANNODE_WHEEL_STEER_FR]->checkTargetReached();
		bTargeted &= m_vpMotor[CANNODE_WHEEL_STEER_BL]->checkTargetReached();
		bTargeted &= m_vpMotor[CANNODE_WHEEL_STEER_BR]->checkTargetReached();
		cnt++;
	}while(!bTargeted && cnt<100);

	for (int i=0; i<NUM_WHEELS;i++)
	{
		nodeVelocitySetPointRadS(CANNODE_WHEEL_DRIVE_FL+i,m_dWheelVelocity[i]);
	}
	nodeCommandSetPoint(CANNODE_WHEEL_DRIVE_GROUP);
}

void Platform_Driver::pltfDriveSpotTurn(double dAngularVelocity)
{
	if (m_DrivingMode!=SPOT_TURN){
		std::cout << "Trying to drive Spot Turn without being in Spot Turn mode. Exiting without driving..." << std::endl;
		return;
	}

	if (SpotTurn( &MyRover,
		dAngularVelocity,
		m_dWheelSteering,
		m_dWheelVelocity ))
	{
		std::cout << "Error in SpotTurn function. Exiting without driving..." << std::endl;
		return;
	}

	nodePositionSetPointRad(CANNODE_WHEEL_STEER_FL,(m_dWheelSteering[0]-PI),PTP_VELOCICTY_DEFAULT);
	nodePositionSetPointRad(CANNODE_WHEEL_STEER_FR,m_dWheelSteering[1],PTP_VELOCICTY_DEFAULT);
	nodePositionSetPointRad(CANNODE_WHEEL_STEER_BL,(m_dWheelSteering[4]+PI),PTP_VELOCICTY_DEFAULT);
	nodePositionSetPointRad(CANNODE_WHEEL_STEER_BR,m_dWheelSteering[5],PTP_VELOCICTY_DEFAULT);
	nodeCommandSetPoint(CANNODE_WHEEL_STEER_GROUP);

	// wait to reach steering angles
	bool bTargeted;
	int cnt=0;
	do
	{
		bTargeted=true;
		usleep(20000);
		bTargeted &= m_vpMotor[CANNODE_WHEEL_STEER_FL]->checkTargetReached();
		bTargeted &= m_vpMotor[CANNODE_WHEEL_STEER_FR]->checkTargetReached();
		bTargeted &= m_vpMotor[CANNODE_WHEEL_STEER_BL]->checkTargetReached();
		bTargeted &= m_vpMotor[CANNODE_WHEEL_STEER_BR]->checkTargetReached();
		cnt++;
	}while(!bTargeted && cnt<100);


	nodeVelocitySetPointRadS(CANNODE_WHEEL_DRIVE_FL+0,-m_dWheelVelocity[0]);
	nodeVelocitySetPointRadS(CANNODE_WHEEL_DRIVE_FL+1,m_dWheelVelocity[1]);
	nodeVelocitySetPointRadS(CANNODE_WHEEL_DRIVE_FL+2,-m_dWheelVelocity[2]);
	nodeVelocitySetPointRadS(CANNODE_WHEEL_DRIVE_FL+3,m_dWheelVelocity[3]);
	nodeVelocitySetPointRadS(CANNODE_WHEEL_DRIVE_FL+4,-m_dWheelVelocity[4]);
	nodeVelocitySetPointRadS(CANNODE_WHEEL_DRIVE_FL+5,m_dWheelVelocity[5]);

	nodeCommandSetPoint(CANNODE_WHEEL_DRIVE_GROUP);
}

void Platform_Driver::pltfDriveSkidTurn(double dVelocity, double dRadiusOfCurvature, int iIsStraightLine)
{
	if (m_DrivingMode!=SKID_TURN){
		std::cout << "Trying to drive Spot Turn without being in Spot Turn mode. Exiting without driving..." << std::endl;
		return;
	}

	if (SkidTurn(
		&MyRover,
		dVelocity,
		dRadiusOfCurvature,
		iIsStraightLine,
		m_dWheelVelocity ))
	{
		std::cout << "Error in SkidTurn function. Exiting without driving..." << std::endl;
		return;
	}

	for (int i=0; i<NUM_WHEELS;i++)
	{
		nodeVelocitySetPointRadS(CANNODE_WHEEL_DRIVE_FL+i,m_dWheelVelocity[i]);
	}
	nodeCommandSetPoint(CANNODE_WHEEL_DRIVE_GROUP);
}

void Platform_Driver::pltfDriveWheelWalk(double *dStepLength, int iGait)
{
	if (m_DrivingMode!=WHEEL_WALKING){
		std::cout << "Trying to drive Wheel Walk without being in Wheel Walking mode. Exiting without driving..." << std::endl;
		return;
	}

	if (WheelWalk( &MyRover,
		dStepLength,
		iGait,
		m_dWalkAngleRad,
		m_dWheelAngleRad
		))
	{
		std::cout << "Error in WheelWalk function. Exiting without driving..." << std::endl;
		return;
	}

	std::cout << "WheelWalk function not implemented yet in Generic Manoeuvre Library. Exiting without driving..." << std::endl;

	// Commented for safety until a validated code for wheel walking capabilities is implemented in the Generic Manoeuvre Library
	for (int i=0; i<NUM_WHEELS;i++)
	{
		nodePositionSetPointRad(CANNODE_WHEEL_DRIVE_FL+i,m_dWheelAngleRad[i],PTP_VELOCICTY_DEFAULT);
		nodePositionSetPointRad(CANNODE_WHEEL_WALK_FL+i,m_dWalkAngleRad[i],PTP_VELOCICTY_DEFAULT);
	}
	nodeCommandSetPoint(CANNODE_WHEEL_DRIVE_GROUP);
	nodeCommandSetPoint(CANNODE_WHEEL_WALK_GROUP);
	//
}
*/

/*
//-----------------------------------------------
void Platform_Driver::directWheelDriveVelocityDegS(int iWheel, double dVelocity)
{
	if (m_DrivingMode!=DIRECT_DRIVE){
		std::cout << "Trying to Direct Drive without being in Direct Drive mode. Exiting without driving..." << std::endl;
		return;
	}
	double dRadS = dVelocity*PI/180;
	nodeVelocityCommandRadS(iWheel,dRadS);
}

//-----------------------------------------------
void Platform_Driver::directWheelSteerAngleDeg(int iWheel, double dAngle)
{
	if (m_DrivingMode!=DIRECT_DRIVE){
		std::cout << "Trying to Direct Drive without being in Direct Drive mode. Exiting without driving..." << std::endl;
		return;
	}
	double dRad = dAngle*PI/180;
	nodePositionCommandRad(iWheel,dRad,PTP_VELOCICTY_DEFAULT);
}

//-----------------------------------------------
void Platform_Driver::directManipJointAngleDeg(int iJoint, double dAngle)
{
	if (m_DrivingMode!=DIRECT_DRIVE){
		std::cout << "Trying to Direct Drive without being in Direct Drive mode. Exiting without driving..." << std::endl;
		return;
	}
	double dRad = dAngle*PI/180;
	nodePositionCommandRad(iJoint,dRad,PTP_VELOCICTY_DEFAULT);
}

//-----------------------------------------------
void Platform_Driver::directWheelWalkJointAngleDeg(int iJoint, double dAngle)
{
	if (m_DrivingMode!=DIRECT_DRIVE){
		std::cout << "Trying to Direct Drive without being in Direct Drive mode. Exiting without driving..." << std::endl;
		return;
	}
	double dRad = dAngle*PI/180;
	nodePositionCommandRad(iJoint,dRad,PTP_VELOCICTY_DEFAULT);
}

//-----------------------------------------------
void Platform_Driver::directMastPanAngleDeg(double dAngle)
{
	if (m_DrivingMode!=DIRECT_DRIVE){
		std::cout << "Trying to Direct Drive without being in Direct Drive mode. Exiting without driving..." << std::endl;
		return;
	}
	double dRad = dAngle*PI/180;
	nodePositionCommandRad(CANNODE_MAST_PTU_PAN,dRad,PTP_VELOCICTY_DEFAULT);
}

//-----------------------------------------------
void Platform_Driver::directMastTiltAngleDeg(double dAngle)
{
	if (m_DrivingMode!=DIRECT_DRIVE){
		std::cout << "Trying to Direct Drive without being in Direct Drive mode. Exiting without driving..." << std::endl;
		return;
	}
	double dRad = dAngle*PI/180;
	nodePositionCommandRad(CANNODE_MAST_PTU_TILT,dRad,PTP_VELOCICTY_DEFAULT);
}
*/

//-----------------------------------------------
void Platform_Driver::nodePositionCommandRad(int iCanIdent, double dPosGearRad, double dVelGearRadS)
{		

    if (m_vpMotor[iCanIdent]->getTypeMotionVariable() != CanDriveItf::MOTIONTYPE_POSCTRL)
        m_vpMotor[iCanIdent]->setTypeMotion(CanDriveItf::MOTIONTYPE_POSCTRL);
	
	m_vpMotor[iCanIdent]->positionCommandRad(dPosGearRad, dVelGearRadS);

}

//-----------------------------------------------
void Platform_Driver::nodePositionSetPointRad(int iCanIdent, double dPosGearRad, double dVelGearRadS)
{

    if (m_vpMotor[iCanIdent]->getTypeMotionVariable() != CanDriveItf::MOTIONTYPE_POSCTRL)
        m_vpMotor[iCanIdent]->setTypeMotion(CanDriveItf::MOTIONTYPE_POSCTRL);

	m_vpMotor[iCanIdent]->positionSetPointRad(dPosGearRad, dVelGearRadS);

}

//-----------------------------------------------
void Platform_Driver::nodeVelocityCommandRadS(int iCanIdent, double dVelGearRadS)
{

    if (m_vpMotor[iCanIdent]->getTypeMotionVariable() != CanDriveItf::MOTIONTYPE_VELCTRL)
        m_vpMotor[iCanIdent]->setTypeMotion(CanDriveItf::MOTIONTYPE_VELCTRL);

	m_vpMotor[iCanIdent]->velocityCommandRadS(dVelGearRadS);

}

//-----------------------------------------------
void Platform_Driver::nodeVelocitySetPointRadS(int iCanIdent, double dVelGearRadS)
{
    if (m_vpMotor[iCanIdent]->getTypeMotionVariable() != CanDriveItf::MOTIONTYPE_VELCTRL)
        m_vpMotor[iCanIdent]->setTypeMotion(CanDriveItf::MOTIONTYPE_VELCTRL);

	m_vpMotor[iCanIdent]->velocitySetPointRadS(dVelGearRadS);
}

//-----------------------------------------------
void Platform_Driver::nodeCommandSetPoint(int iCanIdent)
{

	m_vpMotor[iCanIdent]->commandSetPoint();

}

//-----------------------------------------------
void Platform_Driver::nodeTorqueCommandNm(int iCanIdent, double dTorqueNm)
{
    if (m_vpMotor[iCanIdent]->getTypeMotionVariable() != CanDriveItf::MOTIONTYPE_TORQUECTRL)
        m_vpMotor[iCanIdent]->setTypeMotion(CanDriveItf::MOTIONTYPE_TORQUECTRL);

	m_vpMotor[iCanIdent]->torqueCommandNm(dTorqueNm);

}

//-----------------------------------------------
void Platform_Driver::requestNodePosVel(int iCanIdent)
{
	
	m_vpMotor[iCanIdent]->requestPosVel();

}

//-----------------------------------------------
void Platform_Driver::requestNodeStatus(int iCanIdent)
{

	m_vpMotor[iCanIdent]->requestStatus();

}

//-----------------------------------------------
void Platform_Driver::requestNodeTorque(int iCanIdent)
{

	m_vpMotor[iCanIdent]->requestTorque();

}

//-----------------------------------------------
void Platform_Driver::getNodePositionVelocityRadS(int iCanIdent, double* pdAngleGearRad, double* pdVelGearRadS)
{
	m_vpMotor[iCanIdent]->getPositionVelocityRadS(pdAngleGearRad, pdVelGearRadS);
}

//-----------------------------------------------
void Platform_Driver::getNodeDeltaPositionVelocityRadS(int iCanIdent, double* pdAngleGearRad, double* pdVelGearRadS)
{
	m_vpMotor[iCanIdent]->getDeltaPositionVelocityRadS(pdAngleGearRad, pdVelGearRadS);
}

//-----------------------------------------------
void Platform_Driver::getNodeVelocityRadS(int iCanIdent, double* pdVelocityRadS)
{
	m_vpMotor[iCanIdent]->getVelocityRadS(pdVelocityRadS);
}

//-----------------------------------------------
void Platform_Driver::getNodeStatus(int iCanIdent, int* piStatus, int* piTempCel)
{
	m_vpMotor[iCanIdent]->getStatus(piStatus, piTempCel);
}	

//-----------------------------------------------
void Platform_Driver::getNodeTorque(int iCanIdent, double* pdTorqueNm)
{
	m_vpMotor[iCanIdent]->getTorqueNm(pdTorqueNm);
}

//-----------------------------------------------
bool Platform_Driver::getNodeData(int iCanIdent,double* pdAngleGearRad, double* pdVelGearRadS, double* pdCurrentAmp, double* pdTorqueNm)
{
	if (m_vpMotor[iCanIdent]->status())
	{
		m_vpMotor[iCanIdent]->getData(pdAngleGearRad,pdVelGearRadS,pdCurrentAmp,pdTorqueNm);
		return true;
	}
	return false;
}

//-----------------------------------------------
void Platform_Driver::getNodeAnalogInput(int iCanIdent,double* pdAnalogInput)
{
	m_vpMotor[iCanIdent]->getAnalogInput(pdAnalogInput);
}


//-----------------------------------------------
int Platform_Driver::getNumMotors()
{
	return m_iNumMotors;
}

//-----------------------------------------------
// Thread functions
//-----------------------------------------------

void* msg_handler(void *args)
{
	CanMsg CanMsg;
	int iNodeId=0;

	/*
	 * int* value = reinterpret_cast<int*>(args);
	 * printf("\n in thread %d", *value);
	 */

	while (!m_bExit)
	{
		if (m_pCanCtrl->availableMessages()>0)
		{
			if ((m_pCanCtrl->receiveMsg(&CanMsg) == true))
			{
				if((iNodeId = getNodeFromCobId(CanMsg.getID())) >= 0)
				{
					m_vpMotor[iNodeId]->evalReceivedMsg(CanMsg);
				}
				else
				{
					std::cout << "message_handler: Received CAN_Message with unknown identifier " << CanMsg.getID() << std::endl;
				}
			}
		}
		else
		{
			//* No available messages, release CPU to let other processes proceed with execution.
			//* Try different values of sleep duration for evaluating the resulting performance (will depend on platform HW and bus load).
			usleep(10000);
		}
	}
	return 0;
}

int getNodeFromCobId(int iCobID)
{
	int ID = iCobID & 0x007f;
	for (size_t i=0;i<m_vpMotor.size();i++)
	{
		if (ID==m_vCanNodeIDs.CanId[i])
			return i;
	}
	return -1;
}

//-----------------------------------------------
// Timer functions
//-----------------------------------------------

void timer_handler (int signum)
{
	sendSync(); //* to request pos & velocity (TPDO1) of drives and also current & torque & analog input (TPDO3)
	if (m_bWatchdogActive)
	{
		sendHeartbeat();
	}

	for (size_t i=0;i<m_vpMotor.size();i++)
	{
		((CanDriveWhistle*)m_vpMotor[i])->requestStatus();
	}

	//((CanDriveWhistle*)m_vpMotor[CANNODE_WHEEL_DRIVE_GROUP])->requestStatus();
	//((CanDriveWhistle*)m_vpMotor[CANNODE_WHEEL_STEER_GROUP])->requestStatus();
	//((CanDriveWhistle*)m_vpMotor[CANNODE_WHEEL_WALK_GROUP])->requestStatus();

	//requestSystemInfo();  //! used for debugging

	/*
	m_iTimerDiv++;
	switch(m_iTimerDiv)
	{
		case 1:
			((CanDriveWhistle*)m_vpMotor[CANNODE_WHEEL_DRIVE_FL])->requestStatus();
			break;
		case 2:
			((CanDriveWhistle*)m_vpMotor[CANNODE_WHEEL_STEER_FL])->requestStatus();
			break;
		case 3:
			((CanDriveWhistle*)m_vpMotor[CANNODE_WHEEL_WALK_FL])->requestStatus();
			break;
		case 4:
			//((CanDriveWhistle*)m_vpMotor[CANNODE_MAST_PTU_GROUP])->requestStatus();
			break;
		case 5:
			//((CanDriveWhistle*)m_vpMotor[CANNODE_MANIP_JOINT_GROUP])->requestStatus();
			m_iTimerDiv=0;
			break;
	}
	*/
}

void requestSystemInfo()
{
	//m_vpMotor[CANNODE_WHEEL_DRIVE_FL]->IntprtSetInt(4, 'A', 'N', 1, 0);
	//m_vpMotor[CANNODE_WHEEL_DRIVE_FR]->IntprtSetInt(4, 'A', 'N', 1, 0);
	//m_vpMotor[CANNODE_WHEEL_DRIVE_CL]->IntprtSetInt(4, 'A', 'N', 1, 0); //This Whistle has no system info in the analog output!
	//m_vpMotor[CANNODE_WHEEL_DRIVE_CR]->IntprtSetInt(4, 'A', 'N', 1, 0);
	//m_vpMotor[CANNODE_WHEEL_DRIVE_BL]->IntprtSetInt(4, 'A', 'N', 1, 0);
	//m_vpMotor[CANNODE_WHEEL_DRIVE_BR]->IntprtSetInt(4, 'A', 'N', 1, 0);
	//m_vpMotor[CANNODE_WHEEL_DRIVE_GROUP]->IntprtSetInt(4, 'A', 'N', 1, 0);
	//m_vpMotor[CANNODE_WHEEL_DRIVE_GROUP]->IntprtSetInt(4, 'I', 'Q', 0, 0);
	//m_vpMotor[CANNODE_WHEEL_DRIVE_GROUP]->IntprtSetInt(4, 'C', 'L', 1, 0);


}

void sendSync()
{
	CanMsg msg;
	msg.setID(0x80);
	msg.setLength(0);
	msg.set(0,0,0,0,0,0,0,0);
	m_pCanCtrl->transmitMsg(msg);
}

void sendNMTMsg(BYTE cmd)
{
	CanMsg msg;

	msg.setID(0);
	msg.setLength(2);
	msg.set(cmd,0x00,0,0,0,0,0,0);
	m_pCanCtrl->transmitMsg(msg, false);

	usleep(100000);
}

void sendHeartbeat()
{
	CanMsg msg;
	msg.setID(0x700);
	msg.setLength(5);
	msg.set(0x00,0,0,0,0,0,0,0);
	m_pCanCtrl->transmitMsg(msg);
}
