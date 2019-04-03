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
 * ROS stack name: cob_driver
 * ROS package name: cob_canopen_motor
 * Description:
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Christian Connette, email:christian.connette@ipa.fhg.de
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
 *
 * Date of creation: Feb 2009
 * ToDo: - Assign Adsress of digital input for homing switch "iHomeDigIn" via parameters (in evalReceived Message, Line 116).
 *	   - Homing Event should be defined by a parameterfile and handed to CanDrive... e.g. via the DriveParam.h (in inithoming, Line 531).
 *		 - Check whether "requestStatus" can/should be done in the class implementing the component
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


#include "CanDriveWhistle.h"

//-----------------------------------------------
CanDriveWhistle::CanDriveWhistle()
{

	m_pCanCtrl = NULL;

	m_iTypeMotion = MOTIONTYPE_POSCTRL;

	m_iMotorState = ST_PRE_INITIALIZED;
	m_iNewMotorState = ST_PRE_INITIALIZED;

	m_iStatusRegister = 0;
	m_bCurrentLimitOn = false;
	m_bFailureDetected = false;

	m_StartTime.SetNow();
	m_VelCalcTime.SetNow();
	m_CurrentTime.SetNow();
	
	m_dPosGearMeasRad = 0;
	m_dPosGearMemoRad  = 0;
	m_dVelGearMeasRadS = 0;
	m_dMotorCurrAmps = 0;

	m_bIsInitialized = false;
	m_bIsPresent = false;
	m_bStatusOk = false;
	m_bMotorOn = false;
	m_bTargetReached = false;
	m_bStatusWordReceived = false;

	m_bWatchdogActive = false;

	m_dOldPos = 0;

	m_bLimSwLeft = false; // Not used
	m_bLimSwRight = false; // Not used
	m_bSwitchHome = false; // Not used

	m_iTorqueCtrl = 0; // Not used
	m_iDistSteerAxisToDriveWheelMM=0; // Not Used

}

CanDriveWhistle::~CanDriveWhistle()
{
	if (m_bIsInitialized)
	{
		//delete m_pCanCtrl; ->> This is done only once at Platform driver level not for each Whistle in the CAN network
		m_bIsInitialized=false;
	}
}

//-----------------------------------------------
void CanDriveWhistle::setCanOpenParam( int iTxPDO1, int iTxPDO2, int iRxPDO2, int iTxSDO, int iRxSDO )
{
	m_ParamCanOpen.iTxPDO1 = iTxPDO1;
	m_ParamCanOpen.iTxPDO2 = iTxPDO2;
	m_ParamCanOpen.iRxPDO2 = iRxPDO2;
	m_ParamCanOpen.iTxSDO = iTxSDO;
	m_ParamCanOpen.iRxSDO = iRxSDO;
}

//-----------------------------------------------
void CanDriveWhistle::setCanOpenParam(int iID)
{
	m_ParamCanOpen.iCanID = iID;
	m_ParamCanOpen.iEMCY = CAN_MSG_EMCY_0 + iID;
	m_ParamCanOpen.iTxPDO1 = CAN_MSG_TxPDO1_0 + iID;
	m_ParamCanOpen.iRxPDO1 = CAN_MSG_RxPDO1_0 + iID;
	m_ParamCanOpen.iTxPDO2 = CAN_MSG_TxPDO2_0 + iID;
	m_ParamCanOpen.iRxPDO2 = CAN_MSG_RxPDO2_0 + iID;
	m_ParamCanOpen.iTxPDO3 = CAN_MSG_TxPDO3_0 + iID;
	m_ParamCanOpen.iRxPDO3 = CAN_MSG_RxPDO3_0 + iID;
	m_ParamCanOpen.iTxPDO4 = CAN_MSG_TxPDO4_0 + iID;
	m_ParamCanOpen.iRxPDO4 = CAN_MSG_RxPDO4_0 + iID;
	m_ParamCanOpen.iTxSDO = CAN_MSG_TxSDO_0 + iID;
	m_ParamCanOpen.iRxSDO = CAN_MSG_RxSDO_0 + iID;
	m_ParamCanOpen.iErrorControl = CAN_MSG_ErrorControl_0 + iID;
}

//-----------------------------------------------
void CanDriveWhistle::setDriveName(std::string sName)
{
	m_sName=sName;
}

//-----------------------------------------------
bool CanDriveWhistle::evalReceivedMsg(CanMsg& msg)
{
	bool bRet = false;
	int iTypeOfMsg;

	m_CanMsgLast = msg;

	if ((iTypeOfMsg = getMsgType(msg.getID())) < 0)
	{
		return bRet;
	}

	switch (iTypeOfMsg)
	{
		case CAN_MSG_EMCY_0:
			evalMsgEMCY(msg);
			break;
		case CAN_MSG_ErrorControl_0:
			evalMsgErrorControl(msg);
			break;
		case CAN_MSG_TxPDO1_0:
			evalMsgTxPDO1(msg);
			break;
		case CAN_MSG_RxPDO1_0:
			//* should never receive a message with this ID, this ID is for sending messages to the whistle
			std::cout << "CanDriveWhistle::evalReceivedMsg : Received message with ID of RxPDO1 " << msg.getID() << std::endl;
			break;
		case CAN_MSG_TxPDO2_0:
			evalMsgTxPDO2(msg);
			break;
		case CAN_MSG_RxPDO2_0:
			//* should never receive a message with this ID, this ID is for sending messages to the whistle
			std::cout << "CanDriveWhistle::evalReceivedMsg : Received message with ID of RxPDO2 " << msg.getID() << std::endl;
			break;
		case CAN_MSG_TxPDO3_0:
			evalMsgTxPDO3(msg);
			break;
		case CAN_MSG_RxPDO3_0:
			//* should never receive a message with this ID, this ID is for sending messages to the whistle
			std::cout << "CanDriveWhistle::evalReceivedMsg : Received message with ID of RxPDO3 " << msg.getID() << std::endl;
			break;
		case CAN_MSG_TxPDO4_0:
			//evalMsgTxPDO4(msg);
			break;
		case CAN_MSG_RxPDO4_0:
			//* should never receive a message with this ID, this ID is for sending messages to the whistle
			std::cout << "CanDriveWhistle::evalReceivedMsg : Received message with ID of RxPDO4 " << msg.getID() << std::endl;
			break;
		case CAN_MSG_TxSDO_0:
			evalMsgTxSDO(msg);
			break;
		case CAN_MSG_RxSDO_0:
			//* should never receive a message with this ID, this ID is for sending messages to the whistle
			std::cout << "CanDriveWhistle::evalReceivedMsg : Received message with ID of RxSDO " << msg.getID() << std::endl;
			break;
	}
	bRet=true;

	m_WatchdogTime.SetNow();

	return bRet;
}

//-----------------------------------------------
int CanDriveWhistle::getMsgType(int iID)
{
	int ID = iID & 0x007f;
	if (ID != m_ParamCanOpen.iCanID)
	{
		std::cout << "CanDriveWhistle::getMsgType : Not matching ID on motor: " << m_sName << std::endl;
		return -1;
	}
	else
		ID = iID & 0xff80;
		if (ID == CAN_MSG_EMCY_0)
			return CAN_MSG_EMCY_0;
		else if (ID == CAN_MSG_TxPDO1_0)
			return CAN_MSG_TxPDO1_0;
		else if (ID == CAN_MSG_RxPDO1_0)
			return CAN_MSG_RxPDO1_0;
		else if (ID == CAN_MSG_TxPDO2_0)
			return CAN_MSG_TxPDO2_0;
		else if (ID == CAN_MSG_RxPDO2_0)
			return CAN_MSG_RxPDO2_0;
		else if (ID == CAN_MSG_TxPDO3_0)
			return CAN_MSG_TxPDO3_0;
		else if (ID == CAN_MSG_RxPDO3_0)
			return CAN_MSG_RxPDO3_0;
		else if (ID == CAN_MSG_TxPDO4_0)
			return CAN_MSG_TxPDO4_0;
		else if (ID == CAN_MSG_RxPDO4_0)
			return CAN_MSG_RxPDO4_0;
		else if (ID == CAN_MSG_TxSDO_0)
			return CAN_MSG_TxSDO_0;
		else if (ID == CAN_MSG_RxSDO_0)
			return CAN_MSG_RxSDO_0;
		else if (ID == CAN_MSG_ErrorControl_0)
			return CAN_MSG_ErrorControl_0;
		else
		{
			std::cout << "CanDriveWhistle::getMsgType : Unknown type of message" << std::endl;
			return -1;
		}
}

//-----------------------------------------------
void CanDriveWhistle::evalMsgErrorControl(CanMsg msg)
{
	if (!m_bIsPresent)
	{
		m_bIsPresent=true;
	}
	else if(m_bWatchdogActive)
	{
		/**
		 * Whistles have a minimal implementation of the CANopen Error Control protocol.
		 * No Heartbeat messages should be received at Host apart from the Boot-Up message.
		 */
		m_WatchdogTime.SetNow();
	}
}

//-----------------------------------------------
void CanDriveWhistle::evalMsgEMCY(CanMsg msg)
{
	int errorCode, errorRegister;

	errorCode =  (msg.getAt(1) << 8) | (msg.getAt(0));
	errorRegister = (msg.getAt(2));

	if ((errorCode==0x8130)&&(errorRegister==0x11))
	{
		std::cout << "CanDriveWhistle::evalMsgEMCY: Heartbeat event emergency message received!" << std::endl;
		sendHeartbeat();
	}

	//* ToDo: additional EMCY message cases processing.

}

//-----------------------------------------------
//* eval answers from PDO1: Position & Velocity readings - transmitted after SYNC msg
void CanDriveWhistle::evalMsgTxPDO1(CanMsg msg)
{

	int iPos = (msg.getAt(3) << 24) | (msg.getAt(2) << 16)
			| (msg.getAt(1) << 8) | (msg.getAt(0) );

	m_dPosGearMeasRad = m_DriveParam.getSign() * m_DriveParam.
		PosMotIncrToPosGearRad(iPos);

	int iVel = (msg.getAt(7) << 24) | (msg.getAt(6) << 16)
			| (msg.getAt(5) << 8) | (msg.getAt(4) );

	m_dVelGearMeasRadS = m_DriveParam.getSign() * m_DriveParam.
		VelMotIncrPeriodToVelGearRadS(iVel);

#ifdef DEBUG
	std::cout << "Motor Readings 1: " << m_sName << std::endl;
	std::cout << "Pos: " << m_dPosGearMeasRad << std::endl;
	std::cout << "Vel: " << m_dVelGearMeasRadS << std::endl;
#endif

}

//-----------------------------------------------
//* eval answers from PDO3: Torque & Current readings - transmitted after SYNC msg
void CanDriveWhistle::evalMsgTxPDO3(CanMsg msg)
{
	short iTorque = (msg.getAt(1) << 8) | (msg.getAt(0) );

	short iCurrent = (msg.getAt(3) << 8) | (msg.getAt(2) );

	m_dMotorCurrAmps = m_DriveParam.getNominalCurrent()*iCurrent/1000;

	short iAnalogInput = (msg.getAt(5) << 8) | (msg.getAt(4) );

	m_dAnalogInputVolts = m_DriveParam.getAnalogFactor()*iAnalogInput;

#ifdef DEBUG
	if (m_sName[6]=='D'){
	std::cout << "Motor Readings 3: " << m_sName << std::endl;
	std::cout << "Torque: " << iTorque << std::endl;
	std::cout << "Current: " << m_dMotorCurrAmps << std::endl;
	std::cout << "Analog: " << m_dAnalogInputVolts << std::endl;
	}
#endif

}

//------------------------------------------------
//* eval answers from PDO2: Binary Interpreter
void CanDriveWhistle::evalMsgTxPDO2(CanMsg msg)
{
	int iHomeDigIn= 0x0001;
	int iParam, iFailure, iDigIn;

	if (isBitSet(msg.getAt(3),6))
	{
		std::cout << "CanDriveWhistle::evalReceivedMsg : Received error answer from binary interpreter in motor " << m_sName << " Command sent was: " << msg.getAt(0) << msg.getAt(1) << std::endl;
		// error_handling();
	}

	else if( (msg.getAt(0) == 'P') && (msg.getAt(1) == 'X') ) //* current pos
	{
	}

	else if( (msg.getAt(0) == 'P') && (msg.getAt(1) == 'A') ) //* position absolute
	{
	}

	else if( (msg.getAt(0) == 'M') && (msg.getAt(1) == 'O') ) //* Motor enable
	{
		m_bMotorOn = msg.getAt(4);
	}

	else if( (msg.getAt(0) == 'J') && (msg.getAt(1) == 'V') ) //* current velocity
	{
	}

	else if( (msg.getAt(0) == 'B') && (msg.getAt(1) == 'G') ) //* begin motion
	{
	}

	else if( (msg.getAt(0) == 'U') && (msg.getAt(1) == 'M') ) //* user mode
	{
#ifdef DEBUG
		std::cout << "CanDriveWhistle::evalReceivedMsg : Unit Mode set to: " << msg.getAt(4) << std::endl;
#endif
	}

	else if( (msg.getAt(0) == 'I') && (msg.getAt(1) == 'P') ) //* digital in
	{
		//* Digital inputs are not used in ExoTer rover
		iDigIn = 0x1FFFFF & ( (msg.getAt(7) << 24) | (msg.getAt(6) << 16)
			| (msg.getAt(5) << 8) | (msg.getAt(4)) );

		if( (iDigIn & iHomeDigIn) != 0x0000 )
		{
			m_bLimSwRight = true;
		}
	}

	else if( (msg.getAt(0) == 'S') && (msg.getAt(1) == 'R') ) //* status
	{
		m_iStatusRegister = (msg.getAt(7) << 24) | (msg.getAt(6) << 16)
			| (msg.getAt(5) << 8) | (msg.getAt(4) );

		m_bStatusOk = evalStatusRegister(m_iStatusRegister);
	}

	else if( (msg.getAt(0) == 'M') && (msg.getAt(1) == 'F') ) //* motor failure
	{
		iFailure = (msg.getAt(7) << 24) | (msg.getAt(6) << 16)
			| (msg.getAt(5) << 8) | (msg.getAt(4) );

		evalMotorFailure(iFailure);
	}

	else if( (msg.getAt(0) == 'M') && (msg.getAt(1) == 'S') ) //* motion status
	{
		iParam = (msg.getAt(7) << 24) | (msg.getAt(6) << 16)
			| (msg.getAt(5) << 8) | (msg.getAt(4) );

		if (iParam==0 || iParam==1)
		{
			m_bTargetReached=true;
		}
		else
		{
			m_bTargetReached=false;
		}
#ifdef DEBUG
		std::cout << "motion status value received: " << iParam << std::endl;
#endif
		m_bMotionStatusReceived=true;
	}

	else if( (msg.getAt(0) == 'P') && (msg.getAt(1) == 'M') ) //* profile mode
	{
		iParam = (msg.getAt(7) << 24) | (msg.getAt(6) << 16)
			| (msg.getAt(5) << 8) | (msg.getAt(4) );
#ifdef DEBUG
		std::cout << "pm " << iParam << std::endl;
#endif
	}

	else if( (msg.getAt(0) == 'A') && (msg.getAt(1) == 'C') ) //* acceleration
	{
		iParam = (msg.getAt(7) << 24) | (msg.getAt(6) << 16)
			| (msg.getAt(5) << 8) | (msg.getAt(4) );
#ifdef DEBUG
		std::cout << "ac " << iParam << std::endl;
#endif
	}

	else if( (msg.getAt(0) == 'D') && (msg.getAt(1) == 'C') ) //* deceleration
	{
		iParam = (msg.getAt(7) << 24) | (msg.getAt(6) << 16)
			| (msg.getAt(5) << 8) | (msg.getAt(4) );
#ifdef DEBUG
		std::cout << "dc " << iParam << std::endl;
#endif
	}
	else if( (msg.getAt(0) == 'H') && (msg.getAt(1) == 'M') ) //* homing
	{
		//* ExoTer does not use Homing methods with Limit Switches

		// homing status message (homing armed = 1 / disarmed = 0) is encoded in 5th byte
		if(msg.getAt(4) == 0)
		{
			// if 0 received: elmo disarmed homing after receiving the defined event
			m_bLimSwRight = true;
		}
	}
	else if( (msg.getAt(0) == 'I') && (msg.getAt(1) == 'Q') ) // Active current
	{
		int iVal=0;
		iVal = (msg.getAt(7) << 24) | (msg.getAt(6) << 16)
			| (msg.getAt(5) << 8) | (msg.getAt(4) );
		float* pfVal;
		pfVal=(float*)&iVal;
		m_dMotorCurrAmps = *pfVal;
//#ifdef DEBUG
		std::cout << "Motor " << m_sName << ", IQ " << *pfVal << std::endl;
//#endif
	}
	else if( (msg.getAt(0) == 'A') && (msg.getAt(1) == 'N') ) // Analog Input
	{
		int iVal=0;
		iVal = (msg.getAt(7) << 24) | (msg.getAt(6) << 16)
					| (msg.getAt(5) << 8) | (msg.getAt(4) );
		float* pfVal;
		pfVal=(float*)&iVal;
		m_dAnalogInputVolts=*pfVal;
//#ifdef DEBUG
		std::cout << "Motor " << m_sName << ", AN " << *pfVal << std::endl;
//#endif
	}
	else if( (msg.getAt(0) == 'C') && (msg.getAt(1) == 'L') ) // Analog Input
	{
		int iVal=0;
		iVal = (msg.getAt(7) << 24) | (msg.getAt(6) << 16)
					| (msg.getAt(5) << 8) | (msg.getAt(4) );
		float* pfVal;
		pfVal=(float*)&iVal;
#ifdef DEBUG
		std::cout << "Motor " << m_sName << ", CL " << *pfVal << std::endl;
#endif
	}
	else
	{
		// Add more cases
	}
}

//-----------------------------------------------
//* eval answer from SDO
void CanDriveWhistle::evalMsgTxSDO(CanMsg msg)
{
	if((msg.getAt(0) >> 5) == 2)
	{
		//* Received SDO Upload data (scs = 2)
#ifdef DEBUG
		std::cout << "SDO Upload data received" << std::endl;
#endif
		if (isBitSet(msg.getAt(0), 1))
		{
			//* Only treatment for expedited SDO is implemented
			int iIndex, iSubIndex;
			int iData;

			evalSDO(msg, &iIndex, &iSubIndex);
			iData = getSDODataInt32(msg);

			//* add cases of data processing according to requested SDO data

			/// Status Word
			if (iIndex==0x6041)
			{
				evalStatusWord(iData);
			}

			// other cases
		}
	}
	else if((msg.getAt(0) >> 5) == 3)
	{
		//* Received SDO Download confirmation (scs = 3)
#ifdef DEBUG
		std::cout << "SDO Download confirmation received" << std::endl;
#endif
	}
}

//-----------------------------------------------
bool CanDriveWhistle::init()
{
	//* Send Reset command to:
	//* 1. restart the node in case it was left in an operational state.
	//* 2. receive the boot-up message to confirm the presence of the drive.
	//sendNMTMsg(CMD_NMT_SW_RESET); //! Note: The reset command is implemented in the reset() method now.

	std::cout << "CanDriveWhistle::init: Start init for drive " << m_sName << std::endl;

	int cnt=0;
	while(!m_bIsPresent && cnt < 100)
	{
		usleep(10000);
		cnt++;
	}

	if(!m_bIsPresent)
	{
		std::cout << "CanDriveWhistle::init: Drive not present! " << m_sName << std::endl;
		return false;
	}

	//* Enter NMT operational state to allow sending and receiving PDOs
	sendNMTMsg(CMD_NMT_START);
	m_iMotorState = ST_OPERATION_ENABLED;

	if (m_DriveParam.getIsSteer())
	{
		setTypeMotion(MOTIONTYPE_POSCTRL);
	}
	else
	{
		setTypeMotion(MOTIONTYPE_VELCTRL);
		//* set position counter to zero for the wheel drive motors only. This does not drive the motor to any position.
		IntprtSetInt(8, 'P', 'X', 0, 0);
	}

	//* change this function to adapt to your specific application
	setPDOmappings();

	m_bIsInitialized = true;

	return true;
}

//-----------------------------------------------
void CanDriveWhistle::setDriveLimits()
{
	//* ToDo: Add a function for parameter initialization that takes values from DriveParam

	//* Set Values for Modulo-Counting. Necessary to preserve absolute position for homed motors (after encoder overflow)

	//int iIncrRevWheel = int( (double)m_DriveParam.getGearRatio() * (double)m_DriveParam.getBeltRatio()
	//				* (double)m_DriveParam.getEncIncrPerRevMot() * 3 );
	//IntprtSetInt(8, 'M', 'O', 0, 0);
	//usleep(20000);
	//IntprtSetInt(8, 'X', 'M', 2, iIncrRevWheel * 500);
	//usleep(20000);
	//IntprtSetInt(8, 'X', 'M', 1, -iIncrRevWheel * 500);
	//usleep(20000);
}

//-----------------------------------------------
void CanDriveWhistle::sendNMTMsg(BYTE cmd)
{
	CanMsg msg;

	msg.setID(0);
	msg.setLength(2);
	msg.set(cmd,m_ParamCanOpen.iCanID,0,0,0,0,0,0);
	m_pCanCtrl->transmitMsg(msg, false);

	usleep(100000);
}

//-----------------------------------------------
void CanDriveWhistle::setPDOmappings()
{
	// ---------- set PDO mapping
	// Mapping of TPDO1:
	// - position
	// - velocity
	
	// stop all emissions of TPDO1
	sendSDODownload(0x1A00, 0, 0);
	
	// position 4 byte of TPDO1
	sendSDODownload(0x1A00, 1, 0x60640020);

	// velocity 4 byte of TPDO1
	sendSDODownload(0x1A00, 2, 0x60690020);
	
	// transmission type "sync"
	sendSDODownload(0x1800, 2, 1);
	
	// activate mapped objects
	sendSDODownload(0x1A00, 0, 2);

	// ---------- set PDO mapping
	// Mapping of TPDO3:
	// - effort
	// - current


	// stop all emissions of TPDO3
	sendSDODownload(0x1A02, 0, 0);

	// torque 2 byte of TPDO3
	sendSDODownload(0x1A02, 1, 0x60770010);

	// current 2 byte of TPDO3
	sendSDODownload(0x1A02, 2, 0x60780010);

	// analog input 2 byte of TPDO3
	sendSDODownload(0x1A02, 3, 0x22050110);

	// transmission type "sync"
	sendSDODownload(0x1802, 2, 1);

	// activate mapped objects
	sendSDODownload(0x1A02, 0, 3);


}

//-----------------------------------------------
bool CanDriveWhistle::stop()
{	
	positionCommandRad(m_dPosGearMeasRad,0);

	usleep(20000);

	return true;
}
//-----------------------------------------------
bool CanDriveWhistle::start()
{
	bool bRet = true;

	//* ------------------- Enable motor
	IntprtSetInt(8, 'M', 'O', 0, 1);
	usleep(20000);

	int cnt=0;
	do
	{
		usleep(10000);
		cnt++;
	}
	while(!m_bMotorOn && cnt < 100);

	if(!m_bMotorOn)
	{
		std::cout << "CanDriveWhistle::start: Drive not started! " << m_sName <<std::endl;
		bRet = false;
	}

	//* ------------------- Request status
	IntprtSetInt(4, 'S', 'R', 0, 0);
	usleep(20000);
	
	cnt=0;
	do
	{
		usleep(10000);
		cnt++;
	}
	while(!m_bStatusOk && cnt < 100);

	if(!m_bStatusOk)
	{
		std::cout << "CanDriveWhistle::start: Status NOT OK! " << m_sName << std::endl;
		bRet = false;
	}

	return bRet;
}

//-----------------------------------------------
bool CanDriveWhistle::reset()
{
	//* Reset the whistle with a NMT command
	m_bIsPresent=false;
	m_bIsInitialized = false;
	sendNMTMsg(CMD_NMT_SW_RESET);

	return true;
}
//-----------------------------------------------
bool CanDriveWhistle::shutdown()
{
	bool bRet = true;

	//std::cout << "...shutting down drive " << m_sName << std::endl;

	IntprtSetInt(8, 'M', 'O', 0, 0);
	usleep(20000);

	int cnt=0;
	do
	{
		usleep(10000);
		cnt++;
	}
	while(m_bMotorOn && cnt < 100);

	if(m_bMotorOn)
	{
		std::cout << "CanDriveWhistle::shutdown: Drive still running! " << m_sName << std::endl;
		bRet = false;
	}

	return bRet;
}

//-----------------------------------------------
bool CanDriveWhistle::startWatchdog(bool bStart)
{
	if (bStart == true)
	{
		//* Watchdog setup
		//* Whistle checks Master (Host) hearbeat only
		//* note: the COB-ID for a heartbeat message = 0x700 + Device ID
		const int c_iHeartbeatTimeMS = 15000;
		const int c_iNMTNodeID = 0x00;
		
		//* consumer (Host) heartbeat time
		sendSDODownload(0x1016, 1, (c_iNMTNodeID << 16) | c_iHeartbeatTimeMS);
 		
		//* error behavior after failure: 0=pre-operational, 1=no state change, 2=stopped"
		sendSDODownload(0x1029, 1, 2);
		
		//* motor behavior after heartbeat failure: "quick stop"
		sendSDODownload(0x6007, 0, 3);

		//* activate emergency events: "heartbeat event"
		//* Object 0x2F21 = "Emergency Events" which cause an Emergency Message
		//* Bit 3 is responsible for Heartbeart-Failure.--> Hex 0x08
		sendSDODownload(0x2F21, 0, 0x08);
		usleep(20000);

		m_bWatchdogActive = true;
 
	}
	else
	{	
		//* Motor action after Heartbeat-Error: No Action
		sendSDODownload(0x6007, 0, 0);

		//* Error Behavior: No state change
		sendSDODownload(0x1029, 1, 1);

		//* Deactivate emergency events: "heartbeat event"
		//* Object 0x2F21 = "Emergency Events" which cause an Emergency Message
		//* Bit 3 is responsible for Heartbeart-Failure.
		sendSDODownload(0x2F21, 0, 0x00);
		usleep(20000);

		m_bWatchdogActive = false;
		
	}

	return true;
}

//-----------------------------------------------
double CanDriveWhistle::getTimeToLastMsg()
{
	m_CurrentTime.SetNow();

	return m_CurrentTime - m_WatchdogTime;
}
//-----------------------------------------------
bool CanDriveWhistle::getStatusLimitSwitch()
{
	return m_bLimSwRight;
}
//-----------------------------------------------
bool CanDriveWhistle::initHoming()
{
	/**
	 * This Homing function is not used in ExoTer but the code is kept as valuable example for homing procedures using limit switches.
	 */

	const int c_iPosRef = m_DriveParam.getEncOffset();
	
	// 1. make sure that, if on elmo controller still a pending homing from a previous startup is running (in case of warm-start without switching of the whole robot), this old sequence is disabled
	// disarm homing process
	IntprtSetInt(8, 'H', 'M', 1, 0);

	// always give can and controller some time to understand the command
	usleep(20000);

	// THIS is needed for head_axis on cob3-2!

	//set input logic to 'general purpose'
	IntprtSetInt(8, 'I', 'L', 2, 7);
	usleep(20000);


	// 2. configure the homing sequence
	// 2.a set the value to which the increment counter shall be reseted as soon as the homing event occurs
	// value to load at homing event
	IntprtSetInt(8, 'H', 'M', 2, c_iPosRef);
	usleep(20000);

	// 2.b choose the chanel/switch on which the controller listens for a change or defined logic level (the homing event) (high/low/falling/rising)
	// home event
	// iHomeEvent = 5 : event according to defined FLS switch (for scara arm)
	// iHomeEvent = 9 : event according to definded DIN1 switch (for full steerable wheels COb3)
	// iHomeEvent =11 : event according to ?? (for COb3 Head-Axis)
	IntprtSetInt(8, 'H', 'M', 3, m_DriveParam.getHomingDigIn());
	//IntprtSetInt(8, 'H', 'M', 3, 11); //cob3-2
	usleep(20000);


	// 2.c choose the action that the controller shall perform after the homing event occured
	// HM[4] = 0 : after Event stop immediately
	// HM[4] = 2 : Do nothing!
	IntprtSetInt(8, 'H', 'M', 4, 2);
	usleep(20000);

	// 2.d choose the setting of the position counter (i.e. to the value defined in 2.a) after the homing event occured
	// HM[5] = 0 : absolute setting of position counter: PX = HM[2]
	IntprtSetInt(8, 'H', 'M', 5, 0);
	usleep(20000);

	// 3. let the motor turn some time to give him the possibility to escape the approximation sensor if accidently in home position already at the beginning of the sequence (done in CanCtrlPltf...)

	return true;	
}


//-----------------------------------------------
bool CanDriveWhistle::execHoming()
{

	/**
	 * This Homing function is not used in ExoTer but the code is kept as valuable example for homing procedures using limit switches.
	 */

	int iCnt;
	CanMsg Msg;
	bool bRet = true;

	// 4. arm the homing process -> as soon as the approximation sensor is reached and the homing event occurs the commands set in 2. take effect
	// arm homing process
	IntprtSetInt(8, 'H', 'M', 1, 1);
		
	// 5. clear the can buffer to get rid of all uneccessary and potentially disturbing commands still floating through the wires
	do
	{
		// read from can
		bRet = m_pCanCtrl->receiveMsg(&Msg);
	}
	while(bRet == true);

	// 6. now listen for status of homing, to synchronize program flow -> proceed only after homing was succesful (homing disarmed by elmo) or timeout occured

	// set timeout counter to zero
	iCnt = 0;

	do
	{
		// 6.a ask for status of homing process (armed/disarmed)
		// ask for first byte in Homing Configuration
		IntprtSetInt(4, 'H', 'M', 1, 0);

		// 6.b read message from can
		m_pCanCtrl->receiveMsgRetry(&Msg, 10);
		
		// 6.c see if received message is answer of request and if so what is the status
		if( (Msg.getAt(0) == 'H') && (Msg.getAt(1) == 'M') )
		{	
			// status message (homing armed = 1 / disarmed = 0) is encoded in 5th byte
			if(Msg.getAt(4) == 0)
			{
				// if 0 received: elmo disarmed homing after receiving the defined event
				std::cout << "Got Homing-Signal "  << std::endl;
				m_bLimSwRight = true;
				break;
			}
		}

		// increase count for timeout
		usleep(10000);
		iCnt++;

	}
	while((m_bLimSwRight == false) && (iCnt<2000)); // wait some time
	
	// 7. see why finished (homed or timeout) and log out
	if(iCnt>=2000)
	{
		std::cout << "Homing failed - limit switch " << m_sName << " not reached" << std::endl;
		bRet = false;
	}
	else
	{
		std::cout << "Homing successful - limit switch " << m_sName << " ok" << std::endl;
		bRet = true;
	}
	IntprtSetInt(8, 'I', 'L', 2, 9);  // cob3-2
    usleep(20000);

	return bRet;
}

//-----------------------------------------------
void CanDriveWhistle::Homing()
{
	/**
	 * This is the Homing function that is used in ExoTer.
	 * This simple procedure is possible thanks to the the absolute position initialization by means of the potentiometer.
	 */
	positionCommandRad(0, 0.25);
}

//-----------------------------------------------
void CanDriveWhistle::positionCommandRad(double dPosGearRad, double dVelGearRadS)
{
	int iPosEncIncr;
	int iVelEncIncrPeriod;

	if(m_iTypeMotion == MOTIONTYPE_POSCTRL)
	{

		m_DriveParam.PosVelRadToIncr(dPosGearRad, dVelGearRadS, &iPosEncIncr, &iVelEncIncrPeriod);
		
		if(iVelEncIncrPeriod > m_DriveParam.getVelMax())
		{
			iVelEncIncrPeriod = (int)m_DriveParam.getVelMax();
			std::cout << "CanDriveWhistle::positionCommand : Limit velocity to maximum allowed" << std::endl;
		}
	
		if(iVelEncIncrPeriod < -m_DriveParam.getVelMax())
		{
			iVelEncIncrPeriod = (int)-m_DriveParam.getVelMax();
			std::cout << "CanDriveWhistle::positionCommand : Limit velocity to maximum allowed" << std::endl;
		}

		if(iPosEncIncr > m_DriveParam.getPosMax())
		{
			iPosEncIncr = (int)m_DriveParam.getPosMax();
			std::cout << "CanDriveWhistle::positionCommand : Limit position to maximum allowed" << std::endl;
		}

		if(iPosEncIncr < m_DriveParam.getPosMin())
		{
			iPosEncIncr = (int)m_DriveParam.getPosMin();
			std::cout << "CanDriveWhistle::positionCommand : Limit position to maximum allowed" << std::endl;
		}

		if (iVelEncIncrPeriod==0)
			iVelEncIncrPeriod=m_DriveParam.getPtpVelDefault();

		//* Set max Velocity during PTP Motion
		IntprtSetInt(8, 'S', 'P', 0, iVelEncIncrPeriod);

		//* Set Position Reference for PTP Motion
		if (m_DriveParam.getIsSteer() == true)
			//* Use absolute positioning on motors that have absolute position initialization (Steered motors)
			IntprtSetInt(8, 'P', 'A', 0, iPosEncIncr);
		else
			//* Use relative positioning on motors that don't have absolute position initialization (Driving motors)
			IntprtSetInt(8, 'P', 'R', 0, iPosEncIncr);

		//* Execute command
		IntprtSetInt(4, 'B', 'G', 0, 0);
		
	}
	else
	{	
		std::cout << "CanDriveWhistle::positionCommand : Position Command is not allowed in NON Position control mode!" << std::endl;
	}
	
}

//-----------------------------------------------
void CanDriveWhistle::positionSetPointRad(double dPosGearRad, double dVelGearRadS)
{
	int iPosEncIncr;
	int iVelEncIncrPeriod;

	if(m_iTypeMotion == MOTIONTYPE_POSCTRL)
	{

		m_DriveParam.PosVelRadToIncr(dPosGearRad, dVelGearRadS, &iPosEncIncr, &iVelEncIncrPeriod);

		if(iVelEncIncrPeriod > m_DriveParam.getVelMax())
		{
			iVelEncIncrPeriod = (int)m_DriveParam.getVelMax();
		}

		if(iVelEncIncrPeriod < -m_DriveParam.getVelMax())
		{
			iVelEncIncrPeriod = (int)-m_DriveParam.getVelMax();
		}

		if(iPosEncIncr > m_DriveParam.getPosMax())
		{
			iPosEncIncr = (int)m_DriveParam.getPosMax();
			std::cout << "CanDriveWhistle::positionCommand : Limit position to maximum allowed" << std::endl;
		}

		if(iPosEncIncr < m_DriveParam.getPosMin())
		{
			iPosEncIncr = (int)m_DriveParam.getPosMin();
			std::cout << "CanDriveWhistle::positionCommand : Limit position to maximum allowed" << std::endl;
		}

		if (iVelEncIncrPeriod==0)
			iVelEncIncrPeriod=m_DriveParam.getPtpVelDefault();

		//* Set max Velocity during PTP Motion
		IntprtSetInt(8, 'S', 'P', 0, iVelEncIncrPeriod);

		//* Set Position Reference for PTP Motion
		if (m_DriveParam.getIsSteer() == true)
			//* Use absolute positioning on motors that have absolute position initialization (Steered motors)
			IntprtSetInt(8, 'P', 'A', 0, iPosEncIncr);
		else
			//* Use relative positioning on motors that don't have absolute position initialization (Driving motors)
			IntprtSetInt(8, 'P', 'R', 0, iPosEncIncr);

	}
	else
	{
		std::cout << "CanDriveWhistle::positionSetPoint : Position Command is not allowed in NON Position control mode!" << std::endl;
	}
}

//-----------------------------------------------
void CanDriveWhistle::velocityCommandRadS(double dVelGearRadS)
{
	int iVelEncIncrPeriod;

        //std::cout<<"velocity in library: "<<dVelGearRadS<<" motor is "<<m_sName<<"\n";
	
	if(m_iTypeMotion == MOTIONTYPE_VELCTRL)
	{
		//* calculate motor velocity from joint velocity
		iVelEncIncrPeriod = m_DriveParam.getSign() * m_DriveParam.VelGearRadSToVelMotIncrPeriod(dVelGearRadS);

		if(iVelEncIncrPeriod > m_DriveParam.getVelMax())
		{
			std::cout << "Velocity limit exceeded. Asked for " << iVelEncIncrPeriod << " EncIncrements on " << m_sName << std::endl;
			iVelEncIncrPeriod = (int)m_DriveParam.getVelMax();
		}

		if(iVelEncIncrPeriod < -m_DriveParam.getVelMax())
		{
			std::cout << "Velocity limit exceeded. Asked for " << iVelEncIncrPeriod << " EncIncrements" << std::endl;
			iVelEncIncrPeriod = -1 * (int)m_DriveParam.getVelMax();
		}

                //std::cout<<"sending: "<<iVelEncIncrPeriod<<"\n";
		//* Set Velocity Reference
		IntprtSetInt(8, 'J', 'V', 0, iVelEncIncrPeriod);
		//* Execute command
		IntprtSetInt(4, 'B', 'G', 0, 0);
	}
	else
	{
		std::cout << "CanDriveWhistle::velocityCommand : Velocity Command is not allowed in NON Velocity control mode!" << std::endl;
	}
}

//-----------------------------------------------
void CanDriveWhistle::velocitySetPointRadS(double dVelGearRadS)
{
	int iVelEncIncrPeriod;

	if(m_iTypeMotion == MOTIONTYPE_VELCTRL)
	{
		//* calculate motor velocity from joint velocity
		iVelEncIncrPeriod = m_DriveParam.getSign() * m_DriveParam.VelGearRadSToVelMotIncrPeriod(dVelGearRadS);

		if(iVelEncIncrPeriod > m_DriveParam.getVelMax())
		{
			std::cout << "Velocity limit exceeded. Asked for " << iVelEncIncrPeriod << " EncIncrements on" << m_sName  << std::endl;
			iVelEncIncrPeriod = (int)m_DriveParam.getVelMax();
		}

		if(iVelEncIncrPeriod < -m_DriveParam.getVelMax())
		{
			std::cout << "Velocity limit exceeded. Asked for " << iVelEncIncrPeriod << " EncIncrements on " << m_sName  << std::endl;
			iVelEncIncrPeriod = -1 * (int)m_DriveParam.getVelMax();
		}

		//* Set Velocity Reference
		IntprtSetInt(8, 'J', 'V', 0, iVelEncIncrPeriod);

	}
	else
	{
		std::cout << "CanDriveWhistle::velocitySetPoint : Velocity Command is not allowed in NON Velocity control mode!" << std::endl;
	}
}

//-----------------------------------------------
void CanDriveWhistle::commandSetPoint()
{
	//* Send motion begin command for previously set but not executed motion.
	IntprtSetInt(4, 'B', 'G', 0, 0);
}

//-----------------------------------------------
void CanDriveWhistle::getVelocityRadS(double* dGearVelRadS)
{
	*dGearVelRadS = m_dVelGearMeasRadS;
}

//-----------------------------------------------
void CanDriveWhistle::getPositionVelocityRadS(double* pdAngleGearRad, double* pdVelGearRadS)
{
	*pdAngleGearRad = m_dPosGearMeasRad;
	*pdVelGearRadS = m_dVelGearMeasRadS;
}

//-----------------------------------------------
void CanDriveWhistle::getDeltaPositionVelocityRadS(double* pdAngleGearRad, double* pdVelGearRadS)
{
	*pdAngleGearRad = m_dPosGearMeasRad - m_dPosGearMemoRad;
	*pdVelGearRadS = m_dVelGearMeasRadS;
	m_dPosGearMemoRad = m_dPosGearMeasRad;
}

//-----------------------------------------------
void CanDriveWhistle::getData(double* pdPosGearRad, double* pdVelGearRadS,
								int* piTorqueCtrl, int* piStatusCtrl)
{
	*pdPosGearRad = m_dPosGearMeasRad;
	*pdVelGearRadS = m_dVelGearMeasRadS;
	*piTorqueCtrl = m_iTorqueCtrl;
	*piStatusCtrl = m_iStatusRegister;
}

//-----------------------------------------------
void CanDriveWhistle::getData(double* pdPosGearRad, double* pdVelGearRadS,
		double* pdCurrentAmp, double* pdTorqueNm)
{
	*pdPosGearRad = m_dPosGearMeasRad;
	*pdVelGearRadS = m_dVelGearMeasRadS;
	*pdCurrentAmp = m_dMotorCurrAmps;
	*pdTorqueNm = m_dMotorCurrAmps * m_DriveParam.getCurrToTorque();
}

//-----------------------------------------------
void CanDriveWhistle::getAnalogInput(double* pdAnalogInput)
{
	*pdAnalogInput = m_dAnalogInputVolts;
}

//-----------------------------------------------
double CanDriveWhistle::estimVel(double dPos)
{
	double dVel;
	double dt;

	m_CurrentTime.SetNow();

	dt = m_CurrentTime - m_VelCalcTime;

	dVel = (dPos - m_dOldPos)/dt;

	m_dOldPos = dPos;
	m_VelCalcTime.SetNow();

	return dVel;
}

//-----------------------------------------------
void CanDriveWhistle::requestPosVel()
{
	//* Request of pos and vel done by TPDO1, triggered by SYNC msg
	sendSync();

	//* Note: to request pos by SDO use sendSDOUpload(0x6064, 0)
}

//-----------------------------------------------
void CanDriveWhistle::sendSync()
{
	CanMsg msg;
	msg.setID(0x80);
	msg.setLength(0);
	msg.set(0,0,0,0,0,0,0,0);
	m_pCanCtrl->transmitMsg(msg);
}

//-----------------------------------------------
void CanDriveWhistle::sendHeartbeat()
{
	CanMsg msg;
	msg.setID(0x700);
	msg.setLength(5);
	msg.set(0x00,0,0,0,0,0,0,0);
	m_pCanCtrl->transmitMsg(msg);
}

//-----------------------------------------------
bool CanDriveWhistle::checkWatchdogTime()
{
	double dWatchTime = getTimeToLastMsg();
	if (dWatchTime > CAN_WATCHDOG_TIMEOUT_SEC)
	{
		std::cout << "Reached watchdog timeout of drive " << m_sName << std::endl;
		return true;
	}
	return false;
}

//-----------------------------------------------
void CanDriveWhistle::requestStatus()
{
	IntprtSetInt(4, 'S', 'R', 0, 0);
}

//-----------------------------------------------
void CanDriveWhistle::requestTorque()
{	
   	//* send command for requesting motor active current
 	IntprtSetInt(4, 'I', 'Q', 0, 0);

 	//* Note: For reactive current value use command: IntprtSetInt(4, 'I', 'D', 0, 0);
}

//-----------------------------------------------
void CanDriveWhistle::torqueCommandNm(double dTorqueNm)
{
	if(m_iTypeMotion == MOTIONTYPE_TORQUECTRL)
	{
		//* convert commanded motor torque into amperes
		float fMotCurr = m_DriveParam.getSign() * dTorqueNm / m_DriveParam.getCurrToTorque();

		//* check for limitations
		if  (fMotCurr > m_DriveParam.getCurrMax())
		{
			fMotCurr = m_DriveParam.getCurrMax();
			std::cout << "Torque command too high: " << fMotCurr << " Amps. Torque has been limited to max current." << std::endl;
		}
		if (fMotCurr < -m_DriveParam.getCurrMax())
		{
			fMotCurr = -m_DriveParam.getCurrMax();
			std::cout << "Torque command too high: " << fMotCurr << " Amps. Torque has been limited to min current." << std::endl;
		}

		//* send Command
		m_bTargetReached = false;
		IntprtSetFloat(8, 'T', 'C', 0, fMotCurr);
	}
	else
		std::cout << "CanDriveWhistle::torqueCommand : Torque Command is not allowed in NON torque control mode!" << std::endl;
}

//-----------------------------------------------
void CanDriveWhistle::getTorqueNm(double* dTorqueNm)
{
	//* Note the motor sign
	*dTorqueNm = m_DriveParam.getSign() * m_dMotorCurrAmps * m_DriveParam.getCurrToTorque();
}

//-----------------------------------------------
bool CanDriveWhistle::isError()
{
	if (m_iMotorState != ST_MOTOR_FAILURE)
	{
		if (checkWatchdogTime())
		{
			m_iMotorState = ST_MOTOR_FAILURE;
			m_FailureStartTime.SetNow();
		}
	}
	return (m_iMotorState == ST_MOTOR_FAILURE);
}

//-----------------------------------------------
bool CanDriveWhistle::evalStatusRegister(int iStatus)
{
	bool bNoError;

	//* check Error status
	if(isBitSet(iStatus, 0))
	{
		//* Error detected
		if (m_bFailureDetected == false)
		{
			std::cout << "Error of drive: " << m_sName << std::endl;

			iStatus&=0x0000000E;

			if( iStatus == 2)
				std::cout << "- drive error under voltage" << std::endl;

			else if( iStatus == 4)
				std::cout << "- drive error over voltage" << std::endl;

			else if( iStatus == 10)
				std::cout << "- drive error short circuit" << std::endl;

			else if( iStatus == 12)
				std::cout << "- drive error overheating" << std::endl;

			else if( iStatus == 0) //! For completeness
				std::cout << "- drive error OK" << std::endl;

			m_FailureStartTime.SetNow();

			//* Request detailed description of failure
			IntprtSetInt(4, 'M', 'F', 0, 0);

			if (isBitSet(iStatus, 6)) //! Note: Maybe this case never happens, both bit 0 and 6 up at the same SR.
			{
				std::cout << "Motor " << m_sName << " failure latched" << std::endl;
			}
		}
		m_iNewMotorState = ST_MOTOR_FAILURE;
		m_bFailureDetected = true;
		bNoError = false;
	}
	else if (isBitSet(iStatus, 6))
	{
		//* General failure
		if (m_bFailureDetected == false)
		{
			std::cout << "Motor " << m_sName << " failure latched" << std::endl;

			m_FailureStartTime.SetNow();

			//* Request detailed description of failure
			IntprtSetInt(4, 'M', 'F', 0, 0);
		}
		m_iNewMotorState = ST_MOTOR_FAILURE;
		m_bFailureDetected = true;
		bNoError = false;
	}
	else
	{
		//* No error
		bNoError = true;

		//* Clear flag for failure output
		m_bFailureDetected = false;

		//* Other General status bits

		//* Check Motor is ON
		if(isBitSet(iStatus, 4))
		{
			if (m_iMotorState != ST_OPERATION_ENABLED)
			{
				//std::cout << "Motor " << m_sName << " operation enabled" << std::endl;
			}
			m_iNewMotorState = ST_OPERATION_ENABLED;
		}
		else
		{
			if (m_iMotorState != ST_OPERATION_DISABLED)
			{
				//std::cout << "Motor " << m_sName << " operation disabled" << std::endl;
			}
			m_iNewMotorState = ST_OPERATION_DISABLED;
		}

		//* Current limit
		if(isBitSet(iStatus, 13))
		{
			if (m_bCurrentLimitOn == false)
			{
				//std::cout << "Motor " << m_sName << "current limit on" << std::endl;
			}
			m_bCurrentLimitOn = true;
		}
		else
			m_bCurrentLimitOn = false;
	}

	//* Update state
	m_iMotorState = m_iNewMotorState;

	return bNoError;
}

//-----------------------------------------------
void CanDriveWhistle::evalMotorFailure(int iFailure)
{

	std::cout << "Motor " << m_sName << " has a failure: " << iFailure << std::endl;

	if( isBitSet(iFailure, 2) )
	{
		std::cout << "- feedback loss" << std::endl;
	}

	if( isBitSet(iFailure, 3) )
	{
		std::cout << "- peak current excced" << std::endl;
	}

	if( isBitSet(iFailure, 4) )
	{
		std::cout << "- inhibit" << std::endl;
	}

	if( isBitSet(iFailure, 7) )
	{
		std::cout << "- speed track error" << std::endl;
	}

	if( isBitSet(iFailure, 8) )
	{
		std::cout << "- position track error" << std::endl;
	}

	if( isBitSet(iFailure, 9) )
	{
		std::cout << "- inconsistent database" << std::endl;
	}

	if( isBitSet(iFailure, 11) )
	{
		std::cout << "- heartbeat failure" << std::endl;
	}

	if( isBitSet(iFailure, 12) )
	{
		std::cout << "- servo drive fault" << std::endl;
	}

	if( isBitSet(iFailure, 16) )
	{
		std::cout << "- failed to find electrical zero of the motor" << std::endl;
	}

	if( isBitSet(iFailure, 17) )
	{
		std::cout << "- speed limit exceeded" << std::endl;
	}

	if( isBitSet(iFailure, 18) )
	{
		std::cout << "- stack overflow. Fatal." << std::endl;
	}

	if( isBitSet(iFailure, 19) )
	{
		std::cout << "- CPU exception. Fatal." << std::endl;
	}

	if( isBitSet(iFailure, 21) )
	{
		std::cout << "- motor stuck" << std::endl;
	}

	if( isBitSet(iFailure, 22) )
	{
		std::cout << "- position limit exceeded" << std::endl;
	}

	if( isBitSet(iFailure, 29) )
	{
		std::cout << "- cannot start motor" << std::endl;
	}

}

//-----------------------------------------------
void CanDriveWhistle::setTypeMotionVariable(CanDriveWhistle::MotionType iType)
{
	m_iTypeMotion = iType;
}

CanDriveWhistle::MotionType CanDriveWhistle::getTypeMotionVariable()
{
    return m_iTypeMotion;
}

//-----------------------------------------------
bool CanDriveWhistle::setTypeMotion(MotionType iType)
{
	int iMaxAcc = int(m_DriveParam.getMaxAcc());
	int iMaxDcc = int(m_DriveParam.getMaxDec());
        bool motor_on=false;

	if (m_bMotorOn)
	{
                motor_on=true;
		//* switch off Motor to change Unit-Mode
		IntprtSetInt(8, 'M', 'O', 0, 0);

		int cnt=0;
		do
		{
			usleep(10000);
			cnt++;
		}
		while(m_bMotorOn && cnt < 100);

		if (m_bMotorOn)
		{
			std::cout << "Motor " << m_sName << " Could NOT change Unit Mode to POSITION controlled. Could NOT stop motor" << std::endl;
			return false;
		}
	}

	if (iType == MOTIONTYPE_POSCTRL)
	{
		//* Change to UnitMode = 5 (Single Loop Position Control)
		IntprtSetInt(8, 'U', 'M', 0, 5);
			
		//* set Target Radius to X Increments
		IntprtSetInt(8, 'T', 'R', 1, 15);

		//* set Target Time to X ms
		IntprtSetInt(8, 'T', 'R', 2, 100);

		//* set maximum Acceleration to X Incr/s^2
		IntprtSetInt(8, 'A', 'C', 0, iMaxAcc);

		//* set maximum Deceleration to X Incr/s^2
		IntprtSetInt(8, 'D', 'C', 0, iMaxDcc);

#ifdef DEBUG
		std::cout << "Motor " << m_sName << " Unit Mode switched to: POSITION controlled" << std::endl;
#endif
	}
	else if (iType == MOTIONTYPE_VELCTRL)
	{
		//* Change to UnitMode = 2 Motion Type = VelocityControled
		IntprtSetInt(8, 'U', 'M', 0, 2);

		//* set profiler Mode (only if Unit Mode = 2)
		IntprtSetInt(8, 'P', 'M', 0, 1);

		//* set maximum Acceleration to X Incr/s^2
		IntprtSetInt(8, 'A', 'C', 0, iMaxAcc);

		//* set maximum Deceleration to X Incr/s^2
		IntprtSetInt(8, 'D', 'C', 0, iMaxDcc);

#ifdef DEBUG
		std::cout << "Motor " << m_sName << " Unit Mode switched to: VELOCITY controlled" << std::endl;
#endif
	}
	else
	{
		//* Change to Unit-Mode 1: Torque Controlled
		IntprtSetInt(8, 'U', 'M', 0, 1);

		//* disable external compensation input to avoid noise from that input pin
		IntprtSetInt(8, 'R', 'M', 0, 0);

#ifdef DEBUG
		std::cout << "Motor " << m_sName << " Unit Mode switched to: TORQUE controlled" << std::endl;
#endif
	}
	
	//usleep(100000);
	m_iTypeMotion = iType;
        
        if (motor_on)
	{
                //* switch Motor back ON (as it was before entering the function)
		IntprtSetInt(8, 'M', 'O', 0, 1);

		int cnt=0;
		do
		{
			usleep(10000);
			cnt++;
		}
		while(!m_bMotorOn && cnt < 100);

		if (!m_bMotorOn)
		{
			std::cout << "Motor " << m_sName << " still OFF!!" << std::endl;
			return false;
		}
	}

	return true;
}


//-----------------------------------------------
void CanDriveWhistle::IntprtSetInt(int iDataLen, char cCmdChar1, char cCmdChar2, int iIndex, int iData)
{
	char cIndex[2];
	char cInt[4];
	CanMsg CMsgTr;

	CMsgTr.setID(m_ParamCanOpen.iRxPDO2);
	CMsgTr.setLength(iDataLen);

	cIndex[0] = iIndex;
	//* The two MSB must be 0. Cf. DSP 301 Implementation guide p. 39.
	cIndex[1] = (iIndex >> 8) & 0x3F;

	cInt[0] = iData;
	cInt[1] = iData >> 8;
	cInt[2] = iData >> 16;
	cInt[3] = iData >> 24;

	CMsgTr.set(cCmdChar1, cCmdChar2, cIndex[0], cIndex[1], cInt[0], cInt[1], cInt[2], cInt[3]);
	m_pCanCtrl->transmitMsg(CMsgTr);
}

//-----------------------------------------------
void CanDriveWhistle::IntprtSetFloat(int iDataLen, char cCmdChar1, char cCmdChar2, int iIndex, float fData)
{
	char cIndex[2];
	char cFloat[4];
	CanMsg CMsgTr;
	char* pTempFloat = NULL;
	
	CMsgTr.setID(m_ParamCanOpen.iRxPDO2);
	CMsgTr.setLength(iDataLen);

	cIndex[0] = iIndex;
	//* Sending float values requires bit 6 to be zero and bit 7 one (Elmo Implementation guide)
	//* setting bit 6 to zero with mask 0b10111111->0xBF
	cIndex[1] = (iIndex >> 8) & 0xBF;
	//* setting bit 7 to one with mask 0b10000000 ->0x80
	cIndex[1] = cIndex[1] | 0x80;

	pTempFloat = (char*)&fData;
	for( int i=0; i<4; i++ )
		cFloat[i] = pTempFloat[i];
	
	CMsgTr.set(cCmdChar1, cCmdChar2, cIndex[0], cIndex[1], cFloat[0], cFloat[1], cFloat[2], cFloat[3]);
	m_pCanCtrl->transmitMsg(CMsgTr);
}

//-----------------------------------------------
bool CanDriveWhistle::checkTargetReached()
{
	m_bMotionStatusReceived=false;

	//* Send SDO to read the Status Word of the drive.
	//* m_bStatusWordReceived=false;
	//* sendSDOUpload(0x6041,0); --> Checking bit 10 of StatusWord does not work properly. Don't know why, DSP 402 specifies bit 10 for Target Reached.

	//* Motion Status value will also do the work.
	IntprtSetInt(4, 'M', 'S', 0, 0);

	int cnt=0;
	do
	{
		cnt++;
		usleep(20000);
	}while(!m_bMotionStatusReceived && cnt<100);

	return m_bTargetReached;
}

//-----------------------------------------------
void CanDriveWhistle::evalStatusWord(int iStatusWord)
{
	if (isBitSet(iStatusWord,10))
	{
		//m_bTargetReached=true;
	}
	else
	{
		//m_bTargetReached=false;
	}
	m_bStatusWordReceived=true;
}

//-----------------------------------------------
// SDO Communication Protocol
//-----------------------------------------------

//-----------------------------------------------
void CanDriveWhistle::sendSDOUpload(int iObjIndex, int iObjSubIndex)
{
	CanMsg CMsgTr;
	const int ciInitUploadReq = 0x40;
	
	CMsgTr.setLength(8);
	CMsgTr.setID(m_ParamCanOpen.iRxSDO);

	unsigned char cMsg[8];
	
	cMsg[0] = ciInitUploadReq;
	cMsg[1] = iObjIndex;
	cMsg[2] = iObjIndex >> 8;
	cMsg[3] = iObjSubIndex;
	cMsg[4] = 0x00;
	cMsg[5] = 0x00;
	cMsg[6] = 0x00;
	cMsg[7] = 0x00;

	CMsgTr.set(cMsg[0], cMsg[1], cMsg[2], cMsg[3], cMsg[4], cMsg[5], cMsg[6], cMsg[7]);
	m_pCanCtrl->transmitMsg(CMsgTr);
}

//-----------------------------------------------
void CanDriveWhistle::sendSDODownload(int iObjIndex, int iObjSubIndex, int iData)
{
	CanMsg CMsgTr;

	const int ciInitDownloadReq = 0x20;
	const int ciNrBytesNoData = 0x00;
	const int ciExpedited = 0x02;
	const int ciDataSizeInd = 0x01;
	
	CMsgTr.setLength(8);
	CMsgTr.setID(m_ParamCanOpen.iRxSDO);

	unsigned char cMsg[8];
	
	cMsg[0] = ciInitDownloadReq | (ciNrBytesNoData << 2) | ciExpedited | ciDataSizeInd;
	cMsg[1] = iObjIndex;
	cMsg[2] = iObjIndex >> 8;
	cMsg[3] = iObjSubIndex;
	cMsg[4] = iData;
	cMsg[5] = iData >> 8;
	cMsg[6] = iData >> 16;
	cMsg[7] = iData >> 24;

	CMsgTr.set(cMsg[0], cMsg[1], cMsg[2], cMsg[3], cMsg[4], cMsg[5], cMsg[6], cMsg[7]);
	m_pCanCtrl->transmitMsg(CMsgTr);
}

//-----------------------------------------------
void CanDriveWhistle::evalSDO(CanMsg& CMsg, int* pIndex, int* pSubindex)
{
	*pIndex = (CMsg.getAt(2) << 8) | CMsg.getAt(1);
	*pSubindex = CMsg.getAt(3);
}

//-----------------------------------------------
int CanDriveWhistle::getSDODataInt32(CanMsg& CMsg)
{
	int iData = (CMsg.getAt(7) << 24) | (CMsg.getAt(6) << 16) |
		(CMsg.getAt(5) << 8) | CMsg.getAt(4);

	return iData;
}

//-----------------------------------------------
void CanDriveWhistle::sendSDOAbort(int iObjIndex, int iObjSubIndex, unsigned int iErrorCode)
{
	CanMsg CMsgTr;
	const int ciAbortTransferReq = 0x04 << 5;
	
	CMsgTr.setLength(8);
	CMsgTr.setID(m_ParamCanOpen.iRxSDO);

	unsigned char cMsg[8];
	
	cMsg[0] = ciAbortTransferReq;
	cMsg[1] = iObjIndex;
	cMsg[2] = iObjIndex >> 8;
	cMsg[3] = iObjSubIndex;
	cMsg[4] = iErrorCode;
	cMsg[5] = iErrorCode >> 8;
	cMsg[6] = iErrorCode >> 16;
	cMsg[7] = iErrorCode >> 24;

	CMsgTr.set(cMsg[0], cMsg[1], cMsg[2], cMsg[3], cMsg[4], cMsg[5], cMsg[6], cMsg[7]);
	m_pCanCtrl->transmitMsg(CMsgTr);
}



