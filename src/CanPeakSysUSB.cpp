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
 * ROS stack name: cob3_common
 * ROS package name: generic_can
 * Description:
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Christian Connette, email:christian.connette@ipa.fhg.de
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
 *
 * Date of creation: Feb 2009
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing 
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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

//#define __DEBUG__

#include "CanPeakSysUSB.h"
#include <stdlib.h>
#include <cerrno>
#include <cstring>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

//-----------------------------------------------

CANPeakSysUSB::CANPeakSysUSB()
{
	m_bInitialized = false;

	init();
}

CANPeakSysUSB::CANPeakSysUSB(std::string dev)
{
	m_bInitialized = false;

	init(dev);
}

//-----------------------------------------------
CANPeakSysUSB::~CANPeakSysUSB()
{
	if (m_bInitialized)
	{
		CAN_Close(m_handle);
	}
}

void CANPeakSysUSB::init()
{
	std::string dev;
	dev = "/dev/pcan32";
	init (dev);
}

//-----------------------------------------------
void CANPeakSysUSB::init(std::string dev)
{
	std::string sCanDevice; 
	
	// ToDo Device path taken from configuration file  != 0) -> Done!
	sCanDevice = dev;
	//sCanDevice = "/dev/pcan32";
	
	m_handle = LINUX_CAN_Open(sCanDevice.c_str(), O_RDWR);

	if (!m_handle)
	{
		// Fatal error
		std::cout << "Cannot open CAN on USB: " << strerror(errno) << std::endl;
		sleep(3);
		exit(0);
	}

	// set inteface type and baud rate and then initialize
	setCanItfType(CAN_PEAK_USB);
	setCanBaudRate(BAUD_RATE_1M);

	m_bInitialized = initCAN();

	//  clear the can buffer of previously left messages, probably not necessary as just called LINUX_CAN_Init
	CanMsg Msg;
	bool bRet;
	do
	{
		bRet = receiveMsg(&Msg);
	}
	while(bRet == true);
}

//-----------------------------------------------
void CANPeakSysUSB::close()
{
	if (m_bInitialized)
		{
			CAN_Close(m_handle);
			m_bInitialized=false;
		}
}

//-----------------------------------------------
bool CANPeakSysUSB::isInit()
{
	return m_bInitialized;
}

//-------------------------------------------
bool CANPeakSysUSB::transmitMsg(CanMsg CMsg, bool bBlocking)
{
	TPCANMsg TPCMsg;
	bool bRet = true;

	if (!m_bInitialized)
	{
		std::cout << "CANPeakSysUSB::transmitMsg Error: interface not initialized!" << std::endl;
		return false;
	}
	// copy CMsg to TPCmsg
	TPCMsg.LEN = CMsg.getLength();
	TPCMsg.ID = CMsg.getID();
	TPCMsg.MSGTYPE = CMsg.getType();
	for(int i=0; i<TPCMsg.LEN; i++)
		TPCMsg.DATA[i] = CMsg.getAt(i);
	
	int iRet;
	iRet = LINUX_CAN_Write_Timeout(m_handle, &TPCMsg, 25); //Timeout in micrsoseconds
	
	if(iRet != CAN_ERR_OK) {
#ifdef __DEBUG__
		std::cout << "CANPeakSysUSB::transmitMsg An error occured while sending..." << iRet << std::endl;		
		outputDetailedStatus();
#endif		
		bRet = false;
	}

#ifdef __DEBUG__	
	//is this necessary? try iRet==CAN_Status(m_handle) ?
	iRet = CAN_Status(m_handle);

	if(iRet < 0)
	{
		std::cout <<  "CANPeakSysUSB::transmitMsg, system error: " << iRet << std::endl;
		bRet = false;
	} else if((iRet & CAN_ERR_BUSOFF) != 0) {
		std::cout <<  "CANPeakSysUSB::transmitMsg, BUSOFF detected" << std::endl;
		//Try to restart CAN-Device
		std::cout <<  "Trying to re-init Hardware..." << std::endl;
		bRet = initCAN();
	
	} else if((iRet & CAN_ERR_ANYBUSERR) != 0) {
		std::cout <<  "CANPeakSysUSB::transmitMsg, ANYBUSERR" << std::endl;
	
	} else if( (iRet & (~CAN_ERR_QRCVEMPTY)) != 0) {
		std::cout << "CANPeakSysUSB::transmitMsg, CAN_STATUS: " << iRet << std::endl;
		bRet = false;
	}
#endif

	return bRet;
}

//-------------------------------------------
bool CANPeakSysUSB::receiveMsg(CanMsg* pCMsg)
{
	TPCANRdMsg TPCMsg;
	TPCMsg.Msg.LEN = 8;
	TPCMsg.Msg.MSGTYPE = 0;
	TPCMsg.Msg.ID = 0;
	
	int iRet = CAN_ERR_OK;
	
	bool bRet = false;

	if (!m_bInitialized)
	{
		std::cout << "CANPeakSysUSB::receiveMsg Error: interface not initialized!" << std::endl;
		return bRet;
	}

	iRet = LINUX_CAN_Read_Timeout(m_handle, &TPCMsg, 0);

	if (iRet == CAN_ERR_OK)
	{
		pCMsg->setID(TPCMsg.Msg.ID);
		pCMsg->setLength(TPCMsg.Msg.LEN);
		pCMsg->setData(TPCMsg.Msg.DATA, TPCMsg.Msg.LEN);
		pCMsg->setTimestamp(TPCMsg.dwTime);
		bRet = true;
	}
	else if( (iRet & (~CAN_ERR_QRCVEMPTY)) != 0) //no"empty-queue"-status
	{
		std::cout << "CANPeakSysUSB::receiveMsg, CAN_STATUS: " << iRet << std::endl;
		pCMsg->set(0, 0, 0, 0, 0, 0, 0, 0);
	}
	
	//catch status messages, these could be further processed in overlying software to identify and handle CAN errors
	if( TPCMsg.Msg.MSGTYPE == MSGTYPE_STATUS ) {
		std::cout << "CANPeakSysUSB::receiveMsg, status message catched:\nData is (CAN_ERROR_...) " << TPCMsg.Msg.DATA[3] << std::endl;
		pCMsg->setID(0);
		pCMsg->set(0, 0, 0, 0, 0, 0, 0, 0);
		//ToDo In case of Bus Error re-initialize the CAN Interfaces
	}

	return bRet;
}

//-------------------------------------------
bool CANPeakSysUSB::receiveMsgRetry(CanMsg* pCMsg, int iNrOfRetry)
{
	int i, iRet;

	TPCANRdMsg TPCMsg;
	TPCMsg.Msg.LEN = 8;
	TPCMsg.Msg.MSGTYPE = 0;
	TPCMsg.Msg.ID = 0;

	if (!m_bInitialized)
	{
		std::cout << "CANPeakSysUSB::receiveMsgRetry Error: interface not initialized!" << std::endl;
		return false;
	}

	// wait until msg in buffer
	bool bRet = true;
	iRet = CAN_ERR_OK;
	i=0;
	do
	{
		iRet = LINUX_CAN_Read_Timeout(m_handle, &TPCMsg, 0);

		if(iRet == CAN_ERR_OK)
			break;

		i++;
		usleep(100000);
	}
	while(i < iNrOfRetry);

	// eval return value
	if(iRet != CAN_ERR_OK)
	{
		std::cout << "CANPeakSysUSB::receiveMsgRetry, errorcode= " << nGetLastError() << std::endl;
		pCMsg->set(0, 0, 0, 0, 0, 0, 0, 0);
		bRet = false;
	}
	else
	{
		pCMsg->setID(TPCMsg.Msg.ID);
		pCMsg->setLength(TPCMsg.Msg.LEN);
		pCMsg->setData(TPCMsg.Msg.DATA, TPCMsg.Msg.LEN);
		pCMsg->setTimestamp(TPCMsg.dwTime);
		bRet = true;
	}

	return bRet;
}

//-------------------------------------------
int CANPeakSysUSB::availableMessages()
{
	int reads, writes;
	int rc;

	if (!m_bInitialized)
	{
		std::cout << "CANPeakSysUSB::availableMessages Error: interface not initialized!" << std::endl;
		return -1;
	}

	if ((rc=LINUX_CAN_Extended_Status(m_handle, &reads, &writes)) != CAN_ERR_OK)
	{
		if (rc != CAN_ERR_QRCVEMPTY){
			std::cout << "CANPeakSysUSB::availableMessages: Error reading CAN Status:" << rc << std::endl;
			return -1;
		}
		else return reads;
	}
	else
		return reads;
}

//-------------------------------------------
bool CANPeakSysUSB::initCAN() {
	int iRet = 1;
	bool bRet = true;

	switch(getCanBaudRate())
	{
	case 0:
		iRet = CAN_Init(m_handle, CAN_BAUD_1M, CAN_INIT_TYPE_ST);
		break;
	case 2:
		iRet = CAN_Init(m_handle, CAN_BAUD_500K, CAN_INIT_TYPE_ST);
		break;
	case 4:
		iRet = CAN_Init(m_handle, CAN_BAUD_250K, CAN_INIT_TYPE_ST);
		break;
	case 6:
		iRet = CAN_Init(m_handle, CAN_BAUD_125K, CAN_INIT_TYPE_ST);
		break;
	case 9:
		iRet = CAN_Init(m_handle, CAN_BAUD_50K, CAN_INIT_TYPE_ST);
		break;
	case 11:
		iRet = CAN_Init(m_handle, CAN_BAUD_20K, CAN_INIT_TYPE_ST);
		break;
	case 13:
		iRet = CAN_Init(m_handle, CAN_BAUD_10K, CAN_INIT_TYPE_ST);
		break;
	}

	if(iRet == CAN_ERR_OK)
	{
		std::cout << "CANPeakSysUSB::CANPeakSysUSB(), init ok" << std::endl;
		bRet = true;

	}
	else
	{
		std::cout << "CANPeakSysUSB::CANPeakSysUSB(), error in init" << std::endl;
		bRet = false;
	}
	
	return bRet;
}

void CANPeakSysUSB::outputDetailedStatus() {
	TPDIAG diag;
	
	LINUX_CAN_Statistics(m_handle, &diag);
	
	std::cout << "*************************\n"
			<< "*** Detailed status output of CANPeakSys\n"
			<< "*************************"
			<< "\nIRQ-Level:     " << diag.wIrqLevel
			<< "\nNo reads:      " << diag.dwReadCounter
			<< "\nNo writes:     " << diag.dwWriteCounter
			<< "\nNo interrupts: " << diag.dwIRQcounter
			<< "\nNo errors:     " << diag.dwErrorCounter
			<< "\nError flag:    " << diag.wErrorFlag
			<< "\nLast error:    " << diag.nLastError
			<< std::endl;
}
