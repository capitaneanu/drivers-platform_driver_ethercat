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
 * ToDo:
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


#ifndef CANITF_INCLUDEDEF_H
#define CANITF_INCLUDEDEF_H
//-----------------------------------------------
#include "CanMsg.h"
#include <unistd.h>
//-----------------------------------------------

/**
 * General interface of the CAN bus.
 * \ingroup DriversCanModul	
 */
class CanItf
{

public:
	enum CanItfType {
		CAN_PEAK = 0,
		CAN_PEAK_USB = 1,
		CAN_ESD = 2,
		CAN_DUMMY = 3,
		CAN_BECKHOFF = 4,
		CAN_ETHERCAT = 5 // CanOpen over Ethercat
	};
	
	enum CanBaudRate {
		BAUD_RATE_1M =    0,  //   1 MBit/s
		BAUD_RATE_500K =  2,  // 500 kBit/s
		BAUD_RATE_250K =  4,  // 250 kBit/s
		BAUD_RATE_125K =  6,  // 125 kBit/s
		BAUD_RATE_50K  =  9,  //  50 kBit/s
		BAUD_RATE_20K  =  11,  //  20 kBit/s
		BAUD_RATE_10K  =  13  //  10 kBit/s
	};

	/**
	 * The destructor does not necessarily have to be overwritten.
	 * But it makes sense to close any resources like handles.
	 */
	virtual ~CanItf() {
	}
	
	/**
	 * Initializes the CAN bus.
	 */
	virtual void init() = 0;

	/**
	 * Closes the CAN bus.
	 */
	virtual void close() = 0;

	/**
	 * Checks if the interface is Initialized
	 * @return true if already initialized
	 */
	virtual bool isInit() = 0;

	/**
	 * Returns the number of messages available to read. If no messages have been received returns 0.
	 * @return Number of messages available to read. -1 is an error occurred.
	 */
	virtual int availableMessages() = 0;

	/**
	 * Sends a CAN message.
	 * @param pCMsg CAN message
	 * @param bBlocking specifies whether send should be blocking or non-blocking
	 */
	virtual bool transmitMsg(CanMsg CMsg, bool bBlocking = true) = 0;

	/**
	 * Reads a CAN message.
	 * @return true if a message is available 
	 */
	virtual bool receiveMsg(CanMsg* pCMsg) = 0;

	/**
	 * Reads a CAN message.
	 * The function blocks between the attempts.
	 * @param pCMsg CAN message
	 * @param iNrOfRetry number of retries
	 * @return true if a message is available
	 */
	virtual bool receiveMsgRetry(CanMsg* pCMsg, int iNrOfRetry) = 0;
	
	/**
	 * Set the CAN interface type. This is necessary to implement
	 * a proper CAN bus simulation.
	 * @param iType The CAN interface type.
	 */
	void setCanItfType(CanItfType iType) { m_iCanItfType = iType; }
	
	/**
	 * Get the CAN interface type. This is necessary to implement
	 * a proper CAN bus simulation.
	 * @return The CAN interface type.
	 */
	CanItfType getCanItfType() { return m_iCanItfType; }
	
	/**
	 * Set the CAN Baud Rate. This is necessary to implement
	 * the commmunication with the CAN nodes.
	 * @param iRate The CAN Baud Rate.
	 */
	void setCanBaudRate(CanBaudRate iRate) { m_iCanBaudRate = iRate; }

	/**
	 * Get the CAN Baud Rate. This is necessary to implement
	 * the commmunication with the CAN nodes.
	 * @return The CAN Baud Rate.
	 */
	CanBaudRate getCanBaudRate() { return m_iCanBaudRate; }

private:
	/// The CAN interface type.
	CanItfType m_iCanItfType;
	/// The CAN Baud Rate.
	CanBaudRate m_iCanBaudRate;
};
//-----------------------------------------------

#endif
