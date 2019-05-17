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

#pragma once

#include <base-logging/Logging.hpp>
#include <iostream>

/**
 * Represents a CAN message.
 */

/// Include typedefs from windows.h
typedef unsigned char BYTE;

class CanMsg
{

  private:
    // CAN Message ID. Is the COB-ID. Possible values from 0x00 to 0x7FF for Standard CANopen
    // Messages (Not Extended)
    int m_iID;
    // Length of the Message in number of bytes.
    int m_iLen;
    // Type of message to distinguish between standard and extended. Use 0x00 for Standard message
    // type. See pcan.h
    int m_iType;
    // A CAN message consists of eight bytes.
    BYTE m_bDat[8];
    // Message Time stamp. Set when a message is received. This value is not transmitted over the
    // CAN Bus
    unsigned int m_iTimestamp;

  public:
    /**
     * Default constructor.
     */
    CanMsg()
    {
        m_iID = 0;
        m_iLen = 8;
        m_iType = 0x00;
        m_iTimestamp = 0;
    }

    /**
     * Constructor with data.
     */
    CanMsg(int ID,
           int len,
           BYTE Data0 = 0,
           BYTE Data1 = 0,
           BYTE Data2 = 0,
           BYTE Data3 = 0,
           BYTE Data4 = 0,
           BYTE Data5 = 0,
           BYTE Data6 = 0,
           BYTE Data7 = 0,
           int timeStamp = 0)
    {
        m_iID = ID;
        m_iLen = len;
        m_iType = 0x00;
        m_bDat[0] = Data0;
        m_bDat[1] = Data1;
        m_bDat[2] = Data2;
        m_bDat[3] = Data3;
        m_bDat[4] = Data4;
        m_bDat[5] = Data5;
        m_bDat[6] = Data6;
        m_bDat[7] = Data7;
        m_iTimestamp = timeStamp;
    }

    /**
     * Copy Constructor.
     */
    CanMsg(CanMsg& msg)
    {
        m_iID = msg.m_iID;
        m_iLen = msg.m_iLen;
        m_iType = msg.m_iType;
        m_bDat[0] = msg.m_bDat[0];
        m_bDat[1] = msg.m_bDat[1];
        m_bDat[2] = msg.m_bDat[2];
        m_bDat[3] = msg.m_bDat[3];
        m_bDat[4] = msg.m_bDat[4];
        m_bDat[5] = msg.m_bDat[5];
        m_bDat[6] = msg.m_bDat[6];
        m_bDat[7] = msg.m_bDat[7];
        m_iTimestamp = msg.m_iTimestamp;
    }

    /**
     * Sets the bytes to the telegram.
     */
    void set(BYTE Data0 = 0,
             BYTE Data1 = 0,
             BYTE Data2 = 0,
             BYTE Data3 = 0,
             BYTE Data4 = 0,
             BYTE Data5 = 0,
             BYTE Data6 = 0,
             BYTE Data7 = 0)
    {
        m_bDat[0] = Data0;
        m_bDat[1] = Data1;
        m_bDat[2] = Data2;
        m_bDat[3] = Data3;
        m_bDat[4] = Data4;
        m_bDat[5] = Data5;
        m_bDat[6] = Data6;
        m_bDat[7] = Data7;
    }

    /**
     * Set the byte at the given position.
     */
    void setAt(BYTE data, int iNr) { m_bDat[iNr] = data; }

    /**
     * Gets the bytes of the telegram.
     */
    void get(BYTE* pData0,
             BYTE* pData1,
             BYTE* pData2,
             BYTE* pData3,
             BYTE* pData4,
             BYTE* pData5,
             BYTE* pData6,
             BYTE* pData7)
    {
        *pData0 = m_bDat[0];
        *pData1 = m_bDat[1];
        *pData2 = m_bDat[2];
        *pData3 = m_bDat[3];
        *pData4 = m_bDat[4];
        *pData5 = m_bDat[5];
        *pData6 = m_bDat[6];
        *pData7 = m_bDat[7];
    }

    /**
     * Returns a specific byte of the telegram.
     * @param iNr number of the byte.
     */
    int getAt(int iNr) { return m_bDat[iNr]; }

    /**
     * Sets a block of data to the telegram.
     * @param iLength number of the bytes.
     * @param bData block of Data
     */
    void setData(BYTE bData[8], int iLength = 8)
    {
        m_iLen = iLength;
        for (int i = 0; i < iLength; i++)
        {
            m_bDat[i] = bData[i];
        }
    }

    /**
     * Returns the data of the telegram.
     */
    BYTE* getData() { return m_bDat; }

    /**
     * Prints the telegram.
     */
    void print()
    {
        LOG_INFO_S << "id= " << m_iID << " type= " << m_iType << " len= " << m_iLen
                   << " time= " << m_iTimestamp << " data= " << (int)m_bDat[0] << " "
                   << (int)m_bDat[1] << " " << (int)m_bDat[2] << " " << (int)m_bDat[3] << " "
                   << (int)m_bDat[4] << " " << (int)m_bDat[5] << " " << (int)m_bDat[6] << " "
                   << (int)m_bDat[7];
    }

    /**
     * Get the identifier stored in this message structure.
     * @return the message identifier.
     */
    int getID() { return m_iID; }

    /**
     * Set the message identifier within this message structure.
     * @param id The message identifier. Its value must be in the range [0..2047], i.e.
     * 29-bit identifiers are not supported here.
     */
    void setID(int id)
    {
        if ((0x00 <= id) && (id <= 0x7ff)) m_iID = id;
    }

    /**
     * Get the message length set within this data structure.
     * @return The message length in the range [0..8].
     */
    int getLength() { return m_iLen; }

    /**
     * Set the message length within this message structure.
     * @param len The message length. Its value must be in the range [0..8].
     */
    void setLength(int len)
    {
        if ((0 <= len) && (len <= 8)) m_iLen = len;
    }

    /**
     * Get the message type. By default, the type is 0x00.
     * @return The message type.
     */
    int getType() { return m_iType; }

    /**
     * Set the message type. By default, the type is 0x00.
     * @param type The message type.
     */
    void setType(int type) { m_iType = type; }

    /**
     * Get the message type. By default, the type is 0x00.
     * @return The message type.
     */
    unsigned int getTimestamp() { return m_iTimestamp; }

    /**
     * Set the message type. By default, the type is 0x00.
     * @param type The message type.
     */
    void setTimestamp(unsigned int time) { m_iTimestamp = time; }
};
