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
 * ToDo: - Assign Address of digital input for homing switch "iHomeDigIn" via parameters (in
 *evalReceived Message, Line 116).
 *       - Homing Event should be defined by a parameter file and handed to CanDrive... e.g. via the
 *DriveParam.h (in inithoming, Line 531).
 *		 - Check whether "requestStatus" can/should be done in the class implementing the component
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
#include "CanDriveItf.h"
#include "TimeStamp.h"

#define CAN_MSG_NMT 0x00        /**< CanID for NMT messages */
#define CAN_MSG_SYNC 0x80       /**< CanID for SYNC messages */
#define CAN_MSG_TIMESTAMP 0x100 /**< CanID for TIMESTAMP messages */

#define CAN_MSG_EMCY_0 0x80    /**< Base CanID for EMERGENCY messages. CanID = Base + NodeID */
#define CAN_MSG_TxPDO1_0 0x180 /**< Base CanID for TxPDO1 messages */
#define CAN_MSG_RxPDO1_0 0x200 /**< Base CanID for RxPDO1 messages */
#define CAN_MSG_TxPDO2_0 0x280 /**< Base CanID for TxPDO2 messages */
#define CAN_MSG_RxPDO2_0 0x300 /**< Base CanID for RxPDO2 messages */
#define CAN_MSG_TxPDO3_0 0x380 /**< Base CanID for TxPDO3 messages */
#define CAN_MSG_RxPDO3_0 0x400 /**< Base CanID for RxPDO3 messages */
#define CAN_MSG_TxPDO4_0 0x480 /**< Base CanID for TxPDO4 messages */
#define CAN_MSG_RxPDO4_0 0x500 /**< Base CanID for RxPDO4 messages */
#define CAN_MSG_TxSDO_0 0x580  /**< Base CanID for TxSDO messages */
#define CAN_MSG_RxSDO_0 0x600  /**< Base CanID for RxSDO messages */
#define CAN_MSG_ErrorControl_0 0x700 /**< Base CanID for ErrorControl messages */

#define CMD_NMT_START                                                                         \
    0x01 /**< Definition for Network Start command inside a NMT message. Communications State \
            Machine */
#define CMD_NMT_STOP                                                                         \
    0x02 /**< Definition for Stop Network command inside a NMT message. Communications State \
            Machine */
#define CMD_NMT_PREOP                                                                     \
    0x80 /**< Definition for entering Pre-Operational state command inside a NMT message. \
            Communications State Machine */
#define CMD_NMT_SW_RESET                                                                       \
    0x81 /**< Definition for Software Reset command inside a NMT message. Communications State \
            Machine */
#define CMD_NMT_COMMS_RESET                                                                    \
    0x82 /**< Definition for Communications Reset command inside a NMT message. Communications \
            State Machine */

#define CAN_WATCHDOG_TIMEOUT_SEC                                                                   \
    10 /**< CAN heartbeat control. Time limit in seconds to consider the drive entered a faulty or \
          irresponsive state */

/**
 * Driver class for the motor drive of type Whistle. Implements CanDriveItf "Interface"
 */
class CanDriveWhistle : public CanDriveItf
{
  public:
    /**
     * Internal parameters.
     */

    /**
     * States of the drive Communications State Machine. Transitions occur upon the arrival of NMT
     * messages.
     */
    enum CommsStateWhistle
    {
        ST_PRE_INITIALIZED,
        ST_OPERATION_ENABLED,
        ST_OPERATION_DISABLED,
        ST_MOTOR_FAILURE
    };

    /**
     * States of the drive Communications State Machine. Transitions occur upon the arrival of NMT
     * messages.
     * ToDo: Change the CommsStateWhistle to NMTStateWhistle inside the implemented code
     */
    enum NMTStateWhistle
    {
        ST_PRE_OPERATIONAL,
        ST_OPERATIONAL,
        ST_STOPPED,
    };

    /**
     * States of the drive Communications State Machine. Transitions occur upon the arrival of NMT
     * messages.
     * ToDo: Add this state machine when commanding the Whistle with CANopen Object Dictionary
     * entries instead of Binary Interpreter commands
     */
    enum ControlStateWhistle
    {
        ST_NOT_READY_TO_SWITCH_ON,
        ST_SWITCH_ON_DISABLED,
        ST_READY_TO_SWITCH_ON,
        ST_SWITCHED_ON,
        ST_OPERATION_ENABLE,
        ST_QUICK_STOP,
        ST_FAULT
    };

    /**
     * CAN Node ID and Identifiers of different CAN messages that can be sent or received.
     */
    struct ParamCanOpenType
    {
        int iCanID;
        int iEMCY;
        int iTxPDO1;
        int iRxPDO1;
        int iTxPDO2;
        int iRxPDO2;
        int iTxPDO3;
        int iRxPDO3;
        int iTxPDO4;
        int iRxPDO4;
        int iTxSDO;
        int iRxSDO;
        int iErrorControl;
    };

    /********************************************/ /**
      *  CanDriveItf Interface functions implementation.
      ***********************************************/

    /**
     * Sets the CAN interface.
     */
    void setCanItf(CanItf* pCanItf) { m_pCanCtrl = pCanItf; }

    /**
     * Initializes the driver.
     * Call this function once after construction.
     * @return True if initialization was successful. False otherwise.
     */
    bool init();

    /**
     * Check if the driver is already initialized.
     * This is necessary if a drive gets switched off during runtime.
     * @return true if initialization occurred already, false if not.
     */
    bool isInitialized() { return m_bIsInitialized; }

    /**
     * Enables the motor.
     * After calling the drive accepts velocity and position commands.
     * @return True if drive is started successfully. StatusRegister is also evaluated to ensure a
     * non-faulty state. False otherwise.
     */
    bool start();

    /**
     * Disables the motor.
     * After calling the drive won't accepts velocity and position commands.
     * @return True if drive stopped.
     */
    bool stop();
    /**
     * Resets the drive with an NMT command.
     * The drive changes into the state after initialization.
     * @return True if re-initialization was successful. False otherwise.
     */
    bool reset();

    /**
     * Shutdown the motor.
     * @return True if drive shutdown (motor off).
     */
    bool shutdown();

    /**
     * Disables the brake.
     * This function is not implemented for Whistle,
     * because brakes are released upon power on and
     * shut only at emergency stop.
     */
    bool disableBrake(bool bDisabled) { return true; }

    /**
     * Inits homing procedure.
     * Used only in position mode.
     * @return True if homing procedure initialization was successful.
     * Not used in ExoTer
     * @see Homing
     */
    bool initHoming();

    /**
     * Performs homing procedure.
     * Used only in position mode.
     * @return True if Homing procedure was properly executed.
     * Not used in ExoTer
     * @see Homing
     */
    bool execHoming();

    /**
     * Performs homing procedure for ExoTer rover System HW configuration
     * Homing method for ExoTer is simple: Absolute potentiometer sensor is used at initial stage to
     * set the position value.
     * As from initialization, incremental encoder is used to measure the position relative to the
     * initially set value.
     * This simple procedure is possible thanks to the the absolute position initialization by means
     * of the potentiometer.
     * Assuming a good absolute calibration of the potentiometer, this function just commands the
     * motor to the neutral "zero" position.
     */
    void homing();

    /**
     * @return The elapsed time since the last received message.
     */
    double getTimeToLastMsg();

    /**
     * @return The status of the limit switch needed for homing.
     * True if limit is reached.
     */
    bool getStatusLimitSwitch();

    /**
     * Starts the watchdog.
     * The Whistle provides watchdog functionality which means the drive stops if the watchdog
     * becomes active. To keep the watchdog inactive a heartbeat message has to be sent
     * periodically. The update is done  within a timer.
     * @param bStarted True to start the watchdog and False to inactivate the watchdog.
     * @return True if the setting was successful.
     */
    bool startWatchdog(bool bStart);

    /**
     * Evals a received message.
     * Only messages with fitting identifiers are evaluated.
     * @param msg message to be evaluated.
     * @return True if msg was properly evaluated. Does not mean that there are no errors. The
     * errors should be assessed by function calls raised inside this function.
     */
    bool evalReceivedMsg(CanMsg& msg);

    /**
     * Determines the corresponding type of message for a given msg ID. Checks also if the ID
     * matches the Node ID
     * @param iID the message identifier.
     * @return type of CAN message. -1 if it does not correspond to any of the know types
     */
    int getMsgType(int iID);

    /**
     * Sends position (and velocity) command (PTP Motion).
     * Use this function only in position control mode.
     * @param dPosRad Position command in Radians
     * @param dVelRadS Velocity command in Radians/sec
     */
    void positionCommandRad(double dPosRad, double dVelRadS);

    /**
     * Sets the value for the reference position and velocity (PTP Motion).
     * It does not execute the command. Useful for synchronized motion.
     * Use this function only in position control mode.
     * @param dPosRad Position setpoint in Radians.
     * @param dVelRadS Velocity setpoint in Radians/sec.
     * @see CommandSetPoint()
     */
    void positionSetPointRad(double dPosRad, double dVelRadS);

    /**
     * Sends velocity command.
     * Use this function only in velocity control mode.
     * @param dVelRadS Velocity command in Radians/sec.
     */
    void velocityCommandRadS(double dVelRadS);

    /**
     * Sets the value for the reference velocity.
     * It does not execute the command. Useful for synchronized motion.
     * Use this function only in velocity control mode.
     * @param dVelRadS Velocity setpoint in Radians/sec.
     * @see CommandSetPoint()
     */
    void velocitySetPointRadS(double dVelRadS);

    /**
     * Send execution command to start synchronized motion. For Whistle, sends BG command.
     * @see positionSetPointRad()
     * @see velocitySetPointRadS()
     */
    void commandSetPoint();

    /**
     * Sets the motion type of the drive.
     * @param iType the MotionType selected from the enum. Position, Velocity or Torque control.
     */
    bool setTypeMotion(MotionType iType);

    /**
     * Reads the last received value of the drive position and velocity.
     * @param pdAngleGearRad The value of the current position of the motor is stored in this
     * pointer.
     * @param pdVelGearRadS The value of the current velocity of the motor is stored in this
     * pointer.
     */
    void getPositionVelocityRadS(double* pdPositionRad, double* pdVelocityRadS);

    /**
     * Reads the change of the position and the velocity with respect to the last time that this
     * function was called.
     * @param pdDeltaAngleGearRad The value of the delta position of the motor is stored in this
     * pointer.
     * @param pdDeltaVelGearRadS The value of the delta velocity of the motor is stored in this
     * pointer.
     */
    void getDeltaPositionVelocityRadS(double* pdDeltaPositionRad, double* pdDeltaVelocityRadS);

    /**
     * Reads the last received value of the drive Velocity.
     * @param pdPosGearRad The value of the current Velocity of the motor is stored in this pointer.
     */
    void getVelocityRadS(double* pdVelocityRadS);

    /**
     * Sets the drive parameters.
     * @param driveParam is the object of the DriveParam class that contains the values of the drive
     * parameters.
     */
    void setDriveParam(DriveParam driveParam) { m_DriveParam = driveParam; }

    /**
     * Gets the drive parameters
     * @return Pointer to the object of type DriveParam that is contained in the drive class.
     */
    DriveParam* getDriveParam() { return &m_DriveParam; }

    /**
     * Returns true if an error has been detected.
     * @return boolean with result.
     */
    bool isError();

    /**
     * Dummy implementations for completing the interface.
     */
    unsigned int getError() { return 0; }

    /**
     * Requests position and velocity to the drive.
     */
    void requestPosVel();

    /**
     * Requests status of the drive.
     */
    void requestStatus();

    /**
     * Dummy implementation for completing CanDriveItf.
     */
    void getStatus(int* piStatus, int* piTempCel)
    {
        *piStatus = 0;
        *piTempCel = 0;
    }

    /**
     * Dummy implementation for completing CanDriveItf.
     */
    bool setEMStop()
    {
        LOG_ERROR_S << "The function setEMStop() is not implemented!!!";
        return false;
    }

    /**
     * Dummy implementation for completing CanDriveItf.
     */
    bool resetEMStop()
    {
        LOG_ERROR_S << "The function resetEMStop() is not implemented!!!";
        return false;
    }

    /**
     * Sends an integer value to the Whistle using the built in binary interpreter. Refer to the
     * command reference manual for more info.
     * @param iDataLen The length of iData. From 0 to 4.
     * @param cCmdChar1 Contains the first character of the command for the binary interpreter.
     * @param cCmdChar2 Contains the second character of the command for the binary interpreter.
     * @param iIndex Contains the index of the command (in case necessary, otherwise 0) for the
     * binary interpreter.
     * @param iData Contains the integer value to be transmitted to the drive (in case of set
     * command, otherwise empty).
     */
    void IntprtSetInt(int iDataLen, char cCmdChar1, char cCmdChar2, int iIndex, int iData);

    /**
     * Requests the "active current" to the drive.
     */
    void requestTorque();

    /**
     * Sends Torque command
     * @param dTorqueNm is the required motor torque in Nm.
     */
    void torqueCommandNm(double dTorqueNm);

    /**
     * Reads the last received value of the motor Torque.
     * To update this value call requestTorque before.
     * @param pdTorqueNm The value (in Nm) of the current motor torque is stored in this pointer.
     */
    void getTorqueNm(double* pdTorqueNm);

    /**
     * Checks if the target set point was already reached.
     * @return True if the target set point was already reached.
     */
    bool checkTargetReached();

    /********************************************/ /**
      *  CanDriveWhistle specific functions (not from CanDriveItf)
      ***********************************************/

    /**
     * Default Constructor.
     */
    CanDriveWhistle();

    /**
     * Default Destructor.
     */
    ~CanDriveWhistle();

    /**
     * Returns some received values from the drive.
     * @deprecated use the other functions instead.
     * @param pdPosGearRad position of the drive
     * @param pdVelGearRadS velocity of the drive
     * @param piTorqueCtrl torque
     * @param piStatusCtrl
     */
    void getData(double* pdPosGearRad, double* pdVelGearRadS, int* piTorqueCtrl, int* piStatusCtrl);

    /**
     * Returns data received values from the drive.
     * @param pdPosGearRad position of the drive
     * @param pdVelGearRadS velocity of the drive
     * @param pdCurrentAmp active current of the drive
     * @param pdTorqueNm motor torque
     */
    void getData(double* pdPosGearRad,
                 double* pdVelGearRadS,
                 double* pdCurrentAmp,
                 double* pdTorqueNm);

    bool status() { return m_bStatusOk; }

    void getAnalogInput(double* pdAnalogInput);

    /**
     * Sets the CAN identifiers of the drive node.
     * @deprecated use the other setCanOpenParam function instead.
     * @param iTxPDO1 first transmit process data object
     * @param iTxPDO2 second transmit process data object
     * @param iRxPDO2 second receive process data object
     * @param iTxSDO transmit service data object
     * @param iRxSDO receive service data object
     */
    void setCanOpenParam(int iTxPDO1, int iTxPDO2, int iRxPDO2, int iTxSDO, int iRxSDO);

    /**
     * Sets CAN identifiers of the drive node
     * @param iID Node CAN ID
     */
    void setCanOpenParam(int iID);

    /**
     * Sets the drive's name
     * @param sName string that identifies the drive
     */
    void setDriveName(std::string sName);

    /**
     * Sets just the motion type variable of the drive. Does not send any configuration messages to
     * the Whistle. Use in Group Whistle instaces.
     * @param iType the MotionType selected from the enum. Position, Velocity or Torque control.
     */
    void setTypeMotionVariable(MotionType iType);

    /**
     * Gets just the motion type variable of the drive.
     * @return Motion type of the Whistle. Position, Velocity or Torque control.
     */
    MotionType getTypeMotionVariable();

    /**
     * Sends a heartbeat to the CAN-network to keep all listening watchdogs sleeping
     */
    void sendHeartbeat();

    /**
     * Sends a SYNC msg to the CAN-network.
     * Can be used to trigger the transmission of PDO messages.
     */
    void sendSync();

    /**
     * Sends a float value to the Whistle using the built in interpreter.
     * @param iDataLen The length of iData. From 0 to 4.
     * @param cCmdChar1 Contains the first character of the command for the binary interpreter.
     * @param cCmdChar2 Contains the second character of the command for the binary interpreter.
     * @param iIndex Contains the index of the command (in case necessary, otherwise 0) for the
     * binary interpreter.
     * @param iData Contains the float value to be transmitted to the drive (in case of set command,
     * otherwise empty).
     */
    void IntprtSetFloat(int iDataLen, char cCmdChar1, char cCmdChar2, int iIndex, float fData);

    /**
     * CANopen: Send an SDO message to the drive to initiate an SDO Upload (device to master) in
     * expedited transfer mode, means in only one message.
     * A single SDO message is sent then in response (device to master) with the information of the
     * object requested.
     * @param iObjIndex Index of the Object Dictionary to be uploaded
     * @param iObjSub Subindex of the Object Dictionary to be uploaded
     */
    void sendSDOUpload(int iObjIndex, int iObjSub);

    /**
     * CANopen: Send an SDO message to the drive performing an SDO Download (master to device) in
     * expedited transfer mode, means in only one message.
     * A SDO message is sent then in response (device to master) to acknowledge that the information
     * was properly set in the specified object.
     * @param iObjIndex Index of the Object Dictionary to be downloaded (set value in the Whistle)
     * @param iObjSub Subindex of the Object Dictionary to be downloaded (set value in the Whistle)
     */
    void sendSDODownload(int iObjIndex, int iObjSub, int iData);

    /**
     * CANopen: Internal use. Process a service data object received at the master and retrieves the
     * Index and Sub-index of the object encapsulated
     * @param CMsg CAN message to process
     * @param pIndex Integer pointer where the index of the object is stored
     * @param pSubindex Integer pointer where the sub-index of the object is stored
     */
    void evalSDO(CanMsg& CMsg, int* pIndex, int* pSubindex);

    /**
     * CANopen: Internal use. Process a service data object received at the master and retrieves the
     * integer data of the object encapsulated
     * @param CMsg CAN message to process
     */
    int getSDODataInt32(CanMsg& CMsg);

    /**
     * CANopen: This protocol cancels an active segmented transmission due to the given Error Code.
     * Not used but kept for possible future additional implementation of segmented SDO transfer
     */
    void sendSDOAbort(int iObjIndex, int iObjSubIndex, unsigned int iErrorCode);

    /**
     * CANopen: set up the PDO mapping for the application.
     * Edit the function insides to adapt to the specific application.
     */
    void setPDOmappings();

    /**
     * Set up position limits and modulo counting.
     * Joint software limits and modulo counting (XM[N] command) for avoiding limit overflow
     * Relevant information in DriveParam and in VH[N] - VL[N] commands.
     */
    void setDriveLimits();

    /**
     * Checks the elapsed time since the Watchdog time was reseted. Should correspond to the time
     * the last message from this drive was received.
     * @return True if the timeout is reached. False otherwise.
     */
    bool checkWatchdogTime();

    /**
     * Send message to create a transition in the Communications State Machine of the Whistle.
     * @param cmd Bitfield command to be set in the NMT message.
     */
    void sendNMTMsg(BYTE cmd);

    /**
     * Evaluates a message received of type Error Control, CanID = 0x700+NodeID.
     * @param msg Message to be processed.
     */
    void evalMsgErrorControl(CanMsg msg);

    /**
     * Evaluates a message received of type Emergency, CanID = 0x80+NodeID.
     * @param msg Message to be processed.
     */
    void evalMsgEMCY(CanMsg msg);

    /**
     * Evaluates a message received of type PDO1, CanID = 0x180+NodeID.
     * @param msg Message to be processed.
     */
    void evalMsgTxPDO1(CanMsg msg);

    /**
     * Evaluates a message received of type PDO2, CanID = 0x280+NodeID.
     * @param msg Message to be processed.
     */
    void evalMsgTxPDO2(CanMsg msg);

    /**
     * Evaluates a message received of type PDO3, CanID = 0x380+NodeID.
     * @param msg Message to be processed.
     */
    void evalMsgTxPDO3(CanMsg msg);

    /**
     * Evaluates a message received of type PDO4, CanID = 0x480+NodeID.
     * @param msg Message to be processed.
     */
    void evalMsgTxPDO4(CanMsg msg);

    /**
     * Evaluates a message received of type SDO, CanID = 0x580+NodeID.
     * @param msg Message to be processed.
     */
    void evalMsgTxSDO(CanMsg msg);

    /**
     * Evaluates the Status Register bit-field.
     * @param iStatusRegister is the bit-coded data received in a SR Binary Interpreter message.
     * @return True if no error was detected. Status OK.
     */
    bool evalStatusRegister(int iStatusRegister);

    /**
     * Evaluates the StatusWord bit-field.
     * @param iStatusWord is the bit-coded data received over CANopen message. Requested through SDO
     * or mapped in PDO.
     */
    void evalStatusWord(int iStatusWord);

    /**
     * Evaluates the Motor Failure Register bit-field.
     * @param iFailure is the bit-coded data received in a MF Binary Interpreter message.
     */
    void evalMotorFailure(int iFailure);

    /**
     * Estimates velocity based on the current position, the old previously saved position and the
     * elapsed time.
     * ExoTer does not use this function. Encoder data already provides a velocity measure.
     * @param dPos Current position.
     * @return Estimated value of the velocity.
     */
    double estimVel(double dPos);

  protected:
    /**
     * Parameters
     */

    ParamCanOpenType m_ParamCanOpen; /**< CANopen ID parameters */
    DriveParam m_DriveParam; /**< DriveParam class object with motor specific data. Same servo drive
                                (Whistle class) used to control different types of motor */

    /**
     * Variable members
     */

    CanItf* m_pCanCtrl; /**< Pointer to CanItf class object */

    CanMsg m_CanMsgLast; /**< Last received CAN message. No real use implemented. For now only
                            stores the last message */

    std::string m_sName; /**< String name definition of the drive, i.e, "Manipulator Joint 3" */

    MotionType m_iTypeMotion; /**< Motion Control Type. Choose between Position, Velocity or Torque
                                 Control */

    CommsStateWhistle m_iMotorState;    /**< Communications State Machine. Current State */
    CommsStateWhistle m_iNewMotorState; /**< Communications State Machine. New (commanded) State */

    int m_iStatusRegister;   /**< StatusRegister value */
    bool m_bCurrentLimitOn;  /**< StatusRegister bit for current limit */
    bool m_bFailureDetected; /**< In StatusRegister evaluation, this is set True if failure was
                                detected */

    TimeStamp m_CurrentTime;  /**< Timestamp for CurrentTime. General Purpose */
    TimeStamp m_WatchdogTime; /**< Timestamp for WatchdogTime. Re-Set when a message from a drive is
                                 received */
    TimeStamp m_VelCalcTime;  /**< Timestamp for Velocity estimation function. */
    TimeStamp m_FailureStartTime; /**< Timestamp for the Start time of a failure. General Purpose */
    TimeStamp m_SendTime;         /**< Timestamp for the time a message was sent. General Purpose */
    TimeStamp m_StartTime;        /**< Timestamp for application starting time. General Purpose */

    double m_dPosGearMeasRad;   /**< Position reading value */
    double m_dPosGearMemoRad;   /**< Previously stored position reading value */
    double m_dVelGearMeasRadS;  /**< Velocity reading value */
    double m_dMotorCurrAmps;    /**< Active Current reading value. Amps */
    double m_dAnalogInputVolts; /**< Analog input reading value. Volts */

    bool m_bIsPresent;            /**< Control loop. Flag for network verification */
    bool m_bIsInitialized;        /**< Control loop. Flag for monitoring initialization */
    bool m_bStatusOk;             /**< Control loop. Flag for status register evaluation outcome */
    bool m_bMotorOn;              /**< Control loop. Flag for motor enabled */
    bool m_bTargetReached;        /**< Control loop. Flag for target reached */
    bool m_bStatusWordReceived;   /**< Control loop. Flag for Status Word message reception */
    bool m_bMotionStatusReceived; /**< Control loop. Flag for Motion Status message reception */

    bool m_bWatchdogActive; /**< Flag for monitoring if Watchdog functionality is activated */

    double m_dOldPos; /**< Previously stored position value used for velocity estimation function */

    bool m_bSwitchHome; /**< HomeSwitch boolean state. Useful for homing procedures. Not used in
                           ExoTer */
    bool m_bLimSwLeft; /**< LeftLimitSwitch boolean state. Useful for homing procedures. Not used in
                          ExoTer */
    bool m_bLimSwRight; /**< RightLimitSwitch boolean state. Useful for homing procedures. Not used
                           in ExoTer */

    int m_iTorqueCtrl; /**< Torque command reading value. Used in deprecated function */
    int m_iDistSteerAxisToDriveWheelMM; /**< Steer Mechanism distance to wheel. Use if coupled
                                           Drive-Steer movement is implemented */

    /**
     * Member function to check if a certain bit of variable is set to 1 or not
     * @param iVal variable to check
     * @param iNrBit Position of the bit to check in iVal
     * @return True if the value of the bit in position iNrBit inside iVal is '1'. False if value is
     * '0'.
     */
    bool isBitSet(int iVal, int iNrBit)
    {
        if ((iVal & (1 << iNrBit)) == 0)
            return false;
        else
            return true;
    }
};
