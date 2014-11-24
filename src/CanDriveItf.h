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


#ifndef CANDRIVEITF_INCLUDEDEF_H
#define CANDRIVEITF_INCLUDEDEF_H


//* include files ---------------------------------------------

#include "CanItf.h"
#include "DriveParam.h"


/**
 * Interface description for a drive type of class.
 */
class CanDriveItf
{
public:

	/**
	 * Enum with different types of Motion of the controller, either Position, Velocity or Torque Control.
	 */
	enum MotionType
	{
		MOTIONTYPE_VELCTRL,
                MOTIONTYPE_TORQUECTRL,
		MOTIONTYPE_POSCTRL
	};

	/**
	 * The destructor does not necessarily have to be overwritten.
	 * But it makes sense to close any resources like handles.
	 */
	virtual ~CanDriveItf() {
	}

	/**
	 * Sets the CAN interface for the Drive class.
	 */
	virtual void setCanItf(CanItf* pCanItf) = 0;

	/**
	 * Initializes the driver.
	 * Call this function once after construction.
	 * @return True if initialization was successful. False otherwise.
	 */
	virtual bool init() = 0;
	
	/**
	 * Check if the driver is already initialized.
	 * This is necessary if a drive gets switched off during runtime.
	 * @return true if initialization occurred already, false if not.
	 */
	virtual bool isInitialized() = 0;

	/**
	 * Enables the motor.
	 * After calling the drive accepts velocity and position commands.
	 * @return True if drive is started successfully. StatusRegister is also evaluated to ensure a non-faulty state. False otherwise.
	 */
	virtual bool start() = 0;

	/**
	 * Disables the motor.
	 * After calling the drive won't accepts velocity and position commands.
	 * @return True if drive stopped.
	 */
	virtual bool stop() = 0;

	/**
	 * Resets the drive.
	 * The drive changes into the state after initialization.
	 * @return True if re-initialization was successful. False otherwise.
	 */
	virtual bool reset() = 0;

	/**
	 * Shutdowns the motor.
	 * @return True if drive shutdown (motor off).
	 */	 
	virtual bool shutdown() = 0;

	/**
	 * Disables the brake.
	 * This function is not implemented for Whistle,
	 * because brakes are released upon power on and
	 * shut only at emergency stop.
	 */
	virtual bool disableBrake(bool bDisabled) = 0;

	/**
	 * Inits homing procedure.
	 * Used only in position mode.
	 * @return True if homing procedure initialization was successful.
	 */
	virtual bool initHoming() = 0;

	/**
	 * Performs homing procedure.
	 * Used only in position mode.
	 * @return True if Homing procedure was properly executed.
	 */
	virtual bool execHoming() = 0;

	/**
	 * @return The elapsed time since the last received message.
	 */
	virtual double getTimeToLastMsg() = 0;

	/**
	 * @return The status of the limit switch needed for homing.
	 * True if limit is reached.
	 */
	virtual bool getStatusLimitSwitch() = 0;

	/**
	 * Starts the watchdog.
	 * The Whistle provides watchdog functionality which means the drive stops if the watchdog
	 * becomes active. To keep the watchdog inactive a heartbeat message has to be sent
	 * periodically. The update is done  within a timer.
	 * @param bStarted True to start the watchdog and False to inactivate the watchdog.
	 * @return True if the setting was successful.
	 */
	virtual bool startWatchdog(bool bStarted) = 0;

	/**
	 * Evals a received message.
	 * Only messages with fitting identifiers are evaluated.
	 * @param msg message to be evaluated.
	 * @return True if msg was properly evaluated. Does not mean that there are no errors. The errors should be assessed by function calls raised inside this function.
	 */
	virtual bool evalReceivedMsg(CanMsg& msg) = 0;

	/**
	 * Determines the corresponding type of message for a given msg ID. Checks also if the ID matches the Node ID
	 * @param iID the message identifier.
	 * @return type of CAN message. -1 if it does not correspond to any of the know types
	 */
	virtual int getMsgType(int iID) = 0;

	/**
	 * Sends position (and velocity) command (PTP Motion).
	 * Use this function only in position control mode.
	 * @param dPosRad Position command in Radians
	 * @param dVelRadS Velocity command in Radians/sec
	 */
	virtual void positionCommandRad(double dPosRad, double dVelRadS) = 0;

	/**
	 * Sets the value for the reference position and velocity (PTP Motion).
	 * It does not execute the command. Useful for synchronized motion.
	 * Use this function only in position control mode.
	 * @param dPosRad Position setpoint in Radians.
	 * @param dVelRadS Velocity setpoint in Radians/sec.
	 * @see CommandSetPoint()
	 */
	virtual void positionSetPointRad(double dPosRad, double dVelRadS) = 0;

	/**
	 * Sends velocity command.
	 * Use this function only in velocity control mode.
	 * @param dVelRadS Velocity command in Radians/sec.
	 */
	virtual void velocityCommandRadS(double dVelRadS) = 0;

	/**
	 * Sets the value for the reference velocity.
	 * It does not execute the command. Useful for synchronized motion.
	 * Use this function only in velocity control mode.
	 * @param dVelRadS Velocity setpoint in Radians/sec.
	 * @see CommandSetPoint()
	 */
	virtual void velocitySetPointRadS(double dVelRadS) = 0;

	/**
	 * Send execution command to start synchronized motion.
	 * @see positionSetPointRad()
	 * @see velocitySetPointRadS()
	 */
	virtual void commandSetPoint() = 0;

	/**
	 * Sets the motion type of the drive.
	 * @param iType the MotionType selected from the enum. Position, Velocity or Torque control.
	 */
	virtual bool setTypeMotion(MotionType iType) = 0;

    /**
     * Gets just the motion type variable of the drive.
     * @return Motion type of the Whistle. Position, Velocity or Torque control.
     */
        virtual MotionType getTypeMotionVariable() = 0;

	/**
	 * Reads the last received value of the drive position and velocity.
	 * @param pdAngleGearRad The value of the current position of the motor is stored in this pointer.
	 * @param pdVelGearRadS The value of the current velocity of the motor is stored in this pointer.
	 */
	virtual void getPositionVelocityRadS(double* pdPositionRad, double* pdVelocityRadS) = 0;

	/**
	 * Reads the change of the position and the velocity with respect to the last time that this function was called.
	 * @param pdDeltaAngleGearRad The value of the delta position of the motor is stored in this pointer.
	 * @param pdDeltaVelGearRadS The value of the delta velocity of the motor is stored in this pointer.
	 */
	virtual void getDeltaPositionVelocityRadS(double* pdDeltaPositionRad, double* pdDeltaVelocityRadS) = 0;

	/**
	 * Reads the last received value of the drive Velocity.
	 * @param pdPosGearRad The value of the current Velocity of the motor is stored in this pointer.
	 */
	virtual void getVelocityRadS(double* pdVelocityRadS) = 0;

	/**
	 * Sets the drive parameters.
	 * @param driveParam is the object of the DriveParam class that contains the values of the drive parameters.
	 */
	virtual void setDriveParam(DriveParam driveParam) = 0;

	/**
	 * Gets the drive parameters
	 * @return Pointer to the object of type DriveParam that is contained in the drive class.
	 */
	virtual DriveParam *getDriveParam() = 0;

	/**
	 * Returns true if an error has been detected.
	 * @return boolean with result.
	 */
	virtual bool isError() = 0;
	
	/**
	 * Returns a bitfield containing information about the pending errors.
	 * @return unsigned int with bitcoded error.
	 */
	virtual unsigned int getError() = 0;

	/**
	 * Requests position and velocity to the drive.
	 */
	virtual void requestPosVel() = 0;

	/**
	 * Requests status of the drive.
	 */
	virtual void requestStatus() = 0;

	/**
	 * Returns the measured temperature. 
	 * @param piStatus stores the status of the drive
	 * @param piTempCel stores temperature of drive in Celsius
	 */
	virtual void getStatus(int* piStatus, int* piTempCel) = 0;

	/**
	 * Returns the status boolean of the drive.
	 */
	virtual bool status()=0;

	/**
	 * Enable the emergency stop.
	 * @return true if the result of the process is successful
	 */
	virtual bool setEMStop() = 0;

	/**
	 * Disable the emergency stop.
	 * @return true if the result of the process is successful
	 */
	virtual bool resetEMStop() = 0;

	/**
	 * Sends an integer value to the Whistle using the built in binary interpreter. Refer to the command reference manual for more info.
	 * @param iDataLen The length of iData. From 0 to 4.
	 * @param cCmdChar1 Contains the first character of the command for the binary interpreter.
	 * @param cCmdChar2 Contains the second character of the command for the binary interpreter.
	 * @param iIndex Contains the index of the command (in case necessary, otherwise 0) for the binary interpreter.
	 * @param iData Contains the integer value to be transmitted to the drive (in case of set command, otherwise empty).
	 */
	virtual void IntprtSetInt(int iDataLen, char cCmdChar1, char cCmdChar2, int iIndex, int iData) = 0;

	/**
	 * Requests the "active current" to the drive.
	 */
	virtual void requestTorque() = 0;
	
	/**
	 * Reads the last received value of the motor Torque.
	 * To update this value call requestTorque before.
	 * @param pdTorqueNm The value (in Nm) of the current motor torque is stored in this pointer.
	 */
	virtual void getTorqueNm(double* pdTorqueNm) = 0;
    
    /**
     * Sends Torque command
     * @param dTorqueNm is the required motor torque in Nm.
     */
    virtual void torqueCommandNm(double dTorqueNm) = 0;

    /**
     * Checks if the target set point was already reached.
     * @return True if the target set point was already reached.
     */
    virtual bool checkTargetReached() = 0;

    /**
	 * Returns data received values from the drive.
	 * @param pdPosGearRad position of the drive
	 * @param pdVelGearRadS velocity of the drive
	 * @param pdCurrentAmp active current of the drive
	 * @param pdTorqueNm motor torque
	 */
	virtual void getData(double* pdPosGearRad, double* pdVelGearRadS,
		double* pdCurrentAmp, double* pdTorqueNm)=0;
	/**
	 * Returns received values from analog input.
	 */
	virtual void getAnalogInput(double* pdAnalogInput)=0;

};


//-----------------------------------------------
#endif

