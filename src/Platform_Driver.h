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
 *	 - move implementational details (can cmds) of configureElmoRecorder to CanDriveHarmonica (check whether CanDriveItf has to be adapted then)
 *	 - Check: what is the iRecordingGap, what is its unit
 *	 - Remove Mutex.h search for a Boost lib
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

#ifndef PLATFORMDRIVER_INCLUDEDEF_H
#define PLATFORMDRIVER_INCLUDEDEF_H

//-----------------------------------------------

//* general includes ---------------------------------------------
#include <vector>

//* motion library includes --------------------------------------
#include "CanDriveItf.h"
#include "CanDriveWhistle.h"
#include "CanItf.h"

//* other motion library includes (try to remove) ----------------
#include "Mutex.h"

//* other shared library includes --------------------------------
//#include <GenericRoverManoeuvre.h>

//* definitions --------------------------------------------------
//#define NUM_MOTORS 16							/**< Number of motors in the platform, equal to the number of nodes in the CAN bus*/
//#define NUM_GROUPS 3							/**< Number of node Groups configured in the CAN bus */
//#define NUM_NODES 19							/**< Size of the vector containing all addressable nodes in the platform */
//#define NUM_WHEELS 6							/**< Number of Wheels in the platform */
//#define PTP_VELOCICTY_DEFAULT 0.25				/**< Default velocity used in PTP motions (max velocity of trapezoidal profile) */
//#define WHEEL_RADIUS_MM 70						/**< Radius of Wheels in milimeters */
//#define PLTF_LENGTH 0.530							/**< Length of the platform measured from the center of a front wheel to the center of a back wheel */
//#define PLTF_WIDTH 0.610							/**< Width of the platform measured from the center of a left wheel to the center of a right wheel */

//#define CAN_ITF_TYPE 1							/**< Type of CAN interface device, refer to the CanItf class for other options */
#define TIMER_INTERVAL_SEC 0					/**< Period of the timer function in seconds */
#define TIMER_INTERVAL_USEC 50000				/**< Period of the timer function in microseconds */

//#define WATCHDOG_ACTIVE 0						/**< Flag for configuring the watchdog */

enum MotorStatus {INACTIVE, ACTIVE};
/**
 * List of all can nodes in the platform.
 * Include groups of nodes in order to create a Drive class object to address them altogether.
 * ToDo: Try to remove the dependency of the code from this enumerator as it depends directly in the specific rover.
 */
enum MotorCANNode
{
	CANNODE_WHEEL_DRIVE_FL
	,CANNODE_WHEEL_DRIVE_FR
	,CANNODE_WHEEL_DRIVE_CL
	,CANNODE_WHEEL_DRIVE_CR
	,CANNODE_WHEEL_DRIVE_BL
	,CANNODE_WHEEL_DRIVE_BR
	,CANNODE_WHEEL_STEER_FL
	,CANNODE_WHEEL_STEER_FR
	,CANNODE_WHEEL_STEER_BL
	,CANNODE_WHEEL_STEER_BR
	,CANNODE_WHEEL_WALK_FL
	,CANNODE_WHEEL_WALK_FR
	,CANNODE_WHEEL_WALK_CL
	,CANNODE_WHEEL_WALK_CR
	,CANNODE_WHEEL_WALK_BL
	,CANNODE_WHEEL_WALK_BR
	,CANNODE_MAST_PAN
	,CANNODE_MAST_TILT
	,CANNODE_WHEEL_DRIVE_GROUP
	,CANNODE_WHEEL_STEER_GROUP
	,CANNODE_WHEEL_WALK_GROUP
	,CANNODE_MAST_PTU_GROUP
	,CANNODE_MANIP_JOINT_1
	,CANNODE_MANIP_JOINT_2
	,CANNODE_MANIP_JOINT_3
	,CANNODE_MANIP_JOINT_4
	,CANNODE_MANIP_JOINT_5
	,CANNODE_MANIP_JOINT_GROUP
};

/**
 * List of Drive types. Motor characteristics are grouped depending on the type.
 */
enum MotorType
{
	WHEEL_DRIVE,
	WHEEL_STEER,
	WHEEL_WALK,
	MANIP_JOINT,
	MAST_PAN,
	MAST_TILT
};


/**
 * List of platform driving modes. So far, this are the modes supported by the Generic Manoeuvre Library.

enum PltfDrivingMode
{
	STOPPED,
	STRAIGHT_LINE,
	ACKERMAN,
	SPOT_TURN,
	SKID_TURN,
	WHEEL_WALKING,
	DIRECT_DRIVE
};
*/

/**
 * @deprecated use PltfCanParams instead
 * Structure with Drive node data.
 * See definition of the array of CanNodeType in Platform_Driver.cpp gathering the information of all nodes in the platform.
 */
struct CanNodeType{
	int iCanID;
	std::string sName;
	MotorType type;
};

struct PltfCanParams
{
	std::vector<int> CanId;
	std::vector<std::string> Name;
	std::vector<MotorType> Type;
	std::vector<MotorStatus> Active;
};

/**
 * Parameters characterizing a motor type. Note DriveParam class attribute values to be set.
 */
struct GearMotorParamType
{
	int		iEncIncrPerRevMot;
	double	dGearRatio;
	double	dBeltRatio;
	int		iSign;
	double 	dPosLimitLowIncr;
	double 	dPosLimitHighIncr;
	double	dVelMaxEncIncrS;
	double	dPtpVelDefaultIncrS;
	double	dAccIncrS2;
	double	dDecIncrS2;
	bool	bIsSteer;
	double  dCurrentToTorque;
	double  dCurrMax;
	int		iEncOffsetIncr;
	double	dAnalogFactor;
	double	dNominalCurrent;
};


/**
 * Methods outside the class that are used by other threads of execution (Message Handler & Timer).
 */

/**
 * It takes the CobID of a CAN message and identifies the Node that the message was sent from.
 * @param iCobID Is the CAN message ID that identifies the Communication Object (COB) and the Node ID
 * @return the Node ID
 */
int getNodeFromCobId(int iCobID);

/**
 * Sends a Heartbeat message to the CAN Bus. Used for Life Guarding.
 */
void sendHeartbeat();

/**
 * Sends a NMT message to the CAN Bus.
 */
void sendNMTMsg(BYTE cmd);

/**
 * Sends a SYNC message to the CAN Bus.
 */
void sendSync();

/**
 * Request System Voltage and Current measurements and three passive boggie position readings
 */
void requestSystemInfo();

/**
 * Function that controls the threaded execution of message handling.
 * Messages are then treated asynchronously as they are received and independently of the main threads execution.
 */
void* msg_handler(void *args);

/**
 * Function that catches the Timer signal and is executed every TIMER_INTERVAL_SEC (see definition above).
 * Status monitoring, life guarding and motor readings request is done periodically in this method.
 */
void timer_handler (int signum);


//-----------------------------------------------

/**
 * Represents and Controls all Drive components on an arbitrary platform.
 * Drives shall be connected in a CAN Bus network and comply with the CANopen protocol to control different types of motors.
 */
class Platform_Driver
{
public:

	/** 
	 * Default constructor.
	 */
	Platform_Driver(int num_motors, int num_nodes, int can_dev_type, std::string can_dev_addr, int watchdog);

	/**
	 * Default destructor.
	 */
	~Platform_Driver();

	/** 
	 * Initializes all CAN nodes of the platform and performs homing procedure of the steered motors.
	 * Note: The homing routine is hardware-dependent. This method is now adapted to ExoTer rover configuration.
	 * Re-adapt this before using in different platforms.
	 * @return True if initialization is successful, false otherwise.
	 */
	bool initPltf(GearMotorParamType wheel_drive, GearMotorParamType steer_drive, GearMotorParamType walk_drive, GearMotorParamType pan_drive, GearMotorParamType tilt_drive, GearMotorParamType arm_joint, PltfCanParams can_parameters);

	/**
	 * Sets CAN node and interface configuration and motor parameters for each drive.
	 * (should be adapted to use RoCK component properties)
	 */
	bool readConfiguration(GearMotorParamType wheel_drive, GearMotorParamType steer_drive, GearMotorParamType walk_drive, GearMotorParamType pan_drive, GearMotorParamType tilt_drive, GearMotorParamType arm_joint, PltfCanParams can_parameters);

	/**
	 * Checks for errors in the platform drives.
	 * @return True if an error is found.
	 */
	bool isPltfError();

	/**
	 * Shuts down the platform.
	 * Disables motors, enables brake and disconnects.
	 * @return True if all drives are properly shut down.
	 */
	bool shutdownPltf();

	/**
	 * Shuts down the specific Node.
	 * Disables motor, enables brake and disconnects.
	 * @return True if the drive is properly shut down.
	 */
	bool shutdownNode(int iCanIdent);

	/**
	 * Reinitializes the nodes on the bus.
	 * The function might be necessary after an emergency stop or an hardware failure to re-init drives.
	 * @return True if re-initialization is successful, false otherwise.
	 */
	bool resetPltf();

	/**
	 * Reinitializes the specific node on the bus.
	 * The function might be necessary after an emergency stop or an hardware failure to re-init drives.
	 * @return True if re-initialization is successful, false otherwise.
	 */
	bool resetNode(int iCanIdent);

	/**
	 * Configures and starts the watchdog of all the CAN components.
	 * @param bStart If true the Watchdog is configured and started, if false the watchdog is not configured and not life guarding is performed by the drives.
	 * @return True if configuration is performed properly for all drives.
	 */
	bool startWatchdog(bool bStart);

	/**
	 * Triggers evaluation of the can-buffer.
	 * Looks for messages that arrived to the CAN interface device and calls the functions to process them.
	 * Use in case message handler thread is not running (yet).
	 * @return Value 0 if no errors and -1 if an error occurred.
	 */
	int evalCanBuffer();

	/**
	 * Gets the number of motors in the platform.
	 * @return Integer value of the number of motors (nodes) in the platform
	 */
	int getNumMotors();

	/**
	 * Sets up the timer settings and callback function (timer_handler) to call this method periodically.
	 */
	void configureTimer();

	/**
	 * Sends position (and velocity) command for specific can node (PTP Motion).
	 * Node must be in position control mode
	 * @param iCanIdent selects the can node
	 * @param dPosGearRad position command in radians
	 * @param dVelGearRadS velocity command in radian per second
	 */	
	void nodePositionCommandRad(int iCanIdent, double dPosRad, double dVelRadS=0);

	/**
	 * Sets the value for the reference position and velocity for specific can node (PTP Motion).
	 * It does not execute the command. Useful for synchronized motion.
	 * @param iCanIdent selects a can node
	 * @param dPosGearRad position setpoint in radians
	 * @param dVelGearRadS velocity setpoint in radian per second
	 */
	void nodePositionSetPointRad (int iCanIdent, double dPosRad, double dVelRadS=0);

	/**
	 * Sends velocity command for specific can node.
	 * Node must be in velocity control mode
	 * @param iCanIdent selects the can node
	 * @param dVelGearRadS velocity command in radian per second
	 */	
	void nodeVelocityCommandRadS(int iCanIdent, double dVelRadS);

	/**
	 * Sets the value for the reference velocity for specific can node.
	 * It does not execute the command. Useful for synchronized motion.
	 * @param iCanIdent selects a can node
	 * @param dVelGearRadS velocity setpoint in radian per second
	 */
	void nodeVelocitySetPointRadS(int iCanIdent, double dVelRadS);

	/**
	 * Sends Begin Execution Command to to start synchronized motion
	 * Use to command simultaneously previously set reference position and velocities of a GROUP of nodes
	 * @param iCanIdent selects the can node GROUP. Is the GROUP ID. If a single node is selected only the set point of that node will be executed and no synchronized motion will be achieved.
	 */
	void nodeCommandSetPoint(int iCanIdent);

	/**
	 * Sends torque command for specific can node.
	 * @param iCanIdent selects the can node
	 * @param dTorqueNM motor-torque in Nm
	 */	
	void nodeTorqueCommandNm(int iCanIdent, double dTorqueNm);

	/**
	 * Requests the status of a given can node.
	 */
	void requestNodeStatus(int iCanIdent);

	/**
	 * Requests position and velocity of a given can node.
	 * @param iCanIdent selects the can node
	 */
	void requestNodePosVel(int iCanIdent);

	/**
	 * Requests motor-torque (active current) of a given node.
	 * @param iCanIdent selects the can node
	 */
	void requestNodeTorque(int iCanIdent);

	/**
	 * Gets the position and velocity of a given node.
	 * @param iCanIdent selects the can node
	 * @param pdAngleGearRad The value (in radians) of the current position of the motor is stored in this pointer.
	 * @param pdVelGearRadS The value (in radians/s) of the current velocity of the motor is stored in this pointer.
	 */
	void getNodePositionVelocityRadS(int iCanIdent, double* pdPositionRad, double* pdVelocityRadS);

	/**
	 * Reads the change of the position and the velocity of a given node with respect to the last time that this function was called.
	 * @param iCanIdent selects the can node
	 * @param pdDeltaAngleGearRad The value (in radians) of the delta position of the motor is stored in this pointer.
	 * @param pdDeltaVelGearRadS The value (in radians/s) of the delta velocity of the motor is stored in this pointer.
	 */
	void getNodeDeltaPositionVelocityRadS(int iCanIdent, double* pdDeltaPositionRad, double* pdDeltaVelocityRadS);

	/**
	 * Gets the velocity of a given node.
	 * @param iCanIdent selects the can node
	 * @param pdVelGearRadS The value (in radians/s) of the current velocity of the motor is stored in this pointer.
	 */
	void getNodeVelocityRadS(int iCanIdent, double* pdVelocityRadS);

	/**
	 * Gets the status and temperature in degree celcius.
	 * (Not implemented for CanDriveWhistle)
	 * @param iCanIdent selects the can node
	 */
	void getNodeStatus(int iCanIdent, int* piStatus, int* piTempCel);

	/**
	 * Gets the motor torque (from active current) of a given node.
	 * @param iCanIdent selects the can node
	 * @param pdTorqueNm The value (in Nm) of the current motor torque is stored in this pointer.
	 */
	void getNodeTorque(int iCanIdent, double* pdTorqueNm);


	bool getNodeData(int iCanIdent,double* pdAngleGearRad, double* pdVelGearRadS, double* pdCurrentAmp, double* pdTorqueNm);

	void getNodeAnalogInput(int iCanIdent,double* pdAnalogInput);

	/**
	 * Handles the transition in between different driving modes of the rover platform.
	 * Makes sure that the necessary initial conditions for driving in a given mode are met.
	 * @param Mode from the enum list PltfDrivingMode to be set
	 * @return True if mode was set and initial conditions were met.
	 */
	//bool setDrivingMode(PltfDrivingMode mode);

	/**
	 * **** Generic Manoeuvre Library ****
	 * Drives the rover platform in straight linear motion (forwards or backwards depending on the velocity sign).
	 * @param dVelocity Is the velocity in cm/s at which the rover shall be commanded
	 */
	//void pltfDriveStraightVelocityCmS(double dVelocity);

	/**
	 * **** Generic Manoeuvre Library ****
	 * Drives the rover platform to perform a Generic Ackerman manoeuvre (forwards or backwards depending on the velocity sign).
	 * @param dVelocity Is the velocity in cm/s at which the rover shall be commanded
	 * @param dRotationCenter is the Center of Rotation of the Generic Ackerman manoeuvre. {x,y} position values
	 * @param dPointToControl is the Point to Control of the Generic Ackerman manoeuvre. {x,y} position values
	 */
	//void pltfDriveGenericAckerman(double dVelocity, double *dRotationCenter, double *dPointToControl);

	/**
	 * **** Generic Manoeuvre Library ****
	 * Drives the rover platform to perform a Spot Turn (left or right depending on the velocity sign).
	 * @param dVelocity Is the velocity in radian/s at which the rover shall be commanded
	 */
	//void pltfDriveSpotTurn(double dAngularVelocity);

	/**
	 * **** Generic Manoeuvre Library ****
	 * Drives the rover platform to perform a Skid Turn (left or right depending on the sign of the radius of curvature).
	 * @param dVelocity Is the velocity in cm/s at which the rover shall be commanded
	 * @param dRadiusOfCurvature Is the Radius of Curvature of the turning motion
	 * @param iIsStraightLine determines if a straight line motion is desired
	 */
	//void pltfDriveSkidTurn(double dVelocity, double dRadiusOfCurvature, int iIsStraightLine);

	/**
	 * **** Generic Manoeuvre Library ****
	 * Drives the rover platform to perform a Wheel Walking manoeuvre. **** under development ****
	 * @param dStepLength
	 * @param iGait is the wheel walk gait to be used for the manoeuvre
	 */
	//void pltfDriveWheelWalk(double *dStepLength, int iGait);


	/**
	 * Direct Drive mode motion commands. Use these functions in direct drive control mode only. They control one single motor at a time. Use for drive testing.
	 * @param iWheel or iJoint selects the CAN node motor to be commanded.
	 * @param dVelocity or dAngle is the command for the motor.
	 */
	//void directWheelDriveVelocityDegS(int iWheel, double dVelocity);
	//void directWheelSteerAngleDeg(int iWheel, double dAngle);
	//void directManipJointAngleDeg(int iJoint, double dAngle);
	//void directWheelWalkJointAngleDeg(int iJoint, double dAngle);
	//void directMastPanAngleDeg(double dAngle);
	//void directMastTiltAngleDeg(double dAngle);


//protected:

	/**
	 * Starts up can nodes
	 */
	//void sendNetStartCanOpen();
	
	//PltfDrivingMode m_DrivingMode;							/**< Stores the current driving mode of the rover platform. Default: STOPPED */

	//* Params for all Motor/Gears
	//GearMotorParamType m_GearMotDriveSteer;					/**< Motor parameters for the Single Motor test */
	
	GearMotorParamType m_GearMotWheelDrive;					/**< Parameters of Driving motors */
	GearMotorParamType m_GearMotWheelSteer;					/**< Parameters of Steering motors */
	GearMotorParamType m_GearMotWheelWalk;					/**< Parameters of Walking motors */
	GearMotorParamType m_GearMotManipJoint;					/**< Parameters of Manipulator motors */
	GearMotorParamType m_GearMotMastPan;					/**< Parameters of Mast Pan motor */
	GearMotorParamType m_GearMotMastTilt;					/**< Parameters of Mast Tilt motor */


	//--------------------------------- Variables

	std::string can_address;								/**< Address of the can device interface in the system Ex: "/dev/pcan0" */
	CanMsg m_CanMsgRec;										/**< Stores the last CAN msg received at Host */
//	Mutex m_Mutex;											/**< semafore for accessing shared variables */
	int m_iCanItfType;										/**< Type of CAN interface device */
	int m_iNumMotors;										/**< Number of motors in the platform */
	int m_iNumNodes;										/**< Number of CAN nodes in the platform. Includes the 'virtual' group nodes */
	//double dCanTimeout;									/**< Period of the timer function */
	//bool m_bWatchdogErr;									/**< Watchdog error flag */
	

	// vector with enums (specifying hardware-structure)
	// this has to be adapted in c++ file to your hardware
	//std::vector<int> m_viMotorID;


	//* Generic Manoeuvre Library variables
	//ROVER_PARAM MyRover;									/**< Rover parameters structure */
	//double m_dWheelVelocity[NUM_WHEELS];					/**< Wheel driving velocity commands */
	//double m_dWheelSteering[NUM_WHEELS];					/**< Wheel steering position commands */
	//double m_dWalkAngleRad[NUM_WHEELS];						/**< Walking angle commands */
	//double m_dWheelAngleRad[NUM_WHEELS];					/**< Wheel driving position commands (wheel walking manoeuvre) */


	pthread_t msg_task;										/**< thread variable */
	int rc;													/**< return code of the the thread creation function */

};


//-----------------------------------------------
#endif
