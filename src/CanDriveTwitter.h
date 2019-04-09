#ifndef CANDRIVETWITTER_INCLUDEDEF_H
#define CANDRIVETWITTER_INCLUDEDEF_H

//* include files ---------------------------------------------

#include "CanOverEthercat.h"
#include "DriveParam.h"

/**
 * Interface description for a drive type of class.
 */
class CanDriveTwitter
{
public:
	/**
	 * The constructor
	 */
	CanDriveTwitter(CanOverEthercat *can_interface, std::string name);

	/**
	 * The destructor
	 */
	~CanDriveTwitter();

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
	bool isInitialized();

	/**
	 * Brings the drive to operation enable state. 
	 * After calling the drive accepts velocity and position commands.
	 * @return True if drive is started successfully. StatusRegister is also evaluated to ensure a non-faulty state. False otherwise.
	 */
	bool startup();

	/**
	 * Brings the drive to switch on disabled state. 
	 * After calling the drive won't accepts velocity and position commands.
	 * @return True if drive shutdown successful.
	 */
	bool shutdown();

	/**
	 * Resets the drive.
	 * @return True if re-initialization was successful. False otherwise.
	 */
	bool reset();

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
     * Sends Torque command
	 * Use this function only in torque control mode.
     * @param dTorqueNm is the required motor torque in Nm.
     */
    void torqueCommandNm(double dTorqueNm);

	/**
	 * Send execution command to start synchronized motion.
	 * @see positionSetPointRad()
	 * @see velocitySetPointRadS()
	 */
	void commandSetPoint();

    /**
     * Checks if the target set point was already reached.
     * @return True if the target set point was already reached.
     */
    bool checkTargetReached();

	/**
	 * Reads the last received value of the drive position.
	 * @return The value of the current position of the motor.
	 */
	double getPositionRad();

	/**
	 * Reads the last received value of the drive Velocity.
	 * @return The value of the current Velocity of the motor.
	 */
	double getVelocityRadS();

	/**
	 * Reads the last received value of the motor Torque.
	 * @return The value (in Nm) of the current motor torque is stored in this pointer.
	 */
	double getTorqueNm();
    
	/**
	 * Returns received value from analog input.
	 */
	double getAnalogInput();

    StateTwitter getState()

	/**
	 * Returns true if an error has been detected.
	 * @return boolean with result.
	 */
	bool isError();
	
	/**
	 * Returns a bitfield containing information about the current error.
	 * @return unsigned int with bitcoded error.
	 */
	unsigned int getError();

	/**
	 * Enable the emergency stop.
	 * @return true if the result of the process is successful
	 */
	bool setEmergencyStop();

	/**
	 * Disable the emergency stop.
	 * @return true if the result of the process is successful
	 */
	bool resetEmergencyStop();

	/**
	 * Sets the drive parameters.
	 * @param driveParam is the object of the DriveParam class that contains the values of the drive parameters.
	 */
	void setDriveParam(DriveParam driveParam);

	/**
	 * Gets the drive parameters
	 * @return Pointer to the object of type DriveParam that is contained in the drive class.
	 */
	DriveParam *getDriveParam();

private:
	/**
	 * States of the CANOpen drive state machine. 
	 */
	enum StateTwitter
	{
		ST_NOT_READY_TO_SWITCH_ON,
		ST_SWITCH_ON_DISABLED,
		ST_READY_TO_SWITCH_ON,
		ST_SWITCHED_ON,
		ST_OPERATION_ENABLE,
		ST_QUICK_STOP_ACTIVE,
        ST_FAULT_REACTION_ACTIVE,
		ST_FAULT
	};

	/**
	 * Enum with different operation modes of the controller, either position, velocity or torque control.
	 */
	enum OperationMode
	{
		OM_PROFILE_POSITION = 1,
        OM_PROFILE_VELOCITY = 3,
        OM_PROFILE_TORQUE = 4,
		OM_CYCSYNC_POSITION = 8,
        OM_CYCSYNC_VELOCITY = 9,
        OM_CYCSYNC_TORQUE = 10
	};

    typedef struct RxPDO
    {
        uint16 control_word;
        uint16 operation_mode;
        int32 target_position;
        int32 target_velocity;
        int16 target_torque;
    } RxPDO;
    
    typedef struct TxPDO
    {
        uint16 status_word;
        uint8 operation_mode_display;
        int32 actual_position;
        int32 actual_velocity;
        int16 actual_torque;
        int16 analog_input;
    } TxPDO;

    CanOverEthercat *_can_interface;
    std::string _device_name;
    RxPDO *output;
    TxPDO *input;

	/**
	 * Returns the state of the drive
	 */
	StateTwitter getState();
    OperationMode getOperationMode()
    bool setOperationMode(OperationMode mode)
};

//-----------------------------------------------
#endif

