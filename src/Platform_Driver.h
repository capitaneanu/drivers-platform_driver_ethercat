#ifndef PLATFORMDRIVER_INCLUDEDEF_H
#define PLATFORMDRIVER_INCLUDEDEF_H

//* general includes ---------------------------------------------
#include <string>
#include <vector>

//* other motion library includes (try to remove) ----------------
#include "Mutex.h"

#include "CanDriveTwitter.h"
#include "CanOverEthercat.h"

enum MotorStatus {INACTIVE, ACTIVE};

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
 * Represents and Controls all Drive components on an arbitrary platform.
 * Drives shall be connected in a CAN Bus network and comply with the CANopen protocol to control different types of motors.
 */
class Platform_Driver
{
public:

	/** 
	 * Default constructor.
	 */
	Platform_Driver(unsigned int num_motors, unsigned int num_nodes, unsigned int can_dev_type, std::string can_dev_addr, unsigned int watchdog);

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
	bool initPltf(GearMotorParamType wheel_drive_params, GearMotorParamType steer_drive_params, GearMotorParamType walk_drive_params, GearMotorParamType pan_drive_params, GearMotorParamType tilt_drive_params, GearMotorParamType arm_drive_params, PltfCanParams can_params);

	/**
	 * Sets CAN node and interface configuration and motor parameters for each drive.
	 * (should be adapted to use RoCK component properties)
	 */
	bool readConfiguration(GearMotorParamType wheel_drive_params, GearMotorParamType steer_drive_params, GearMotorParamType walk_drive_params, GearMotorParamType pan_drive_params, GearMotorParamType tilt_drive_params, GearMotorParamType arm_drive_params, PltfCanParams can_params);

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
	bool shutdownNode(unsigned int drive_id);
	
    /**
	 * Starts the specific Node.
	 * Enables the motor, check the status.
	 * @return True if the drive is properly started.
	 */
	bool startNode(unsigned int drive_id);


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
	bool resetNode(unsigned int drive_id);

	/**
	 * Sends position (and velocity) command for specific can node (PTP Motion).
	 * Node must be in position control mode
	 * @param drive_id selects the can node
	 * @param dPosGearRad position command in radians
	 * @param dVelGearRadS velocity command in radian per second
	 */	
	void nodePositionCommandRad(unsigned int drive_id, double dPosRad);

	/**
	 * Sends velocity command for specific can node.
	 * Node must be in velocity control mode
	 * @param drive_id selects the can node
	 * @param dVelGearRadS velocity command in radian per second
	 */	
	void nodeVelocityCommandRadS(unsigned int drive_id, double dVelRadS);

	/**
	 * Sends torque command for specific can node.
	 * @param drive_id selects the can node
	 * @param dTorqueNM motor-torque in Nm
	 */	
	void nodeTorqueCommandNm(unsigned int drive_id, double dTorqueNm);

	/**
	 * Requests the status of a given can node.
	 */
	void requestNodeStatus(unsigned int drive_id);

	/**
	 * Requests position and velocity of a given can node.
	 * @param drive_id selects the can node
	 */
	void requestNodePosVel(unsigned int drive_id);

	/**
	 * Requests motor-torque (active current) of a given node.
	 * @param drive_id selects the can node
	 */
	void requestNodeTorque(unsigned int drive_id);

	/**
	 * Gets the position and velocity of a given node.
	 * @param drive_id selects the can node
	 * @param pdAngleGearRad The value (in radians) of the current position of the motor is stored in this pointer.
	 */
	void getNodePositionRad(unsigned int drive_id, double* pdPositionRad);

	/**
	 * Gets the velocity of a given node.
	 * @param drive_id selects the can node
	 * @param pdVelGearRadS The value (in radians/s) of the current velocity of the motor is stored in this pointer.
	 */
	void getNodeVelocityRadS(unsigned int drive_id, double* pdVelocityRadS);

	/**
	 * Gets the motor torque (from active current) of a given node.
	 * @param drive_id selects the can node
	 * @param pdTorqueNm The value (in Nm) of the current motor torque is stored in this pointer.
	 */
	void getNodeTorque(unsigned int drive_id, double* pdTorqueNm);


	bool getNodeData(unsigned int drive_id, double* pdAngleGearRad, double* pdVelGearRadS, double* pdCurrentAmp, double* pdTorqueNm);

	void getNodeAnalogInput(unsigned int drive_id, double* pdAnalogInput);

private:
	std::string _can_address;						/**< Address of the can device interface in the system */
    CanOverEthercat* _can_interface;				/**< CAN interface device class object (PeakSysUSB) */
    std::vector<CanDriveTwitter *> _can_drives;		/**< Motor controllers. Pointer to each motor's CanDrive-Itf */
    PltfCanParams _can_parameters;	    	/**< Array of CanNodeTypes. Keeps information of all Node IDs and high level description */
    unsigned int _num_nodes;
};


//-----------------------------------------------
#endif
