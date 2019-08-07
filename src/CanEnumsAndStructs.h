#pragma once

#include <string>
#include <vector>

namespace platform_driver_ethercat
{

enum MotorStatus
{
    INACTIVE,
    ACTIVE
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
    MAST_TILT,
    FT_SENSOR,
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
    int iEncIncrPerRevMot;
    double dGearRatio;
    double dBeltRatio;
    int iSign;
    double dPosLimitLowIncr;
    double dPosLimitHighIncr;
    double dVelMaxEncIncrS;
    double dPtpVelDefaultIncrS;
    double dAccIncrS2;
    double dDecIncrS2;
    bool bIsSteer;
    double dCurrentToTorque;
    double dCurrMax;
    int iEncOffsetIncr;
    double dAnalogFactor;
    double dNominalCurrent;
};

/**
 * List of all can nodes in the platform.
 * Include groups of nodes in order to create a Drive class object to address them altogether.
 * ToDo: Try to remove the dependency of the code from this enumerator as it depends directly in the
 * specific rover.
 */
enum MotorCANNode
{
    CANNODE_WHEEL_DRIVE_FL,
    CANNODE_WHEEL_DRIVE_FR,
    CANNODE_WHEEL_DRIVE_CL,
    CANNODE_WHEEL_DRIVE_CR,
    CANNODE_WHEEL_DRIVE_BL,
    CANNODE_WHEEL_DRIVE_BR,
    CANNODE_WHEEL_STEER_FL,
    CANNODE_WHEEL_STEER_FR,
    CANNODE_WHEEL_STEER_BL,
    CANNODE_WHEEL_STEER_BR,
    CANNODE_WHEEL_WALK_FL,
    CANNODE_WHEEL_WALK_FR,
    CANNODE_WHEEL_WALK_CL,
    CANNODE_WHEEL_WALK_CR,
    CANNODE_WHEEL_WALK_BL,
    CANNODE_WHEEL_WALK_BR,
    CANNODE_MAST_PAN,
    CANNODE_MAST_TILT,
    CANNODE_WHEEL_DRIVE_GROUP,
    CANNODE_WHEEL_STEER_GROUP,
    CANNODE_WHEEL_WALK_GROUP,
    CANNODE_MAST_PTU_GROUP,
    CANNODE_MANIP_JOINT_1,
    CANNODE_MANIP_JOINT_2,
    CANNODE_MANIP_JOINT_3,
    CANNODE_MANIP_JOINT_4,
    CANNODE_MANIP_JOINT_5,
    CANNODE_MANIP_JOINT_GROUP
};

/**
 * List of platform driving modes. So far, this are the modes supported by the Generic Manoeuvre
Library.

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
 * See definition of the array of CanNodeType in PlatformDriverPcan.cpp gathering the information of
 * all nodes in the platform.
 */
struct CanNodeType
{
    int iCanID;
    std::string sName;
    MotorType type;
};

}
