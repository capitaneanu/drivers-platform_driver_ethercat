#include "CANOverEthercat.hpp"
#include "ecrt.h"

CANOverEthercat::CANOverEthercat()
: _isInitialized(false)
{
    init();
}

CANOverEthercat::CANOverEthercat(const std::string& devName)
: _isInitialized(false)
{
    init(devName);
}

CANOverEthercat::~CANOverEthercat()
{
    close();
}
void CANOverEthercat::init()
{
    _isInitialized = true;
}
void CANOverEthercat::init(const std::string& devName)
{
    _isInitialized = true;
}
void CANOverEthercat::close()
{
    _isInitialized = false;
}
bool CANOverEthercat::isInit()
{
    return _isInitialized;
}
int  CANOverEthercat::availableMessages()
{
    return 0;
}
bool CANOverEthercat::transmitMsg(CanMsg CMsg, bool bBlocking = true)
{
    return false;
}
bool CANOverEthercat::receiveMsg(CanMsg* pCMsg)
{
    return false;
}
bool CANOverEthercat::receiveMsgRetry(CanMsg* pCMsg, int iNrOfRetry)
{
    return false;
}
