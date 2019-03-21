#include "CANOverEthercat.h"
#include "soem/ethercat.h"

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
     /* initialise SOEM, bind socket to ifname */
     if (ec_init(devName.c_str()))
     {
        printf("ec_init on %s succeeded.\n",devName.c_str());
        /* find and auto-config slaves */
        if ( ec_config_init(FALSE) > 0 )
        {
            printf("%d slaves found and configured.\n",ec_slavecount);
//            ec_config_map(&IOmap);
            ec_configdc();
            printf("Slaves mapped, state to SAFE_OP.\n");
        }
     }

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
bool CANOverEthercat::transmitMsg(CanMsg CMsg, bool bBlocking)
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
