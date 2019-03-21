#ifndef CANOVERETHERCAT_INCLUDEDEF_H
#define CANOVERETHERCAT_INCLUDEDEF_H
//-----------------------------------------------
#include "CanItf.h"

class CANOverEthercat : public CanItf
{
public:
	CANOverEthercat();
	CANOverEthercat(const std::string& devName);
	~CANOverEthercat();
	void init();
	void init(const std::string& devName);
	void close();
	bool isInit();
	int availableMessages();
	bool transmitMsg(CanMsg CMsg, bool bBlocking = true);
	bool receiveMsg(CanMsg* pCMsg);
	bool receiveMsgRetry(CanMsg* pCMsg, int iNrOfRetry);
private:
    bool _isInitialized;
};

#endif
