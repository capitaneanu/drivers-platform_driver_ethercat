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
    static void *ecatcheck(void *ptr);
private:
    char _io_map[4096];
    static int _expected_wkc;
    static volatile int _wkc;
    static bool _is_initialized;
    pthread_t _ecatcheck_thread_handle;
};

#endif
