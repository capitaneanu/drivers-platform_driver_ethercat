#ifndef CANOVERETHERCAT_INCLUDEDEF_H
#define CANOVERETHERCAT_INCLUDEDEF_H
//-----------------------------------------------

class CanOverEthercat
{
public:
	CanOverEthercat(const std::string& devName);
	~CanOverEthercat();
	void init();
	void init(const std::string& devName);
	void close();
	bool isInit();
	int availableMessages();
	bool transmitMsg(CanMsg CMsg, bool bBlocking = false);
	bool receiveMsg(CanMsg* pCMsg);
	bool receiveMsgRetry(CanMsg* pCMsg, int iNrOfRetry);
    static void *pdo_cycle(void *ptr);
private:
    char _io_map[4096];
    static int _expected_wkc;
    static volatile int _wkc;
    static bool _is_initialized;
    pthread_t _thread_handle;
    static bool _pdo_update;
    std::queue<CanMsg> _sdo_messages;
};

#endif
