#ifndef CANOVERETHERCAT_INCLUDEDEF_H
#define CANOVERETHERCAT_INCLUDEDEF_H

#include <string>

class CanOverEthercat
{
public:
	CanOverEthercat(const std::string& devName);
	~CanOverEthercat();
	void close();
	bool isInit();
    bool sdoRead(uint16 slave, uint16 idx, uint8 sub, int *data);
    bool sdoWrite(uint16 slave, uint16 idx, uint8 sub, int fieldsize, int data);
    char* getInputPdoPtr(uint16 slave);
    char* getOutputPdoPtr(uint16 slave);
private:
    char _io_map[4096];
    bool _is_initialized;
    static int _expected_wkc;
    static volatile int _wkc;
    pthread_t _thread_handle;

	void init(const std::string& devName);
    void driveSetup(uint16 slave);
    static void *pdoCycle(void *ptr);
};

#endif
