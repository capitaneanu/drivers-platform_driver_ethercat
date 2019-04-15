#ifndef CANOVERETHERCAT_INCLUDEDEF_H
#define CANOVERETHERCAT_INCLUDEDEF_H

#include <string>

class CanOverEthercat
{
public:
	CanOverEthercat(const std::string device_name);
	~CanOverEthercat();
	bool init();
	void close();
	bool isInit();
    unsigned char *getInputPdoPtr(uint16_t slave);
    unsigned char *getOutputPdoPtr(uint16_t slave);

    static bool sdoRead(uint16_t slave, uint16_t idx, uint8_t sub, int *data);
    static bool sdoWrite(uint16_t slave, uint16_t idx, uint8_t sub, int fieldsize, int data);
private:
    char _io_map[4096];
    std::string _device_name;
    bool _is_initialized;
    pthread_t _thread_handle;

    static int _expected_wkc;
    static volatile int _wkc;

    static int driveSetup(uint16_t slave);
    static void *pdoCycle(void *ptr);
};

#endif
