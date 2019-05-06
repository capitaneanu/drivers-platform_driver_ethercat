#ifndef CANOVERETHERCAT_INCLUDEDEF_H
#define CANOVERETHERCAT_INCLUDEDEF_H

#include <map>
#include <string>
#include <thread>

class CanDriveTwitter;

class CanOverEthercat
{
public:
	CanOverEthercat(const std::string device_name);
	~CanOverEthercat();
	bool init();
	void close();
	bool isInit();
    bool addDrive(CanDriveTwitter *drive);
    unsigned char *getInputPdoPtr(uint16_t slave);
    unsigned char *getOutputPdoPtr(uint16_t slave);

    static bool sdoRead(uint16_t slave, uint16_t idx, uint8_t sub, int *data);
    static bool sdoWrite(uint16_t slave, uint16_t idx, uint8_t sub, int fieldsize, int data);
private:
    char _io_map[4096];
    std::string _device_name;
    bool _is_initialized;
    std::map<unsigned int, CanDriveTwitter *> _drives_twitter;
    std::thread _ethercat_thread;

    static int _expected_wkc;
    static volatile int _wkc;

    void pdoCycle();
};

#endif
