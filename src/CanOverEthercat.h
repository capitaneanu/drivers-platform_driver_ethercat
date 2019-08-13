#pragma once

#include <map>
#include <string>
#include <thread>

namespace platform_driver_ethercat
{

class CanDevice;

class CanOverEthercat
{
  public:
    CanOverEthercat(const std::string interface_address, const unsigned int num_slaves);
    ~CanOverEthercat();
    bool init();
    void close();
    bool isInit();
    bool addDevice(CanDevice* device);
    unsigned char* getInputPdoPtr(uint16_t slave);
    unsigned char* getOutputPdoPtr(uint16_t slave);

    static bool sdoRead(uint16_t slave, uint16_t idx, uint8_t sub, int* data);
    static bool sdoWrite(uint16_t slave, uint16_t idx, uint8_t sub, int fieldsize, int data);

  private:
    const std::string interface_address_;
    const unsigned int num_slaves_;
    char io_map_[4096];
    bool is_initialized_;
    std::map<unsigned int, CanDevice*> devices_;
    std::thread ethercat_thread_;

    static int expected_wkc_;
    static volatile int wkc_;

    void pdoCycle();
};

}
