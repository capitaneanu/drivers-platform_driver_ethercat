#pragma once

namespace platform_driver_ethercat
{

class CanOverEthercat;

class CanDevice
{
  public:
    CanDevice(CanOverEthercat* can_interface, unsigned int can_id, std::string device_name);
    virtual ~CanDevice() = 0;

    virtual bool configure() = 0;
    virtual bool startup() = 0;
    virtual bool shutdown() = 0;
    virtual bool reset() = 0;
    virtual bool isError() = 0;

    virtual void setInputPdo(unsigned char* input_pdo) = 0;
    virtual void setOutputPdo(unsigned char* output_pdo) = 0;

    unsigned int getCanId();
    std::string getDeviceName();

  protected:
    CanOverEthercat* _can_interface;
    unsigned int _can_id;
    std::string _device_name;
};

}
