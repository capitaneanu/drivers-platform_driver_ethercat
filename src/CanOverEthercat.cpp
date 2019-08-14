#include "CanOverEthercat.h"
#include "CanDevice.h"
#include "ethercat.h"

using namespace platform_driver_ethercat;

const int EC_TIMEOUTMON = 500;

int CanOverEthercat::expected_wkc_ = 0;
volatile int CanOverEthercat::wkc_ = 0;

CanOverEthercat::CanOverEthercat(const std::string interface_address, const unsigned int num_slaves)
    : interface_address_(interface_address), num_slaves_(num_slaves), is_initialized_(false)
{
}

CanOverEthercat::~CanOverEthercat() { close(); }

bool CanOverEthercat::init()
{
    if (isInit()) return true;

    is_initialized_ = false;

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(interface_address_.c_str()))
    {
        printf("ec_init on %s succeeded.\n", interface_address_.c_str());
        /* find and auto-config slaves */
        if (ec_config_init(FALSE) > 0)
        {
            printf("%d slaves found.\n", ec_slavecount);

            if (num_slaves_ != ec_slavecount)
            {
                printf("Expected number of slaves is different from number of slaves found.\n");
                return false;
            }

            if (devices_.size() > ec_slavecount)
            {
                printf("Number of added devices (%d) is greater than number of slaves found.\n",
                       devices_.size());
                return false;
            }

            /* configure all devices via sdo */
            for (auto& device : devices_)
            {
                unsigned int slave_id = device.first;

                if (slave_id > ec_slavecount)
                {
                    printf("Slave id %d outside range.\n", slave_id);
                    return false;
                }

                device.second->configure();
            }

            // Disable complete access
            // Workaround for bug of FT sensors according to
            // https://github.com/OpenEtherCATsociety/SOEM/issues/251
            for (int i = 1; i <= ec_slavecount; i++)
            {
                ec_slave[i].CoEdetails &= ~ECT_COEDET_SDOCA;
            }

            ec_config_map(&io_map_);
            ec_configdc();

            /* set pointers to pdo map for all devices */
            for (auto& device : devices_)
            {
                unsigned int slave_id = device.first;

                device.second->setInputPdo(ec_slave[slave_id].inputs);
                device.second->setOutputPdo(ec_slave[slave_id].outputs);
            }

            printf("Slaves mapped, state to SAFE_OP.\n");
            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

            expected_wkc_ = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expected_wkc_);

            printf("Request operational state for all slaves\n");
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            /* request OP state for all slaves */
            ec_writestate(0);
            int chk = 40;
            /* wait for all slaves to reach OP state */
            do
            {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

            if (ec_slave[0].state == EC_STATE_OPERATIONAL)
            {
                printf("Operational state reached for all slaves.\n");

                /* create thread for pdo cycle */
                // pthread_create(&_thread_handle, NULL, &pdoCycle, NULL);
                ethercat_thread_ = std::thread(&CanOverEthercat::pdoCycle, this);

                is_initialized_ = true;
                return true;
            }
            else
            {
                printf("Not all slaves reached operational state.\n");
                is_initialized_ = false;

                ec_readstate();

                for (int i = 1; i <= ec_slavecount; i++)
                {
                    if (ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                               i,
                               ec_slave[i].state,
                               ec_slave[i].ALstatuscode,
                               ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }

                printf("Close socket\n");
                ec_close();

                return false;
            }
        }
        else
        {
            printf("No slaves found.\n");

            printf("Close socket\n");
            ec_close();

            return false;
        }
    }
    else
    {
        printf("ec_init on %s not succeeded.\n", interface_address_.c_str());
        return false;
    }
}

void CanOverEthercat::close()
{
    if (isInit())
    {
        printf("\nRequest init state for all slaves\n");
        ec_slave[0].state = EC_STATE_INIT;
        /* request INIT state for all slaves */
        ec_writestate(0);

        // cancel pdo_cycle thread
        // pthread_cancel(_thread_handle);
        // pthread_join(_thread_handle, NULL);

        is_initialized_ = false;
    }

    printf("Close socket\n");
    ec_close();
}

bool CanOverEthercat::isInit() { return is_initialized_; }

bool CanOverEthercat::addDevice(std::shared_ptr<CanDevice> device)
{
    if (isInit())
    {
        printf("EtherCAT interface already initialized. Drive cannot be added afterwards.\n");
        return false;
    }

    devices_.insert(std::make_pair(device->getSlaveId(), device));

    return true;
}

bool CanOverEthercat::sdoRead(uint16_t slave, uint16_t idx, uint8_t sub, int* data)
{
    int fieldsize = sizeof(data);

    int wkc = ec_SDOread(slave, idx, sub, FALSE, &fieldsize, data, EC_TIMEOUTRXM);

    if (wkc == 1)
        return true;
    else
        return false;
}

bool CanOverEthercat::sdoWrite(uint16_t slave, uint16_t idx, uint8_t sub, int fieldsize, int data)
{
    int wkc = ec_SDOwrite(slave, idx, sub, FALSE, fieldsize, &data, EC_TIMEOUTRXM);

    if (wkc == 1)
        return true;
    else
        return false;
}

unsigned char* CanOverEthercat::getInputPdoPtr(uint16_t slave) { return ec_slave[slave].inputs; }

unsigned char* CanOverEthercat::getOutputPdoPtr(uint16_t slave) { return ec_slave[slave].outputs; }

void CanOverEthercat::pdoCycle()
{
    int currentgroup = 0;

    while (1)
    {
        ec_send_processdata();
        wkc_ = ec_receive_processdata(EC_TIMEOUTRET);

        while (EcatError) printf("%s", ec_elist2string());

        if ((wkc_ < expected_wkc_) || ec_group[currentgroup].docheckstate)
        {
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();

            for (int slave = 1; slave <= ec_slavecount; slave++)
            {
                if ((ec_slave[slave].group == currentgroup)
                    && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state > EC_STATE_NONE)
                    {
                        // devices_[slave]->configure();

                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d reconfigured\n", slave);
                        }
                    }
                    else if (!ec_slave[slave].islost)
                    {
                        /* re-check state */
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (ec_slave[slave].state == EC_STATE_NONE)
                        {
                            ec_slave[slave].islost = TRUE;
                            printf("ERROR : slave %d lost\n", slave);
                        }
                    }
                }
                if (ec_slave[slave].islost)
                {
                    if (ec_slave[slave].state == EC_STATE_NONE)
                    {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d recovered\n", slave);
                        }
                    }
                    else
                    {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d found\n", slave);
                    }
                }
            }
            if (!ec_group[currentgroup].docheckstate)
                printf("OK : all slaves resumed OPERATIONAL.\n");
        }

        osal_usleep(10000);  // roughly 100 Hz
    }
}
