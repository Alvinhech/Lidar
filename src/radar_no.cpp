#include <stdio.h>
#include <stdlib.h>
#include<iostream>

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>

static inline void delay(_word_size_t ms)
{
    while (ms>=1000)
    {
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}
#endif

using namespace rp::standalone::rplidar;

#define N_SIZE 360 

bool checkRPLIDARHealth(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result))
    { 
        // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR)
        {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            return false;
        }
        else
        {
            return true;   
        }
    } 
    else
    {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

#include <signal.h>
bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

int main(int argc, const char * argv[]) 
{
    RPlidarDriver * drv;
    const char * opt_com_path = NULL;
    _u32         opt_com_baudrate = 115200;
    u_result     op_result;

    printf("Ultra simple LIDAR data grabber for RPLIDAR.\n"
           "Version: "RPLIDAR_SDK_VERSION"\n");

    // read serial port from the command line...
    if (argc>1) opt_com_path = argv[1]; // or set to a fixed value: e.g. "com3" 

    // read baud rate from the command line if specified...
    if (argc>2) opt_com_baudrate = strtoul(argv[2], NULL, 10);

    if (!opt_com_path) 
    {
#ifdef _WIN32
        // use default com port
        opt_com_path = "\\\\.\\com3";
#else
        opt_com_path = "/dev/ttyUSB0";
#endif
    }
    
    bool b_valid = false;
    int com_no = -1;
    while(com_no < 20)
    {
        com_no++;

#ifdef _WIN32
        std::string opt_com_path = std::string("\\\\.\\com") + to_string(com_no);
#else
        std::string opt_com_path = std::string("/dev/ttyUSB") + std::to_string(com_no);
#endif

        // create the driver instance
        drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
        rplidar_response_device_info_t devinfo;

        if (!drv) 
        {
            fprintf(stderr, "insufficent memory, exit\n");
            com_no--;
            continue;
            //exit(-2);
        }

        // make connection...
        if (IS_FAIL(drv->connect(opt_com_path.c_str(), opt_com_baudrate)))
        {
            RPlidarDriver::DisposeDriver(drv);
            continue;
        }
    	
        // retrieving the device info
        op_result = drv->getDeviceInfo(devinfo);

        if (IS_FAIL(op_result)) 
        {
            fprintf(stderr, "Error, cannot get device info.\n");
            
            RPlidarDriver::DisposeDriver(drv);
            continue;
        }

        // print out the device serial number, firmware and hardware version number..
        printf("RPLIDAR S/N: ");
        char buf[32];
        int offset = 0;
        for (int pos = 0; pos < 16 ;++pos) 
        {
            int j = sprintf(buf+offset,"%02X",devinfo.serialnum[pos]);
            offset += j;
            printf("%02X", devinfo.serialnum[pos]);
        }
        
        printf("\n"
                "Firmware Ver: %d.%02d\n"
                "Hardware Rev: %d\n"
                , devinfo.firmware_version>>8
                , devinfo.firmware_version & 0xFF
                , (int)devinfo.hardware_version);
 
        if(!b_valid)
        {
            b_valid = true;
            break;
        }

    }

    if(!b_valid)
    {
        std::cout<<"fail to connect radar..."<<std::endl;
        return -1;
    }

    // check health...
    if (!checkRPLIDARHealth(drv)) 
    {
        goto on_finished;
    }

	signal(SIGINT, ctrlc);
    
	drv->startMotor();
    // start scan...
    drv->startScan();
    
    // fetech result and print it out...
    while (1) 
    {   
        rplidar_response_measurement_node_t nodes[N_SIZE*2];
        size_t  count = _countof(nodes);
            
        op_result = drv->grabScanData(nodes, count);

        if (IS_OK(op_result)) 
        {
            
            drv->ascendScanData(nodes, count);
           
            for (int pos = 0; pos < (int)count ; ++pos) 
            { 
                int pos_int = (int)((nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f + 0.5f);
                float distance = nodes[pos].distance_q2/4.0f/10.0f;
                //std::cout<<pos_int<<":"<<distance<<std::endl;
            }
        }

        if (ctrl_c_pressed){
			break;
        }

    }

    drv->stop();
    drv->stopMotor();
    // done!
on_finished:
    RPlidarDriver::DisposeDriver(drv);
    return 0;
}



