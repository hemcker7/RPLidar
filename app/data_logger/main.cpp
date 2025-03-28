/*
 *  SLAMTEC LIDAR
 *  Data Logger for RPLidar S2
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2020 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <fstream>
#include <ctime>

#include "sl_lidar.h" 
#include "sl_lidar_driver.h"
#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(sl_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}
#endif

using namespace sl;

void print_usage(int argc, const char * argv[])
{
    printf("Usage:\n"
           " For serial channel\n %s --channel --serial <com port> [baudrate] [output_file]\n"
           " The baudrate used by different models is as follows:\n"
           "  A1(115200),A2M7(256000),A2M8(115200),A2M12(256000),"
           "A3(256000),S1(256000),S2(1000000),S3(1000000)\n"
           " For udp channel\n %s --channel --udp <ipaddr> [port NO.] [output_file]\n"
           " The T1 default ipaddr is 192.168.11.2,and the port NO.is 8089. Please refer to the datasheet for details.\n"
           , argv[0], argv[0]);
}

bool checkSLAMTECLIDARHealth(ILidarDriver * drv)
{
    sl_result     op_result;
    sl_lidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (SL_IS_OK(op_result)) {
        printf("SLAMTEC Lidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, slamtec lidar internal error detected. Please reboot the device to retry.\n");
            return false;
        } else {
            return true;
        }
    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

int main(int argc, const char * argv[]) {
    const char * opt_is_channel = NULL; 
    const char * opt_channel = NULL;
    const char * opt_channel_param_first = NULL;
    const char * output_file = NULL;
    sl_u32         opt_channel_param_second = 0;
    sl_u32         baudrateArray[2] = {115200, 256000};
    sl_result     op_result;
    int          opt_channel_type = CHANNEL_TYPE_SERIALPORT;
    bool useArgcBaudrate = false;
    IChannel* _channel = NULL;
    time_t now = 0;
    char* dt = NULL;
    char filename[100] = {0};
    time_t current_time = 0;
    std::ofstream outFile;
    ILidarDriver * drv = NULL;
    sl_lidar_response_device_info_t devinfo;
    bool connectSuccess = false;

    // Scan data variables
    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = 0;
    float last_angle = 0;
    bool scan_complete = false;
    const int BARCOUNT = 360;  // One bar per degree
    bool angle_histogram[BARCOUNT] = {false};  // Track which angles we've seen
    int points_per_degree[BARCOUNT] = {0};  // Track number of points per degree
    const int MAX_POINTS_PER_DEGREE = 5;  // Maximum points to save per degree
    int scan_count = 0;
    bool first_scan = true;
    bool skip_next = false;  // Flag to skip every other point

    printf("RPLidar S2 Data Logger\n"
           "Version: %s\n", SL_LIDAR_SDK_VERSION);

    if (argc < 4) {
        print_usage(argc, argv);
        return -1;
    }

    opt_is_channel = argv[1];
    opt_channel = argv[2];
    opt_channel_param_first = argv[3];
    if (argc > 4) {
        opt_channel_param_second = strtoul(argv[4], NULL, 10);
    }
    if (argc > 5) {
        output_file = argv[5];
    } else {
        // Generate default filename with timestamp
        time(&now);
        struct tm *tm_info = localtime(&now);
        strftime(filename, sizeof(filename), "lidar_data_%Y%m%d_%H%M%S.csv", tm_info);
        output_file = filename;
    }

    if(strcmp(opt_is_channel, "--channel")==0) {
        if(strcmp(opt_channel, "-s")==0||strcmp(opt_channel, "--serial")==0) {
            useArgcBaudrate = true;
        }
        else if(strcmp(opt_channel, "-u")==0||strcmp(opt_channel, "--udp")==0) {
            opt_channel_type = CHANNEL_TYPE_UDP;
        }
        else {
            print_usage(argc, argv);
            return -1;
        }
    }
    else {
        print_usage(argc, argv);
        return -1;
    }

    if(opt_channel_type == CHANNEL_TYPE_SERIALPORT) {
        if (!opt_channel_param_first) {
#ifdef _WIN32
            opt_channel_param_first = "\\\\.\\com3";
#elif __APPLE__
            opt_channel_param_first = "/dev/tty.SLAB_USBtoUART";
#else
            opt_channel_param_first = "/dev/ttyUSB0";
#endif
        }
    }

    // create the driver instance
    drv = *createLidarDriver();

    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }

    if(opt_channel_type == CHANNEL_TYPE_SERIALPORT) {
        if(useArgcBaudrate) {
            _channel = (*createSerialPortChannel(opt_channel_param_first, opt_channel_param_second));
            if (SL_IS_OK((drv)->connect(_channel))) {
                op_result = drv->getDeviceInfo(devinfo);
                if (SL_IS_OK(op_result)) {
                    connectSuccess = true;
                }
                else {
                    delete drv;
                    drv = NULL;
                }
            }
        }
        else {
            size_t baudRateArraySize = (sizeof(baudrateArray))/ (sizeof(baudrateArray[0]));
            for(size_t i = 0; i < baudRateArraySize; ++i) {
                _channel = (*createSerialPortChannel(opt_channel_param_first, baudrateArray[i]));
                if (SL_IS_OK((drv)->connect(_channel))) {
                    op_result = drv->getDeviceInfo(devinfo);
                    if (SL_IS_OK(op_result)) {
                        connectSuccess = true;
                        break;
                    }
                    else {
                        delete drv;
                        drv = NULL;
                    }
                }
            }
        }
    }
    else if(opt_channel_type == CHANNEL_TYPE_UDP) {
        _channel = *createUdpChannel(opt_channel_param_first, opt_channel_param_second);
        if (SL_IS_OK((drv)->connect(_channel))) {
            op_result = drv->getDeviceInfo(devinfo);
            if (SL_IS_OK(op_result)) {
                connectSuccess = true;
            }
            else {
                delete drv;
                drv = NULL;
            }
        }
    }

    if (!connectSuccess) {
        (opt_channel_type == CHANNEL_TYPE_SERIALPORT)?
            (fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
                , opt_channel_param_first)):(fprintf(stderr, "Error, cannot connect to the specified ip addr %s.\n"
                , opt_channel_param_first));
        goto on_finished;
    }

    // print out the device serial number, firmware and hardware version number
    printf("SLAMTEC LIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }
    printf("\n"
            "Firmware Ver: %d.%02d\n"
            "Hardware Rev: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version);

    // check health...
    if (!checkSLAMTECLIDARHealth(drv)) {
        goto on_finished;
    }

    signal(SIGINT, ctrlc);

    drv->setMotorSpeed();
    // start scan...
    drv->startScan(0,1);

    printf("Successfully started scan. Saving data to %s\n", output_file);
    outFile.open(output_file);
    if (!outFile.is_open()) {
        fprintf(stderr, "Error, cannot open output file %s.\n", output_file);
        goto on_finished;
    }

    // Write header to file
    outFile << "timestamp,angle,distance,quality,scan_number\n";

    // fetch result and print it out...
    while (1) {
        count = _countof(nodes);
        op_result = drv->grabScanDataHq(nodes, count, 0);  // Added timeout parameter
        
        if (SL_IS_OK(op_result)) {
            drv->ascendScanData(nodes, count);
            scan_count++;  // Increment scan count for each batch of data
            
            for (int pos = 0; pos < (int)count ; ++pos) {
                float current_angle = (nodes[pos].angle_z_q14 * 90.f) / 16384.f;
                float current_distance = nodes[pos].dist_mm_q2/4.0f;
                
                // Only record data points with valid measurements
                if (current_distance > 0 && !skip_next) {
                    // Convert angle to integer degree (0-359)
                    int degree = (int)current_angle;
                    if (degree >= 0 && degree < BARCOUNT) {
                        // Only save if we haven't reached max points for this degree
                        if (points_per_degree[degree] < MAX_POINTS_PER_DEGREE) {
                            time(&current_time);
                            outFile << current_time << "," 
                                   << current_angle << ","
                                   << current_distance << ","
                                   << (int)nodes[pos].quality << ","
                                   << scan_count << "\n";
                            points_per_degree[degree]++;
                        }
                    }
                }
                skip_next = !skip_next;  // Toggle skip flag
                
                // Reset points per degree counter when we complete a full scan
                if (current_angle < last_angle) {  // We've wrapped around
                    memset(points_per_degree, 0, sizeof(points_per_degree));
                    scan_count++;
                }
                last_angle = current_angle;
            }
            
            // Print progress
            printf("Scan #%d - Collected %d data points\n", scan_count, (int)count);
        }

        if (ctrl_c_pressed) {
            break;
        }
        
        delay(50);  // Add small delay between scans
    }

    drv->stop();
    outFile.close();
    printf("Scan stopped. Data saved to %s\n", output_file);

on_finished:
    if(drv) {
        delete drv;
        drv = NULL;
    }
    return 0;
} 