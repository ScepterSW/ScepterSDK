#include "rclcpp/rclcpp.hpp"
#include "Scepter_api.h"
int main() 
{
    ScStatus status = ScStatus::SC_OK;
    // Initialise the API
    status = scInitialize();
    if (status != ScStatus::SC_OK)
    {
        RCLCPP_INFO(rclcpp::get_logger("sc_enumerate_devices"),"scInitialize failed! %d" ,status);
        return 1;
    }
    int ind = 0;
    int waitCnt = 10;
    uint32_t device_count = 0;//The number of available devices
    while(ind < waitCnt)
    {
        status = scGetDeviceCount(&device_count, 3000);
        if (status != ScStatus::SC_OK)
        {
            RCLCPP_INFO(rclcpp::get_logger("sc_enumerate_devices"), "scGetDeviceCount failed! %d" ,status);
            break;
        }

        if (0 == device_count)
        {
            RCLCPP_INFO(rclcpp::get_logger("sc_enumerate_devices"), "The device count is 0, Please attach the camera, wait progress: %d/%d", ind + 1, waitCnt);
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("sc_enumerate_devices"), "Find device successfully, The device count is : %d" ,device_count); 
            break;
        }
        ind++;
    }

    ScDeviceInfo* pPsDeviceInfoList = new ScDeviceInfo[device_count];
    status =  scGetDeviceInfoList(device_count, pPsDeviceInfoList);
    if (status != ScStatus::SC_OK)
    {
        RCLCPP_INFO(rclcpp::get_logger("sc_enumerate_devices"), "scGetDeviceInfoList failed! %d" ,status);
        return 1;
    }

    for (int ind = 0; ind < (int)device_count; ind++)
    {
        RCLCPP_INFO(rclcpp::get_logger("sc_enumerate_devices"), "device %d ip %s seriesnumber %s status %d", (ind + 1), pPsDeviceInfoList[ind].ip, pPsDeviceInfoList[ind].serialNumber, pPsDeviceInfoList[ind].status);
    }

    status = scShutdown();
    if (status != ScStatus::SC_OK)
    {
        RCLCPP_INFO(rclcpp::get_logger("sc_enumerate_devices"), "scShutdown failed! %d" ,status);
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
