#include <iostream>
#include "ros/ros.h"
#include "dynamic_reconfigure/server.h"
#include <scepter_manager.hpp>

int main(int argc, char *argv[]) 
{
    ros::init(argc, argv, "scepter_manager");

    int32_t device_index = ros::param::param<int32_t>("~device_index", 0);

    ScepterManager manager = ScepterManager(device_index);
    dynamic_reconfigure::Server<scepter_param::Sceptertof_roscppConfig> server;
    dynamic_reconfigure::Server<scepter_param::Sceptertof_roscppConfig>::CallbackType f;
 
    f = boost::bind(&ScepterManager::paramCallback,&manager,_1,_2);
    server.setCallback(f);
    manager.run();

    return 0;
}
