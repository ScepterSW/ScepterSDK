#include <iostream>
#include "ros/ros.h"
#include "dynamic_reconfigure/server.h"
#include <scepter_manager.hpp>

int main(int argc, char *argv[]) 
{
    if(argc>=3)
    {
        ros::init(argc, argv, argv[1]);

        ScepterManager manager = ScepterManager(argv[2],argv[1]);
        dynamic_reconfigure::Server<scepter_param::Sceptertof_roscppConfig> server;
        dynamic_reconfigure::Server<scepter_param::Sceptertof_roscppConfig>::CallbackType f;
    
        f = boost::bind(&ScepterManager::paramCallback,&manager,_1,_2);
        server.setCallback(f);
        manager.run();
    }
    else
    {
        ROS_INFO("argc is: %d , needed >=3",argc);
    }
    return 0;
}
