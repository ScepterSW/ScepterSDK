#ifndef SCEPTER_MANAGER_H
#define SCEPTER_MANAGER_H


#include <csignal>
#include <iostream>
// 包含头文件ros/ros.h,ROS提供的C++客户端库，在后面的编译配置中要添加相应的依赖库roscpp

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
// 使用std_msgs的内置消息或者自定义数据来传递图像数据
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

// opencv读取的图像数据传给sensor_msgs的图像消息
#ifdef JAZZY
  #include <cv_bridge/cv_bridge.hpp>
#else
  #include <cv_bridge/cv_bridge.h>
#endif

// 用来发布和订阅图像信息
#ifdef JAZZY
  #include <image_transport/image_transport.hpp>
#else
  #include <image_transport/image_transport.h>
#endif
#include "std_msgs/msg/string.hpp"
#include <rcl_interfaces/msg/parameter_event.hpp>
#include "pcl_conversions/pcl_conversions.h"

 
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
 
#include <opencv2/opencv.hpp>
#include "Scepter_api.h"
using namespace std;
using namespace cv;


class ScepterManager : public rclcpp::Node{
public:
    explicit ScepterManager(int32_t device_index,std::string topic_name);
    bool initDCAM(int32_t device_index = 0);
    bool shutDownDCAM();
private:
    static void sigsegv_handler(int sig);
    bool publicImage(const ScFrameReady& psReadFrame);
    void set_sensor_intrinsics();
    void timeout();
private:
    sensor_msgs::msg::CameraInfo color_info_, depth_info_, ir_info_, alignedDepth_info_, alignedColor_info_, pointclound2_info_ , depth2colorpointclound2info;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_, depth_pub_, ir_pub_, alignedDepth_pub_, alignedColor_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr colorinfo_pub_, depthinfo_pub_, irinfo_pub_, alignedDepthinfo_pub_, alignedColorinfo_pub_, pointclound2info_pub_, depth2colorpointclound2info_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointclound2_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr depth2colorpointclound2_pub_;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

private:
    std::string camera_name_;
    // SDK var
    int32_t device_index_;
    uint16_t slope_;
    int color_width_, color_height_;
    ScDeviceHandle deviceHandle_;
    unsigned int sessionIndex_;  
    ScSensorIntrinsicParameters depth_intrinsics_{}, color_intrinsics_{};
    ScSensorExtrinsicParameters extrinsics_{};
    bool depthCloudPointFlag_;
    bool depth2ColorCloudPointFlag_;
    sensor_msgs::msg::CameraInfo info_msg;
    ScFrame frameArr[5];
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pubArr[5];
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pubCameraInfoArr[5];
    sensor_msgs::msg::CameraInfo cameraInfoArr[5];
};


#endif //SCEPTER_MANAGER_H
