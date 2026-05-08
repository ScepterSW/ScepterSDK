#ifndef SCEPTER_MANAGER_H
#define SCEPTER_MANAGER_H

#include <csignal>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#ifdef JAZZY
#include <cv_bridge/cv_bridge.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#elif defined(HUMBLE)
#include <cv_bridge/cv_bridge.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <image_transport/image_transport.hpp>
#include <std_msgs/msg/string.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <opencv2/opencv.hpp>
#include <atomic>
#include "Scepter_api.h"
#include "camera_settings.h"
using namespace std;
using namespace cv;

class ScepterManager : public rclcpp::Node{
public:
    explicit ScepterManager(rclcpp::NodeOptions node_options);
    ~ScepterManager();
    bool init_open_camera();
private:
    bool publicImage(const ScFrameReady& psReadFrame);
    void set_sensor_intrinsics();
    void timeout();
    void declare_parameters();
    bool config_camera_params();
    void init_topic_list();
    void init_TF_info();
    void publishTF();
    void publishStaticTF();
    void updateImageTopic(rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& image_topic, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr& image_info_topic, bool modify, int index, const char* type);
    void updateCloudPointTopic(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& point_cloud_topic, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr& point_clouds_info_topic, bool modify, const char* type);
private:
    static std::atomic<int> nodeCnt;
    bool hdrEnabled = false;
    bool wdrEnabled = false;
    std::string camera_name_;
    std::string camera_sn_;
    ScDeviceHandle deviceHandle_;
    ScSensorIntrinsicParameters depth_intrinsics_{}, color_intrinsics_{};
    ScSensorExtrinsicParameters extrinsics_{};
    sensor_msgs::msg::CameraInfo info_msg;
    CameraSetting cameraSetting_;
    std::string camera_frame, depth_frame,ir_frame, color_frame, aligneddepth_frame, alignedcolor_frame, points_frame,depth2colorpoints_frame;
    tf2::Quaternion quaternion_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    ScFrame frameArr[5];
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pubArr[5]; // depth ir color depth2color color2depth
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pubCameraInfoArr[5]; // depth ir color depth2color color2depth
    sensor_msgs::msg::CameraInfo cameraInfoArr[5]; // depth ir color depth2color color2depth
    sensor_msgs::msg::CameraInfo pointclound2_info_, depth2colorpointclound2info;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointclound2_pub_, depth2colorpointclound2_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pointclound2info_pub_, depth2colorpointclound2info_pub_;
    rclcpp::TimerBase::SharedPtr timer_, timer_tf_;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

};

#endif //SCEPTER_MANAGER_H
