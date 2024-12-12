#ifndef SCEPTER_MANAGER_H
#define SCEPTER_MANAGER_H


#include <csignal>
#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include <sensor_msgs/CameraInfo.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <dynamic_reconfigure/server.h>

#include <opencv2/opencv.hpp>
#include "Scepter_api.h"
#include "ScepterROS_MultiCameras/Sceptertof_roscppConfig.h"

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;
using namespace cv;


class ScepterManager {
public:
    explicit ScepterManager(const std::string &ip = "192.168.1.101",const std::string &topic_name = "Scepter");
    void run();
    void paramCallback(scepter_param::Sceptertof_roscppConfig& config,uint32_t level);
    scepter_param::Sceptertof_roscppConfig config_;
private:
    static void sigsegv_handler(int sig);
    void checkScStatus(ScStatus status, const std::string &message_on_fail);
    void set_sensor_intrinsics();
    bool fillImagePtr(const ros::Time& time, const ScFrameType type, ScFrame& frame, sensor_msgs::CameraInfoPtr& cameraInfoPtr, sensor_msgs::ImagePtr& imagePtr);
    void updateColorIntrinsicParameters();
    void publishCloudPoint(const ros::Time& time, const ScFrame& srcFrame, ros::Publisher& pub, sensor_msgs::CameraInfoPtr& cameraInfoPtr, ScFrame* frameArr = nullptr);
    std::string camera_ip_;
    std::string camera_name_;
    ros::NodeHandle color_nh_, depth_nh_, ir_nh_, alignedDepth_nh_, alignedColor_nh_, depthCloudPoint_nh_,depth2colorCloudPoint_nh_;;
    std::shared_ptr<image_transport::ImageTransport> color_it_, depth_it_, ir_it_, alignedDepth_it_, alignedColor_it_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> color_info_, depth_info_, ir_info_, alignedDepth_info_, alignedColor_info_, depth_point_cloud_info_,depth2color_point_cloud_info_;;
    sensor_msgs::CameraInfoPtr cameraInfo_Ary[7];

    image_transport::CameraPublisher color_pub_, depth_pub_, ir_pub_, alignedDepth_pub_, alignedColor_pub_;
    ros::Publisher depthCloudPointPub_, depth2colorCloudPointPub_;
    ros::Publisher depthCloudPointCameraInfoPub_, depth2colorCloudPointCameraPub_;
    int32_t device_index_;
    uint16_t slope_;
    int color_width_, color_height_;
    ScDeviceHandle deviceHandle_;
    unsigned int sessionIndex_;  
    ScSensorIntrinsicParameters depth_intrinsics_{}, color_intrinsics_{};
    ScSensorExtrinsicParameters extrinsics_{};
    bool hdrEnabled;
    bool wdrEnabled;
};


#endif //SCEPTER_MANAGER_H
