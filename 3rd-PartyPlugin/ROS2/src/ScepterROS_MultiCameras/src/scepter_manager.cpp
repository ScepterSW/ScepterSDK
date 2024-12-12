#include "scepter_manager.hpp"
#include <thread>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#ifdef JAZZY
  #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#else
  #include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#endif
#include <tf2_ros/static_transform_broadcaster.h>

using namespace std::chrono_literals;

ScepterManager::ScepterManager(std::string ip, std::string camera_name) :
        Node(camera_name),
        camera_ip_(ip),    
        camera_name_(camera_name),        
        color_width_(-1),
        color_height_(-1),
        slope_(1450),
        deviceHandle_(0),
        depthCloudPointFlag_(true),
        depth2ColorCloudPointFlag_(false)
{
    signal(SIGSEGV, ScepterManager::sigsegv_handler);
    RCLCPP_INFO(this->get_logger(), "camera_name: %s" , camera_name_.c_str());
    color_pub_ = this->create_publisher<sensor_msgs::msg::Image>(camera_name_+"/color/image_raw", 30);
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(camera_name_+"/depth/image_raw", 30);
    ir_pub_ = this->create_publisher<sensor_msgs::msg::Image>(camera_name_+"/ir/image_raw", 30);
    alignedDepth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(camera_name_+"/transformedDepth/image_raw", 30);
    alignedColor_pub_ = this->create_publisher<sensor_msgs::msg::Image>(camera_name_+"/transformedColor/image_raw", 30);
    
    colorinfo_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_name_+"/color/camera_info", 30);
    depthinfo_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_name_+"/depth/camera_info", 30);
    irinfo_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_name_+"/ir/camera_info", 30);
    alignedDepthinfo_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_name_+"/transformedDepth/camera_info", 30);
    alignedColorinfo_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_name_+"/transformedColor/camera_info", 30);
    pointclound2info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_name_+"/depth/points/camera_info", 30);
    depth2colorpointclound2info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_name_+"/depth2color/points/camera_info", 30);
   
    pointclound2_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(camera_name_+"/depth/points", 30);
    depth2colorpointclound2_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(camera_name_+"/depth2color/points", 30);
    memset(frameArr, 0,sizeof(ScFrame) * 5);
    pubArr[0] = depth_pub_; pubArr[1] = ir_pub_; pubArr[2] = color_pub_; pubArr[3] = alignedDepth_pub_; pubArr[4] = alignedColor_pub_; 
    pubCameraInfoArr[0] = depthinfo_pub_; pubCameraInfoArr[1] = irinfo_pub_; pubCameraInfoArr[2] = colorinfo_pub_; pubCameraInfoArr[3] = alignedDepthinfo_pub_; pubCameraInfoArr[4] = alignedColorinfo_pub_;
    this->declare_parameter<bool>("DepthCloudPointFlag", true);
    this->declare_parameter<bool>("Depth2ColorCloudPointFlag", false);
    if(initDCAM())
    {
        timer_ = this->create_wall_timer(
          100ms, std::bind(&ScepterManager::timeout, this));
    }
    parameter_callback_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params) {
        rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            ScStatus status = ScStatus::SC_OK;
            for (const auto &param : params) {
                if (param.get_name() == "DepthCloudPointFlag") {
                    RCLCPP_INFO(this->get_logger(), "%s DepthCloudPointFlag updated: %d", camera_name_.c_str(), param.as_bool());
                    depthCloudPointFlag_ = param.as_bool();
                }
                if (param.get_name() == "Depth2ColorCloudPointFlag") {
                    RCLCPP_INFO(this->get_logger(), "%s Depth2ColorCloudPointFlag updated: %d", camera_name_.c_str(), param.as_bool());
                    depth2ColorCloudPointFlag_ = param.as_bool();
                }
            }
            return result;
        }
    );
}
void ScepterManager::sigsegv_handler(int sig) 
{
    signal(SIGSEGV, SIG_DFL);
    cout<<"Segmentation fault, stopping camera driver : %" << sig <<endl;
    rclcpp::shutdown();
}
bool ScepterManager::initDCAM()
{ 
    ScStatus status = ScStatus::SC_OK;
    // Initialise the API
    status =scInitialize();
    if (status != ScStatus::SC_OK)
    {
        RCLCPP_INFO(this->get_logger(),"scInitialize failed! %d" ,status);
        return false;
    }
   // Get number of available devices
    uint32_t device_count = 0;
GET:
    int checkDeviceSec = 0;
    status = scGetDeviceCount(&device_count, 3000);
    if (status != ScStatus::SC_OK)
    {
        RCLCPP_INFO(this->get_logger(), "scGetDeviceCount failed! %d" ,status);
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Get device count: %d" ,device_count); 
    if (0 == device_count)
    {
        this_thread::sleep_for(chrono::seconds(1));
        goto GET;
    }
    ScDeviceInfo* pPsDeviceInfoList = new ScDeviceInfo[device_count];
    status =  scGetDeviceInfoList(device_count,pPsDeviceInfoList);
    if (status != ScStatus::SC_OK)
    {
        RCLCPP_INFO(this->get_logger(), "scGetDeviceInfoList failed! %d" ,status);
        return false;
    }
    ScDeviceInfo* pPsDeviceInfo = nullptr;
    for(int i=0;i<device_count;i++)
    {
        if(string(pPsDeviceInfoList[i].ip) == this->camera_ip_)
        {
            pPsDeviceInfo = &pPsDeviceInfoList[i];
            RCLCPP_INFO(this->get_logger(), "sn:%s,ip:%s", pPsDeviceInfo->serialNumber, pPsDeviceInfo->ip);
            break;
        }   
    }
    if(pPsDeviceInfo == nullptr)
    {
        RCLCPP_INFO(this->get_logger(), "pPsDeviceInfo nullptr");
        goto GET;	
    }
    
    status = scOpenDeviceBySN(pPsDeviceInfo->serialNumber, &deviceHandle_);
    if (status != ScStatus::SC_OK)
    {
        RCLCPP_INFO(this->get_logger(), "scOpenDeviceBySN failed! %d" ,status);
        return false;
    }

    char buffer[2048];
    getcwd(buffer, sizeof(buffer));
    string path(buffer);
    path = path + "/" + camera_name_ + "_parameter.json";
    status = scSetParamsByJson(deviceHandle_, const_cast<char*>(path.c_str()));
    if (status != ScStatus::SC_OK)
    {
        RCLCPP_INFO(this->get_logger(), "scSetParamsByJson ret %d Please create json file at %s if you need it", status, const_cast<char*>(path.c_str()));
    }
    else
    {
        RCLCPP_INFO(this->get_logger(),"Successfully load json %s", const_cast<char*>(path.c_str()));
    }

    scStartStream(deviceHandle_);
    
    /* add user define api call start*/
    // such as call the scSetSpatialFilterEnabled
   
    /*
    status= scSetSpatialFilterEnabled(deviceHandle_,true);
    RCLCPP_INFO( "SetSpatialFilterEnabled status: " << status);
    */
  
    /* add user define api call end*/
 
    set_sensor_intrinsics();
    cameraInfoArr[0] = depth_info_; cameraInfoArr[1] = ir_info_; cameraInfoArr[2] = color_info_; cameraInfoArr[3] = alignedDepth_info_; cameraInfoArr[4] = alignedColor_info_;
    const int BufLen = 64;
    char fw[BufLen] = { 0 };
    scGetFirmwareVersion(deviceHandle_, fw, BufLen);
    RCLCPP_INFO(this->get_logger(), "fw: %s " , fw);

    scSetTransformColorImgToDepthSensorEnabled(deviceHandle_, true);
    scSetTransformDepthImgToColorSensorEnabled(deviceHandle_, true);
    RCLCPP_INFO(this->get_logger(), "------ the camera is runing ok ------" );
    return true;
}
void ScepterManager::timeout() 
{    
    // Get next frame set
    ScFrameReady psReadFrame = {0};
    ScStatus status =  scGetFrameReady(deviceHandle_, 1200, &psReadFrame);
    if (status != SC_OK)
    {
        RCLCPP_INFO(this->get_logger(), "scGetFrameReady failed! %d" ,status);
        return;
    }
    publicImage(psReadFrame);
}

bool ScepterManager::shutDownDCAM()
{ 
    bool bret = true;
    ScStatus status = ScStatus::SC_OK;
     
    status = scStopStream(deviceHandle_);
    if (status != ScStatus::SC_OK)
    {
        RCLCPP_INFO(this->get_logger(), "scStopStream failed! %d" ,status);
        bret = false;
    }
    status = scCloseDevice(&deviceHandle_);
    if (status != ScStatus::SC_OK)
    {
        RCLCPP_INFO(this->get_logger(), "scCloseDevice failed! %d" ,status);
        bret = false;
    }
    status = scShutdown();
    if (status != ScStatus::SC_OK)
    {
        RCLCPP_INFO(this->get_logger(), "scShutdown failed! %d" ,status);
        bret = false;
    }
    return bret;
}

bool ScepterManager::publicImage(const ScFrameReady& psReadFrame)
{    
    std::string camera_frame(this->camera_name_ + "_frame"), 
                color_frame(this->camera_name_ + "_color_frame"),
                aligneddepth_frame(this->camera_name_ + "_transformedDepth_frame"),
                alignedcolor_frame(this->camera_name_ + "_transformedColor_frame"),
                depth_frame(this->camera_name_ + "_depth_frame"),
                ir_frame(this->camera_name_ + "_ir_frame"),
                points_frame(this->camera_name_ + "_points_frame"),
                depth2colorpoints_frame(this->camera_name_ + "_depth2colorpoints_frame");
    bool ret = false;
    memset(frameArr, 0,sizeof(ScFrame)*5);
    ScFrame frame = {0};
    if (psReadFrame.depth == 1)
    {
        scGetFrame(deviceHandle_, SC_DEPTH_FRAME, &frame);
        memcpy(&frameArr[0], &frame, sizeof(ScFrame));   
    }
    if (psReadFrame.ir == 1)
    {
        scGetFrame(deviceHandle_, SC_IR_FRAME, &frame);
        memcpy(&frameArr[1], &frame, sizeof(ScFrame));  
    }
    if (psReadFrame.color == 1)
    {
        scGetFrame(deviceHandle_, SC_COLOR_FRAME, &frame);
        memcpy(&frameArr[2], &frame, sizeof(ScFrame));  
    }
    if (psReadFrame.transformedDepth == 1)
    {
        scGetFrame(deviceHandle_, SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME, &frame);
        memcpy(&frameArr[3], &frame, sizeof(ScFrame));  
    }
    if (psReadFrame.transformedColor == 1)
    {
        scGetFrame(deviceHandle_, SC_TRANSFORM_COLOR_IMG_TO_DEPTH_SENSOR_FRAME, &frame);
        memcpy(&frameArr[4], &frame, sizeof(ScFrame));  
    }
    for (int i = 0; i < 5; i++)
    {
        frame = frameArr[i];
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &pub = pubArr[i];
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr &pubCameraInfo = pubCameraInfoArr[i];
        sensor_msgs::msg::CameraInfo& cameraInfo = cameraInfoArr[i];
        ScFrameType type = frame.frameType;
        if (frame.pFrameData != NULL)
        {
            int cvMatType = CV_16UC1;
            std::string imageEncodeType = sensor_msgs::image_encodings::TYPE_16UC1;
            switch (type)
            {
            case SC_IR_FRAME:
            {
                cvMatType = CV_8UC1;
                imageEncodeType = sensor_msgs::image_encodings::TYPE_8UC1;
                cv::Mat mat = cv::Mat(frame.height, frame.width, cvMatType, frame.pFrameData);
                cv_bridge::CvImage cvi_;
                cvi_.header.stamp = rclcpp::Clock().now();
                cvi_.header.frame_id = ir_frame;
                cvi_.encoding = "8UC1";
                cvi_.image = mat;
                sensor_msgs::msg::Image im_msg;
                cvi_.toImageMsg(im_msg);
                pub->publish(im_msg);
                pubCameraInfo->publish(cameraInfo);
                ret = true;
            }
                break;
            case SC_DEPTH_FRAME:
            {
                cvMatType = CV_16UC1;
                imageEncodeType = sensor_msgs::image_encodings::TYPE_16UC1;
                cv::Mat mat = cv::Mat(frame.height, frame.width, cvMatType, frame.pFrameData);
                cv_bridge::CvImage cvi_;
                cvi_.header.stamp = rclcpp::Clock().now();
                cvi_.header.frame_id = depth_frame;
                cvi_.encoding = "16UC1";
                cvi_.image = mat;
                sensor_msgs::msg::Image im_msg;     
                cvi_.toImageMsg(im_msg);
                pub->publish(im_msg);
                pubCameraInfo->publish(cameraInfo);
                if(depthCloudPointFlag_ == true) {
                    ScFrame &srcFrame = frame;
                    const int len = srcFrame.width * srcFrame.height;
                    ScVector3f* worldV = new ScVector3f[len];
                    scConvertDepthFrameToPointCloudVector(deviceHandle_, &srcFrame, worldV); //Convert Depth frame to World vectors.

                    sensor_msgs::msg::PointCloud2 output_msg;
                    pcl::PointCloud<pcl::PointXYZRGB> cloud;
                    cloud.points.resize(len);
                    for (int i = 0; i < len; i++)
                    { 
                        if (0 != worldV[i].z && worldV[i].z !=65535)
                        {
                            cloud.points[i].x = worldV[i].x/1000;
                            cloud.points[i].y = worldV[i].y/1000;
                            cloud.points[i].z = worldV[i].z/1000;
                            cloud.points[i].r = 255;
                            cloud.points[i].g = 255;
                            cloud.points[i].b = 255;
                        
                        }
                    }
                    delete [] worldV;
                    pcl::toROSMsg(cloud, output_msg);
                    output_msg.header.frame_id=points_frame;
                    output_msg.header.stamp = rclcpp::Clock().now();
                    pointclound2_pub_->publish(output_msg);
                    pointclound2info_pub_->publish(pointclound2_info_);
                }
                ret = true;
            }
            break;
            case SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME:
            {
                cvMatType = CV_16UC1;
                imageEncodeType = sensor_msgs::image_encodings::TYPE_16UC1;
                cv::Mat mat = cv::Mat(frame.height, frame.width, cvMatType, frame.pFrameData);
                cv_bridge::CvImage cvi_;
                cvi_.header.stamp = rclcpp::Clock().now();
                cvi_.header.frame_id = depth_frame;
                cvi_.encoding = "16UC1";
                cvi_.image = mat;
                sensor_msgs::msg::Image im_msg;     
                cvi_.toImageMsg(im_msg);
                pub->publish(im_msg);
                pubCameraInfo->publish(cameraInfo);
                if(depth2ColorCloudPointFlag_ == true)
                {
                    const int len = frame.width * frame.height;
                    ScVector3f* worldV = new ScVector3f[len];
                    scConvertDepthFrameToPointCloudVector(deviceHandle_, &frame, worldV);
                    sensor_msgs::msg::PointCloud2 output_msg;
                    pcl::PointCloud<pcl::PointXYZRGB> cloud;
                    cloud.points.resize(len);
                    cvMatType = CV_8UC3;
                    imageEncodeType = sensor_msgs::image_encodings::BGR8;
                    cv::Mat mat = cv::Mat(frameArr[2].height, frameArr[2].width, cvMatType, frameArr[2].pFrameData);
                    for (int i = 0; i < len; i++)
                    { 
                        int row = (i / mat.cols);
                        int col = (i % mat.cols);
                        uchar* data = mat.ptr<uchar>(row); // bgr == 1 col
                        if (0 != worldV[i].z && worldV[i].z !=65535)
                        {
                            cloud.points[i].x = worldV[i].x/1000;
                            cloud.points[i].y = worldV[i].y/1000;
                            cloud.points[i].z = worldV[i].z/1000;
                            cloud.points[i].r = data[3*col + 2];
                            cloud.points[i].g = data[3*col + 1];
                            cloud.points[i].b = data[3*col];
                        
                        }
                    }
                    delete [] worldV;
                    pcl::toROSMsg(cloud, output_msg);
                    output_msg.header.frame_id=depth2colorpoints_frame;
                    output_msg.header.stamp = rclcpp::Clock().now();
                    depth2colorpointclound2_pub_->publish(output_msg);
                    depth2colorpointclound2info_pub_->publish(depth2colorpointclound2info);
                }
                ret = true;
            }
                break;
            case SC_COLOR_FRAME:
            case SC_TRANSFORM_COLOR_IMG_TO_DEPTH_SENSOR_FRAME:
            {
                cvMatType = CV_8UC3;
                imageEncodeType = sensor_msgs::image_encodings::BGR8;
                cv::Mat mat = cv::Mat(frame.height, frame.width, cvMatType, frame.pFrameData);
                cv_bridge::CvImage cvi_;
                cvi_.header.stamp = rclcpp::Clock().now();
                cvi_.header.frame_id = color_frame;
                cvi_.encoding = "bgr8";
                cvi_.image = mat;
                sensor_msgs::msg::Image im_msg;
                cvi_.toImageMsg(im_msg);
                pub->publish(im_msg);
                pubCameraInfo->publish(cameraInfo);
                ret = true;
            }    
                break;
            default:
                ret = false;
                break;
            }

        }
    }

    return ret;
}

void ScepterManager::set_sensor_intrinsics() 
{
    std::string camera_frame(this->camera_name_ + "_frame"), 
                color_frame(this->camera_name_ + "_color_frame"),
                aligneddepth_frame(this->camera_name_ + "_transformedDepth_frame"),
                alignedcolor_frame(this->camera_name_ + "_transformedColor_frame"),
                depth_frame(this->camera_name_ + "_depth_frame"),
                ir_frame(this->camera_name_ + "_ir_frame"),
                points_frame(this->camera_name_ + "_points_frame"),
                depth2colorpoints_frame(this->camera_name_ + "_depth2colorpoints_frame");

    // Get camera parameters (extrinsic)
    scGetSensorExtrinsicParameters(deviceHandle_, &this->extrinsics_);

    // Setup tf broadcaster
    static tf2_ros::StaticTransformBroadcaster tf_broadcaster(this);
 

    // PsCameraExtrinsicParameters to ROS transform
 
    tf2::Matrix3x3 rotation_matrix(extrinsics_.rotation[0], extrinsics_.rotation[1], extrinsics_.rotation[2],
                                  extrinsics_.rotation[3], extrinsics_.rotation[4], extrinsics_.rotation[5],
                                  extrinsics_.rotation[6], extrinsics_.rotation[7], extrinsics_.rotation[8]);
    double roll, pitch, yaw;
    rotation_matrix.getRPY(roll, pitch, yaw);
    tf2::Quaternion quaternion;
    quaternion.setRPY(roll, pitch, yaw);
    // Publish static TFs
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = rclcpp::Clock().now();
    msg.transform.rotation.w = 1.0;

    msg.header.frame_id = camera_frame;
    msg.child_frame_id = points_frame;
    tf_broadcaster.sendTransform(msg);

    msg.header.frame_id = camera_frame;
    msg.child_frame_id = depth2colorpoints_frame;
    tf_broadcaster.sendTransform(msg);

    msg.header.frame_id = camera_frame;
    msg.child_frame_id = ir_frame;
    tf_broadcaster.sendTransform(msg);

    // Camera base to Color Frame
    msg.header.frame_id = camera_frame;
    msg.child_frame_id = color_frame;
    tf_broadcaster.sendTransform(msg);

    // Color Frame to Aligned Frame
    msg.header.frame_id = color_frame;
    msg.child_frame_id = depth_frame;
    tf_broadcaster.sendTransform(msg);
    
    msg.header.frame_id = depth_frame;
    msg.child_frame_id = alignedcolor_frame;
    tf_broadcaster.sendTransform(msg);

    // Color Frame to Depth Frame
    msg.transform.translation.x = extrinsics_.translation[0] / 1000;
    msg.transform.translation.y = extrinsics_.translation[1] / 1000;
    msg.transform.translation.z = extrinsics_.translation[2] / 1000;
    msg.transform.rotation = tf2::toMsg(quaternion);
    msg.header.frame_id = color_frame;
    msg.child_frame_id = aligneddepth_frame;
    tf_broadcaster.sendTransform(msg);

    // Get camera parameters (intrinsic)
    scGetSensorIntrinsicParameters(deviceHandle_, SC_TOF_SENSOR, &this->depth_intrinsics_);
    scGetSensorIntrinsicParameters(deviceHandle_, SC_COLOR_SENSOR, &this->color_intrinsics_);

    // Initialise camera info messages
    info_msg.distortion_model = "plumb_bob";
    info_msg.header.frame_id = color_frame;
    info_msg.d = {color_intrinsics_.k1, color_intrinsics_.k2, color_intrinsics_.p1, color_intrinsics_.p2,
                  color_intrinsics_.k3};
    info_msg.k = {color_intrinsics_.fx, 0, color_intrinsics_.cx, 
                    0, color_intrinsics_.fy, color_intrinsics_.cy,
                    0, 0, 1};
    info_msg.p = {color_intrinsics_.fx, 0, color_intrinsics_.cx,
                    0, 0, color_intrinsics_.fy,
                    color_intrinsics_.cy, 0, 0, 
                    0, 1, 0};
    info_msg.r.fill(0);
    info_msg.r[0] = 1;
    info_msg.r[4] = 1;
    info_msg.r[8] = 1;
    color_info_=info_msg;
    alignedDepth_info_=info_msg;
    colorinfo_pub_->publish(color_info_);
    alignedDepthinfo_pub_->publish(alignedDepth_info_);

    info_msg.header.frame_id = depth2colorpoints_frame;
    depth2colorpointclound2info = info_msg;
    depth2colorpointclound2info_pub_->publish(depth2colorpointclound2info);

    info_msg.header.frame_id = depth_frame;
    info_msg.d = {depth_intrinsics_.k1, depth_intrinsics_.k2, depth_intrinsics_.p1, depth_intrinsics_.p2,
                  depth_intrinsics_.k3};
    info_msg.k = {depth_intrinsics_.fx, 0, depth_intrinsics_.cx,
                    0, depth_intrinsics_.fy, depth_intrinsics_.cy,
                    0, 0, 1};
    info_msg.p = {depth_intrinsics_.fx, 0, depth_intrinsics_.cx,
                    0, 0, depth_intrinsics_.fy,
                    depth_intrinsics_.cy, 0, 0,
                    0, 1, 0};
    
    depth_info_=info_msg;
    alignedColor_info_=info_msg;

    info_msg.header.frame_id = ir_frame;
    ir_info_=info_msg;
    depthinfo_pub_->publish(depth_info_);
    irinfo_pub_->publish(ir_info_);
    alignedColorinfo_pub_->publish(alignedColor_info_);
    
    info_msg.header.frame_id = points_frame;
    pointclound2_info_ = info_msg;
    pointclound2info_pub_->publish(pointclound2_info_);
}
