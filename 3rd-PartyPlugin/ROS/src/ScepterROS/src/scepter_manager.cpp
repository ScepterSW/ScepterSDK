#include "scepter_manager.hpp"

//define call back function
void ScepterManager::paramCallback(scepter_param::Sceptertof_roscppConfig& config,uint32_t level)
{
    ROS_INFO("Request: %d %d %d %d %d %d %d %d %d %d %d %d",
                config.FrameRate,
                config.IRGMMGain,
                config.ColorResloution,
                config.ToFManual,
                config.ToFExposureTime,
                config.ColorManual,
                config.ColorExposureTime,
                config.WorkMode,
                config.SoftwareTrigger,
                config.XDRMode,
                config.DepthCloudPoint,
                config.Depth2ColorCloudPoint);

    if(config_.XDRMode != config.XDRMode)
    {
        config_.XDRMode = config.XDRMode;
        ScStatus status = ScStatus::SC_OK;
        switch (config_.XDRMode)
        {
        case 0:
            hdrEnabled = false;
            wdrEnabled = false;
            status = scSetHDRModeEnabled(deviceHandle_, hdrEnabled);
            ROS_INFO_STREAM( "scSetHDRModeEnabled status: " << status << " hdrEnabled " << hdrEnabled);
            status = scSetWDRModeEnabled(deviceHandle_, wdrEnabled);
            ROS_INFO_STREAM( "scSetWDRModeEnabled status: " << status << " wdrEnabled " << wdrEnabled);
            break;
        case 1:
            hdrEnabled = true;
            wdrEnabled = false;
            status = scSetWDRModeEnabled(deviceHandle_, wdrEnabled);
            ROS_INFO_STREAM( "scSetWDRModeEnabled status: " << status << " wdrEnabled " << wdrEnabled);
            status = scSetHDRModeEnabled(deviceHandle_, hdrEnabled);
            ROS_INFO_STREAM( "scSetHDRModeEnabled status: " << status << " hdrEnabled " << hdrEnabled);
            break;
        case 2:
            hdrEnabled = false;
            wdrEnabled = true;
            status = scSetHDRModeEnabled(deviceHandle_, hdrEnabled);
            ROS_INFO_STREAM( "scSetHDRModeEnabled status: " << status << " hdrEnabled " << hdrEnabled);
            status = scSetWDRModeEnabled(deviceHandle_, wdrEnabled);
            ROS_INFO_STREAM( "scSetWDRModeEnabled status: " << status << " wdrEnabled " << wdrEnabled);
            break;
        }
    }
    if(config_.IRGMMGain != config.IRGMMGain)
    {
        config_.IRGMMGain = config.IRGMMGain;
        ScStatus status= scSetIRGMMGain(deviceHandle_,config_.IRGMMGain);
        ROS_INFO_STREAM( "SetIRGMMGain status: " << status);
    }
    if(config_.FrameRate != config.FrameRate)
    {
        config_.FrameRate = config.FrameRate;
        ScStatus status= scSetFrameRate(deviceHandle_,config_.FrameRate);
        ROS_INFO_STREAM( "SetFrameRate status: " << status);
        if (status != SC_OK)
        {
            if (hdrEnabled)
            {
                ROS_INFO_STREAM( "FrameRate number invaild with HDR enabled");
            }
            if (wdrEnabled)
            {
                ROS_INFO_STREAM( "FrameRate number invaild with WDR enabled");
            }
        }
    }
    if(config_.ColorResloution != config.ColorResloution)
    {
        config_.ColorResloution = config.ColorResloution;
        int w = 640;
        int h = 480;
        switch (config_.ColorResloution)
        {
        case 0:
            w = 1600;
            h = 1200;
            break;
        case 1:
            w = 800;
            h = 600;
            break;
        case 2:
            w = 640;
            h = 480;
            break;
        }
        ScStatus status= scSetColorResolution(deviceHandle_,w,h);
        ROS_INFO_STREAM( "SetColorResolution status: " << status);
        if(status == SC_OK)
        {
            ROS_INFO_STREAM( "updateColorIntrinsicParameters as color resloution changed");
            updateColorIntrinsicParameters();
        }
    }
    
    if(config_.ToFManual != config.ToFManual)
    {
        config_.ToFManual = config.ToFManual;
        ScStatus status= scSetExposureControlMode(deviceHandle_,SC_TOF_SENSOR,(ScExposureControlMode)config_.ToFManual);
        ROS_INFO_STREAM( "SetExposureControlMode tof status: " << status);
        if (status != SC_OK)
        {
            if (hdrEnabled)
            {
                ROS_INFO_STREAM( "Can not set Tof ExposureControlMode with HDR enabled");
            }
            if (wdrEnabled)
            {
                ROS_INFO_STREAM( "Can not set Tof ExposureControlMode with WDR enabled");
            }
        }
        if(config_.ToFManual != 1)
        {
          config_.ToFExposureTime = 0;  
        }
    }
    if(config_.ToFExposureTime != config.ToFExposureTime&&config_.ToFManual==1)
    {
        config_.ToFExposureTime = config.ToFExposureTime;
        int exposureTime = 0;
		int retry = 0;
		while (0 == exposureTime && retry < 3)
        {
			scGetMaxExposureTime(deviceHandle_, SC_TOF_SENSOR, &exposureTime);
            retry++;
        }
        if (0 != exposureTime && config_.ToFExposureTime <= exposureTime)
        {
			exposureTime = config_.ToFExposureTime;
		}
        else
        {
            ROS_INFO_STREAM( "SetExposureTime tof Max Value: " << exposureTime);
        }
        ScStatus status= scSetExposureTime(deviceHandle_,SC_TOF_SENSOR,exposureTime);
        ROS_INFO_STREAM( "SetExposureTime tof status: " << status);
        if (status != SC_OK)
        {
            if (hdrEnabled)
            {
                ROS_INFO_STREAM( "Can not set Tof ExposureTime with HDR enabled");
            }
            if (wdrEnabled)
            {
                ROS_INFO_STREAM( "Can not set Tof ExposureTime with WDR enabled");
            }
        }
    }
    if(config_.ColorManual != config.ColorManual)
    {
        config_.ColorManual = config.ColorManual;
        ScStatus status= scSetExposureControlMode(deviceHandle_,SC_COLOR_SENSOR,(ScExposureControlMode)config_.ColorManual);
        ROS_INFO_STREAM( "SetExposureControlMode color status: " << status);
        if(config_.ColorManual != 1)
        {
          config_.ColorExposureTime = 0;  
        }
    }
    if(config_.ColorExposureTime != config.ColorExposureTime&&config_.ColorManual==1)
    {
		config_.ColorExposureTime = config.ColorExposureTime;
        int exposureTime = 0;
		int retry = 0;
		while (0 == exposureTime && retry < 3)
        {
			scGetMaxExposureTime(deviceHandle_, SC_COLOR_SENSOR, &exposureTime);
            retry++;
        }
        if (0 != exposureTime && config_.ColorExposureTime <= exposureTime)
        {
			exposureTime = config_.ColorExposureTime;
		}
        else
        {
            ROS_INFO_STREAM( "SetExposureTime color Max Value: " << exposureTime);
        }
        ScStatus status= scSetExposureTime(deviceHandle_,SC_COLOR_SENSOR,exposureTime);
        ROS_INFO_STREAM( "SetExposureTime color status: " << status);
    }
    if(config_.WorkMode != config.WorkMode)
    {
        config_.WorkMode = config.WorkMode;
        ScStatus status= scSetWorkMode(deviceHandle_,(ScWorkMode)config_.WorkMode);
        ROS_INFO_STREAM( "SetWorkMode status: " << status);
    }

    config_.SoftwareTrigger = config.SoftwareTrigger;

    if(config_.DepthCloudPoint != config.DepthCloudPoint)
    {   
        config_.DepthCloudPoint = config.DepthCloudPoint;
    }
    if(config_.Depth2ColorCloudPoint != config.Depth2ColorCloudPoint)
    {   
        config_.Depth2ColorCloudPoint = config.Depth2ColorCloudPoint;
    }
}

ScepterManager::ScepterManager(int32_t device_index, const string &camera_name) :
        color_nh_(camera_name + "/color"),
        depth_nh_(camera_name + "/depth"),
        ir_nh_(camera_name + "/ir"),
        alignedDepth_nh_(camera_name + "/transformedDepth"),        
        alignedColor_nh_(camera_name + "/transformedColor"),
        depthCloudPoint_nh_(camera_name + "/depthCloudPoint"),
        depth2colorCloudPoint_nh_(camera_name + "/depth2colorCloudPoint"),
        camera_name_(camera_name),
        color_info_(new camera_info_manager::CameraInfoManager(color_nh_)),
        depth_info_(new camera_info_manager::CameraInfoManager(depth_nh_)),
        ir_info_(new camera_info_manager::CameraInfoManager(ir_nh_)),
        alignedDepth_info_(new camera_info_manager::CameraInfoManager(alignedDepth_nh_)),
        alignedColor_info_(new camera_info_manager::CameraInfoManager(alignedColor_nh_)),
        depth_point_cloud_info_(new camera_info_manager::CameraInfoManager(depthCloudPoint_nh_)),
        depth2color_point_cloud_info_(new camera_info_manager::CameraInfoManager(depth2colorCloudPoint_nh_)),
        color_it_(new image_transport::ImageTransport(color_nh_)),
        depth_it_(new image_transport::ImageTransport(depth_nh_)),
        ir_it_(new image_transport::ImageTransport(ir_nh_)),
        alignedDepth_it_(new image_transport::ImageTransport(alignedDepth_nh_)),
        alignedColor_it_(new image_transport::ImageTransport(alignedColor_nh_)),
        color_width_(-1),
        color_height_(-1),
        slope_(1450),
        deviceHandle_(0),
        sessionIndex_(0),
        hdrEnabled(false),
        wdrEnabled(false)
{
    signal(SIGSEGV, ScepterManager::sigsegv_handler);

    // Initialise the API
    checkScStatus(scInitialize(), "Initialize failed!");

    // Get number of available devices
    uint32_t device_count = 0;
GET:
    int checkDeviceSec = 0;
	ScStatus status = scGetDeviceCount(&device_count, 3000);
	if (status != ScStatus::SC_OK || device_count < 1)
	{
        ROS_INFO("check device cost:%d second.", checkDeviceSec++);
        ros::Duration(1).sleep();
		goto GET;	
	}
    ROS_INFO("Get device count: %d", device_count);

    // Verify device index selection
    this->device_index_ = device_index;
    if (this->device_index_ < 0
        || this->device_index_ >= device_count)
        {
        throw std::runtime_error(
                "Device index outside of available devices range 0-" + std::to_string(device_count));
        }
       
	ScDeviceInfo* pPsDeviceInfoList = new ScDeviceInfo[device_count];
	status = scGetDeviceInfoList(device_count, pPsDeviceInfoList);
	if (status != ScStatus::SC_OK)
	{
		ROS_INFO("scGetDeviceInfoList failed! %d", status);
		delete[] pPsDeviceInfoList;
		pPsDeviceInfoList = NULL;
		goto GET;
	}

	ScDeviceInfo* pPsDeviceInfo = &pPsDeviceInfoList[0];

    // Attempt to open the device
    checkScStatus(scOpenDeviceBySN(pPsDeviceInfo->serialNumber, &deviceHandle_), "OpenDevice failed!");

    ROS_INFO("Successfully connected to device %d", this->device_index_);

    status= scStartStream(deviceHandle_);
    ROS_INFO_STREAM( "Start Depth Frame status: " << status);
    /* add user define api call start*/
    // such as call the scSetSpatialFilterEnabled
   
    /*
    status= scSetSpatialFilterEnabled(deviceHandle_,true);
    ROS_INFO_STREAM( "SetSpatialFilterEnabled status: " << status);
    */

    // such as call the scSetParamsByJson     
   
    /*
    char buffer[2048];
    getcwd(buffer, sizeof(buffer));
    string path(buffer);
    path = path + "/parameter.json";
    status = scSetParamsByJson(deviceHandle_, const_cast<char*>(path.c_str()));
    if (status != ScStatus::SC_OK)
    {
        ROS_INFO("scSetParamsByJson ret %d Please create json file at %s if you need it", status, const_cast<char*>(path.c_str()));
    }
    else
    {
        ROS_INFO("Successfully load json %s", const_cast<char*>(path.c_str()));
    }
    */
   
    /* add user define api call end*/

    int rate=0;
    status= scGetFrameRate(deviceHandle_,&rate);
    config_.FrameRate=rate;
    ROS_INFO_STREAM( "GetFrameRate status: " << status);
    uint8_t gmmgain=0;
    status= scGetIRGMMGain(deviceHandle_,&gmmgain);
    config_.IRGMMGain = gmmgain;
    ROS_INFO_STREAM( "GetIRGMMGain status: " << status);
    
    ScWorkMode mode;
    status= scGetWorkMode(deviceHandle_,&mode);
    config_.WorkMode= mode;
    ROS_INFO_STREAM( "GetWorkMode status: " << status);
    
    {
        ScExposureControlMode pControlMode;
        status= scGetExposureControlMode(deviceHandle_,SC_TOF_SENSOR,&pControlMode);
        config_.ToFManual = pControlMode;
        ROS_INFO_STREAM( "GetExposureControlMode tof status: " << status);

        if(config_.ToFManual == SC_EXPOSURE_CONTROL_MODE_MANUAL)
        {
            int nExposureTime = 0;
            status= scGetExposureTime(deviceHandle_,SC_TOF_SENSOR,&nExposureTime);
            config_.ToFExposureTime = nExposureTime;
            ROS_INFO_STREAM( "GetExposureTime ToF status: " << status);
        }
    }
    {
        ScExposureControlMode pControlMode;
        status= scGetExposureControlMode(deviceHandle_,SC_COLOR_SENSOR,&pControlMode);
        config_.ColorManual = pControlMode;
        ROS_INFO_STREAM( "GetExposureControlMode color status: " << status);
        if(pControlMode == SC_EXPOSURE_CONTROL_MODE_MANUAL)
        {
            int nExposureTime = 0;
            status= scGetExposureTime(deviceHandle_,SC_COLOR_SENSOR,&nExposureTime);
            config_.ColorExposureTime= nExposureTime;
            ROS_INFO_STREAM( "GetExposureTime color status: " << status);
        }
    }

    bool hdrEnable = false;
    status = scGetHDRModeEnabled(deviceHandle_, &hdrEnable);
    ROS_INFO_STREAM( "hdrEnable status: " << status << " hdrEnable " << hdrEnable);
    bool wdrEnable = false;
    status = scGetWDRModeEnabled(deviceHandle_, &wdrEnable);
    ROS_INFO_STREAM( "wdrEnable status: " << status << " wdrEnable " << wdrEnable);
    if (hdrEnable == false && wdrEnable == false)
    {
        config_.XDRMode = 0;
    }
    else if (hdrEnable == true && wdrEnable == false)
    {
        config_.XDRMode = 1;
    }
    else if (hdrEnable == false && wdrEnable == true)
    {
        config_.XDRMode = 2;
    }
    else
    {
        config_.XDRMode = -1;
        ROS_ERROR("XDRMode init error");
    }
    config_.DepthCloudPoint = false;
    config_.Depth2ColorCloudPoint = false;
    ROS_INFO("ctl: %d %d %d %d %d %d %d %d %d %d %d",
                config_.FrameRate,
                config_.IRGMMGain,
                config_.ColorResloution,
                config_.ToFManual,
                config_.ToFExposureTime,
                config_.ColorManual,
                config_.ColorExposureTime,
                config_.WorkMode,
                config_.XDRMode,
                config_.DepthCloudPoint,
                config_.Depth2ColorCloudPoint);
}
 
void ScepterManager::run() 
{
    // Initialise ROS nodes
    set_sensor_intrinsics();

    cameraInfo_Ary[0] = boost::make_shared<sensor_msgs::CameraInfo>(depth_info_->getCameraInfo());
    cameraInfo_Ary[1] = boost::make_shared<sensor_msgs::CameraInfo>(ir_info_->getCameraInfo());
    cameraInfo_Ary[2] = boost::make_shared<sensor_msgs::CameraInfo>(color_info_->getCameraInfo());
    cameraInfo_Ary[3] = boost::make_shared<sensor_msgs::CameraInfo>(alignedDepth_info_->getCameraInfo());
    cameraInfo_Ary[4] = boost::make_shared<sensor_msgs::CameraInfo>(alignedColor_info_->getCameraInfo());
    cameraInfo_Ary[5] = boost::make_shared<sensor_msgs::CameraInfo>(depth_point_cloud_info_->getCameraInfo());
    cameraInfo_Ary[6] = boost::make_shared<sensor_msgs::CameraInfo>(depth2color_point_cloud_info_->getCameraInfo());

    // CameraPublisher 	advertiseCamera (const std::string &base_topic, uint32_t queue_size, bool latch=false)
    this->color_pub_ = this->color_it_->advertiseCamera("image_raw", 30);
    this->depth_pub_ = this->depth_it_->advertiseCamera("image_raw", 30);
    this->ir_pub_ = this->ir_it_->advertiseCamera("image_raw", 30);
    this->alignedDepth_pub_ = this->alignedDepth_it_->advertiseCamera("image_raw", 30);
    this->alignedColor_pub_ = this->alignedColor_it_->advertiseCamera("image_raw", 30);
    this->depthCloudPointPub_ = depthCloudPoint_nh_.advertise<sensor_msgs::PointCloud2>("cloud_points",30);
    this->depth2colorCloudPointPub_ = depth2colorCloudPoint_nh_.advertise<sensor_msgs::PointCloud2>("cloud_points",30);
    this->depthCloudPointCameraInfoPub_ = depthCloudPoint_nh_.advertise<sensor_msgs::CameraInfo>("camera_info",30);
    this->depth2colorCloudPointCameraPub_ = depth2colorCloudPoint_nh_.advertise<sensor_msgs::CameraInfo>("camera_info",30);
    
    // Containers for frames
    ScStatus status;
    ros::Time now = ros::Time::now();
    int missed_frames = 0;

    ScFrame frameArr[5];
    memset(frameArr, 0,sizeof(ScFrame) * 5);

    sensor_msgs::ImagePtr msg_Ary[5]; // depth_msg,ir_msg,color_msg,alignedDetph_msg,alignedColor_msg
    image_transport::CameraPublisher pub_Ary[5] = {this->depth_pub_,this->ir_pub_,this->color_pub_,this->alignedDepth_pub_,this->alignedColor_pub_};
    while (ros::ok()) {
        ros::spinOnce();
        // Get next frame set
        if(config_.WorkMode == SC_SOFTWARE_TRIGGER_MODE && config_.SoftwareTrigger ==1)
        {
            scSoftwareTriggerOnce(deviceHandle_);
        }
        ScFrameReady psReadFrame = {0};
        ScStatus status =  scGetFrameReady(deviceHandle_, 1200, &psReadFrame);
        if (status != SC_OK)
        {
            continue;
        }

        now = ros::Time::now();
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
        bool ret = false;
        for(int ind = 0; ind  < 5; ind++)
        {
            frame = frameArr[ind];
            ScFrameType type = frame.frameType;
            if (frame.pFrameData != NULL)
            {
                ret = fillImagePtr(now, type, frame, cameraInfo_Ary[ind], msg_Ary[ind]);
                if(ret)
                {
                    pub_Ary[ind].publish(msg_Ary[ind],cameraInfo_Ary[ind]);
                    if (config_.DepthCloudPoint == true && type == SC_DEPTH_FRAME)
                    {
                        publishCloudPoint(now, frame, depthCloudPointPub_, cameraInfo_Ary[5], nullptr);
                    }
                    if (config_.Depth2ColorCloudPoint == true && type == SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME)
                    {
                        publishCloudPoint(now, frame, depth2colorCloudPointPub_, cameraInfo_Ary[6], frameArr);
                    }
                }
                else
                {
                    ROS_INFO_STREAM( "fill image failed for type : " << type );
                }
            }
        }
    }

    status = scStopStream(deviceHandle_);
    ROS_INFO_STREAM( "Stop Depth Frame status: " << status);
    status = scCloseDevice(&deviceHandle_);
    ROS_INFO_STREAM( "CloseDevice status: " << status);
    status = scShutdown();
    ROS_INFO_STREAM( "Shutdown status: " << status );
}

void ScepterManager::publishCloudPoint(const ros::Time& time, const ScFrame& srcFrame, ros::Publisher& pub, sensor_msgs::CameraInfoPtr& cameraInfoPtr, ScFrame* frameArr)
{
    ScFrameType type = srcFrame.frameType;
    if (!(type == SC_DEPTH_FRAME || type == SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME))
    {
        ROS_INFO_STREAM( "invaid frame type : " << type );
        return;
    }
    const int len = srcFrame.width * srcFrame.height;
    ScVector3f* worldV = new ScVector3f[len];
    scConvertDepthFrameToPointCloudVector(deviceHandle_, &srcFrame, worldV); //Convert Depth frame to World vectors.

    sensor_msgs::PointCloud2 output_msg;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    cloud.points.resize(len);
    if (type == SC_DEPTH_FRAME)
    {
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
    }
    else if (type == SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME)
    {
        int cvMatType = CV_8UC3;
        std::string imageEncodeType = sensor_msgs::image_encodings::BGR8;
        cv::Mat mat = cv::Mat((*(frameArr + 2)).height, (*(frameArr + 2)).width, cvMatType, (*(frameArr + 2)).pFrameData);
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
    }
    delete [] worldV;
    pcl::toROSMsg(cloud, output_msg);
    output_msg.header.frame_id = this->camera_name_ + "_points_frame";
    if (type == SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME)
    {
        output_msg.header.frame_id = this->camera_name_ + "_depth2colorpoints_frame";
    }
    output_msg.header.stamp = time;
    cameraInfoPtr->height = srcFrame.height;
    cameraInfoPtr->width = srcFrame.width;
    cameraInfoPtr->header.stamp = time;
    pub.publish(output_msg);
    if (type == SC_DEPTH_FRAME)
    {
        this->depthCloudPointCameraInfoPub_.publish(cameraInfoPtr);
    }
    else if (type == SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME)
    {
        this->depth2colorCloudPointCameraPub_.publish(cameraInfoPtr);
    }
    //pub.publish(output_msg, cameraInfoPtr);
}

bool ScepterManager::fillImagePtr(const ros::Time& time, const ScFrameType type, ScFrame& frame,sensor_msgs::CameraInfoPtr& cameraInfoPtr, sensor_msgs::ImagePtr& imagePtr)
{
    bool ret = false;

    if (frame.pFrameData != NULL)
    {
        int cvMatType = CV_16UC1;
        std::string imageEncodeType = sensor_msgs::image_encodings::TYPE_16UC1;
        switch (type)
        {
        case SC_IR_FRAME:
            cvMatType = CV_8UC1;
            imageEncodeType = sensor_msgs::image_encodings::TYPE_8UC1;
            break;
        case SC_DEPTH_FRAME:
        case SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME:
            cvMatType = CV_16UC1;
            imageEncodeType = sensor_msgs::image_encodings::TYPE_16UC1;
            break;
        case SC_COLOR_FRAME:
 	case SC_TRANSFORM_COLOR_IMG_TO_DEPTH_SENSOR_FRAME:
            cvMatType = CV_8UC3;
            imageEncodeType = sensor_msgs::image_encodings::BGR8;
            break;
        default:
            return ret;
        }

        cv::Mat mat = cv::Mat(frame.height, frame.width, cvMatType, frame.pFrameData);
        cameraInfoPtr->height = frame.height;
        cameraInfoPtr->width = frame.width;
        cameraInfoPtr->header.stamp = time;
        imagePtr = cv_bridge::CvImage(cameraInfoPtr->header, imageEncodeType, mat).toImageMsg();
        ret = true;
    }

    return ret;
}

void ScepterManager::sigsegv_handler(int sig)
{
    signal(SIGSEGV, SIG_DFL);
    ROS_ERROR("Segmentation fault, stopping camera driver (%d).", sig);    
    ros::shutdown();
}

void ScepterManager::checkScStatus(ScStatus status, const std::string &message_on_fail)
{
    if (status == ScStatus::SC_OK)
        return;
    ROS_ERROR(message_on_fail.c_str()); 
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
    checkScStatus(scGetSensorExtrinsicParameters(deviceHandle_, &this->extrinsics_),
                      "Could not get extrinsics!");

    // Setup tf broadcaster
    static tf2_ros::StaticTransformBroadcaster tf_broadcaster;
    ros::Time now = ros::Time::now();

    // PsCameraExtrinsicParameters to ROS transform
    tf::Transform transform;
    tf::Matrix3x3 rotation_matrix(extrinsics_.rotation[0], extrinsics_.rotation[1], extrinsics_.rotation[2],
                                  extrinsics_.rotation[3], extrinsics_.rotation[4], extrinsics_.rotation[5],
                                  extrinsics_.rotation[6], extrinsics_.rotation[7], extrinsics_.rotation[8]);
    double roll, pitch, yaw;
    rotation_matrix.getRPY(roll, pitch, yaw);
    geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

    // Publish static TFs
    geometry_msgs::TransformStamped msg;
    msg.header.stamp = now;
    msg.transform.rotation.w = 1.0;

    // Camera base to Color Frame
    msg.header.frame_id = camera_frame;
    msg.child_frame_id = color_frame;
    tf_broadcaster.sendTransform(msg);

    msg.header.frame_id = camera_frame;
    msg.child_frame_id = points_frame;
    tf_broadcaster.sendTransform(msg);

    msg.header.frame_id = camera_frame;
    msg.child_frame_id = depth2colorpoints_frame;
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
    msg.transform.rotation = orientation;
    msg.header.frame_id = color_frame;
    msg.child_frame_id = aligneddepth_frame;
    tf_broadcaster.sendTransform(msg);

    // Get camera parameters (intrinsic)
    checkScStatus(scGetSensorIntrinsicParameters(deviceHandle_, SC_TOF_SENSOR, &this->depth_intrinsics_),
                      "Could not get depth intrinsics!");
    checkScStatus(scGetSensorIntrinsicParameters(deviceHandle_, SC_COLOR_SENSOR, &this->color_intrinsics_),
                      "Could not get color intrinsics!");

    // Initialise camera info messages
    sensor_msgs::CameraInfo info_msg;
    info_msg.distortion_model = "plumb_bob";
    info_msg.header.frame_id = color_frame;
    info_msg.D = {color_intrinsics_.k1, color_intrinsics_.k2, color_intrinsics_.p1, color_intrinsics_.p2,
                  color_intrinsics_.k3};
    info_msg.K = {color_intrinsics_.fx, 0, color_intrinsics_.cx, 
                    0, color_intrinsics_.fy, color_intrinsics_.cy,
                    0, 0, 1};
    info_msg.P = {color_intrinsics_.fx, 0, color_intrinsics_.cx,
                    0, 0, color_intrinsics_.fy,
                    color_intrinsics_.cy, 0, 0, 
                    0, 1, 0};
    info_msg.R.fill(0);
    info_msg.R[0] = 1;
    info_msg.R[4] = 1;
    info_msg.R[8] = 1;
    color_info_->setCameraInfo(info_msg);
    alignedDepth_info_->setCameraInfo(info_msg);

    info_msg.header.frame_id = depth2colorpoints_frame;
    depth2color_point_cloud_info_->setCameraInfo(info_msg);

    info_msg.header.frame_id = depth_frame;
    info_msg.D = {depth_intrinsics_.k1, depth_intrinsics_.k2, depth_intrinsics_.p1, depth_intrinsics_.p2,
                  depth_intrinsics_.k3};
    info_msg.K = {depth_intrinsics_.fx, 0, depth_intrinsics_.cx,
                    0, depth_intrinsics_.fy, depth_intrinsics_.cy,
                    0, 0, 1};
    info_msg.P = {depth_intrinsics_.fx, 0, depth_intrinsics_.cx,
                    0, 0, depth_intrinsics_.fy,
                    depth_intrinsics_.cy, 0, 0,
                    0, 1, 0};
    
    depth_info_->setCameraInfo(info_msg);
    info_msg.header.frame_id = ir_frame;
    ir_info_->setCameraInfo(info_msg);
    alignedColor_info_->setCameraInfo(info_msg);

    info_msg.header.frame_id = points_frame;
    depth_point_cloud_info_->setCameraInfo(info_msg);

    ROS_INFO("Successfully received intrinsic and extrinsic parameters for device %d", this->device_index_);

    checkScStatus(scSetTransformColorImgToDepthSensorEnabled(deviceHandle_, true),
                      "Could not SetTransformColorImgToDepthSensorEnabled!");
    checkScStatus(scSetTransformDepthImgToColorSensorEnabled(deviceHandle_, true),
                      "Could not SetTransformDepthImgToColorSensorEnabled!");
}

void ScepterManager::updateColorIntrinsicParameters()
{
    std::string color_frame(this->camera_name_ + "_color_frame"),
                depth2colorpoints_frame(this->camera_name_ + "_depth2colorpoints_frame");
    checkScStatus(scGetSensorIntrinsicParameters(deviceHandle_, SC_COLOR_SENSOR, &this->color_intrinsics_),
                    "Could not get color intrinsics!");

    // Initialise camera info messages
    sensor_msgs::CameraInfo info_msg;
    info_msg.distortion_model = "plumb_bob";
    info_msg.header.frame_id = color_frame;
    info_msg.D = {color_intrinsics_.k1, color_intrinsics_.k2, color_intrinsics_.p1, color_intrinsics_.p2,
                color_intrinsics_.k3};
    info_msg.K = {color_intrinsics_.fx, 0, color_intrinsics_.cx, 
                    0, color_intrinsics_.fy, color_intrinsics_.cy,
                    0, 0, 1};
    info_msg.P = {color_intrinsics_.fx, 0, color_intrinsics_.cx,
                    0, 0, color_intrinsics_.fy,
                    color_intrinsics_.cy, 0, 0, 
                    0, 1, 0};
    info_msg.R.fill(0);
    info_msg.R[0] = 1;
    info_msg.R[4] = 1;
    info_msg.R[8] = 1;
    color_info_->setCameraInfo(info_msg);
    alignedDepth_info_->setCameraInfo(info_msg);

    info_msg.header.frame_id = depth2colorpoints_frame;
    depth2color_point_cloud_info_->setCameraInfo(info_msg);

    cameraInfo_Ary[2].reset();
    cameraInfo_Ary[2] = boost::make_shared<sensor_msgs::CameraInfo>(color_info_->getCameraInfo());
    cameraInfo_Ary[3].reset();
    cameraInfo_Ary[3] = boost::make_shared<sensor_msgs::CameraInfo>(alignedDepth_info_->getCameraInfo());
    cameraInfo_Ary[6].reset();
    cameraInfo_Ary[6] = boost::make_shared<sensor_msgs::CameraInfo>(depth2color_point_cloud_info_->getCameraInfo());
}
