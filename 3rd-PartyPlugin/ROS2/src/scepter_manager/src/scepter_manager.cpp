#include "scepter_manager.hpp"
#include <thread>
#include "tf2/LinearMath/Matrix3x3.h"
#include <rclcpp_components/register_node_macro.hpp>
#include<sstream>
#ifdef FOXY
#include "rmw/qos_profiles.h"
#else
#include "rclcpp/qos.hpp"
#endif
#include<iostream>
using namespace std::chrono_literals;
const int fixInterval = 1; // milliseconds
std::atomic<int> ScepterManager::nodeCnt(0);
ScepterManager::ScepterManager(rclcpp::NodeOptions node_options):
        Node("camera_name", node_options.allow_undeclared_parameters(true)),
        camera_name_(""),
        camera_sn_(""),
        deviceHandle_(nullptr)
{
    declare_parameters();
    this->get_parameter("camera_name", camera_name_);
    RCLCPP_INFO(this->get_logger(), "camera name: %s" , camera_name_.c_str());
    this->get_parameter<std::string>("camera_sn", camera_sn_);
    RCLCPP_INFO(this->get_logger(), "camera sn: %s" , camera_sn_.c_str());
    timer_ = nullptr;
    timer_tf_ = nullptr;
    if(init_open_camera())
    {
        RCLCPP_INFO(this->get_logger(), "Init and open camera success");
        //config_camera_params();
        init_topic_list();
        init_TF_info();
        set_sensor_intrinsics();
        int interval = (1000 / cameraSetting_.get_framerate()) + fixInterval;
        timer_ = this->create_wall_timer(std::chrono::milliseconds(interval), std::bind(&ScepterManager::timeout, this));
        RCLCPP_INFO(this->get_logger(), "Init timer success with interval %d", interval);
        auto on_shutdown_callback = [this]() {
            std::cout << "Shutting down." << std::endl;
            if (timer_ != nullptr)
            {
                if (!timer_->is_canceled())
                {
                    std::cout << "cancel timer in shutdown." << std::endl;
                    timer_->cancel();
                }
                timer_ = nullptr;
            }
            if (timer_tf_ != nullptr)
            {
                if (!timer_tf_->is_canceled())
                {
                    std::cout << "cancel timer_tf in shutdown." << std::endl;
                    timer_tf_->cancel();
                }
                timer_tf_ = nullptr;
            }
        };
        this->get_node_options().context()->on_shutdown(on_shutdown_callback);
        RCLCPP_INFO(this->get_logger(), "------ camera %s is runing ok ------", camera_name_.c_str());
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "init_open_camera failed");
        return;
    }

    parameter_callback_handle_ = this->add_on_set_parameters_callback( [this] (const std::vector<rclcpp::Parameter> &params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        if (deviceHandle_ == nullptr)
        {
            RCLCPP_INFO(this->get_logger(), "deviceHandle is null.");
            result.successful = false;
            return result;
        }

        ScStatus status = ScStatus::SC_OK;
        for (const auto &param : params) {
            RCLCPP_INFO(this->get_logger(), "param name %s", param.get_name().c_str());
            if (param.get_name() == "depth_publish") {
                RCLCPP_INFO(this->get_logger(), "depth_publish updated: %d", param.as_bool());
                cameraSetting_.set_depth_publish(param.as_bool());
                if(param.as_bool() == true)
                {
                    updateImageTopic(pubArr[0], pubCameraInfoArr[0], true, 0, "depth");
                }
                else
                {
                    updateImageTopic(pubArr[0], pubCameraInfoArr[0], false, 0, "depth");
                }
            }
            else if (param.get_name() == "ir_publish") {
                RCLCPP_INFO(this->get_logger(), "ir_publish updated: %d", param.as_bool());
                cameraSetting_.set_ir_publish(param.as_bool());
                if(param.as_bool() == true)
                {
                    updateImageTopic(pubArr[1], pubCameraInfoArr[1], true, 1, "ir");
                }
                else
                {
                    updateImageTopic(pubArr[1], pubCameraInfoArr[1], false, 1, "ir");
                }
            }
            else if (param.get_name() == "color_publish") {
                RCLCPP_INFO(this->get_logger(), "color_publish updated: %d", param.as_bool());
                cameraSetting_.set_color_publish(param.as_bool());
                if(param.as_bool() == true)
                {
                    updateImageTopic(pubArr[2], pubCameraInfoArr[2], true, 2, "color");
                }
                else
                {
                    updateImageTopic(pubArr[2], pubCameraInfoArr[2], false, 2, "color");
                }
            }
            else if (param.get_name() == "transformed_color") {
                RCLCPP_INFO(this->get_logger(), "transformed_color updated: %d", param.as_bool());
                cameraSetting_.set_transformed_color_publish(param.as_bool());
                status = scSetTransformColorImgToDepthSensorEnabled(deviceHandle_, param.as_bool());
                if (status != ScStatus::SC_OK)
                {
                    RCLCPP_INFO(this->get_logger(), "scSetTransformColorImgToDepthSensorEnabled failed! %d" ,status);
                    result.successful = false;
                    continue;
                }
                if(param.as_bool() == true)
                {
                    updateImageTopic(pubArr[4], pubCameraInfoArr[4], true, 4, "transformedColor");
                }
                else
                {
                    updateImageTopic(pubArr[4], pubCameraInfoArr[4], false, 4, "transformedColor");
                }
            }
            else if (param.get_name() == "transformed_depth") {
                RCLCPP_INFO(this->get_logger(), "transformed_depth updated: %d", param.as_bool());
                cameraSetting_.set_transformed_depth_publish(param.as_bool());
                status = scSetTransformDepthImgToColorSensorEnabled(deviceHandle_, param.as_bool());
                if (status != ScStatus::SC_OK)
                {
                    RCLCPP_INFO(this->get_logger(), "scSetTransformDepthImgToColorSensorEnabled failed! %d" ,status);
                    result.successful = false;
                    continue;
                }
                if(param.as_bool() == true)
                {
                    updateImageTopic(pubArr[3], pubCameraInfoArr[3], true, 3, "transformedDepth");
                }
                else
                {
                    updateImageTopic(pubArr[3], pubCameraInfoArr[3], false, 3, "transformedDepth");
                }
            }
            else if (param.get_name() == "depth_cloud_point") {
                RCLCPP_INFO(this->get_logger(), "%s depth_cloud_point updated: %d", camera_name_.c_str(), param.as_bool());
                cameraSetting_.set_depth_cloud_point(param.as_bool());
                if(param.as_bool() == true)
                {
                    updateCloudPointTopic(pointclound2_pub_, pointclound2info_pub_, true, "depth");
                }
                else
                {
                    updateCloudPointTopic(pointclound2_pub_, pointclound2info_pub_, false, "depth");
                }

            }
            else if (param.get_name() == "depth2color_cloud_point") {
                RCLCPP_INFO(this->get_logger(), "%s depth2color_cloud_point updated: %d", camera_name_.c_str(), param.as_bool());
                cameraSetting_.set_depth2color_cloud_point(param.as_bool());
                if(param.as_bool() == true)
                {
                    updateCloudPointTopic(depth2colorpointclound2_pub_, depth2colorpointclound2info_pub_, true, "depth2color");
                }
                else
                {
                    updateCloudPointTopic(depth2colorpointclound2_pub_, depth2colorpointclound2info_pub_, false, "depth2color");
                }
            }
            else if (param.get_name() == "work_mode") {
                RCLCPP_INFO(this->get_logger(), "work_mode updated: %d", (int)param.as_int());
                cameraSetting_.set_workmode(param.as_int());
                status = scSetWorkMode(deviceHandle_, (ScWorkMode)(param.as_int()));
                if (status != ScStatus::SC_OK)
                {
                    RCLCPP_INFO(this->get_logger(), "scSetWorkMode failed! %d" ,status);
                    result.successful = false;
                    continue;
                }
                else
                {
                    if (param.as_int() == 0 || param.as_int() == 1)
                    {
                        if (timer_ != nullptr)
                        {
                            if(!timer_->is_canceled())
                            {
                                RCLCPP_INFO(this->get_logger(), "timer cancel in work_mode");
                                timer_->cancel();
                            }
                            timer_ = nullptr;
                        }
                        int interval = (1000 / cameraSetting_.get_framerate()) + fixInterval;
                        RCLCPP_INFO(this->get_logger(), "timer restart in workmode with interval %d", interval);
                        timer_ = this->create_wall_timer(std::chrono::milliseconds(interval), std::bind(&ScepterManager::timeout, this));
                    }
                    else
                    {
                        if (timer_ != nullptr)
                        {
                            if(!timer_->is_canceled())
                            {
                                RCLCPP_INFO(this->get_logger(), "timer cancel in work_mode");
                                timer_->cancel();
                            }
                            timer_ = nullptr;
                        }
                    }
                }
            }
            else if (param.get_name() == "xdr_mode") {
                RCLCPP_INFO(this->get_logger(), "xdr_mode updated: %d", (int)param.as_int());
                cameraSetting_.set_xdrmode(param.as_int());
                ScStatus hdrStatus = ScStatus::SC_OK, wdrStatus = ScStatus::SC_OK;
                switch (param.as_int())
                {
                    case 0:
                        hdrEnabled = false;
                        wdrEnabled = false;
                        hdrStatus = scSetHDRModeEnabled(deviceHandle_, hdrEnabled);
                        RCLCPP_INFO(this->get_logger(), "scSetHDRModeEnabled status: %d hdrEnabled %d", hdrStatus, hdrEnabled);
                        wdrStatus = scSetWDRModeEnabled(deviceHandle_, wdrEnabled);
                        RCLCPP_INFO(this->get_logger(), "scSetWDRModeEnabled status: %d wdrEnabled %d", wdrStatus, wdrEnabled);
                        break;
                    case 1:
                        hdrEnabled = true;
                        wdrEnabled = false;
                        wdrStatus = scSetWDRModeEnabled(deviceHandle_, wdrEnabled);
                        RCLCPP_INFO(this->get_logger(), "scSetWDRModeEnabled status: %d wdrEnabled %d", wdrStatus, wdrEnabled);
                        hdrStatus = scSetHDRModeEnabled(deviceHandle_, hdrEnabled);
                        RCLCPP_INFO(this->get_logger(), "scSetHDRModeEnabled status: %d hdrEnabled %d", hdrStatus, hdrEnabled);
                        break;
                    case 2:
                        hdrEnabled = false;
                        wdrEnabled = true;
                        hdrStatus = scSetHDRModeEnabled(deviceHandle_, hdrEnabled);
                        RCLCPP_INFO(this->get_logger(), "scSetHDRModeEnabled status: %d hdrEnabled %d", hdrStatus, hdrEnabled);
                        wdrStatus = scSetWDRModeEnabled(deviceHandle_, wdrEnabled);
                        RCLCPP_INFO(this->get_logger(), "scSetWDRModeEnabled status: %d wdrEnabled %d", wdrStatus, wdrEnabled);
                        break;
                }

                if (hdrStatus != ScStatus::SC_OK || wdrStatus != ScStatus::SC_OK)
                {
                    RCLCPP_INFO(this->get_logger(), "scSetXDRModeEnabled failed! hdrStatus: %d wdrStatus %d" ,hdrStatus, wdrStatus);
                    result.successful = false;
                    continue;
                }
                else
                {
                    int frame = 0;
                    status = scGetFrameRate(deviceHandle_, &frame);
                    RCLCPP_INFO(this->get_logger(), "scGetFrameRate %d status %d", frame, status);
                    if (status == ScStatus::SC_OK)
                    {
                        cameraSetting_.set_framerate(frame);
                        int interval = (1000 / frame) + fixInterval;
                        if (timer_ != nullptr)
                        {
                            if (!timer_->is_canceled())
                            {
                                RCLCPP_INFO(this->get_logger(), "timer cancel in xdr");
                                timer_->cancel();
                            }
                            timer_ = nullptr;
                        }
                        RCLCPP_INFO(this->get_logger(), "timer restart in xrd_mode with interval %d", interval);
                        timer_ = this->create_wall_timer(std::chrono::milliseconds(interval), std::bind(&ScepterManager::timeout, this));
                    }
                }
            }
            else if (param.get_name() == "framerate") {
                RCLCPP_INFO(this->get_logger(), "framerate updated: %d", (int)param.as_int());
                status = scSetFrameRate(deviceHandle_, param.as_int());
                if (status != ScStatus::SC_OK)
                {
                    RCLCPP_INFO(this->get_logger(), "scSetFrameRate failed! %d" ,status);
                    scGetHDRModeEnabled(deviceHandle_, &hdrEnabled);
                    scGetWDRModeEnabled(deviceHandle_, &wdrEnabled);
                    if (hdrEnabled)
                    {
                        RCLCPP_INFO(this->get_logger(),"framerate number invaild with HDR enabled");
                    }
                    if (wdrEnabled)
                    {
                        RCLCPP_INFO(this->get_logger(),"framerate number invaild with WDR enabled");
                    }
                    result.successful = false;
                    continue;
                }
                else
                {
                    cameraSetting_.set_framerate(param.as_int());
                }

                int interval = (1000 / param.as_int()) + fixInterval;
                if (timer_ != nullptr)
                {
                    if(!timer_->is_canceled())
                    {
                        RCLCPP_INFO(this->get_logger(), "timer cancel in framerate");
                        timer_->cancel();
                    }
                    timer_ = nullptr;
                }
                RCLCPP_INFO(this->get_logger(), "timer restart in framerate with interval %d", interval);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(interval), std::bind(&ScepterManager::timeout, this));
            }
            else if (param.get_name() == "color_resolution") {
                RCLCPP_INFO(this->get_logger(), "color_resolution updated: %d", (int)param.as_int());
                cameraSetting_.set_color_resolution(param.as_int());
                int width = 640;
                int height = 480;
                switch (param.as_int())
                {
                case 0:
                    width = 1600;
                    height = 1200;
                    break;
                case 1:
                    width = 800;
                    height = 600;
                    break;
                case 2:
                    width = 640;
                    height = 480;
                    break;
                }
                status= scSetColorResolution(deviceHandle_, width, height);
                if (status != ScStatus::SC_OK)
                {
                    RCLCPP_INFO(this->get_logger(), "scSetColorResolution failed! %d" ,status);
                    result.successful = false;
                    continue;
                }
            }
            else if (param.get_name() == "software_trigger") {
                if (cameraSetting_.get_workmode() != 2)
                {
                    RCLCPP_INFO(this->get_logger(), "workMode check failed, work mode need set to 2 but current workmode is: %d", cameraSetting_.get_workmode());
                    result.successful = false;
                    continue;
                }
                RCLCPP_INFO(this->get_logger(), "SoftwareTrigger called");
                status = scSoftwareTriggerOnce(deviceHandle_);
                if (status != ScStatus::SC_OK)
                {
                    RCLCPP_INFO(this->get_logger(), "SoftwareTrigger failed! %d" ,status);
                    result.successful = false;
                    continue;
                }
                ScFrameReady psReadFrame;
                memset(&psReadFrame, 0x0, sizeof(ScFrameReady));
                ScStatus status =  scGetFrameReady(deviceHandle_, 1200, &psReadFrame);
                if (status != SC_OK)
                {
                    RCLCPP_INFO(this->get_logger(), "scGetFrameReady failed! %d" ,status);
                }
                else
                {
                    publicImage(psReadFrame);
                }
            }
            else if (param.get_name() == "SingleFrameDelayTest") {
                if (cameraSetting_.get_workmode() != 2)
                {
                    RCLCPP_INFO(this->get_logger(), "workMode check failed, work mode need set to 2 but current workmode is: %d", cameraSetting_.get_workmode());
                    result.successful = false;
                    continue;
                }
                RCLCPP_INFO(this->get_logger(), "SingleFrameDelayTest updated: %lld  now %lld diff %d", (uint64_t)param.as_double(), (uint64_t)(this->get_clock()->now().seconds() * 1000),((uint64_t)(this->get_clock()->now().seconds() * 1000) - (uint64_t)param.as_double()));
                {
                    status = scSoftwareTriggerOnce(deviceHandle_);
                    if (status != ScStatus::SC_OK)
                    {
                        RCLCPP_INFO(this->get_logger(), "SoftwareTrigger failed! %d" ,status);
                        result.successful = false;
                        continue;
                    }
                    ScFrameReady psReadFrame;
                    memset(&psReadFrame, 0x0, sizeof(ScFrameReady));
                    ScStatus status =  scGetFrameReady(deviceHandle_, 1200, &psReadFrame);
                    if (status != SC_OK)
                    {
                        RCLCPP_INFO(this->get_logger(), "scGetFrameReady failed! %d" ,status);
                        result.successful = false;
                        continue;
                    }
                    if (1 == psReadFrame.depth)
                    {
                        ScFrame frame;
                        memset(&frame, 0x0, sizeof(ScFrame));
                        scGetFrame(deviceHandle_, SC_DEPTH_FRAME, &frame);
                        if(frame.pFrameData != nullptr && pubArr[0] != nullptr)
                        {
                            int cvMatType = CV_16UC1;
                            std::string imageEncodeType = sensor_msgs::image_encodings::TYPE_16UC1;
                            cv::Mat mat = cv::Mat(frame.height, frame.width, cvMatType, frame.pFrameData);
                            cv_bridge::CvImage cvi_;
                            cvi_.header.stamp = this->get_clock()->now();
                            cvi_.header.frame_id = this->camera_name_ + "_depth_frame";
                            cvi_.encoding = "16UC1";
                            cvi_.image = mat;
                            sensor_msgs::msg::Image im_msg;
                            cvi_.toImageMsg(im_msg);
                            pubArr[0]->publish(im_msg);
                            //RCLCPP_INFO( this->get_logger(), " get frame index %d now %lld", frame.frameIndex, (uint64_t)(this->get_clock()->now().seconds() * 1000));
                        }
                    }
                }
            }
        }
        return result;
    });
}

ScepterManager::~ScepterManager()
{
    std::cout << "ScepterManager deconstruction." << std::endl;
    nodeCnt--;
    ScStatus status = ScStatus::SC_OK;
    if (deviceHandle_ != nullptr)
    {
        status = scStopStream(deviceHandle_);
        if (status != ScStatus::SC_OK)
        {
            std::cout << "scStopStream failed status " << status << std::endl;
        }
        else
        {
            std::cout << "scStopStream success status " << status << std::endl;
        }
        status = scCloseDevice(&deviceHandle_);
        if (status != ScStatus::SC_OK)
        {
            std::cout << "scCloseDevice failed status " << status << std::endl;
        }
        else
        {
            std::cout << "scCloseDevice success status " << status << std::endl;
        }
        deviceHandle_ = nullptr;
    }
    if(nodeCnt.load() == 0)
    {
        status = scShutdown();
        if (status != ScStatus::SC_OK)
        {
            std::cout << "scShutdown failed status " << status << std::endl;
        }
        else
        {
            std::cout << "scShutdown success status " << status << std::endl;
        }
    }
}

void ScepterManager::declare_parameters()
{
    auto framerate_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    rcl_interfaces::msg::IntegerRange framerate_range;
    framerate_range.set__from_value(0).set__to_value(30).set__step(1);
    framerate_descriptor.integer_range = {framerate_range};

    auto workmode_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    rcl_interfaces::msg::IntegerRange workmode_range;
    workmode_range.set__from_value(0).set__to_value(2).set__step(1);
    workmode_descriptor.description = "Work Mode\n 0: Active Mode\n 1: Hardwaretrigger Mode\n 2: Softwaretigger Mode";
    workmode_descriptor.integer_range = {workmode_range};

    auto resolution_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    rcl_interfaces::msg::IntegerRange resolution_range;
    resolution_range.set__from_value(0).set__to_value(2).set__step(1);
    resolution_descriptor.description = "Color Resolution\n 0: 1600*1200 \n 1: 800*600\n 2: 640*480";
    resolution_descriptor.integer_range = {resolution_range};

    auto xdr_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    rcl_interfaces::msg::IntegerRange xdr_range;
    xdr_range.set__from_value(0).set__to_value(2).set__step(1);
    xdr_descriptor.description = "XDR Mode\n 0: hdr disable wdr disable \n 1: hdr enable wdr diable \n 2: hdr disable wdr enable";
    xdr_descriptor.integer_range = {xdr_range};

    auto camera_name_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    camera_name_descriptor.description = "camear name";
    camera_name_descriptor.read_only = true;

    auto camera_sn_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    camera_sn_descriptor.description = "camear series number";
    camera_sn_descriptor.read_only = true;

    auto depth_publish_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    depth_publish_descriptor.description = "whether publish depth image.";

    auto ir_publish_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    ir_publish_descriptor.description = "whether publish ir image.";

    auto color_publish_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    color_publish_descriptor.description = "whether publish color image.";

    auto trasnsformed_color_publish_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    trasnsformed_color_publish_descriptor.description = "whether enable color to depth transformation and publish it.";

    auto trasnsformed_depth_publish_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    trasnsformed_depth_publish_descriptor.description = "whether enable depth to color transformation and publish it.";

    auto depth_cloud_point_publish_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    depth_cloud_point_publish_descriptor.description = "whether publish depth cloud point.";

    auto depth2color_cloud_point_publish_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    depth2color_cloud_point_publish_descriptor.description = "whether publish depth2color cloud point, must works with the transformed_depth enabled.";

    cameraSetting_ = CameraSetting(this->declare_parameter<std::string>("camera_name","", camera_name_descriptor), this->declare_parameter<std::string>("camera_sn","", camera_sn_descriptor),
    this->declare_parameter<int>("framerate", 10, framerate_descriptor), this->declare_parameter<int>("work_mode", 0, workmode_descriptor), this->declare_parameter<int>("color_resolution", 0, resolution_descriptor),
    this->declare_parameter<int>("xdr_mode", 0, xdr_descriptor), this->declare_parameter<bool>("depth_publish", false, depth_publish_descriptor), this->declare_parameter<bool>("ir_publish", false, ir_publish_descriptor),
    this->declare_parameter<bool>("color_publish", false, color_publish_descriptor), this->declare_parameter<bool>("transformed_color", false, trasnsformed_color_publish_descriptor), this->declare_parameter<bool>("transformed_depth", false, trasnsformed_depth_publish_descriptor),
    this->declare_parameter<bool>("depth_cloud_point", false, depth_cloud_point_publish_descriptor), this->declare_parameter<bool>("depth2color_cloud_point", false, depth2color_cloud_point_publish_descriptor));


    auto software_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    software_descriptor.description = "Only works with Softwaretigger work mode enabled.";
    this->declare_parameter<bool>("software_trigger", false, software_descriptor);
    //this->declare_parameter<double>("SingleFrameDelayTest", 0); // only used in test mode
    return;
}

bool ScepterManager::init_open_camera()
{ 
    ScStatus status = ScStatus::SC_OK;
    if(nodeCnt.load() == 0)
    {
        // Initialise the API
        status = scInitialize();
        if (status != ScStatus::SC_OK)
        {
            RCLCPP_INFO(this->get_logger(),"scInitialize failed! %d" ,status);
            return false;
        }
    }
    nodeCnt++;
    uint32_t device_count = 0;
GET:
    status = scGetDeviceCount(&device_count, 3000);
    if (status != ScStatus::SC_OK)
    {
        RCLCPP_INFO(this->get_logger(), "scGetDeviceCount failed! %d" ,status);
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "The device count is: %d" ,device_count); 
    if (0 == device_count)
    {
        RCLCPP_INFO(this->get_logger(), "Try again."); 
        this_thread::sleep_for(chrono::seconds(3));
        goto GET;
    }

    ScDeviceInfo* pPsDeviceInfoList = new ScDeviceInfo[device_count];
    status =  scGetDeviceInfoList(device_count, pPsDeviceInfoList);
    if (status != ScStatus::SC_OK)
    {
        RCLCPP_INFO(this->get_logger(), "scGetDeviceInfoList failed! %d" ,status);
        delete [] pPsDeviceInfoList;
        pPsDeviceInfoList = nullptr;
        return false;
    }
    std::string cameraSN = "";
    for (int ind = 0 ; ind < (int)device_count; ind++)
    {
        if(strcmp(pPsDeviceInfoList[ind].serialNumber, cameraSetting_.get_camera_sn().c_str()) == 0)
        {
            cameraSN = pPsDeviceInfoList[ind].serialNumber;
            break;
        }
    }

    if (cameraSN.length() == 0)
    {
        RCLCPP_INFO(this->get_logger(), "No match camera was found, please attach the device with sn: %s", camera_sn_.c_str());
        delete [] pPsDeviceInfoList;
        pPsDeviceInfoList = nullptr;
        goto GET;
    }

    status = scOpenDeviceBySN(cameraSN.c_str(), &deviceHandle_);
    if (status != ScStatus::SC_OK)
    {
        RCLCPP_INFO(this->get_logger(), "scOpenDeviceBySN %s failed! %d", cameraSN.c_str(),status);
        return false;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "scOpenDeviceBySN success, SN: %s", cameraSN.c_str());
    }
    char buffer[2048];
    getcwd(buffer, sizeof(buffer));
    string path(buffer);
    path = path + "/" + camera_name_ + "_parameter.json";
    status = scSetParamsByJson(deviceHandle_, const_cast<char*>(path.c_str()));
    if (status != ScStatus::SC_OK)
    {
        RCLCPP_INFO(this->get_logger(), "scSetParamsByJson ret %d Please create json file at %s if you need it", status, const_cast<char*>(path.c_str()));
        config_camera_params();
    }
    else
    {
        RCLCPP_INFO(this->get_logger(),"Successfully load json %s", const_cast<char*>(path.c_str()));
        ScStatus status = ScStatus::SC_OK;
        ScWorkMode workMode;
        status = scGetWorkMode(deviceHandle_, &workMode);
        if (status != ScStatus::SC_OK)
        {
            RCLCPP_INFO(this->get_logger(), "scGetWorkMode failed status %d", status);
        }
        else
        {
            cameraSetting_.set_workmode((int)workMode);
            RCLCPP_INFO(this->get_logger(), "work_mode: %d", cameraSetting_.get_workmode());
            this->set_parameter(rclcpp::Parameter("work_mode", cameraSetting_.get_workmode()));
        }
        int xdr_mode = 0;
        bool hdrEnabled = false;
        bool wdrEnabled = false;
        scGetHDRModeEnabled(deviceHandle_, &hdrEnabled);
        RCLCPP_INFO(this->get_logger(), "hdrEnabled: %d", hdrEnabled);
        scGetWDRModeEnabled(deviceHandle_, &wdrEnabled);
        RCLCPP_INFO(this->get_logger(), "wdrEnabled: %d",  wdrEnabled);
        if (hdrEnabled)
        {
            xdr_mode = 1;
        }
        else if (wdrEnabled)
        {
            xdr_mode = 2;
        }
        else
        {
            xdr_mode = 0;
        }
        cameraSetting_.set_xdrmode(xdr_mode);
        RCLCPP_INFO(this->get_logger(), "xdr_mode: %d", cameraSetting_.get_xdrmode());
        this->set_parameter(rclcpp::Parameter("xdr_mode", cameraSetting_.get_xdrmode()));
        int frame = 0;
        status = scGetFrameRate(deviceHandle_, &frame);
        if (status != ScStatus::SC_OK)
        {
            RCLCPP_INFO(this->get_logger(), "scGetFrameRate failed status %d", status);
        }
        else
        {
            cameraSetting_.set_framerate(frame);
            RCLCPP_INFO(this->get_logger(), "framerate: %d", cameraSetting_.get_framerate());
            this->set_parameter(rclcpp::Parameter("framerate", cameraSetting_.get_framerate()));
        }

        int width = 0;
        int height = 0;
        int color_resolution = 0;
        scGetColorResolution(deviceHandle_, &width, &height);
        if (width == 1600 && height == 1200)
        {
            color_resolution = 0;
        }
        else if (width == 800 && height == 600)
        {
            color_resolution = 1;
        }
        else
        {
            color_resolution = 2;
        }
        cameraSetting_.set_color_resolution(color_resolution);
        RCLCPP_INFO(this->get_logger(), "color_resolution: %d", cameraSetting_.get_color_resolution());
        this->set_parameter(rclcpp::Parameter("color_resolution", cameraSetting_.get_color_resolution()));
    }

    status = scStartStream(deviceHandle_);
    if (status != ScStatus::SC_OK)
    {
        RCLCPP_INFO(this->get_logger(), "scStartStream %s failed! %d", cameraSN.c_str(),status);
        return false;
    }
    /* add user define api call start*/
    // such as call the scSetSpatialFilterEnabled

    /*
    status= scSetSpatialFilterEnabled(deviceHandle_,true);
    RCLCPP_INFO( "SetSpatialFilterEnabled status: " << status);
    */
    /* add user define api call end*/

    const int BufLen = 64;
    char fw[BufLen] = { 0 };
    scGetFirmwareVersion(deviceHandle_, fw, BufLen);
    RCLCPP_INFO(this->get_logger(), "fw: %s " , fw);
    return true;
}

bool ScepterManager::config_camera_params()
{
    // use param init camera
    ScStatus status = ScStatus::SC_OK;
    status = scSetWorkMode(deviceHandle_, (ScWorkMode)cameraSetting_.get_workmode());
    if (status != ScStatus::SC_OK)
    {
        RCLCPP_INFO(this->get_logger(), "scSetWorkMode failed status %d", status);
        cameraSetting_.set_workmode(0);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "scSetWorkMode: %d", cameraSetting_.get_workmode());
    }
    ScStatus hdrStatus = ScStatus::SC_OK, wdrStatus = ScStatus::SC_OK;
    switch (cameraSetting_.get_xdrmode())
    {
        case 0:
            hdrEnabled = false;
            wdrEnabled = false;
            hdrStatus = scSetHDRModeEnabled(deviceHandle_, hdrEnabled);
            RCLCPP_INFO(this->get_logger(), "scSetHDRModeEnabled status: %d hdrEnabled %d", hdrStatus, hdrEnabled);
            wdrStatus = scSetWDRModeEnabled(deviceHandle_, wdrEnabled);
            RCLCPP_INFO(this->get_logger(), "scSetWDRModeEnabled status: %d wdrEnabled %d", wdrStatus, wdrEnabled);
            break;
        case 1:
            hdrEnabled = true;
            wdrEnabled = false;
            wdrStatus = scSetWDRModeEnabled(deviceHandle_, wdrEnabled);
            RCLCPP_INFO(this->get_logger(), "scSetWDRModeEnabled status: %d wdrEnabled %d", wdrStatus, wdrEnabled);
            hdrStatus = scSetHDRModeEnabled(deviceHandle_, hdrEnabled);
            RCLCPP_INFO(this->get_logger(), "scSetHDRModeEnabled status: %d hdrEnabled %d", hdrStatus, hdrEnabled);
            break;
        case 2:
            hdrEnabled = false;
            wdrEnabled = true;
            hdrStatus = scSetHDRModeEnabled(deviceHandle_, hdrEnabled);
            RCLCPP_INFO(this->get_logger(), "scSetHDRModeEnabled status: %d hdrEnabled %d", hdrStatus, hdrEnabled);
            wdrStatus = scSetWDRModeEnabled(deviceHandle_, wdrEnabled);
            RCLCPP_INFO(this->get_logger(), "scSetWDRModeEnabled status: %d wdrEnabled %d", wdrStatus, wdrEnabled);
            break;
    }

    if (cameraSetting_.get_workmode() == 0)
    {
        status = scSetFrameRate(deviceHandle_, cameraSetting_.get_framerate());
        if (status != ScStatus::SC_OK)
        {
            RCLCPP_INFO(this->get_logger(), "scSetFrameRate failed status %d", status);
            cameraSetting_.set_framerate(10);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "scSetFrameRate frameRate: %d", cameraSetting_.get_framerate());
        }
    }

    int width = 640;
    int height = 480;
    switch (cameraSetting_.get_color_resolution())
    {
    case 0:
        width = 1600;
        height = 1200;
        break;
    case 1:
        width = 800;
        height = 600;
        break;
    case 2:
        width = 640;
        height = 480;
        break;
    }
    status= scSetColorResolution(deviceHandle_, width, height);
    if (status != ScStatus::SC_OK)
    {
        RCLCPP_INFO(this->get_logger(), "scSetColorResolution failed! %d" ,status);
        cameraSetting_.set_color_resolution(2);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "scSetColorResolution width: %d  height: %d", width, height);
    }

    return true;
}

void ScepterManager::init_topic_list()
{
#ifdef FOXY
    auto qosVol = rclcpp::QoS(rclcpp::KeepLast(10));
    qosVol.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qosVol.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE); // Set to RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT to get a lower delay if necessary
#else
    auto qosVol = rclcpp::QoS(rclcpp::KeepLast(10)).durability(rclcpp::DurabilityPolicy::Volatile).reliability(rclcpp::ReliabilityPolicy::Reliable); //Set to rclcpp::ReliabilityPolicy::BestEffort to get a lower delay if necessary
#endif
    if(cameraSetting_.get_depth_publish() == true) {
        pubArr[0] = this->create_publisher<sensor_msgs::msg::Image>(camera_name_+"/"+camera_sn_ +"/depth/image_raw", qosVol);
        pubCameraInfoArr[0] = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_name_+"/"+camera_sn_ +"/depth/camera_info", qosVol);
    }
    if(cameraSetting_.get_ir_publish() == true) {
        pubArr[1] = this->create_publisher<sensor_msgs::msg::Image>(camera_name_+"/"+camera_sn_ +"/ir/image_raw", qosVol);
        pubCameraInfoArr[1] = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_name_+"/"+camera_sn_ +"/ir/camera_info", qosVol);
    }
    if(cameraSetting_.get_color_publish() == true) {
        pubArr[2] = this->create_publisher<sensor_msgs::msg::Image>(camera_name_+"/"+ camera_sn_ +"/color/image_raw", qosVol);
        pubCameraInfoArr[2] = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_name_+"/"+camera_sn_ +"/color/camera_info", qosVol);
    }
    if(cameraSetting_.get_transformed_depth_publish() == true) {
        ScStatus status = scSetTransformDepthImgToColorSensorEnabled(deviceHandle_, true);
        RCLCPP_INFO(this->get_logger(), "Enable transformDepthImgToColorSensor status: %d", status);
        pubArr[3] = this->create_publisher<sensor_msgs::msg::Image>(camera_name_+"/"+camera_sn_ +"/transformedDepth/image_raw", qosVol);
        pubCameraInfoArr[3] = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_name_+"/"+camera_sn_ +"/transformedDepth/camera_info", qosVol);
    }
    if(cameraSetting_.get_transformed_color_publish() == true) {
        ScStatus status = scSetTransformColorImgToDepthSensorEnabled(deviceHandle_, true);
        RCLCPP_INFO(this->get_logger(), "Enable transformColorImgToDepthSensor status: %d", status);
        pubArr[4] = this->create_publisher<sensor_msgs::msg::Image>(camera_name_+"/"+camera_sn_ +"/transformedColor/image_raw", qosVol);
        pubCameraInfoArr[4] = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_name_+"/"+camera_sn_ +"/transformedColor/camera_info", qosVol);
    }
    if (cameraSetting_.get_depth_cloud_point() == true){
        pointclound2_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(camera_name_+"/"+camera_sn_ +"/depth/points", qosVol);
        pointclound2info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_name_+"/"+camera_sn_ +"/depth/points/camera_info", qosVol);
    }
    if (cameraSetting_.get_depth2color_cloud_point() == true)
    {
        depth2colorpointclound2_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(camera_name_+"/"+camera_sn_ +"/depth2color/points", qosVol);
        depth2colorpointclound2info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_name_+"/"+camera_sn_ +"/depth2color/points/camera_info", qosVol);
    }

    RCLCPP_INFO(this->get_logger(), "init topic success");
    return;
}

void ScepterManager::updateImageTopic(rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& image_topic, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr& image_info_topic, bool modify, int index, const char* type)
{
    if (modify ==  true)
    {
#ifdef FOXY
    auto qosVol = rclcpp::QoS(rclcpp::KeepLast(10));
    qosVol.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qosVol.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE); // Set to RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT to get a lower delay if necessary
#else
    auto qosVol = rclcpp::QoS(rclcpp::KeepLast(10)).durability(rclcpp::DurabilityPolicy::Volatile).reliability(rclcpp::ReliabilityPolicy::Reliable); //Set to rclcpp::ReliabilityPolicy::BestEffort to get a lower delay if necessary
#endif
        std::stringstream ssm;
        if (image_topic == nullptr)
        {
            ssm << camera_name_ << "/" << camera_sn_ << "/" << type << "/image_raw";
            image_topic = this->create_publisher<sensor_msgs::msg::Image>(ssm.str().c_str(), qosVol);
            pubArr[index] = image_topic;
            ssm.str("");
            ssm.clear();
        }
        if (image_info_topic == nullptr)
        {
            ssm << camera_name_ << "/" << camera_sn_ << "/" << type << "/camera_info";
            image_info_topic = this->create_publisher<sensor_msgs::msg::CameraInfo>(ssm.str().c_str(), qosVol);
            pubCameraInfoArr[index] = image_info_topic;
            ssm.str("");
            ssm.clear();
        }
    }
    else
    {
        if (image_topic != nullptr)
        {
            RCLCPP_INFO(this->get_logger(), "%s image reset", type);
            image_topic.reset();
            pubArr[index].reset();
        }
        if (image_info_topic != nullptr)
        {
            RCLCPP_INFO(this->get_logger(), "%s info reset", type);
            image_info_topic.reset();
            pubCameraInfoArr[index].reset();
        }
    }
    return;
}

void ScepterManager::updateCloudPointTopic(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& point_cloud_topic, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr& point_clouds_info_topic, bool modify, const char* type)
{
    if (modify ==  true)
    {
#ifdef FOXY
    auto qosVol = rclcpp::QoS(rclcpp::KeepLast(10));
    qosVol.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qosVol.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE); // Set to RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT to get a lower delay if necessary
#else
    auto qosVol = rclcpp::QoS(rclcpp::KeepLast(10)).durability(rclcpp::DurabilityPolicy::Volatile).reliability(rclcpp::ReliabilityPolicy::Reliable); //Set to rclcpp::ReliabilityPolicy::BestEffort to get a lower delay if necessary
#endif
        std::stringstream ssm;
        if (point_cloud_topic == nullptr)
        {
            ssm << camera_name_ << "/" << camera_sn_ << "/" << type << "/points";
            point_cloud_topic = this->create_publisher<sensor_msgs::msg::PointCloud2>(ssm.str().c_str(), qosVol);
            ssm.str("");
            ssm.clear();
        }
        if (point_clouds_info_topic == nullptr)
        {
            ssm << camera_name_ << "/" << camera_sn_ << "/" << type << "/points/camera_info";
            point_clouds_info_topic = this->create_publisher<sensor_msgs::msg::CameraInfo>(ssm.str().c_str(), qosVol);
            ssm.str("");
            ssm.clear();
        }
    }
    else
    {
        if (point_cloud_topic != nullptr)
        {
            RCLCPP_INFO(this->get_logger(), "%s image reset", type);
            point_cloud_topic.reset();
        }
        if (point_clouds_info_topic != nullptr)
        {
            RCLCPP_INFO(this->get_logger(), "%s info reset", type);
            point_clouds_info_topic.reset();
        }
    }
    return;
}
void ScepterManager::timeout() 
{
    if (deviceHandle_ != nullptr)
    {
        if (cameraSetting_.get_workmode() == 2)
        {
            if (timer_ != nullptr)
            {
                if(!timer_->is_canceled())
                {
                    RCLCPP_INFO(this->get_logger(), "timer cancel in timeout");
                    timer_->cancel();
                }
                timer_ = nullptr;
            }
        }
        else
        {
            // Get next frame set
            ScFrameReady psReadFrame;
            memset(&psReadFrame, 0x0, sizeof(ScFrameReady));
            ScStatus status =  scGetFrameReady(deviceHandle_, 1200, &psReadFrame);
            if (status != SC_OK)
            {
                RCLCPP_INFO(this->get_logger(), "scGetFrameReady failed! %d" ,status);
            }
            else
            {
                publicImage(psReadFrame);
            }
        }
    }
    return;
}

bool ScepterManager::publicImage(const ScFrameReady& psReadFrame)
{
    if (deviceHandle_ == nullptr)
    {
        return false;
    }

    bool ret = false;
    memset(frameArr, 0,sizeof(ScFrame)*5);
    ScFrame frame;
    memset(&frame, 0x0, sizeof(ScFrame));
    rclcpp::Time current_time = this->get_clock()->now();
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
        std::unique_ptr<sensor_msgs::msg::CameraInfo> cameraInfo = std::make_unique<sensor_msgs::msg::CameraInfo>(cameraInfoArr[i]);
        cameraInfo->header.stamp = current_time;
        ScFrameType type = frame.frameType;
        if (frame.pFrameData != NULL)
        {
            int cvMatType = CV_16UC1;
            std::string imageEncodeType = sensor_msgs::image_encodings::TYPE_16UC1;
            switch (type)
            {
            case SC_IR_FRAME:
            {
                if (cameraSetting_.get_ir_publish() == true && pub != nullptr && pubCameraInfo != nullptr) {
                    cvMatType = CV_8UC1;
                    imageEncodeType = sensor_msgs::image_encodings::TYPE_8UC1;
                    cv::Mat mat = cv::Mat(frame.height, frame.width, cvMatType, frame.pFrameData);
                    cv_bridge::CvImage cvi_;
                    cvi_.header.stamp = current_time;
                    cvi_.header.frame_id = ir_frame;
                    cvi_.encoding = "8UC1";
                    cvi_.image = mat;
                    auto im_msg = std::make_unique<sensor_msgs::msg::Image>(*cvi_.toImageMsg());
                    pub->publish(std::move(im_msg));
                    pubCameraInfo->publish(std::move(cameraInfo));
                }
                ret = true;
            }
                break;
            case SC_DEPTH_FRAME:
            {
                if (cameraSetting_.get_depth_publish() == true && pub != nullptr && pubCameraInfo != nullptr) {
                    cvMatType = CV_16UC1;
                    imageEncodeType = sensor_msgs::image_encodings::TYPE_16UC1;
                    cv::Mat mat = cv::Mat(frame.height, frame.width, cvMatType, frame.pFrameData);
                    cv_bridge::CvImage cvi_;
                    cvi_.header.stamp = current_time;
                    cvi_.header.frame_id = depth_frame;
                    cvi_.encoding = "16UC1";
                    cvi_.image = mat;
                    auto im_msg = std::make_unique<sensor_msgs::msg::Image>(*cvi_.toImageMsg());
                    pub->publish(std::move(im_msg));
                    pubCameraInfo->publish(std::move(cameraInfo));
                }
                if(cameraSetting_.get_depth_cloud_point() == true && pointclound2_pub_ != nullptr && pointclound2info_pub_ != nullptr) {
                    ScFrame &srcFrame = frame;
                    const int len = srcFrame.width * srcFrame.height;
                    ScVector3f* worldV = new ScVector3f[len];
                    scConvertDepthFrameToPointCloudVector(deviceHandle_, &srcFrame, worldV); //Convert Depth frame to World vectors.

                    auto output_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
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
                    pcl::toROSMsg(cloud, *output_msg);
                    output_msg->header.frame_id=points_frame;
                    output_msg->header.stamp = current_time;
                    pointclound2_pub_->publish(std::move(output_msg));
                    pointclound2_info_.header.stamp = current_time;
                    std::unique_ptr<sensor_msgs::msg::CameraInfo> cameraInfoPc = std::make_unique<sensor_msgs::msg::CameraInfo>(pointclound2_info_);
                    pointclound2info_pub_->publish(std::move(cameraInfoPc));
                }
                ret = true;
            }
            break;
            case SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME:
            {
                if (cameraSetting_.get_transformed_depth_publish() == true && pub != nullptr && pubCameraInfo != nullptr)
                {
                    cvMatType = CV_16UC1;
                    imageEncodeType = sensor_msgs::image_encodings::TYPE_16UC1;
                    cv::Mat mat = cv::Mat(frame.height, frame.width, cvMatType, frame.pFrameData);
                    cv_bridge::CvImage cvi_;
                    cvi_.header.stamp = current_time;
                    cvi_.header.frame_id = aligneddepth_frame;
                    cvi_.encoding = "16UC1";
                    cvi_.image = mat;
                    auto im_msg = std::make_unique<sensor_msgs::msg::Image>(*cvi_.toImageMsg());
                    pub->publish(std::move(im_msg));
                    pubCameraInfo->publish(std::move(cameraInfo));
                    if(cameraSetting_.get_depth2color_cloud_point() == true && depth2colorpointclound2_pub_ != nullptr && depth2colorpointclound2info_pub_ != nullptr) // inorder to publish depth2color cloud point must enable TransformedDepthFlag_ first
                    {
                        const int len = frame.width * frame.height;
                        ScVector3f* worldV = new ScVector3f[len];
                        scConvertDepthFrameToPointCloudVector(deviceHandle_, &frame, worldV);
                        auto output_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
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
                        pcl::toROSMsg(cloud, *output_msg);
                        output_msg->header.frame_id = depth2colorpoints_frame;
                        output_msg->header.stamp = current_time;
                        depth2colorpointclound2_pub_->publish(std::move(output_msg));
                        depth2colorpointclound2info.header.stamp = current_time;
                        std::unique_ptr<sensor_msgs::msg::CameraInfo> cameraInfoPc = std::make_unique<sensor_msgs::msg::CameraInfo>(depth2colorpointclound2info);
                        depth2colorpointclound2info_pub_->publish(std::move(cameraInfoPc));
                    }
                }
                ret = true;
            }
                break;
            case SC_COLOR_FRAME:
            {
                if (cameraSetting_.get_color_publish() == true && pub != nullptr && pubCameraInfo != nullptr)
                {
                    cvMatType = CV_8UC3;
                    imageEncodeType = sensor_msgs::image_encodings::BGR8;
                    cv::Mat mat = cv::Mat(frame.height, frame.width, cvMatType, frame.pFrameData);
                    cv_bridge::CvImage cvi_;
                    cvi_.header.stamp = current_time;
                    cvi_.header.frame_id = color_frame;
                    cvi_.encoding = "bgr8";
                    cvi_.image = mat;
                    auto im_msg = std::make_unique<sensor_msgs::msg::Image>(*cvi_.toImageMsg());
                    pub->publish(std::move(im_msg));
                    pubCameraInfo->publish(std::move(cameraInfo));
                }
                ret = true;
            }    
                break;
            case SC_TRANSFORM_COLOR_IMG_TO_DEPTH_SENSOR_FRAME:
            {
                if (cameraSetting_.get_transformed_color_publish() && pub != nullptr && pubCameraInfo != nullptr)
                {
                    cvMatType = CV_8UC3;
                    imageEncodeType = sensor_msgs::image_encodings::BGR8;
                    cv::Mat mat = cv::Mat(frame.height, frame.width, cvMatType, frame.pFrameData);
                    cv_bridge::CvImage cvi_;
                    cvi_.header.stamp = current_time;
                    cvi_.header.frame_id = alignedcolor_frame;
                    cvi_.encoding = "bgr8";
                    cvi_.image = mat;
                    auto im_msg = std::make_unique<sensor_msgs::msg::Image>(*cvi_.toImageMsg());
                    pub->publish(std::move(im_msg));
                    pubCameraInfo->publish(std::move(cameraInfo));
                }
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

void ScepterManager::init_TF_info()
{
    // frame_id
    camera_frame = this->camera_name_ + "_frame";
    depth_frame = this->camera_name_ + "_depth_frame";
    ir_frame = this->camera_name_ + "_ir_frame";
    color_frame = this->camera_name_ + "_color_frame";
    aligneddepth_frame = this->camera_name_ + "_transformedDepth_frame";
    alignedcolor_frame = this->camera_name_ + "_transformedColor_frame";
    points_frame = this->camera_name_ + "_points_frame";
    depth2colorpoints_frame = this->camera_name_ + "_depth2colorpoints_frame";

    ScStatus status = ScStatus::SC_OK;
    status = scGetSensorExtrinsicParameters(deviceHandle_, &this->extrinsics_);
    if (status != ScStatus::SC_OK)
    {
        RCLCPP_INFO(this->get_logger(), "scGetSensorExtrinsicParameters failed status %d", status);
    }
    else
    {
        tf2::Matrix3x3 rotation_matrix(extrinsics_.rotation[0], extrinsics_.rotation[1], extrinsics_.rotation[2],
                                    extrinsics_.rotation[3], extrinsics_.rotation[4], extrinsics_.rotation[5],
                                    extrinsics_.rotation[6], extrinsics_.rotation[7], extrinsics_.rotation[8]);
        double roll, pitch, yaw;
        rotation_matrix.getRPY(roll, pitch, yaw);
        quaternion_.setRPY(roll, pitch, yaw);
        bool intraProcessComms = this->get_node_options().use_intra_process_comms();
        RCLCPP_INFO(this->get_logger(), "Intra-process comms enabled: %s", intraProcessComms ? "YES" : "NO");
        if(intraProcessComms == false)
        {
            static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
            publishStaticTF();
        }
        else
        {
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
            timer_tf_ = this->create_wall_timer(std::chrono::seconds(1), [this]() {this->publishTF();});
        }
    }
    return;
}

void ScepterManager::publishStaticTF()
{
    // Publish static TFs
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = this->get_clock()->now();
    msg.transform.rotation.w = 1.0;

    msg.header.frame_id = camera_frame;
    msg.child_frame_id = points_frame;
    static_tf_broadcaster_->sendTransform(msg);

    msg.header.frame_id = camera_frame;
    msg.child_frame_id = depth2colorpoints_frame;
    static_tf_broadcaster_->sendTransform(msg);

    msg.header.frame_id = camera_frame;
    msg.child_frame_id = ir_frame;
    static_tf_broadcaster_->sendTransform(msg);

    // Camera base to Color Frame
    msg.header.frame_id = camera_frame;
    msg.child_frame_id = color_frame;
    static_tf_broadcaster_->sendTransform(msg);

    // Color Frame to Aligned Frame
    msg.header.frame_id = color_frame;
    msg.child_frame_id = depth_frame;
    static_tf_broadcaster_->sendTransform(msg);
    
    msg.header.frame_id = depth_frame;
    msg.child_frame_id = alignedcolor_frame;
    static_tf_broadcaster_->sendTransform(msg);

    // Color Frame to Depth Frame
    msg.transform.translation.x = extrinsics_.translation[0] / 1000;
    msg.transform.translation.y = extrinsics_.translation[1] / 1000;
    msg.transform.translation.z = extrinsics_.translation[2] / 1000;
    msg.transform.rotation = tf2::toMsg(quaternion_);
    msg.header.frame_id = color_frame;
    msg.child_frame_id = aligneddepth_frame;
    static_tf_broadcaster_->sendTransform(msg);
}

void ScepterManager::publishTF()
{
    // Publish TFs
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = this->get_clock()->now();
    msg.transform.rotation.w = 1.0;

    msg.header.frame_id = camera_frame;
    msg.child_frame_id = points_frame;
    tf_broadcaster_->sendTransform(msg);

    msg.header.frame_id = camera_frame;
    msg.child_frame_id = depth2colorpoints_frame;
    tf_broadcaster_->sendTransform(msg);

    msg.header.frame_id = camera_frame;
    msg.child_frame_id = ir_frame;
    tf_broadcaster_->sendTransform(msg);

    // Camera base to Color Frame
    msg.header.frame_id = camera_frame;
    msg.child_frame_id = color_frame;
    tf_broadcaster_->sendTransform(msg);

    // Color Frame to Aligned Frame
    msg.header.frame_id = color_frame;
    msg.child_frame_id = depth_frame;
    tf_broadcaster_->sendTransform(msg);
    
    msg.header.frame_id = depth_frame;
    msg.child_frame_id = alignedcolor_frame;
    tf_broadcaster_->sendTransform(msg);

    // Color Frame to Depth Frame
    msg.transform.translation.x = extrinsics_.translation[0] / 1000;
    msg.transform.translation.y = extrinsics_.translation[1] / 1000;
    msg.transform.translation.z = extrinsics_.translation[2] / 1000;
    msg.transform.rotation = tf2::toMsg(quaternion_);
    msg.header.frame_id = color_frame;
    msg.child_frame_id = aligneddepth_frame;
    tf_broadcaster_->sendTransform(msg);
}
void ScepterManager::set_sensor_intrinsics() 
{
    // Get camera parameters (intrinsic)
    ScStatus status = ScStatus::SC_OK;
    status = scGetSensorIntrinsicParameters(deviceHandle_, SC_TOF_SENSOR, &this->depth_intrinsics_);
    if (status != ScStatus::SC_OK)
    {
        RCLCPP_INFO(this->get_logger(), "scGetSensorIntrinsicParameters  tof sensor failed status %d", status);
        return;
    }
    status = scGetSensorIntrinsicParameters(deviceHandle_, SC_COLOR_SENSOR, &this->color_intrinsics_);
    if (status != ScStatus::SC_OK)
    {
        RCLCPP_INFO(this->get_logger(), "scGetSensorIntrinsicParameters color sensor failed status %d", status);
        return;
    }

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
    cameraInfoArr[2]=info_msg; // color
    cameraInfoArr[3]=info_msg; // depth2color

    info_msg.header.frame_id = depth2colorpoints_frame;
    depth2colorpointclound2info = info_msg;

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
    
    cameraInfoArr[0]=info_msg; // depth
    cameraInfoArr[4]=info_msg; // color2depth

    info_msg.header.frame_id = ir_frame;
    cameraInfoArr[1]=info_msg; // ir

    info_msg.header.frame_id = points_frame;
    pointclound2_info_ = info_msg;
}

RCLCPP_COMPONENTS_REGISTER_NODE(ScepterManager)
