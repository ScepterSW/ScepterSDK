/*
    This client is used to demo how to set software trigger mode and subscribe to the triggered image.
*/
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sstream>
#include <unordered_map>
#include <functional>
#include <opencv2/opencv.hpp>
#ifdef JAZZY
  #include <cv_bridge/cv_bridge.hpp>
#else
  #include <cv_bridge/cv_bridge.h>
#endif
#include <vector>
#include <thread>
#include <map>
#include <iostream>
using namespace std::chrono_literals;
using namespace std;
class DeviceSWTriggerModeNode;
stringstream ssm;
static bool isGetRes = false;
static int frameIndex = 0;
const char* winName = "Dynamic Window";
static bool subFlag = false;
rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr softwareTriggerSub_;

class DeviceSWTriggerModeNode:public rclcpp::Node {
public:
        DeviceSWTriggerModeNode(std::string nodeName, rclcpp::NodeOptions& node_options):Node(nodeName, node_options.automatically_declare_parameters_from_overrides(true))
        {
            RCLCPP_INFO(this->get_logger(), "DeviceSWTriggerMode");
            this->node_name = this->get_parameter("node_name").as_string();
            RCLCPP_INFO(this->get_logger(), "node_name %s " ,this->node_name.c_str());
            asyncParam_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, this->node_name);
        }
        bool connect_server() {
            while(!asyncParam_client_->wait_for_service(1s))//wait for server 1s
            {
                if(!rclcpp::ok())
                {
                    RCLCPP_INFO(rclcpp::get_logger("DeviceSWTriggerMode"), "stop client!");
                    return false;
                }
                RCLCPP_INFO(rclcpp::get_logger("DeviceSWTriggerMode"), "async connecting server......");
            }
            return true;
        }
        std::string getNodeName() const {return this->node_name;}
        std::string getCameraSN() const {return this->camera_sn;}
        void SetCameraSN(std::string sn) {this->camera_sn = sn;}
        rclcpp::AsyncParametersClient::SharedPtr& getClient() { return this->asyncParam_client_;}
private:
    rclcpp::AsyncParametersClient::SharedPtr asyncParam_client_;
    std::string node_name = "";
    std::string camera_sn = "";
};

void softwareTrigger_image_callback(const sensor_msgs::msg::Image::SharedPtr img)
{
    frameIndex++;
    ssm.clear();
    ssm.str("");
    std::string frameId = img->header.frame_id;
    std::string imageEncodeType = img->encoding;
    cv_bridge::CvImagePtr cv_img = cv_bridge::toCvCopy(img, imageEncodeType);
    ssm << "./src/device_sw_trigger_mode/";
    ssm << frameId;
    ssm << "_softwareTrigger_";
    ssm << frameIndex;
    ssm << ".png";
    RCLCPP_INFO(rclcpp::get_logger("DeviceSWTriggerMode"), "save software triggered image %s", ssm.str().c_str());
    cv::imwrite(ssm.str(), cv_img->image);
    cv::imshow(winName, cv_img->image);
    return;
}

void setWorkMode(std::shared_ptr<DeviceSWTriggerModeNode>& node)
{
    if (node == nullptr)
    {
        isGetRes = true;
        return;
    }
    RCLCPP_INFO(rclcpp::get_logger("DeviceSWTriggerMode"), "softwareTrigger");
    auto future = node->getClient()->set_parameters({
        rclcpp::Parameter("depth_publish",true),
        rclcpp::Parameter("ir_publish",true),
        rclcpp::Parameter("color_publish",true),
        rclcpp::Parameter("work_mode",2)
    });
    return;
}

bool getCameraSN(std::shared_ptr<DeviceSWTriggerModeNode>& node)
{
    std::vector<std::string> param_names = {"camera_sn"};
    auto future = node->getClient()->get_parameters(param_names);
    auto status =  rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(5));
    if (status == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        RCLCPP_INFO(node->get_logger(), "Response received param %s val %s", response[0].get_name().c_str(), response[0].value_to_string().c_str());
        node->SetCameraSN(response[0].value_to_string());
    } else {
        RCLCPP_WARN(node->get_logger(), "Failed to get camera_sn");
        return false;
    }
    return true;
}

void showMenu() {
    RCLCPP_INFO(rclcpp::get_logger("DeviceSWTriggerMode"), "Please choose which topic you want to subscribe: ");
    RCLCPP_INFO(rclcpp::get_logger("DeviceSWTriggerMode"), "<I/i>: subcribe color raw image ");
    RCLCPP_INFO(rclcpp::get_logger("DeviceSWTriggerMode"), "<D/d>: subcribe depth raw image");
    RCLCPP_INFO(rclcpp::get_logger("DeviceSWTriggerMode"), "<C/c>: subcribe color raw image");
    RCLCPP_INFO(rclcpp::get_logger("DeviceSWTriggerMode"), "<Q/q>: exit");
    RCLCPP_INFO(rclcpp::get_logger("DeviceSWTriggerMode"), "Your choice: ");
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    auto node = std::make_shared<DeviceSWTriggerModeNode>("DeviceSWTriggerMode", node_options);
    bool flag = node->connect_server();
    if(!flag)
    {
        RCLCPP_INFO(rclcpp::get_logger("DeviceSWTriggerMode"), "connect server failed");
        return -1;
    }
    bool ret = getCameraSN(node);
    if (ret == false)
    {
        RCLCPP_INFO(rclcpp::get_logger("DeviceSWTriggerMode"), "getCameraSN failed");
        return -1;
    }
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    cv::namedWindow(winName, cv::WINDOW_AUTOSIZE);
    cv::Mat blank_image(480, 480, CV_8UC3, cv::Scalar(255, 255, 255));
    std::string text = "Press <SPACE> KEY to trigger once.";
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 0.5;
    int thickness = 1;
    cv::Scalar color(0, 0, 255);
    cv::Point textOrg(10, 30); 
    cv::putText(blank_image, text, textOrg, fontFace, fontScale, color, thickness);
    text = "Press <ESC> KEY to quit.";
    textOrg.x = 10;
    textOrg.y = 50;
    cv::putText(blank_image, text, textOrg, fontFace, fontScale, color, thickness);
    setWorkMode(node);

    std::map<char, std::string> topicMap = {{'D',"depth"}, {'I',"ir"}, {'C',"color"}};
    bool running = true;
    char input;
    while(running)
    {
        showMenu();
        std::cin >> input;
        input = toupper(input);
        if (input == 'Q')
        {
            running = false;
            auto future = node->getClient()->set_parameters({rclcpp::Parameter("work_mode",0)});
            auto status =  executor.spin_until_future_complete(future, std::chrono::seconds(5));
            if (status == rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_INFO(node->get_logger(), "Response received");
            } else {
                auto response = future.get();
                RCLCPP_WARN(node->get_logger(), "Failed to set work_mode as %s",response[0].reason.c_str());
            }
            RCLCPP_INFO(rclcpp::get_logger("DeviceSWTriggerMode"), "exit software trigger");
            break;
        }
        else
        {
            if (topicMap.find(input) != topicMap.end())
            {
                RCLCPP_INFO(rclcpp::get_logger("DeviceSWTriggerMode"), "subscribe camera topic type %s", topicMap[input].c_str());
                cv::imshow(winName, blank_image);
            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("DeviceSWTriggerMode"), "Your input %c is invalid, please check your input.", input);
                continue;
            }

            isGetRes = false;
            while(isGetRes == false) {
                executor.spin_some();
                int key = cv::waitKey(0);
                if (key == 27 || key == 81 || key == 113 || key == -1)
                {
                    isGetRes = true;
                    subFlag = false;
                    softwareTriggerSub_.reset();
                    //cv::destroyWindow(winName);
                    cv::destroyAllWindows();
                    break;
                }
                else if (key == 32)
                {
                    RCLCPP_INFO(rclcpp::get_logger("DeviceSWTriggerMode"), "trigger once");
                    if (subFlag == false)
                    {
                    #ifdef FOXY
                        auto qosVol = rclcpp::QoS(rclcpp::KeepLast(1));
                        qosVol.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
                        qosVol.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
                    #else
                        auto qosVol = rclcpp::QoS(rclcpp::KeepLast(1)).durability(rclcpp::DurabilityPolicy::Volatile).reliability(rclcpp::ReliabilityPolicy::Reliable);
                    #endif
                        std::string colorTopic =  node->getNodeName() + "/" + node->getCameraSN() + "/" + topicMap[input] + "/image_raw";
                        softwareTriggerSub_ = node->create_subscription<sensor_msgs::msg::Image>(colorTopic.c_str(), qosVol, softwareTrigger_image_callback);
                        subFlag = true;
                    }
                    auto future = node->getClient()->set_parameters({rclcpp::Parameter("software_trigger", true)});
                    auto status =  executor.spin_until_future_complete(future, std::chrono::seconds(5));
                    if (status == rclcpp::FutureReturnCode::SUCCESS) {
                        RCLCPP_INFO(node->get_logger(), "Response received");
                    } else {
                        auto response = future.get();
                        RCLCPP_WARN(node->get_logger(), "Failed to set work_mode as %s",response[0].reason.c_str());
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    continue;
                }
                else
                {
                    RCLCPP_INFO(rclcpp::get_logger("DeviceSWTriggerMode"), "press Space key to trigger again or press Q/q/Esc key to exit.");
                    continue;
                }
            }
        }
    }
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
