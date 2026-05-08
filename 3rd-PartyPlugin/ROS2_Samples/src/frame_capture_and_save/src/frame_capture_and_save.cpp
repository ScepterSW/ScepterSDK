/*
    This client is used to demo how to subscribe to the raw image(depth/color/ir).
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
using namespace std::chrono_literals;
using namespace std;
stringstream ssm;
static bool isGetRes = false;
static int frameIndex = 0;
static int frameSaveCount = 10;
rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr colorImageSub_;
rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthImageSub_;
rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr irImageSub_;

class FrameCaptureAndSaveNode:public rclcpp::Node {
public:
        FrameCaptureAndSaveNode(std::string nodeName, rclcpp::NodeOptions& node_options):Node(nodeName, node_options.automatically_declare_parameters_from_overrides(true))
        {
            RCLCPP_INFO(this->get_logger(), "FrameCaptureAndSave");
            this->node_name = this->get_parameter("node_name").as_string();
            RCLCPP_INFO(this->get_logger(), "node_name %s " ,this->node_name.c_str());
            asyncParam_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, this->node_name);
        }
        bool connect_server() {
            while(!asyncParam_client_->wait_for_service(1s))//wait for server 1s
            {
                if(!rclcpp::ok())
                {
                    RCLCPP_INFO(rclcpp::get_logger("FrameCaptureAndSave"), "stop client!");
                    return false;
                }
                RCLCPP_INFO(rclcpp::get_logger("FrameCaptureAndSave"), "async connecting server......");
            }
            return true;
        }

        std::string getNodeName() const {return this->node_name;}
        std::string getCameraSN() const {return this->cameraSN;}
        void SetCameraSN(std::string sn) {this->cameraSN = sn;}
        rclcpp::AsyncParametersClient::SharedPtr& getClient() { return this->asyncParam_client_;}
private:
    rclcpp::AsyncParametersClient::SharedPtr asyncParam_client_;
    std::string node_name = "";
    std::string cameraSN = "";
};

void image_callback(const sensor_msgs::msg::Image::SharedPtr img)
{
    if (isGetRes == true)
    {
        return;
    }
    frameIndex++;
    ssm << "./src/frame_capture_and_save/";
    ssm << img->header.frame_id;
    ssm << "_";
    ssm << (frameIndex % frameSaveCount);
    ssm << ".png";
    std::string imageEncodeType = img->encoding;
    cv_bridge::CvImagePtr cv_img = cv_bridge::toCvCopy(img, imageEncodeType);
    if (ssm.str().find("depth_") != std::string::npos)
    {
        cv_img->image.convertTo(cv_img->image, CV_8U, 255.0 / 3000);
        applyColorMap(cv_img->image, cv_img->image, cv::COLORMAP_RAINBOW);
    }
    cv::imwrite(ssm.str(), cv_img->image);
    ssm.clear();
    ssm.str("");
    cv::imshow(img->header.frame_id, cv_img->image);
    //RCLCPP_INFO(rclcpp::get_logger("FrameCaptureAndSave"), "img->header sec %lld , nano %lld ", img->header.stamp.sec, img->header.stamp.nanosec);
    int key = cv::waitKey(10);
    if (key == 27 || key == 81 || key == 113)
    {
        RCLCPP_INFO(rclcpp::get_logger("FrameCaptureAndSave"), "exit subscribe image ");
        isGetRes = true;
        //cv::destroyWindow(img->header.frame_id);
        cv::destroyAllWindows();
    }
    return;
}

void subscribeColorImage(std::shared_ptr<FrameCaptureAndSaveNode>& node)
{
    if (node == nullptr)
    {
        isGetRes = true;
        return;
    }
    RCLCPP_INFO(rclcpp::get_logger("FrameCaptureAndSave"), "subscribeColorImage");
    node->getClient()->set_parameters({
        rclcpp::Parameter("framerate",10),
        rclcpp::Parameter("color_publish",true),
        rclcpp::Parameter("work_mode",0)
    });
#ifdef FOXY
    auto qosVol = rclcpp::QoS(rclcpp::KeepLast(1));
    qosVol.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qosVol.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
#else
    auto qosVol = rclcpp::QoS(rclcpp::KeepLast(1)).durability(rclcpp::DurabilityPolicy::Volatile).reliability(rclcpp::ReliabilityPolicy::Reliable);
#endif
    std::string colorTopic = node->getNodeName() + "/" + node->getCameraSN() + std::string("/color/image_raw");
    colorImageSub_ = node->create_subscription<sensor_msgs::msg::Image>(colorTopic.c_str(), qosVol, image_callback);
    return;
}

void subscribeDepthImage(std::shared_ptr<FrameCaptureAndSaveNode>& node)
{
    if (node == nullptr)
    {
        isGetRes = true;
        return;
    }
    RCLCPP_INFO(rclcpp::get_logger("FrameCaptureAndSave"), "subscribeDepthImage");
    node->getClient()->set_parameters({
        rclcpp::Parameter("framerate",10),
        rclcpp::Parameter("depth_publish",true),
        rclcpp::Parameter("work_mode",0)
    });
#ifdef FOXY
    auto qosVol = rclcpp::QoS(rclcpp::KeepLast(1));
    qosVol.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qosVol.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
#else
    auto qosVol = rclcpp::QoS(rclcpp::KeepLast(1)).durability(rclcpp::DurabilityPolicy::Volatile).reliability(rclcpp::ReliabilityPolicy::Reliable);
#endif
    std::string depthTopic = node->getNodeName() + "/" + node->getCameraSN() + std::string("/depth/image_raw");
    colorImageSub_ = node->create_subscription<sensor_msgs::msg::Image>(depthTopic.c_str(), qosVol, image_callback);
    return;
}

void subscribeIrImage(std::shared_ptr<FrameCaptureAndSaveNode>& node)
{
    if (node == nullptr)
    {
        isGetRes = true;
        return;
    }

    RCLCPP_INFO(rclcpp::get_logger("FrameCaptureAndSave"), "subscriIrbeImage");
    node->getClient()->set_parameters({
        rclcpp::Parameter("framerate",10),
        rclcpp::Parameter("ir_publish",true),
        rclcpp::Parameter("work_mode",0)
    });
#ifdef FOXY
    auto qosVol = rclcpp::QoS(rclcpp::KeepLast(1));
    qosVol.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qosVol.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
#else
    auto qosVol = rclcpp::QoS(rclcpp::KeepLast(1)).durability(rclcpp::DurabilityPolicy::Volatile).reliability(rclcpp::ReliabilityPolicy::Reliable);
#endif
    std::string irTopic = node->getNodeName() + "/" + node->getCameraSN() + std::string("/ir/image_raw");
    colorImageSub_ = node->create_subscription<sensor_msgs::msg::Image>(irTopic.c_str(), qosVol, image_callback);
    return;
}

void clearSubscribes()
{
    if (colorImageSub_.get() != nullptr)
    {
        colorImageSub_.reset();
    }
    if (depthImageSub_.get() != nullptr)
    {
        depthImageSub_.reset();
    }
    if(irImageSub_.get() != nullptr)
    {
        irImageSub_.reset();
    }
}

bool getCameraSN(std::shared_ptr<FrameCaptureAndSaveNode>& node)
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
    RCLCPP_INFO(rclcpp::get_logger("FrameCaptureAndSave"), "You can exit the display window through press Q/q key or ESC key in the display window");
    RCLCPP_INFO(rclcpp::get_logger("FrameCaptureAndSave"), "Please press a key to run an example: ");
    RCLCPP_INFO(rclcpp::get_logger("FrameCaptureAndSave"), "<I/i>: subcribe color raw image ");
    RCLCPP_INFO(rclcpp::get_logger("FrameCaptureAndSave"), "<D/d>: subcribe depth raw image");
    RCLCPP_INFO(rclcpp::get_logger("FrameCaptureAndSave"), "<C/c>: subcribe color raw image");
    RCLCPP_INFO(rclcpp::get_logger("FrameCaptureAndSave"), "<Q/q>: exit");
    RCLCPP_INFO(rclcpp::get_logger("FrameCaptureAndSave"), "Your choice: ");
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    auto node = std::make_shared<FrameCaptureAndSaveNode>("FrameCaptureAndSave", node_options);
    bool flag = node->connect_server();
    if(!flag)
    {
        RCLCPP_INFO(rclcpp::get_logger("FrameCaptureAndSave"), "connect server failed");
        return -1;
    }
    bool ret = getCameraSN(node);
    if (ret == false)
    {
        RCLCPP_INFO(rclcpp::get_logger("FrameCaptureAndSave"), "getCameraSN failed");
        return -1;
    }
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::unordered_map<char, std::function<void(std::shared_ptr<FrameCaptureAndSaveNode>& node)>> functionMap;
    functionMap['I'] = subscribeIrImage;
    functionMap['D'] = subscribeDepthImage;
    functionMap['C'] = subscribeColorImage;

    char input;
    bool running = true;
    rclcpp::Rate loop_rate(500);
    while (running) {
        clearSubscribes();
        showMenu();
        std::cin >> input;
        input = toupper(input);
        isGetRes = false;
        if (functionMap.find(input) != functionMap.end()) {
            functionMap[input](node);
        } else if (input == 'Q') {
            running = false;
            RCLCPP_INFO(rclcpp::get_logger("FrameCaptureAndSave"), "Exiting program. Goodbye!");
            break;
        } else {
            RCLCPP_INFO(rclcpp::get_logger("FrameCaptureAndSave"), "Invalid input, please try again.");
            continue;
        }
        while(isGetRes == false)
        {
            executor.spin_some();
            loop_rate.sleep();
        }
    }
    clearSubscribes();
    rclcpp::shutdown();
    return 0;
}
