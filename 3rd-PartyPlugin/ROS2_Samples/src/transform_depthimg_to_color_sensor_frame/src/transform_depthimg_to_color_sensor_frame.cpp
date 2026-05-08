/*
    This client is used to demo how to subscribe to the transformed depth image.
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

rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr transformedDepthImageSub_;

class TransformDepthImgToColorSensorFrameNode:public rclcpp::Node {
public:
        TransformDepthImgToColorSensorFrameNode(std::string nodeName, rclcpp::NodeOptions& node_options):Node(nodeName, node_options.automatically_declare_parameters_from_overrides(true))
        {
            RCLCPP_INFO(this->get_logger(), "TransformDepthImgToColorSensorFrame");
            this->node_name = this->get_parameter("node_name").as_string();
            RCLCPP_INFO(this->get_logger(), "node_name %s " ,this->node_name.c_str());
            asyncParam_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, this->node_name);
        }
        bool connect_server() {
            while(!asyncParam_client_->wait_for_service(1s))//wait for server 1s
            {
                if(!rclcpp::ok())
                {
                    RCLCPP_INFO(rclcpp::get_logger("TransformDepthImgToColorSensorFrame"), "stop client!");
                    return false;
                }
                RCLCPP_INFO(rclcpp::get_logger("TransformDepthImgToColorSensorFrame"), "async connecting server......");
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
    frameIndex++;
    ssm << "./src/transform_depthimg_to_color_sensor_frame/";
    ssm << img->header.frame_id;
    ssm << "_";
    ssm << (frameIndex % 10);
    ssm << ".png";
    std::string imageEncodeType = img->encoding;
    cv_bridge::CvImagePtr cv_img = cv_bridge::toCvCopy(img, imageEncodeType);
    cv::imwrite(ssm.str(), cv_img->image);
    ssm.clear();
    ssm.str("");
    cv::imshow(img->header.frame_id, cv_img->image);
    int key = cv::waitKey(10);
    if (key == 27 || key == 81 || key == 113)
    {
        RCLCPP_INFO(rclcpp::get_logger("TransformDepthImgToColorSensorFrame"), "exit subscribe image ");
        isGetRes = true;
        cv::destroyWindow(img->header.frame_id);
    }
    return;
}

void subscribeTransformedDepthImage(std::shared_ptr<TransformDepthImgToColorSensorFrameNode>& node)
{
    if (node == nullptr)
    {
        isGetRes = true;
        return;
    }
    RCLCPP_INFO(rclcpp::get_logger("TransformDepthImgToColorSensorFrame"), "subscribeTransformedDepthImage");
    node->getClient()->set_parameters({
        rclcpp::Parameter("framerate",10),
        rclcpp::Parameter("transformed_depth",true),
        rclcpp::Parameter("work_mode",0)
    });
    std::string transformedDepthTopic = node->getNodeName() + "/" + node->getCameraSN() + std::string("/transformedDepth/image_raw");
#ifdef FOXY
    auto qosVol = rclcpp::QoS(rclcpp::KeepLast(1));
    qosVol.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qosVol.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
#else
    auto qosVol = rclcpp::QoS(rclcpp::KeepLast(1)).durability(rclcpp::DurabilityPolicy::Volatile).reliability(rclcpp::ReliabilityPolicy::Reliable);
#endif
    transformedDepthImageSub_ = node->create_subscription<sensor_msgs::msg::Image>(transformedDepthTopic.c_str(), qosVol, image_callback);
    return;
}

bool getCameraSN(std::shared_ptr<TransformDepthImgToColorSensorFrameNode>& node)
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

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    auto node = std::make_shared<TransformDepthImgToColorSensorFrameNode>("TransformDepthImgToColorSensorFrame", node_options);
    bool flag = node->connect_server();
    if(!flag)
    {
        RCLCPP_INFO(rclcpp::get_logger("TransformDepthImgToColorSensorFrame"), "connect server failed");
        return -1;
    }
    rclcpp::Rate loop_rate(500);
    bool ret = getCameraSN(node);
    if (ret == false)
    {
        RCLCPP_INFO(rclcpp::get_logger("TransformDepthImgToColorSensorFrame"), "getCameraSN failed");
        return -1;
    }
    isGetRes = false;
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    subscribeTransformedDepthImage(node);
    while(isGetRes == false)
    {
        executor.spin_some();
        loop_rate.sleep();
    }
    transformedDepthImageSub_.reset();
    rclcpp::shutdown();
    return 0;
}
