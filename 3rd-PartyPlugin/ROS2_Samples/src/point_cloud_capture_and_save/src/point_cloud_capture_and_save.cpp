/*
    This client is used to demo how to subscribe to the depth cloud point.
*/
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <sstream>
#include <unordered_map>
#include <functional>
#ifdef HUMBLE
#include <pcl/common/io.h>
#else
#include <pcl/io/io.h>
#endif
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <memory>
using namespace std;
using namespace std::chrono_literals;
stringstream ssm;
static uint64_t pointCloudIndex = 0;
static uint64_t pointCloudSaveCnt = 10;
static bool isGetRes = false;

rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depthCloudSub_;

class PointCloudCaptureAndSaveNode:public rclcpp::Node {
public:
        PointCloudCaptureAndSaveNode(std::string nodeName, rclcpp::NodeOptions& node_options):Node(nodeName, node_options.automatically_declare_parameters_from_overrides(true))
        {
            RCLCPP_INFO(this->get_logger(), "PointCloudCaptureAndSave");
            this->node_name = this->get_parameter("node_name").as_string();
            RCLCPP_INFO(this->get_logger(), "node_name %s " ,this->node_name.c_str());
            asyncParam_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, this->node_name);
        }
        bool connect_server() {
            while(!asyncParam_client_->wait_for_service(1s))//wait for server 1s
            {
                if(!rclcpp::ok())
                {
                    RCLCPP_INFO(rclcpp::get_logger("PointCloudCaptureAndSave"), "stop client!");
                    return false;
                }
                RCLCPP_INFO(rclcpp::get_logger("PointCloudCaptureAndSave"), "async connecting server......");
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

void cloudPoint_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloudPoint)
{
    pointCloudIndex++;
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> pclCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::fromROSMsg(*cloudPoint, *pclCloud);
    ssm << "./src/point_cloud_capture_and_save/";
    ssm << cloudPoint->header.frame_id;
    ssm << "_";
    ssm << (pointCloudIndex % pointCloudSaveCnt);
    ssm << ".pcd";
    pcl::io::savePCDFileASCII(ssm.str().c_str(), *pclCloud);
    ssm.clear();
    ssm.str("");
    if (pointCloudIndex > pointCloudSaveCnt)
    {
        RCLCPP_INFO(rclcpp::get_logger("PointCloudCaptureAndSave"), "exit point cloud");
        isGetRes = true;
    }
    return;
}

void subscribeDepthCloudPoint(std::shared_ptr<PointCloudCaptureAndSaveNode>& node)
{
    if (node == nullptr)
    {
        isGetRes = true;
        return;
    }
    RCLCPP_INFO(rclcpp::get_logger("PointCloudCaptureAndSave"), "subscribeDepthCloudPoint");
    node->getClient()->set_parameters({
        rclcpp::Parameter("framerate",10),
        rclcpp::Parameter("depth_cloud_point",true),
        rclcpp::Parameter("work_mode",0)
    });
    std::string depthCloudPointTopic = node->getNodeName() + "/" + node->getCameraSN() + std::string("/depth/points");
#ifdef FOXY
    auto qosVol = rclcpp::QoS(rclcpp::KeepLast(1));
    qosVol.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qosVol.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
#else
    auto qosVol = rclcpp::QoS(rclcpp::KeepLast(1)).durability(rclcpp::DurabilityPolicy::Volatile).reliability(rclcpp::ReliabilityPolicy::Reliable);
#endif
    depthCloudSub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(depthCloudPointTopic.c_str(), qosVol, cloudPoint_callback);
    return;
}

bool getCameraSN(std::shared_ptr<PointCloudCaptureAndSaveNode>& node)
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
    auto node = std::make_shared<PointCloudCaptureAndSaveNode>("PointCloudCaptureAndSave", node_options);
    bool flag = node->connect_server();
    if(!flag)
    {
        RCLCPP_INFO(rclcpp::get_logger("PointCloudCaptureAndSave"), "connect server failed");
        return -1;
    }
    rclcpp::Rate loop_rate(500);
    bool ret = getCameraSN(node);
    if (ret == false)
    {
        RCLCPP_INFO(rclcpp::get_logger("PointCloudCaptureAndSave"), "getCameraSN failed");
        return -1;
    }
    isGetRes = false;
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    subscribeDepthCloudPoint(node);
    while(isGetRes == false)
    {
        executor.spin_some();
        loop_rate.sleep();
    }
    depthCloudSub_.reset();
    rclcpp::shutdown();
    return 0;

}
