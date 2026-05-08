#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include "thread"
using namespace std::chrono_literals;
#ifdef _WIN32
    #include <sys/timeb.h>
#else
    #include <time.h>
#endif
#include <fstream>
using namespace std;
static uint64_t endTimestamp   = 0;
static uint64_t startTimestamp = 0;
static uint64_t apiInterval    = 0;
ofstream csvWriter;
static uint64_t frameIndex = 0;
static bool isGetImg = false;
uint64_t        getCurrentTIme()
{
#ifdef _WIN32
    timeb timeInfo;
    ftime(&timeInfo);
    return (timeInfo.time * 1000 + timeInfo.millitm);
#else
    timespec timeInfo;
    clock_gettime(CLOCK_REALTIME, &timeInfo);
    return (timeInfo.tv_sec * 1000 + timeInfo.tv_nsec / 1000000);
#endif
}
void timeBegin()
{
    startTimestamp = getCurrentTIme();
    RCLCPP_INFO(rclcpp::get_logger("ScepterSingleFrameDelayTest"), "startTimestamp %ld", startTimestamp);
    isGetImg = false;
}

void timeEnd()
{
    endTimestamp = getCurrentTIme();
    frameIndex++;
    apiInterval  = (endTimestamp - startTimestamp);
    csvWriter << frameIndex << "," << apiInterval << endl;
    RCLCPP_INFO(rclcpp::get_logger("ScepterSingleFrameDelayTest"), "endTimestamp %ld apiInterval %ld", endTimestamp, apiInterval);
}

void image_callback(const sensor_msgs::msg::Image::SharedPtr img)
{
    timeEnd();
    isGetImg = true;
    (void)img; // handle the image here if necessary.
    return;
}

class ScepterSingleFrameDelayTest:public rclcpp::Node{
public:
        ScepterSingleFrameDelayTest(std::string nodeName, rclcpp::NodeOptions& node_options):Node(nodeName, node_options.automatically_declare_parameters_from_overrides(true))
        {
            RCLCPP_INFO(this->get_logger(),"ScepterSingleFrameDelayTest");
            this->node_name = this->get_parameter("node_name").as_string();
            RCLCPP_INFO(this->get_logger(), "node_name %s " ,this->node_name.c_str());
            asyncParam_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, this->node_name);
        }
        bool connect_server(){
            while(!asyncParam_client_->wait_for_service(1s))//wait for server 1s
            {
                if(!rclcpp::ok())
                {
                    RCLCPP_INFO(rclcpp::get_logger("ScepterSingleFrameDelayTest"), "stop client!");
                    return false;
                }
                RCLCPP_INFO(rclcpp::get_logger("ScepterSingleFrameDelayTest"), "async connecting server......");
            }
            return true;
        }

        std::string getNodeName() {return this->node_name;}
        std::string getCameraSN() const {return this->camera_sn;}
        void SetCameraSN(std::string sn) {this->camera_sn = sn;}
        rclcpp::AsyncParametersClient::SharedPtr& getClient() { return this->asyncParam_client_;}
    rclcpp::AsyncParametersClient::SharedPtr asyncParam_client_;
    std::string node_name = "";
    std::string camera_sn = "";
};

bool getCameraSN(std::shared_ptr<ScepterSingleFrameDelayTest>& node)
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
    auto node = std::make_shared<ScepterSingleFrameDelayTest>("ScepterSingleFrameDelayTest", node_options);
    bool flag = node->connect_server();
    if(!flag)
    {
        RCLCPP_INFO(rclcpp::get_logger("ScepterSingleFrameDelayTest"), "connect server failed");
        return -1;
    }
    bool ret = getCameraSN(node);
    if (ret == false)
    {
        RCLCPP_INFO(rclcpp::get_logger("ScepterSingleFrameDelayTest"), "getCameraSN failed");
        return -1;
    }
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    string fileName = "";
    csvWriter.open(fileName.append("SingleFrameDelayTest.csv"));
    if (!csvWriter.is_open())
    {
        RCLCPP_INFO(rclcpp::get_logger("ScepterSingleFrameDelayTest"), "csv file open failed");
        return -1;
    }
    csvWriter << "frameIndex,TotalDelay" << endl;
    node->asyncParam_client_->set_parameters({
        rclcpp::Parameter("work_mode",2)
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subhandle_;
    std::string depthTopic = node->getNodeName() + "/" + node->getCameraSN() + std::string("/depth/image_raw");
    RCLCPP_INFO(rclcpp::get_logger("ScepterSingleFrameDelayTest"), "depthTopic %s", depthTopic.c_str());
    #ifdef FOXY
        auto qosVol = rclcpp::QoS(rclcpp::KeepLast(1));
        qosVol.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        qosVol.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    #else
        auto qosVol = rclcpp::QoS(rclcpp::KeepLast(1)).durability(rclcpp::DurabilityPolicy::Volatile).reliability(rclcpp::ReliabilityPolicy::Reliable);
    #endif
    subhandle_ = node->create_subscription<sensor_msgs::msg::Image>(depthTopic.c_str(), qosVol, image_callback);
    rclcpp::Rate loop_rate(1000);
	uint32_t testCnt = 0;
    RCLCPP_INFO(rclcpp::get_logger("ScepterSingleFrameDelayTest"), "Please input the number of tests: ");
	cin >> testCnt;
    for (int i = 0; i < (int)testCnt; i++)
    {
        timeBegin();
        node->asyncParam_client_->set_parameters({rclcpp::Parameter("SingleFrameDelayTest", (double)startTimestamp)});
        while (isGetImg == false)
        {
            executor.spin_some();
            loop_rate.sleep();
        }
    }
    auto future = node->asyncParam_client_->set_parameters({rclcpp::Parameter("work_mode",0)});
    auto status =  executor.spin_until_future_complete(future, std::chrono::seconds(5));
    if (status == rclcpp::FutureReturnCode::SUCCESS) {
        //auto response = future.get();
        RCLCPP_INFO(node->get_logger(), "Response received, work mode update done.");
    } else {
        RCLCPP_WARN(node->get_logger(), "Request timed out or node shutdown");
    }
    RCLCPP_INFO(node->get_logger(), "Test done.");
    rclcpp::shutdown();
    return 0;
}
