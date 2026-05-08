#include <ros/ros.h>
#include <ros/master.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/StrParameter.h>
#include <dynamic_reconfigure/Config.h>
#include <thread>
#include <fstream>
#ifdef _WIN32
    #include <sys/timeb.h>
#else
    #include <time.h>
#endif
using namespace std;
static uint64_t endTimestamp   = 0;
static uint64_t startTimestamp = 0;
static uint64_t apiInterval    = 0;
static bool isGetImg = false;
ofstream csvWriter;
static uint64_t frameIndex = 0;
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
    isGetImg = false;
    startTimestamp = getCurrentTIme();
    ROS_INFO_STREAM("timeBegin " << startTimestamp);
}

void timeEnd()
{
    endTimestamp = getCurrentTIme();
    frameIndex++;
    apiInterval  = (endTimestamp - startTimestamp);
    csvWriter << frameIndex << "," << apiInterval << endl;
    ROS_INFO_STREAM("endTimestamp " << endTimestamp << " apiInterval " << apiInterval);
}
void imageCallback(const sensor_msgs::ImageConstPtr& tem_msg)
{
    timeEnd();
    isGetImg = true;
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "singleFrameDelayTestClient");
    ros::NodeHandle nh;

    std::vector<std::string> nodes;
    ros::master::getNodes(nodes);

    ros::ServiceClient client;

    bool isNodeExist = false;
    for (const auto& node : nodes) {
        if (node.compare("/scepter_manager") == 0)
        {
            isNodeExist = true;
            break;
        }
    }

    if (isNodeExist == false)
    {
        ROS_INFO_STREAM("cann't find scepter_manager node ");
        return -1;
    }

    string fileName = "";
    csvWriter.open(fileName.append("SingleFrameDelayTest.csv"));
    if (!csvWriter.is_open())
    {
        ROS_INFO_STREAM("csv file open failed");
        return -1;
    }
    csvWriter << "frameIndex,TotalDelay" << endl;

    std::string service_name = "/scepter_manager/set_parameters";
    client = nh.serviceClient<dynamic_reconfigure::Reconfigure>(service_name);
    dynamic_reconfigure::Reconfigure srv;
    dynamic_reconfigure::Config config;

    dynamic_reconfigure::IntParameter FrameRate;
    FrameRate.name = "FrameRate";
    FrameRate.value = 30;
    config.ints.push_back(FrameRate); 

    dynamic_reconfigure::IntParameter  WorkMode;
    WorkMode.name = "WorkMode";
    WorkMode.value = 2;
    config.ints.push_back(WorkMode); 
    dynamic_reconfigure::IntParameter  ColorResloution;
    ColorResloution.name = "ColorResloution";
    ColorResloution.value = 0;
    config.ints.push_back(ColorResloution);
    srv.request.config = config;
    if (client.call(srv))
    {
        ROS_INFO_STREAM("Set parameters for node succeeded.");
    }
    else
    {
        ROS_WARN_STREAM("Failed to set parameters for node");
        return -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/Scepter/depth/image_raw", FrameRate.value, imageCallback);
    ros::Rate loop_rate(1000); // interval = 1000 / 100
    dynamic_reconfigure::BoolParameter SoftwareTrigger;
    SoftwareTrigger.name = "SoftwareTrigger";
    SoftwareTrigger.value = true;
    config.ints.clear();
    config.bools.push_back(SoftwareTrigger); 
    srv.request.config = config;
    ROS_INFO_STREAM("Please input the number of tests: ");
    uint32_t testCnt = 0;
    cin >> testCnt;
    for (int i = 0; i < testCnt; i++)
    {
        timeBegin();
        if (client.call(srv))
        {
            while(isGetImg == false)
            {
                ros::spinOnce(); // execute callback queue once then exit
                loop_rate.sleep();
            }
        }
        else
        {
            ROS_WARN_STREAM("Failed to set software trigger parameters for node ");
            break;
        }
    }
    csvWriter.close();
    config.ints.clear();
    config.bools.clear();
    WorkMode.name = "WorkMode";
    WorkMode.value = 0;
    config.ints.push_back(WorkMode);
    SoftwareTrigger.name = "SoftwareTrigger";
    SoftwareTrigger.value = false;
    config.bools.push_back(SoftwareTrigger); 
    srv.request.config = config;
    client.call(srv);
    return 0;
}
