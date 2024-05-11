#include "rclcpp/rclcpp.hpp"
#include <scepter_manager.hpp>

int main(int argc, char *argv[]) 
{
    rclcpp::init(argc, argv);

    int32_t device_index = 0;
    auto node = std::make_shared<ScepterManager>(device_index,"Scepter");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
