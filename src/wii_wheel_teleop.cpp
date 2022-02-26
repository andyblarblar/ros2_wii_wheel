#include "../include/wii_wheel/wii_wheel_teleop.hpp"

#include <memory>
#include <functional>

namespace WiiWheel
{
    WiiWheelTeleop::WiiWheelTeleop(rclcpp::NodeOptions options)
        : Node("wii_wheel", options)
    {
        wii_joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
            "/wiimote/joy", 10,
            std::bind(&WiiWheelTeleop::wii_joy_callback, this, std::placeholders::_1));

        wheel_joy_pub = this->create_publisher<sensor_msgs::msg::Joy>(
            "joy", rclcpp::SensorDataQoS());       
    }

    void WiiWheelTeleop::wii_joy_callback(const sensor_msgs::msg::Joy::ConstSharedPtr msg)
    { //axis 2(pitch) is forward, 1(roll) is turn 
      //Button 1 is go, button 3 is reverse

    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;
    auto wheel_node = std::make_shared<WiiWheel::WiiWheelTeleop>(options);
    exec.add_node(wheel_node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
