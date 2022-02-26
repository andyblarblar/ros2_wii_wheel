#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace WiiWheel
{
    class WiiWheelTeleop : public rclcpp::Node
    {
    public:
        explicit WiiWheelTeleop(rclcpp::NodeOptions options);

    private:
        void wii_joy_callback(const sensor_msgs::msg::Joy::ConstSharedPtr msg);    
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr wii_joy_sub;
        rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr wheel_joy_pub;
    };
}
