#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joy_feedback_array.hpp"

namespace WiiWheel
{
    class WiiWheelTeleop : public rclcpp::Node
    {
    public:
        explicit WiiWheelTeleop(rclcpp::NodeOptions options);
        /// Constant linear velocity
        float velocity{1};
        /// Max angular velocity
        float max_turn{3};
        /// Value we divide the raw accelerometer value by
        float steering_ratio{3};

    private:
        void wii_joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);    
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr wii_joy_sub;
        rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr wheel_joy_pub;

        /// Publishes rumble and LED feeback.
        rclcpp::Publisher<sensor_msgs::msg::JoyFeedbackArray>::SharedPtr joy_feedback_pub;

        /// If a zeroed joy message was sent after buttons stoped being pressed.
        bool sent_zero_msg{false};

        /// Publishes the next joy feedback command for when we are changing speed.
        void send_next_vibrate_cmd(bool accell) {
            static uint8_t last_led{}; // Cannot be negative as per call site. 

            last_led += accell ? 1 : -1;

            sensor_msgs::msg::JoyFeedbackArray msg{};

            sensor_msgs::msg::JoyFeedback led1;
            sensor_msgs::msg::JoyFeedback led2;
            sensor_msgs::msg::JoyFeedback led3;
            sensor_msgs::msg::JoyFeedback led4;
            sensor_msgs::msg::JoyFeedback rum;

            led1.type = led1.TYPE_LED;
            led1.intensity = 0;
            led1.id = 0;
            led2.type = led1.TYPE_LED;
            led2.intensity = 0;
            led2.id = 1;
            led3.type = led1.TYPE_LED;
            led3.intensity = 0;
            led3.id = 2;
            led4.type = led1.TYPE_LED;
            led4.intensity = 0;
            led4.id = 3;
            rum.type = rum.TYPE_RUMBLE;
            rum.intensity = 1;
            rum.id = 0;

            // Rotate through leds to show what way you are changing
            msg.array = {led1, led2, led3, led4, rum};
            msg.array[last_led % 4].intensity = 1;

            joy_feedback_pub->publish(msg);
        }
        
        /// Clears rumble activity.
        void send_clear_vibrate_cmd() {
            sensor_msgs::msg::JoyFeedbackArray msg{};
            sensor_msgs::msg::JoyFeedback rum;

            rum.type = rum.TYPE_RUMBLE;
            rum.intensity = 0;
            rum.id = 0;

            msg.array = {rum};

            joy_feedback_pub->publish(msg);
        }
    };
}
