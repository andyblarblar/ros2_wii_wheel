#include "../include/wii_wheel/wii_wheel_teleop.hpp"

#include <algorithm>
#include <memory>
#include <functional>
#include <rclcpp/logging.hpp>

namespace WiiWheel
{
    WiiWheelTeleop::WiiWheelTeleop(rclcpp::NodeOptions options)
        : Node("wii_wheel", options)
    {
        wii_joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
            "/wiimote/joy", 10,
            std::bind(&WiiWheelTeleop::wii_joy_callback, this, std::placeholders::_1));

        wheel_joy_pub = this->create_publisher<sensor_msgs::msg::Joy>(
            "joy", rclcpp::SystemDefaultsQoS());       
    }

    void WiiWheelTeleop::wii_joy_callback(const sensor_msgs::msg::Joy::ConstSharedPtr msg)
    { 
      //axis 1(pitch) is forward, 0(roll) is turn 
      //Button 1 is go, button 3 is reverse

      sensor_msgs::msg::Joy msg_new {*msg};

      // If 2 is held down, then accelerate
      if (msg_new.buttons[1]) {
        //Get roll as pitch (controller is horizontal)
        std::swap(msg_new.axes[0], msg_new.axes[1]);
        //Set pitch to a constant value bc 2 is held down
        msg_new.axes[1] = 1.;

        //Clamp turn to ensure we dont spin out
        msg_new.axes[0] = std::clamp(msg_new.axes[0]/5.f, -3.f, 3.f);

        wheel_joy_pub->publish(msg_new);
        return;
      } 
      // If B is held down, then reverse
      else if (msg_new.buttons[3]) {
        //Get roll as pitch (controller is horizontal)
        std::swap(msg_new.axes[0], msg_new.axes[1]);
        //Set pitch to a constant reverse value bc b is down
        msg_new.axes[1] = -1.;

        //Clamp turn to ensure we dont spin out
        msg_new.axes[0] = std::clamp(msg_new.axes[0]/5.f, -3.f, 3.f);

        wheel_joy_pub->publish(msg_new);
        return;
      }

      //Just pub nothing if neither are held down
      msg_new.axes[1] = 0.;
      msg_new.axes[0] = 0.;
      msg_new.axes[2] = 0.; //No idea what this does but clear anyway lol
      wheel_joy_pub->publish(msg_new);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;
    auto wheel_node = std::make_shared<WiiWheel::WiiWheelTeleop>(options);
    exec.add_node(wheel_node);

    RCLCPP_INFO(wheel_node->get_logger(), "I'm using tilt controls!");
    
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
