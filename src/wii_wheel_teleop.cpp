#include "../include/wii_wheel/wii_wheel_teleop.hpp"

#include <algorithm>
#include <memory>
#include <functional>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/msg/detail/joy_feedback_array__struct.hpp>

namespace WiiWheel
{
    WiiWheelTeleop::WiiWheelTeleop(rclcpp::NodeOptions options)
        : Node("wii_wheel", options)
    {
        wii_joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
            "/wiimote/joy", 5,
            std::bind(&WiiWheelTeleop::wii_joy_callback, this, std::placeholders::_1));

        wheel_joy_pub = this->create_publisher<sensor_msgs::msg::Joy>(
            "joy", rclcpp::SystemDefaultsQoS());
            
        joy_feedback_pub = this->create_publisher<sensor_msgs::msg::JoyFeedbackArray>(
            "joy/set_feedback", rclcpp::SystemDefaultsQoS());

        dvelocity = this->declare_parameter("dvelocity", 1.);       
        max_turn = this->declare_parameter("max_turn", 3);       
        steering_ratio = this->declare_parameter("steering_ratio", 4);       
    }

    void WiiWheelTeleop::wii_joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    { 
      //axis 1(pitch) is forward, 0(roll) is turn 
      //Button 1 is go, button 3 is reverse
      //Button 4 is speed up, button 5 is speed down
 
      // Early return if nothing is pressed, but send zero message first to avoid joy getting 'stuck'.
      if (!std::any_of(msg->buttons.begin(), msg->buttons.end(), [](bool button){return button;})) {
        if (!sent_zero_msg) {
          msg->axes[1] = 0.;
          msg->axes[0] = 0.;
          msg->axes[2] = 0.;
          wheel_joy_pub->publish(*msg);
          sent_zero_msg = true;
        }
        return;
      }

      sent_zero_msg = false;

      static bool rumbled_last = false;
      // The plus and minus buttons change speed
      if (msg->buttons[4]) {
        velocity += dvelocity;
        send_next_vibrate_cmd(true);
        rumbled_last = true;
      }
      // Ensure we dont have negative speed
      else if (msg->buttons[5] && velocity - dvelocity > 0){
        velocity -= dvelocity;
        send_next_vibrate_cmd(false);
        rumbled_last = true;
      }
      else if (rumbled_last) {
        send_clear_vibrate_cmd();
        rumbled_last = false;
      }

      // If 2 is held down, then accelerate
      if (msg->buttons[1]) {
        //Get roll as pitch (controller is horizontal)
        std::swap(msg->axes[0], msg->axes[1]);
        //Set pitch to a constant value bc 2 is held down
        msg->axes[1] = velocity;

        //Clamp turn to ensure we dont spin out
        msg->axes[0] = std::clamp(msg->axes[0]/steering_ratio, -max_turn, max_turn);

        wheel_joy_pub->publish(*msg);
        return;
      } 
      // If B is held down, then reverse
      else if (msg->buttons[3]) {
        //Get roll as pitch (controller is horizontal)
        std::swap(msg->axes[0], msg->axes[1]);
        //Set pitch to a constant reverse value bc b is down
        msg->axes[1] = -velocity;

        //Clamp turn to ensure we dont spin out
        msg->axes[0] = std::clamp(msg->axes[0]/steering_ratio, -max_turn, max_turn);

        wheel_joy_pub->publish(*msg);
        return;
      }
      // Forward message with no velocity if any other button. This allows downstream mapping.
      else {
        msg->axes[0] = 0.;
        msg->axes[1] = 0.;
        msg->axes[2] = 0.;
        wheel_joy_pub->publish(*msg);
      }
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
