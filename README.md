# wii_wheel_teleop
From package '[wii_wheel](https://github.com/andyblarblar/ros2_wii_wheel)'
# File
`./src/wii_wheel_teleop.cpp`

## Summary 
 Allows for teleop by tilting the wiimote like when using a wii wheel. This is implemented by intercepting the joy messages
from the wiimote node. The included launch file launches this node for you. 

During operation, move forward with 2, backward with B, increase speed with +, and decrease speed with -.

## Topics

### Publishes
- `joy`: The wheel joy messages.
- `joy/set_feedback`: Rumble control messages, sent when changing speed.

### Subscribes
- `/wiimote/joy`: The original joy messages from the wiimote node.

## Params
- `velocity`: Starting speed. Default 1
- `max_turn`: Max angular speed. Default 3
- `steering_ratio`: Value the raw accelerometer data is divided by. Default 3

# Launch 
 `./launch/wheel_teleop.launch.py` 
 Launches both teleop and the wiimote node. The wiimote node will prompt for you to press 1+2 on a wiimote to connect it.
This will only connect to gen1 pre wii motion plus inside wiimotes with a nunchuk in (I have no idea).
 

## Args
- `velocity`: Starting speed. Default 1
- `max_turn`: Max angular speed. Default 3
- `steering_ratio`: Value the raw accelerometer data is divided by. Default 3

