# Line Follower Robot — ROS 2 + OpenCV + Arduino (L298N)

## Overview
A simple **vision-based line follower** built with **ROS 2, OpenCV, and Arduino (L298N motor driver)**.  
The robot can run both in **Gazebo simulation** and on **real hardware**.

### Features
- Camera-based line detection using **OpenCV**
- Adaptive thresholding for variable lighting
- PID-based steering control
- Differential drive in simulation (Gazebo plugin)
- Hardware integration with Arduino (L298N motor driver)

### ROS 2 Nodes
- `camera_node` → publishes camera feed  
- `line_detector` → extracts line offset from image  
- `controller_node` → PID control + motor commands  

---

## Repository Structure
line_follower/
├─ hardware/
│ ├─ wiring.md
│ └─ bom.csv
├─ microcontroller/
│ └─ src/motor_controller.ino # Arduino firmware
├─ ros2_pkg/
│ ├─ package.xml
│ ├─ setup.py
│ ├─ setup.cfg
│ ├─ resource/line_follower_nodes
│ └─ line_follower_nodes/
│ ├─ camera_node.py
│ ├─ line_detector.py
│ ├─ controller_node.py
│ └─ launch/
│ ├─ sim.launch.py
│ ├─ hw.launch.py
│ └─ gazebo.launch.py
├─ gazebo/
│ ├─ track_world.sdf # World with black line
│ └─ models/
│ ├─ track_line/ # Simple black line on white ground
│ └─ line_follower_bot/ # Robot with camera + diff drive
└─ README.md

---

## Run in Simulation
Build and source your workspace:
```bash
colcon build
source install/setup.bash

Start the simulation (simple ROS 2 camera + PID loop, no Gazebo movement):
ros2 launch line_follower_nodes sim.launch.py

Run the full Gazebo world with robot + diff drive plugin:
ros2 launch line_follower_nodes gazebo.launch.py

---

Run in Simulation

Flash microcontroller/src/motor_controller.ino to Arduino.

Connect Arduino (e.g., /dev/ttyUSB0) and USB camera.

Launch hardware mode:

ros2 launch line_follower_nodes hw.launch.py serial_port:=/dev/ttyUSB0 camera_index:=0

PID Tuning

Tune parameters in controller_node.py or via launch:

kp → proportional gain (responsiveness)

kd → derivative gain (stability)

base_speed → forward speed

Example:

ros2 launch line_follower_nodes hw.launch.py kp:=0.015 kd:=0.003 base_speed:=90

Notes

Simulation uses a simple black line on a white ground in Gazebo.

Hardware commands are sent over serial:
"M,<left_pwm>,<right_pwm>\n" with values in [-127, 127].

Works with both Arduino UNO/Mega and ESP32 (with small pin changes).