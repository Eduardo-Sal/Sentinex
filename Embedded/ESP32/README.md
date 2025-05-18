# A-ASMR ESP32 Firmware

This folder contains the firmware for the ESP32 microcontroller used in the Sentinex autonomous security robot. It handles:

- Motor and servo control
- Ultrasonic distance measurement
- Serial communication with the Jetson Nano running a micro-ROS agent

## Dependencies

Install the following Arduino libraries:

- ESP32Servo  
  https://github.com/jkb-git/ESP32Servo

- micro_ros_arduino  
  https://github.com/micro-ROS/micro_ros_arduino

These libraries include ROS 2 message types such as:
- std_msgs
- geometry_msgs
- nav_msgs
- sensor_msgs
- tf2_msgs

## Setup Instructions (Arduino IDE)

### 1. Install Required Libraries

Clone the libraries into your Arduino `libraries` folder:

```bash
git clone https://github.com/micro-ROS/micro_ros_arduino.git ~/Arduino/libraries/micro_ros_arduino
git clone https://github.com/jkb-git/ESP32Servo.git ~/Arduino/libraries/ESP32Servo
```

### 2. Build ROS 2 Message Headers

Set up micro-ROS headers by running:

```bash
cd ~/Arduino/libraries/micro_ros_arduino
ros2 run micro_ros_setup create_firmware_ws.sh arduino
ros2 run micro_ros_setup build_firmware.sh
```

### 3. Configure and Upload via Arduino IDE

Use the following settings:

- Board: ESP32 Dev Module  
- Upload Speed: 115200 (or higher)  
- Port: Your device’s serial port (e.g., `/dev/cu.SLAB_USBtoUART` on macOS)

Upload the sketch using **Sketch > Upload**.

## Communication with Jetson Nano

This firmware communicates over USB serial with a Jetson Nano, which runs a Python-based micro-ROS agent using `pyserial`. The ESP32 acts as a micro-ROS client node, sending and receiving data through standard ROS 2 messages.

## File Summary

- `Motor_Speed_Control_Odom.ino`: Motor control for ROS2  
- `Ultrasonic_Servo_Sound_Reset_Button.ino`: Ultrasonic, microphone, servo angle control, and pairing logic for ROS2  
- `odometry.h`: Odometry support header  
- `odometry.cpp`: Odometry implementation for tracking robot position and orientation using velocity data  
- `README.md`: This setup guide