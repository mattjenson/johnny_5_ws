Johnny 5 Robot Hardware Overview

This document outlines the hardware components used to build the Johnny 5 robot, running ROS 2 Jazzy on a Raspberry Pi 5 (16 GB) with Ubuntu 24.04 and an official Raspberry Pi AI HAT+.

⸻

1. Compute Platform
	•	Raspberry Pi 5 (16 GB)
	•	Ubuntu 24.04 LTS
	•	ROS 2 Jazzy
	•	Raspberry Pi AI HAT+
	•	Provides hardware acceleration for ML workloads and extra I/O headers

⸻

2. Power
	•	Lithium Polymer Battery Pack
	•	Amazon Link
	•	Provides stable 5 V output for Pi and peripherals

⸻

3. Drive System
	•	Motor Driver
	•	Polol Dual VNH5019 Motor Driver
	•	Controls two DC motors via PWM and direction inputs
	•	DC Track Motors (×2)
	•	12 V DC Gear Motor
	•	Mounted on caterpillar tracks for differential drive

⸻

4. Sensors & Navigation
	•	Tilt Sensors (×5)
	•	Analog Tilt Sensor Module
	•	Placed on base, torso, head, and both arms
	•	I²C Multiplexer
	•	TCA9548A 8-Channel I²C Multiplexer
	•	Enables multiple identical I²C tilt sensors on one bus
	•	Digital Compass (Magnetometer)
	•	HMC5883L 3-Axis Compass
	•	For heading estimation and orientation

⸻

5. Vision
	•	Camera Module
	•	Raspberry Pi Camera Module 3
	•	High-res stills and 1080p video for perception nodes

⸻

6. Wiring & Prototyping
	•	Breadboard
	•	Standard Solderless Breadboard
	•	40-Pin Ribbon Cable
	•	GPIO Extension Cable
	•	Routes signals from under the AI HAT+
	•	4-Pin DuPont Cables
	•	Male-to-Female Jumper Wires
	•	For tilt sensor connections

⸻

7. Recommended Software Libraries

ROS 2 Core Libraries
	•	C++ (rclcpp)
	•	Core client library for C++ ROS 2 nodes
	•	Message packages: sensor_msgs, geometry_msgs, tf2_ros
	•	Python (rclpy)
	•	Core client library for Python ROS 2 nodes
	•	Use with rosdep install ros-jazzy-example-pkg for dependencies

Sensor Drivers & Utilities
	•	I²C & GPIO Access
	•	C++: pigpio or native Linux i2c-dev via ioctl
	•	Python: smbus2, RPi.GPIO or pigpio Python bindings
	•	Tilt Sensor (Analog)
	•	Python: adafruit-circuitpython-tilt (AnalogSensor I²C read)
	•	Multiplexer (TCA9548A)
	•	Python: adafruit-circuitpython-tca9548a
	•	Compass (HMC5883L)
	•	Python: adafruit-circuitpython-hmc5883l
	•	C++: RTIMULib or custom driver using i2c-dev

Vision & CV
	•	OpenCV
	•	C++: cv_bridge, image_transport
	•	Python: opencv-python, picamera2 for direct camera access
	•	TensorFlow Lite / ONNX Runtime
	•	Hardware-accelerated inference on AI HAT+

Others & Utilities
	•	Eigen (C++) for linear algebra
	•	Boost (C++) for utilities and threading
	•	NumPy (Python) for array handling
	•	yaml_cpp or yaml (Python) for config parsing

⸻

Ensure all firmware and libraries match ROS 2 Jazzy and Ubuntu 24.04 compatibility.