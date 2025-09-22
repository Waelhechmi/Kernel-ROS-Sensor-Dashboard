# Kernel-ROS Sensor Dashboard

## Description
Complete real-time sensor monitoring system that reads data from I2C devices through custom kernel modules and provides multiple interfaces for visualization and data publishing.

## Features
- TMP102 Temperature Sensor (±0.5°C precision)
- BMP280 Pressure and Temperature Sensor
- ADS1115 16-bit ADC (4 channels)
- Custom Linux Kernel Modules
- Qt GUI Dashboard with real-time gauges
- ROS2 Node for sensor data publishing
- Command-line monitoring tool

## Project Structure
kernel_modules/
├── tmp102.c, tmp102.ko
├── bmp280.c, bmp280.ko
├── ads1115.c, ads1115.ko
└── Makefile

user_application/
├── user.c
└── user_dashboard

graphicsviewer/
├── main.cpp
├── mainwindow.cpp, mainwindow.h, mainwindow.ui
├── SensorsReader.cpp, SensorsReader.h
├── SensorDashboardGUI.pro
└── ui_mainwindow.h

ros2_ws/src/sensors_publisher/
├── src/sensor_publisher.cpp
├── CMakeLists.txt
└── package.xml

## Installation

### Prerequisites
sudo apt update
sudo apt install build-essential linux-headers-$(uname -r)
sudo apt install qt5-default cmake
sudo apt install ros-humble-desktop python3-colcon-common-extensions

### 1. Kernel Modules
cd kernel_modules
make
sudo insmod tmp102.ko
sudo insmod bmp280.ko
sudo insmod ads1115.ko

### 2. User Application
cd user_application
gcc -o user_dashboard user.c

### 3. Qt GUI
cd graphicsviewer
qmake SensorDashboardGUI.pro
make

### 4. ROS2 Node
cd ros2_ws
colcon build --packages-select sensors_publisher
source install/setup.bash

## Usage

### Command-line
./user_dashboard
Output:
TMP102: Temperature = 25.50 °C
BMP280: Temperature = 24.80 °C, Pressure = 1013.25 hPa
ADS1115: Channel 0 = 1.234 V

### Qt GUI
cd graphicsviewer
./SensorDashboardGUI

### ROS2
ros2 run sensors_publisher sensor_publisher
ros2 topic echo /tmp102/temperature

## ROS2 Topics
- /tmp102/temperature (std_msgs/Float32) - 2Hz
- /bmp280/temperature (std_msgs/Float32) - 1Hz
- /bmp280/pressure (std_msgs/Float32) - 1Hz
- /ads1115/voltage (std_msgs/Float32MultiArray) - 4Hz

## Update Intervals
- TMP102: 500ms
- BMP280: 1 second
- ADS1115: 250ms

## Device Files
- /dev/tmp102
- /dev/bmp280
- /dev/ads1115

## Troubleshooting

### Kernel Modules
dmesg | tail -20
i2cdetect -y 1
sudo chmod 666 /dev/tmp102 /dev/bmp280 /dev/ads1115

### ROS2
echo $ROS_DISTRO
source /opt/ros/humble/setup.bash

### Qt
qmake --version
sudo apt install libqt5charts5-dev

## Dependencies
- Linux kernel 4.15+
- I2C interface enabled
- ROS2 Humble or newer
- Qt5 development libraries
- GCC compiler

## Git Branches
- main: Stable production code
- graphics: GUI development
- dev-kernel: Kernel module development
- ros_publisher_value: ROS2 node features

## Future Features
- Real-time plotting in Qt GUI
- Logging sensor values to file
- ROS2 services for calibration
- Web-based dashboard interface
- Mobile app support

## Notes
- Runs on Linux with ROS2 Humble or later
- Requires Qt5 for GUI
- Kernel modules must be loaded before running GUI or ROS2 node
- I2C devices must be properly connected and detected

## Support
For issues:
- Check kernel messages with dmesg
- Verify I2C device detection
- Ensure proper permissions on device files
- Confirm ROS2 environment is sourced



## Version


Compatibility: Linux Kernel 4.15+, ROS2 Humble, Qt5.12+
