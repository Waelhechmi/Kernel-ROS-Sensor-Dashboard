# Kernel-ROS Sensor Dashboard

## Description

This project is a complete system to read I2C/TMP sensors and publish their values via ROS2, with a Qt graphical interface to visualize data in real-time.

It integrates:

- Kernel modules for sensors: `TMP102`, `BMP280`, `ADS1115`
- A user application to read sensor values (`user_dashboard.c`)
- A Qt GUI (`graphicsviewer`) with gauges and labels for each sensor
- A ROS2 node (`sensors_publisher`) to publish each sensor value on independent topics

---

## Project Structure

Kernel-ROS-Sensor-Dashboard/
├── ads1115.c
├── ads1115.ko
├── bmp280.c
├── bmp280.ko
├── tmp102.c
├── tmp102.ko
├── graphicsviewer/
│ ├── main.cpp
│ ├── mainwindow.cpp
│ ├── mainwindow.h
│ ├── mainwindow.ui
│ ├── SensorsReader.cpp
│ ├── SensorsReader.h
│ ├── SensorDashboardGUI.pro
│ └── ui_mainwindow.h
├── ros2_ws/
│ └── src/
│ └── sensors_publisher/
│ ├── src/sensor_publisher.cpp
│ ├── CMakeLists.txt
│ └── package.xml
├── user.c
└── README.md


---

## Kernel Modules

Each sensor has a kernel module exposing data via `/dev`:

- TMP102 → `/dev/tmp102`
- BMP280 → `/dev/bmp280`
- ADS1115 → `/dev/ads1115`

### Build and insert modules


make
sudo insmod tmp102.ko
sudo insmod bmp280.ko
sudo insmod ads1115.ko 

To remove 
sudo rmmod tmp102
sudo rmmod bmp280
sudo rmmod ads1115

User Application

user.c reads the sensor files and prints values in the terminal:

gcc -o user_dashboard user.c
./user_dashboard

Example output:

TMP102: Temp = 25.50 °C
BMP280: Temp = 24.80 °C, Pressure = 1013.25 hPa
ADS1115: Voltage = 1.234 V

Qt Graphical Interface

The graphicsviewer folder contains the GUI for sensor visualization:

Build

qmake SensorDashboardGUI.pro
make
./SensorDashboardGUI


Each sensor has a gauge and a label showing its real-time value.

Updates are independent via timers (TMP102 every 500 ms, BMP280 every 1 s, ADS1115 every 250 ms).



ROS2 Node: sensors_publisher

Publishes each sensor value on a separate ROS2 topic:

Sensor	ROS2 Topic
TMP102	/tmp102/value
BMP280 Temp	/bmp280/temperature
BMP280 Pressure	/bmp280/pressure
ADS1115	/ads1115/voltage


Build 
cd ros2_ws
colcon build --packages-select sensors_publisher
source install/setup.bash


Run 
ros2 run sensors_publisher sensor_publisher

Example of reading a topic

ros2 topic echo /bmp280/temperature


Git Branches

main → Contains the full stable project

ros_publisher_value → Contains only the ROS2 node for publishing sensor values

Future Features

Real-time plotting in the Qt GUI

Logging sensor values to a file

ROS2 services for calibration or configuration of sensors

Notes

The project runs on Linux with ROS2 Humble or later.

Requires Qt5 for the GUI.

Kernel modules must be loaded before running the GUI or the ROS2 node.




