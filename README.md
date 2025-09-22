project:
  name: "Kernel-ROS Sensor Dashboard"
  description: >
    Kernel-ROS Sensor Dashboard is a complete system to read sensor values from I2C devices 
    and visualize/publish them in real-time. It integrates kernel modules, a user-space 
    application, a Qt GUI, and a ROS2 node for publishing sensor data.
  features:
    - Kernel modules:
        TMP102: "Temperature sensor"
        BMP280: "Temperature and Pressure sensor"
        ADS1115: "Analog-to-Digital Converter for voltage readings"
    - User-space application: "user_dashboard.c"
    - Qt GUI: "graphicsviewer folder with real-time gauges and labels"
    - ROS2 node: "sensors_publisher publishes each sensor value on separate topics"

project_structure:
  - ads1115.c
  - ads1115.ko
  - bmp280.c
  - bmp280.ko
  - tmp102.c
  - tmp102.ko
  - graphicsviewer:
      files:
        - main.cpp
        - mainwindow.cpp
        - mainwindow.h
        - mainwindow.ui
        - SensorsReader.cpp
        - SensorsReader.h
        - SensorDashboardGUI.pro
        - ui_mainwindow.h
  - ros2_ws/src/sensors_publisher:
      files:
        - src/sensor_publisher.cpp
        - CMakeLists.txt
        - package.xml
  - user.c
  - README.md

kernel_modules:
  devices:
    TMP102: "/dev/tmp102"
    BMP280: "/dev/bmp280"
    ADS1115: "/dev/ads1115"
  build:
    commands:
      - "make"
      - "sudo insmod tmp102.ko"
      - "sudo insmod bmp280.ko"
      - "sudo insmod ads1115.ko"
  remove:
    commands:
      - "sudo rmmod tmp102"
      - "sudo rmmod bmp280"
      - "sudo rmmod ads1115"

user_application:
  file: "user.c"
  build: "gcc -o user_dashboard user.c"
  run: "./user_dashboard"
  example_output: |
    TMP102: Temp = 25.50 °C
    BMP280: Temp = 24.80 °C, Pressure = 1013.25 hPa
    ADS1115: Voltage = 1.234 V

qt_gui:
  folder: "graphicsviewer"
  build:
    - "qmake SensorDashboardGUI.pro"
    - "make"
  run: "./SensorDashboardGUI"
  update_intervals:
    TMP102: "500 ms"
    BMP280: "1 s"
    ADS1115: "250 ms"

ros2_node:
  name: "sensors_publisher"
  topics:
    TMP102: "/tmp102/value"
    BMP280_temperature: "/bmp280/temperature"
    BMP280_pressure: "/bmp280/pressure"
    ADS1115: "/ads1115/voltage"
  build:
    - "cd ros2_ws"
    - "colcon build --packages-select sensors_publisher"
    - "source install/setup.bash"
  run: "ros2 run sensors_publisher sensor_publisher"
  example_echo: "ros2 topic echo /bmp280/temperature"

git_branches:
  main: "Full stable project (kernel + GUI + ROS2)"
  graphics: "GUI development branch"
  dev-kernel: "Kernel module development branch"
  ros_publisher_value: "ROS2 node publishing sensor values"

future_features:
  - "Real-time plotting in Qt GUI"
  - "Logging sensor values to a file"
  - "ROS2 services for sensor calibration/configuration"

notes:
  - "Runs on Linux with ROS2 Humble or later"
  - "Requires Qt5 for the GUI"
  - "Kernel modules must be loaded before running GUI or ROS2 node"

