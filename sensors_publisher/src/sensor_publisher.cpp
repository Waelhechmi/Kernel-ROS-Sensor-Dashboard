#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <iostream>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

struct BmpData {
    int32_t temperature_centi;
    int32_t pressure_pa;
};

class SensorPublisher : public rclcpp::Node {
public:
    SensorPublisher() : Node("sensor_publisher") {
        tmp_pub_ = create_publisher<std_msgs::msg::Float32>("tmp102/value", 10);
        bmp_temp_pub_ = create_publisher<std_msgs::msg::Float32>("bmp280/temperature", 10);
        bmp_press_pub_ = create_publisher<std_msgs::msg::Float32>("bmp280/pressure", 10);
        ads_pub_ = create_publisher<std_msgs::msg::Float32>("ads1115/voltage", 10);

        timer_ = create_wall_timer(500ms, std::bind(&SensorPublisher::publishValues, this));
    }

private:
    void publishValues() {
        publishTMP();
        publishBMP();
        publishADS();
    }

    void publishTMP() {
        int fd = open("/dev/tmp102", O_RDONLY);
        if(fd < 0) {
            RCLCPP_WARN(get_logger(), "Cannot open /dev/tmp102");
            return;
        }
        int16_t tmp_raw;
        if(read(fd, &tmp_raw, sizeof(tmp_raw)) > 0) {
            float temp_C = (tmp_raw & 0x0FFF) * 0.0625f;
            std_msgs::msg::Float32 msg;
            msg.data = temp_C;
            tmp_pub_->publish(msg);
        }
        close(fd);
    }

    void publishBMP() {
        int fd = open("/dev/bmp280", O_RDONLY);
        if(fd < 0) {
            RCLCPP_WARN(get_logger(), "Cannot open /dev/bmp280");
            return;
        }
        BmpData bmp;
        if(read(fd, &bmp, sizeof(bmp)) > 0) {
            float temp_C = bmp.temperature_centi / 100.0f;
            float pres_hPa = bmp.pressure_pa / 100.0f;

            std_msgs::msg::Float32 tmsg, pmsg;
            tmsg.data = temp_C;
            pmsg.data = pres_hPa;
            bmp_temp_pub_->publish(tmsg);
            bmp_press_pub_->publish(pmsg);
        }
        close(fd);
    }

    void publishADS() {
        int fd = open("/dev/ads1115", O_RDONLY);
        if(fd < 0) {
            RCLCPP_WARN(get_logger(), "Cannot open /dev/ads1115");
            return;
        }
        int16_t ads_raw;
        if(read(fd, &ads_raw, sizeof(ads_raw)) > 0) {
            float voltage = ads_raw / 32768.0f * 3.3f;
            std_msgs::msg::Float32 msg;
            msg.data = voltage;
            ads_pub_->publish(msg);
        }
        close(fd);
    }

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr tmp_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr bmp_temp_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr bmp_press_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ads_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
