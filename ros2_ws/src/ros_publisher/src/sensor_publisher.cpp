#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "RawSensorsReader.h"
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

class SensorPublisher : public rclcpp::Node {
public:
    SensorPublisher() : Node("sensor_publisher") {
        tmp_pub = create_publisher<std_msgs::msg::Float32>("tmp102", 10);
        bmp_temp_pub = create_publisher<std_msgs::msg::Float32>("bmp280_temperature", 10);
        bmp_pres_pub = create_publisher<std_msgs::msg::Float32>("bmp280_pressure", 10);
        ads_pub = create_publisher<std_msgs::msg::Float32>("ads1115_voltage", 10);

        reader = std::make_shared<RawSensorsReader>();
        timer_ = create_wall_timer(500ms, std::bind(&SensorPublisher::publishSensors, this));
    }

private:
    void publishSensors() {
        float tmp;
        if(reader->readTMP(tmp)) {
            auto msg = std_msgs::msg::Float32();
            msg.data = tmp;
            tmp_pub->publish(msg);
        }

        BmpData bmp;
        if(reader->readBMP(bmp)) {
            auto msg_temp = std_msgs::msg::Float32();
            msg_temp.data = bmp.temperature;
            bmp_temp_pub->publish(msg_temp);

            auto msg_pres = std_msgs::msg::Float32();
            msg_pres.data = bmp.pressure;
            bmp_pres_pub->publish(msg_pres);
        }

        float voltage;
        if(reader->readADS(voltage)) {
            auto msg = std_msgs::msg::Float32();
            msg.data = voltage;
            ads_pub->publish(msg);
        }
    }

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr tmp_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr bmp_temp_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr bmp_pres_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ads_pub;

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<RawSensorsReader> reader;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorPublisher>());
    rclcpp::shutdown();
    return 0;
}
