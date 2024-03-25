#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "../lib/wit_lib/wit_lib.h"
#include "../../nereo_interfaces/msg/ImuData.hpp" // To fix

using namespace std::chrono_literals;

class PublisherIMU: public rclcpp::Node
{
    private:
        size_t count_;
        rclcpp::Publisher<nereo_interfaces::msg::ImuData>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        void timer_callback()
        {
            auto message = nereo_interfaces::msg::ImuData();
            
            // Getting data from IMU
            if (!GetAcc(message.data.Acc))
                RCLCPP_ERROR(this->get_logger(), "Error while getting ACCELERATION data");
            
            if (!GetAngle(message.data.Angle))
                RCLCPP_ERROR(this->get_logger(), "Error while getting ANGLE data");

            if (!GetAngVel(message.data.AngVel))
                RCLCPP_ERROR(this->get_logger(), "Error while getting ANGULAR VELOCITY data");

            if (!GetTEMP(message.data.Temp))
                RCLCPP_ERROR(this->get_logger(), "Error while getting TEMPERATURE data");
        }

    public:
        PublisherIMU(): Node("imu_publisher"), count_()
        {
            publisher_ = this->create_publisher<nereo_interfaces::msg::ImuData>("imu_topic", 10);
            timer_ = this->create_wall_timer(200ms, std::bind(&publisherIMU::timer_callback, this));
            WitInit();
        }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublisherIMU>());
    rclcpp::shutdown();
    return 0;
}