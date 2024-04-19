#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "../lib/wit_lib/wit_lib.h"
#include "sensor_msgs/msg/imu.hpp"
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

using namespace std::chrono_literals;

// Struct di appoggio
typedef struct {
    uint16_t Acc[3];
    uint16_t Angle[3];
    uint16_t AngVel[3];
    uint16_t Temp;
} imuValues;

class PublisherIMU: public rclcpp::Node
{
    private:
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr dataIMUpublisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr diagnostic_timer_;
        diagnostic_updater::Updater diagnostic_updater_;

        // Status indicators
        bool imu_acc_error = false;
        bool imu_angle_error = false;
        bool imu_ang_vel_error = false;

        void timer_callback()
        {
            imuValues imu;
            auto dataIMUmessage = sensor_msgs::msg::Imu();

            imu_acc_error = getAcc(imu.Acc);
            imu_ang_vel_error = getAngVel(imu.AngVel);
            imu_angle_error = getAngle(imu.Angle);

            dataIMUmessage.data.header.stamp = self.get_clock().now().seconds;
            dataIMUmessage.data.header.frame_id = "Frame ID IMU";

            for (int i = 0; i < 3; i++){
                dataIMUmessage.data.angular_velocity[i] = imu.AngVel[i];
                dataIMUmessage.data.linear_acceleration[i] = imu.Acc[i];
                dataIMUmessage.data.orientation[i] = imu.Angle[i];
            }

            // Linear acceleration covariance
            // Angular velocity covariance
            // Orientation covariance


            // Publish ...

            dataIMUpublisher_->publish(dataIMUmessage);
        }

        void diagnostic_check(diagnostic_updater::DiagnosticStatusWrapper& stat) {
            if (!imu_acc_error && !imu_angle_error && !imu_ang_vel_error)
                stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "IMU functioning normally");
            else {
                stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "IMU encountered errors");

                if (imu_acc_error)
                    stat.add("Acceleration Error", "Error while getting ACCELERATION (IMU)");
                
                if (imu_angle_error)
                    stat.add("Angle Error", "Error while getting ANGLE data");
                
                if (imu_ang_vel_error)
                    stat.add("Angular Velocity Error", "Error while getting ANGULAR VELOCITY data");
            }
        }

    public:
        PublisherIMU(): Node("imu_publisher"), diagnostic_updater_(this)
        {
            dataIMUpublisher_ = this->create_publisher<sensor_msgs::msg::Imu>("dataIMU_topic", 10);
            timer_ = this->create_wall_timer(200ms, std::bind(&publisherIMU::timer_callback, this));

            WitInit();

            diagnostic_updater_.setHardwareID("IMU_Sensor");
            diagnostic_updater_.add("IMU Status", this, &PublisherIMU::diagnostic_check);

            auto diagnostic_timer_ = this->create_wall_timer(200ms, std::bind(&PublisherIMU::timer_callback, this));
        };
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublisherIMU>());
    rclcpp::shutdown();
    return 0;
}