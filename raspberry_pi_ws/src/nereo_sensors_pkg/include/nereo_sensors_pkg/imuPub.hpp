#ifndef IMU_PUB_H
#define IMU_PUB_H

#include <chrono>
#include <memory>
#include <queue>
#include <string>
#include <thread>

#include "imu_libs/WT61P.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

#define MAXN 20
#define WT61P_IIC_ADDR 0x50

char *i2c_device = "/dev/i2c-1";

typedef struct {
    float x;
    float y;
    float z;
} Vec3;

typedef struct {
    Vec3 acc;
    Vec3 angles;
    Vec3 ang_vel;
} ImuValues;

typedef double float64;

enum Status {OK, WARN, ERROR, STALE};

//void calcCovMatrix(std::queue<Vec3> window, float64 *matrix);

class PublisherIMU: public rclcpp::Node
{
    private:
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        
        rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr imu_diagnostic_publisher_;

        /* Status indicators
        bool imu_acc_error = false;
        bool imu_angle_error = false;
        bool imu_ang_vel_error = false;*/
        Status communication_state = OK;

        std::queue<Vec3> acceleration_window;
        std::queue<Vec3> angular_velocity_window;
        std::queue<Vec3> angles_window;

        float64 matrix[9];

        void timer_callback();

    public:
        PublisherIMU();
};

#endif
