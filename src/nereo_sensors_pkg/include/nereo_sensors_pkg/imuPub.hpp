#ifndef IMU_PUB_H
#define IMU_PUB_H

#include <chrono>
#include <memory>
#include <queue>
#include <string>

#include "imu_libs/WT61P.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

#define MAXN 20

typedef struct {
    float x;
    float y;
    float z;
} vec3;

typedef struct {
    vec3 acc;
    vec3 angles;
    vec3 ang_vel;
} imuValues;

typedef double float64;

enum status {OK, WARN, ERROR, STALE};

void calcCovMatrix(std::queue<vec3> window, float64 *matrix);

class PublisherIMU: public rclcpp::Node
{
    private:
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr dataIMUpublisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        
        rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnosticPublisher_;

        // Status indicators
        bool imu_acc_error = false;
        bool imu_angle_error = false;
        bool imu_ang_vel_error = false;

        status communicationState = OK;

        std::queue<vec3> dataWindowAcc;
        std::queue<vec3> dataWindowAngVel;
        std::queue<vec3> dataWindowAngle;

        vec3 Arr;

        float64 matrix[9];

        void timer_callback();

    public:
        PublisherIMU();
};

#endif