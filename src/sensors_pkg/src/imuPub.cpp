#include <chrono>
#include <iostream>
#include <memory>
#include <queue>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "../lib/wit_lib/wit_lib.h"
#include "sensor_msgs/msg/imu.hpp"
#include "diagnostic_array.hpp"

#define MAXN 20

using namespace std::chrono_literals;

typedef struct {
    float Acc[3];
    float Angle[3];
    float AngVel[3];
} imuValues;

typedef struct {
    float x;
    float y;
    float z;
} vec3;

typedef double float64;

enum status {OK, WARN, ERROR, STALE};

void calcCovMatrix(std::queue<vec3> window, float64 *matrix){

    std::queue<vec3> copy = window;

    vec3 mean;
    vec3 sum = {0, 0, 0};

    while(!copy.empty()){
        sum.x += copy.front().x;
        sum.y += copy.front().y;
        sum.z += copy.front().z;

        copy.pop();
    }

    mean.x = sum.x / MAXN;
    mean.y = sum.y / MAXN;
    mean.z = sum.z / MAXN;

    copy = window;

    sum = {0, 0, 0};

    // VARIANCE
    while (!copy.empty()){
        sum.x += (copy.front().x - mean.x)*(copy.front().x - mean.x);
        sum.y += (copy.front().y - mean.y)*(copy.front().y - mean.y);
        sum.z += (copy.front().z - mean.z)*(copy.front().z - mean.z);

        copy.pop();
    }

    matrix[0] = sum.x / MAXN;
    matrix[4] = sum.y / MAXN;
    matrix[8] = sum.z / MAXN;

    
    sum = {0, 0, 0};

    // COVARIANCE
    while(!copy.empty()){
        sum.x += (copy.front().x - mean.x)*(copy.front().y - mean.y);
        sum.y += (copy.front().x - mean.x)*(copy.front().z - mean.z);
        sum.z += (copy.front().y - mean.y)*(copy.front().z - mean.z);

    }

    matrix[1] = sum.x / MAXN; matrix[3] = sum.x / MAXN;
    matrix[2] = sum.y / MAXN; matrix[6] = sum.y / MAXN;
    matrix[5] = sum.z / MAXN; matrix[7] = sum.z / MAXN;
}

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

        status communicationState = INFO;

        std::queue<vec3> dataWindowAcc;
        std::queue<vec3> dataWindowAngVel;
        std::queue<vec3> dataWindowAngle;

        vec3 Arr;

        float64 matrix[9];

        void timer_callback()
        {

            // IMU Data
            int diagnosticSize = 0;
            imuValues imu;
            auto dataIMUmessage = sensor_msgs::msg::Imu();

            // vvv Valgono 0 nel caso di errore di acquisizione vvv

            imu_acc_error = getAcc(imu.Acc);
            imu_ang_vel_error = getAngVel(imu.AngVel);
            imu_angle_error = getAngle(imu.Angle);

            dataIMUmessage.header.stamp = this->get_clock()->now();
            dataIMUmessage.header.frame_id = "IMU Data";
            
            dataIMUmessage.angular_velocity.x = imu.AngVel[0];
            dataIMUmessage.angular_velocity.y = imu.AngVel[1];
            dataIMUmessage.angular_velocity.z = imu.AngVel[2];
            
            dataIMUmessage.linear_acceleration.x = imu.Acc[0];
            dataIMUmessage.linear_acceleration.y = imu.Acc[1];
            dataIMUmessage.linear_acceleration.z = imu.Acc[2];

            dataIMUmessage.orientation.x = imu.Angle[0];
            dataIMUmessage.orientation.y = imu.Angle[1];
            dataIMUmessage.orientation.z = imu.Angle[2];

            // Capire cosa mettere nella quarta componente del quaternione

            // Linear acceleration covariance
            calcCovMatrix(dataWindowAcc, matrix);

            for (int i = 0; i < 9; i++)
                dataIMUmessage.linear_acceleration_covariance[i] = matrix[i];

            // Angular velocity covariance
            calcCovMatrix(dataWindowAngVel, matrix);

            for (int i = 0; i < 9; i++)
                dataIMUmessage.angular_velocity_covariance[i] = matrix[i];

            // Orientation covariance - SBAGLIATA - CONVERSIONE IN QUATERNIONI E POI MATRICE
            calcCovMatrix(dataWindowAngle, matrix);
            
            for (int i = 0; i < 9; i++)
                dataIMUmessage.orientation_covariance[i] = matrix[i];

            // Diagnostic
            auto diagnosticMessage = diagnostic_msgs::msg::DiagnosticArray();
            
            diagnosticMessage.header.stamp = this->get_clock()->now();
            diagnosticMessage.header.frame_id = "IMU Diagnostic";

            // GENERAL ERROR ENCOUTERED
            if (!imu_acc_error || !imu_ang_vel_error || !imu_angle_error){
                diagnosticMessage.status[diagnosticSize].level = ERROR;
                diagnosticMessage.status[diagnosticSize++].name = "Acquisition error";
                diagnosticMessage.status[diagnosticSize].message = "Errors encountered while acquiring data from IMU";
            }
            else{
                diagnosticMessage.status[diagnosticSize].level = OK;
                diagnosticMessage.status[diagnosticSize++].name = "Everything OK";
                diagnosticMessage.status[diagnosticSize].message = "All data acquired correctly";
            }

            // ACCELERATION ERROR
            if (!imu_acc_error){
                diagnosticMessage.status[diagnosticSize].level = ERROR;
                diagnosticMessage.status[diagnosticSize].name = "Acceleration error";
                diagnosticMessage.status[diagnosticSize++].message = "Error while acquiring Acceleration";
            }

            // ANGULAR VELOCITY ERROR
            if (!imu_ang_vel_error){
                diagnosticMessage.status[diagnosticSize].level = ERROR;
                diagnosticMessage.status[diagnosticSize].name = "Angular velocity error";
                diagnosticMessage.status[diagnosticSize++].message = "Error while acquiring Angular velocity";
            }

            // ANGLE ERROR
            if (!imu_angle_error){
                diagnosticMessage.status[diagnosticSize].level = ERROR;
                diagnosticMessage.status[diagnosticSize].name = "Angle error";
                diagnosticMessage.status[diagnosticSize++].message = "Error while acquiring Angle";
            }

            dataIMUpublisher_->publish(dataIMUmessage);
            diagnosticPublisher_->publish(diagnosticMessage);
        }

    public:
        PublisherIMU(): Node("imu_publisher")
        {            
            dataIMUpublisher_ = this->create_publisher<sensor_msgs::msg::Imu>("dataIMU_topic", 10);
            diagnosticPublisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnosticIMU_topic", 10);

            timer_ = this->create_wall_timer(200ms, std::bind(&PublisherIMU::timer_callback, this));

            WitInit();
        };
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublisherIMU>());
    rclcpp::shutdown();
    return 0;
}

/*Diagnostic status
- [0]: General INFO
- [1 - 3]: Errors encountered (Acc, AngVel, Angle)
- */