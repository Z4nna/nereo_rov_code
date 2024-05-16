#include "nereo_sensors_pkg/imuPub.hpp"
using namespace std::chrono_literals;

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublisherIMU>());
    rclcpp::shutdown();
    return 0;
}

void calcCovMatrix(std::queue<Vec3> window, float64 *matrix) {

    std::queue<Vec3> copy = window;

    Vec3 mean;
    Vec3 sum = {0, 0, 0};

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

void PublisherIMU::timer_callback()
{
    auto dataIMUmessage = sensor_msgs::msg::Imu();

    dataIMUmessage.header.stamp = this->get_clock()->now();
    dataIMUmessage.header.frame_id = "IMU Data";
    
    WT61P_read_angular_vel();
    dataIMUmessage.angular_velocity.x = WT61P_get_angular_vel_x();
    dataIMUmessage.angular_velocity.y = WT61P_get_angular_vel_y();
    dataIMUmessage.angular_velocity.z = WT61P_get_angular_vel_z();
    
    WT61P_read_acc();
    dataIMUmessage.linear_acceleration.x = WT61P_get_acc_x();
    dataIMUmessage.linear_acceleration.y = WT61P_get_acc_y();
    dataIMUmessage.linear_acceleration.z = WT61P_get_acc_z();

    /// fix this part: use tf2 to convert from euler angles to quaternion
    WT61P_read_angle();
    Vec3 angles = { WT61P_get_pitch(), WT61P_get_roll(), WT61P_get_yaw() };
    
    dataIMUmessage.orientation.x = angles.x;
    dataIMUmessage.orientation.y = angles.y; 
    dataIMUmessage.orientation.z = angles.z;

    /*
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
        auto diagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus();
        diagnosticStatus.level = ERROR;
        diagnosticStatus.name = "IMU data acquisition.";
        diagnosticStatus.message = "Errors encountered while acquiring data from IMU";
        diagnosticMessage.status.push_back(diagnosticStatus);
    }
    else{
        auto diagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus();
        diagnosticStatus.level = OK;
        diagnosticStatus.name = "IMU data acquisition.";
        diagnosticStatus.message = "All data acquired correctly.";
        diagnosticMessage.status.push_back(diagnosticStatus);
    }

    // ACCELERATION ERROR
    if (!imu_acc_error){
        auto diagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus();
        diagnosticStatus.level = ERROR;
        diagnosticStatus.name = "IMU acceleration acquisition.";
        diagnosticStatus.message = "Error while acquiring Acceleration";
        diagnosticMessage.status.push_back(diagnosticStatus);
    }

    // ANGULAR VELOCITY ERROR
    if (!imu_ang_vel_error)
    {
        auto diagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus();
        diagnosticStatus.level = ERROR;
        diagnosticStatus.name = "IMU Angular velocity acquisition.";
        diagnosticStatus.message = "Error while acquiring Angular velocity";
        diagnosticMessage.status.push_back(diagnosticStatus);
    }

    // ANGLE ERROR
    if (!imu_angle_error)
    {
        auto diagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus();
        diagnosticStatus.level = ERROR;
        diagnosticStatus.name = "IMU Angle acquisition.";
        diagnosticStatus.message = "Error while acquiring Angular velocity";
        diagnosticMessage.status.push_back(diagnosticStatus);
    }
    diagnosticPublisher_->publish(diagnosticMessage);
    */   
   dataIMUpublisher_->publish(dataIMUmessage);
}

PublisherIMU::PublisherIMU(): Node("imu_publisher")
{            
    dataIMUpublisher_ = this->create_publisher<sensor_msgs::msg::Imu>("dataIMU_topic", 10);
    diagnosticPublisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnosticIMU_topic", 10);

    timer_ = this->create_wall_timer(200ms, std::bind(&PublisherIMU::timer_callback, this));

    int ret = WT61P_begin(i2c_device, WT61P_IIC_ADDR);
};

/*Diagnostic status
- [0]: General INFO
- [1 - 3]: Errors encountered (Acc, AngVel, Angle)
- */
