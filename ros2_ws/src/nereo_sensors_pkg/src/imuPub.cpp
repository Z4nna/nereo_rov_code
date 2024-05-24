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
    tf2::Quaternion tf2_quat, tf2_quat_from_msg;

    auto imu_data_message = sensor_msgs::msg::Imu();

    imu_data_message.header.stamp = this->get_clock()->now();
    imu_data_message.header.frame_id = "IMU Data";
    
    WT61P_read_angular_vel();
    imu_data_message.angular_velocity.x = WT61P_get_angular_vel_x();
    imu_data_message.angular_velocity.y = WT61P_get_angular_vel_y();
    imu_data_message.angular_velocity.z = WT61P_get_angular_vel_z();
    
    WT61P_read_acc();
    imu_data_message.linear_acceleration.x = WT61P_get_acc_x();
    imu_data_message.linear_acceleration.y = WT61P_get_acc_y();
    imu_data_message.linear_acceleration.z = WT61P_get_acc_z();

    WT61P_read_angle();
    Vec3 angles = { WT61P_get_pitch(), WT61P_get_roll(), WT61P_get_yaw() };

    // tf2 quaternion conversion

    tf2_quat.setRPY(angles.x * 0.0174533, angles.y * 0.0174533, angles.z * 0.0174533); // Conversion from degrees to radians

    tf2_quat.normalize();

    imu_data_message.orientation = tf2::toMsg(tf2_quat);


    /*
    // Linear acceleration covariance
    calcCovMatrix(acceleration_window, matrix);

    for (int i = 0; i < 9; i++)
        imu_data_message.linear_acceleration_covariance[i] = matrix[i];

    // Angular velocity covariance
    calcCovMatrix(angular_velocity_window, matrix);

    for (int i = 0; i < 9; i++)
        imu_data_message.angular_velocity_covariance[i] = matrix[i];

    // Orientation covariance - SBAGLIATA - CONVERSIONE IN QUATERNIONI E POI MATRICE
    calcCovMatrix(angles_window, matrix);
    
    for (int i = 0; i < 9; i++)
        imu_data_message.orientation_covariance[i] = matrix[i];

    // Diagnostic
    auto imu_diagnostic_message = diagnostic_msgs::msg::DiagnosticArray();
    
    imu_diagnostic_message.header.stamp = this->get_clock()->now();
    imu_diagnostic_message.header.frame_id = "IMU Diagnostic";

    // GENERAL ERROR ENCOUTERED
    if (!imu_acc_error || !imu_ang_vel_error || !imu_angle_error){
        auto diagnostic_status = diagnostic_msgs::msg::DiagnosticStatus();
        diagnostic_status.level = ERROR;
        diagnostic_status.name = "IMU data acquisition.";
        diagnostic_status.message = "Errors encountered while acquiring data from IMU";
        imu_diagnostic_message.status.push_back(diagnostic_status);
    }
    else{
        auto diagnostic_status = diagnostic_msgs::msg::DiagnosticStatus();
        diagnostic_status.level = OK;
        diagnostic_status.name = "IMU data acquisition.";
        diagnostic_status.message = "All data acquired correctly.";
        imu_diagnostic_message.status.push_back(diagnostic_status);
    }

    // ACCELERATION ERROR
    if (!imu_acc_error){
        auto diagnostic_status = diagnostic_msgs::msg::DiagnosticStatus();
        diagnostic_status.level = ERROR;
        diagnostic_status.name = "IMU acceleration acquisition.";
        diagnostic_status.message = "Error while acquiring Acceleration";
        imu_diagnostic_message.status.push_back(diagnostic_status);
    }

    // ANGULAR VELOCITY ERROR
    if (!imu_ang_vel_error)
    {
        auto diagnostic_status = diagnostic_msgs::msg::DiagnosticStatus();
        diagnostic_status.level = ERROR;
        diagnostic_status.name = "IMU Angular velocity acquisition.";
        diagnostic_status.message = "Error while acquiring Angular velocity";
        imu_diagnostic_message.status.push_back(diagnostic_status);
    }

    // ANGLE ERROR
    if (!imu_angle_error)
    {
        auto diagnostic_status = diagnostic_msgs::msg::DiagnosticStatus();
        diagnostic_status.level = ERROR;
        diagnostic_status.name = "IMU Angle acquisition.";
        diagnostic_status.message = "Error while acquiring Angular velocity";
        imu_diagnostic_message.status.push_back(diagnostic_status);
    }
    diagnostic_publisher_->publish(imu_diagnostic_message);
    */
   
   imu_data_publisher_->publish(imu_data_message);
}

PublisherIMU::PublisherIMU(): Node("imu_publisher")
{            
    imu_data_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);
    imu_diagnostic_publisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("imu_diagnostic", 10);

    timer_ = this->create_wall_timer(200ms, std::bind(&PublisherIMU::timer_callback, this));

    int ret = WT61P_begin(i2c_device, WT61P_IIC_ADDR);
};

/*
Diagnostic status
- [0]: General INFO
- [1 - 3]: Errors encountered (Acc, AngVel, Angle)
*/