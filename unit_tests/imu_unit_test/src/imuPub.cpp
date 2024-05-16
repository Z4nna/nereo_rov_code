#include "nereo_sensors_pkg/imuPub.hpp"

char *i2c_device = "/dev/i2c-1";

typedef struct {
    Vec3 angles;
    Vec3 angular_velocity;
    Vec3 linear_acceleration;
} ImuValues;

int main(int argc, char const *argv[])
{
    std::cout << "Started main." << std::endl;
    int ret = WT61P_begin(i2c_device, WT61P_IIC_ADDR );
    if (ret) 
    {
        std::cout << "Error initializing the IMU." << std::endl;
    } else 
    {
        std::cout << "WT61P initialized correctly." << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));
    for (;;std::this_thread::sleep_for(std::chrono::milliseconds(500)))
    {
        ImuValues data;
        
        WT61P_read_angular_vel();
        data.angular_velocity.x = WT61P_get_angular_vel_x();
        data.angular_velocity.y = WT61P_get_angular_vel_y();
        data.angular_velocity.z = WT61P_get_angular_vel_z();

        WT61P_read_acc();
        data.linear_acceleration.x = WT61P_get_acc_x();
        data.linear_acceleration.y = WT61P_get_acc_y();
        data.linear_acceleration.z = WT61P_get_acc_z();

        WT61P_read_angle();
        data.angles = { WT61P_get_pitch(), WT61P_get_roll(), WT61P_get_yaw() };

        std::cout << "Pitch: " << data.angles.x << "    Roll: " << data.angles.y << "    Yaw: " << data.angles.z << std::endl;
        std::cout << "AngVel - X: " << data.angular_velocity.x << "    AngVel - Y: " << data.angular_velocity.y << "    AngVel - Z: " << data.angular_velocity.z << std::endl;
        std::cout << "LinAcc - X: " << data.linear_acceleration.x << "    LinAcc - Y: " << data.linear_acceleration.y << "    LinAcc - Z: " << data.linear_acceleration.z << std::endl; 
    }
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

/*
void timer_callback()
{
   
    

    // Linear acceleration covariance
    calcCovMatrix(dataWindowAcc, matrix);

    for (int i = 0; i < 9; i++)
        data.linear_acceleration_covariance[i] = matrix[i];

    // Angular velocity covariance
    calcCovMatrix(dataWindowAngVel, matrix);

    for (int i = 0; i < 9; i++)
        data.angular_velocity_covariance[i] = matrix[i];

    // Orientation covariance - SBAGLIATA - CONVERSIONE IN QUATERNIONI E POI MATRICE
    calcCovMatrix(dataWindowAngle, matrix);
    
    for (int i = 0; i < 9; i++)
        data.orientation_covariance[i] = matrix[i];

    // Diagnostic
    

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

    dataIMUpublisher_->publish(data);
    diagnosticPublisher_->publish(diagnosticMessage);
} */

/*Diagnostic status
- [0]: General INFO
- [1 - 3]: Errors encountered (Acc, AngVel, Angle)
- */
