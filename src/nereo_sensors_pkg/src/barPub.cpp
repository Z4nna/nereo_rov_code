#include "nereo_sensors_pkg/barPub.hpp"
using namespace std::chrono_literals;

int fd; // File descriptor for I2C communication

//=========================================================================================================

// MAIN FUNCTION

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublisherBAR>());
    rclcpp::shutdown();
    return 0;
}

// NODE CLASS
void PublisherBAR::timer_callback()
{
    RCLCPP_INFO(this->get_logger(), "Entered timer callback!");
    uint16_t wait_us = 0;

    auto messageTemp = sensor_msgs::msg::Temperature();
    auto messagePress = sensor_msgs::msg::FluidPressure();

    int bar_press_error = 0;
    int bar_temp_error = 0;

    // This function will return a microsecond duration when we can safely attempt to read the value
    wait_us = ms5837_start_conversion(&BARsensor, SENSOR_PRESSURE, OSR_8192); // Pressure reading
    RCLCPP_INFO(this->get_logger(), "Bar started conversion");
    delayFor(wait_us);
    bar_press_error = ms5837_read_conversion(&BARsensor);
    RCLCPP_INFO(this->get_logger(), "Bar read conversion");

    wait_us = ms5837_start_conversion(&BARsensor, SENSOR_TEMPERATURE, OSR_8192); // Temperature reading
    delayFor(wait_us);
    bar_temp_error = ms5837_read_conversion(&BARsensor);

    ms5837_calculate(&BARsensor); // Once collected data it will convert them
    //RCLCPP_INFO(this->get_logger(), "Bar calculate");

    // TEMPERATURE
    messageTemp.temperature = ms5837_temperature_celcius(&BARsensor);
    messageTemp.variance = 0; // 0: Unknown variance
    messageTemp.header.stamp = this->get_clock()->now(); // Time data type from builtin_interfaces (???)
    messageTemp.header.frame_id = "Frame ID Barometer - Temperature";
    RCLCPP_INFO(this->get_logger(), "Bar temperature");

    // PRESSURE
    messagePress.fluid_pressure = ms5837_pressure_pascal(&BARsensor);
    messagePress.variance = 0; // 0: Unknown variance
    messagePress.header.stamp = this->get_clock()->now();
    messagePress.header.frame_id = "Frame ID Barometer - Pressure";
    RCLCPP_INFO(this->get_logger(), "Bar pressure");

    // DIAGNOSTIC
    auto diagnosticMessage = diagnostic_msgs::msg::DiagnosticArray();

    diagnosticMessage.header.stamp = this->get_clock()->now();
    diagnosticMessage.header.frame_id = "Barometer Diagnostic";

    // GENERAL ERROR ENCOUTERED
    RCLCPP_INFO(this->get_logger(), "Header set");
    if (!bar_press_error || !bar_temp_error){
        diagnostic_msgs::msg::DiagnosticStatus diagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus();
        RCLCPP_INFO(this->get_logger(), "If 0.0");
        diagnosticStatus.level = ERROR;
        RCLCPP_INFO(this->get_logger(), "If 0.1");
        diagnosticStatus.name = "Acquisition error";
        RCLCPP_INFO(this->get_logger(), "If 0.2");
        diagnosticStatus.message = "Errors encountered while acquiring data from BAROMETER";
        RCLCPP_INFO(this->get_logger(), "If 0.3");
        diagnosticMessage.status.push_back(diagnosticStatus);
    }
    else{
        auto diagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus();
        diagnosticStatus.level = OK;
        diagnosticStatus.name = "Barometer data acquisition.";
        diagnosticStatus.message = "Pressure and temperature data acquired correctly";
        diagnosticMessage.status.push_back(diagnosticStatus);
    }
    RCLCPP_INFO(this->get_logger(), "Exited if-else 0");
    // PRESSURE ERROR
    if (!bar_press_error){
        RCLCPP_INFO(this->get_logger(), "If 1");
        diagnostic_msgs::msg::DiagnosticStatus diagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus();
        diagnosticStatus.level = ERROR;
        diagnosticStatus.name = "Pressure acquisition error";
        diagnosticStatus.message = "Error while acquiring Pressure";
        diagnosticMessage.status.push_back(diagnosticStatus);
    }

    // TEMPERATURE ERROR
    if (!bar_temp_error){
        RCLCPP_INFO(this->get_logger(), "If 2");
        diagnostic_msgs::msg::DiagnosticStatus diagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus();
        diagnosticStatus.level = ERROR;
        diagnosticStatus.name = "Temperature acquisition error";
        diagnosticStatus.message = "Error while acquiring Temperature";
        diagnosticMessage.status.push_back(diagnosticStatus);
    }

    // PUBLISHING
    RCLCPP_INFO(this->get_logger(), "Ready to publish!");
    tempPublisher_->publish(messageTemp);
    pressPublisher_->publish(messagePress);
    diagnosticPublisher_->publish(diagnosticMessage);
    RCLCPP_INFO(this->get_logger(), "Published!");
}

PublisherBAR::PublisherBAR(): Node("bar_publisher")
{
    tempPublisher_ = this->create_publisher<sensor_msgs::msg::Temperature>("barTemp_topic", 10);
    pressPublisher_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("barPress_topic", 10);
    diagnosticPublisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnosticBAR_topic", 10);
    RCLCPP_INFO(this->get_logger(), "Publishers created");
    
    timer_ = this->create_wall_timer(300ms, std::bind(&PublisherBAR::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Timer created");

    fd = IICInit(&BARsensor);
    RCLCPP_INFO(this->get_logger(), "I2C initiated");

    ms5837_i2c_set_read_fn(&BARsensor, IICreadBytes);
    ms5837_i2c_set_write_fn(&BARsensor, IICwriteBytes);
    RCLCPP_INFO(this->get_logger(), "Set rw i2c functions");

    ms5837_reset(&BARsensor);
    //RCLCPP_INFO(this->get_logger(), "Bar reset");

    delayFor(20);  // It is necessary to give the barometer time to finish the initialization

    ms5837_read_calibration_data(&BARsensor);
    //RCLCPP_INFO(this->get_logger(), "Bar read cal data.");
}

int32_t IICInit(ms5837_t *sensor)
{
  fd = wiringPiI2CSetup(sensor->i2c_address); // WT61P_ADDRESS is slave address
  if (fd == -1){
    return -1; // Failed to open I2C device
  }
    return 0;
}

int32_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t *data, uint8_t length)
{
    if (fd == -1){
        return 0;
    }

    for (uint32_t i = 0; i < length; ++i){
        int byte = wiringPiI2CReadReg8(fd, reg + i);
        if (byte == -1){
            return 0;
        }
        data[i] = static_cast<uint8_t>(byte);
    }
    return 1;
}

int32_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t *data, uint8_t length)
{
    if (fd == -1){
        return 0;
    }

    for (uint8_t i = 0; i < length; ++i){
        int result = wiringPiI2CWriteReg8(fd, reg + i, data[i]);
        if (result == -1){
            return 0;
        }
    }
    return 1;
}

void delayFor(int ms){
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
