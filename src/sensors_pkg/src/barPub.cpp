#include <chrono>
#include <memory>
#include <string>
#include <stdint.h>
#include <stdbool.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "diagnostic_array.hpp"
#include "../lib/MS5837/ms5837.h"
#include "../lib/wiringPi/wiringPiI2C.h"

using namespace std::chrono_literals;


// Status values for diagnostic array
enum status {OK, WARN, ERROR, STALE};

// I2C FUNCTIONS
int32_t WitInit();
int32_t IICreadBytes(uint8_t, uint8_t, uint8_t *, uint32_t);
int32_t IICwriteBytes(uint8_t, uint8_t, uint8_t *, uint32_t);

// Delay function
void delayFor(int);

int fd; // File descriptor for I2C communication

// NODE CLASS
class PublisherBAR: public rclcpp::Node
{
    private:
        rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr tempPublisher_;
        rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressPublisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnosticPublisher_;

        ms5837_t BARsensor = { 0 };

        void timer_callback()
        {

            uint16_t wait_us = 0;

            int diagnosticSize = 0;

            auto messageTemp = sensor_msgs::msg::Temperature();
            auto messagePress = sensor_msgs::msg::FluidPressure();

            int bar_press_error = 0;
            int bar_temp_error = 0;

            // This function will return a microsecond duration when we can safely attempt to read the value
            wait_us = ms5837_start_conversion(&BARsensor, SENSOR_PRESSURE, OSR_8192); // Pressure reading
            delayFor(wait_us);
            bar_press_error = ms5837_read_conversion(&BARsensor);

            wait_us = ms5837_start_conversion(&BARsensor, SENSOR_TEMPERATURE, OSR_8192); // Temperature reading
            delayFor(wait_us);
            bar_temp_error = ms5837_read_conversion(&BARsensor);

            ms5837_calculate(&BARsensor); // Once collected data it will convert them

            // TEMPERATURE
            messageTemp.temperature = ms5837_temperature_celcius(&BARsensor);
            messageTemp.variance = 0; // 0: Unknown variance
            messageTemp.header.stamp = this->get_clock()->now(); // Time data type from builtin_interfaces (???)
            messageTemp.header.frame_id = "Frame ID Barometer - Temperature";

            // PRESSURE
            messagePress.fluid_pressure = ms5837_pressure_pascal(&BARsensor);
            messagePress.variance = 0; // 0: Unknown variance
            messagePress.header.stamp = this->get_clock()->now();
            messagePress.header.frame_id = "Frame ID Barometer - Pressure";

            // DIAGNOSTIC
            auto diagnosticMessage = diagnostic_msgs::msg::DiagnosticArray();

            diagnosticMessage.header.stamp = this->get_clock()->now();
            diagnosticMessage.header.frame_id = "Barometer Diagnostic";

            // GENERAL ERROR ENCOUTERED
            if (!bar_press_error || !bar_temp_error){
                diagnosticMessage.status[diagnosticSize].level = ERROR;
                diagnosticMessage.status[diagnosticSize].name = "Acquisition error";
                diagnosticMessage.status[diagnosticSize++].message = "Errors encountered while acquiring data from BAROMETER";
            }
            else{
                diagnosticMessage.status[diagnosticSize].level = OK;
                diagnosticMessage.status[diagnosticSize].name = "Everything OK";
                diagnosticMessage.status[diagnosticSize++].message = "All data acquired correctly";
            }

            // PRESSURE ERROR
            if (!bar_press_error){
                diagnosticMessage.status[diagnosticSize].level = ERROR;
                diagnosticMessage.status[diagnosticSize].name = "Pressure error";
                diagnosticMessage.status[diagnosticSize++].message = "Error while acquiring Pressure";
            }

            // TEMPERATURE ERROR
            if (!bar_temp_error){
                diagnosticMessage.status[diagnosticSize].level = ERROR;
                diagnosticMessage.status[diagnosticSize].name = "Temperature error";
                diagnosticMessage.status[diagnosticSize++].message = "Error while acquiring Temperature";
            }

            // PUBLISHING
            tempPublisher_->publish(messageTemp);
            pressPublisher_->publish(messagePress);
            diagnosticPublisher_->publish(diagnosticMessage);
        }

    public:
        PublisherBAR(): Node("bar_publisher")
        {
            tempPublisher_ = this->create_publisher<sensor_msgs::msg::Temperature>("barTemp_topic", 10);
            pressPublisher_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("barPress_topic", 10);
            diagnosticPublisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnosticBAR_topic", 10);

            timer_ = this->create_wall_timer(300ms, std::bind(&PublisherBAR::timer_callback, this));

            fd = WitInit(&BARsensor);

            ms5837_i2c_set_read_fn(&BARsensor, IICreadBytes);
            ms5837_i2c_set_write_fn(&BARsensor, IICwriteBytes);

            ms5837_reset(&BARsensor);

            delayFor(20);  // It is necessary to give the barometer time to finish the initialization

            ms5837_read_calibration_data(&BARsensor);
        }
};

//=========================================================================================================

// MAIN FUNCTION

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublisherBAR>());
    rclcpp::shutdown();
    return 0;
}


int32_t WitInit(ms5837_t *sensor)
{
  fd = wiringPiI2CSetup(sensor->i2c_address); // WT61P_ADDRESS is slave address
  if (fd == -1){
    return -1; // Failed to open I2C device
  }
    return 0;
}

int32_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length)
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

int32_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length)
{
    if (fd == -1){
        return 0;
    }

    for (uint32_t i = 0; i < length; ++i){
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