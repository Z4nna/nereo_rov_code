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
#include <Wire.h>
#include "lib/MS5837/ms5837.h"

using namespace std::chrono_literals;

// Barometer init
ms5837_t BARsensor = { 0 }; // I put this here because it seemed not visible for timer_callback when declared into public

// NODE CLASS
class PublisherBAR: public rclcpp::Node
{
    private:
        rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr tempPublisher_;
        rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressPublisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnosticPublisher_;

        void timer_callback()
        {
            uint16_t wait_us = 0;

            auto messageTemp = sensor_msgs::msg::Temperature();
            auto messagePress = sensor_msgs::msg::FluidPressure();

            // This function will return a microsecond duration when we can safely attempt to read the value
            wait_us = ms5837_start_conversion(&BARsensor, SENSOR_PRESSURE, OSR_8192); // Pressure reading
            delayMicroseconds(wait_us);
            ms5837_read_conversion(&BARsensor);

            wait_us = ms5837_start_conversion(&BARsensor, SENSOR_TEMPERATURE, OSR_8192); // Temperature reading
            delayMicroseconds(wait_us);
            ms5837_read_conversion(&BARsensor);

            ms5837_calculate(&BARsensor); // Once collected data it will convert them

            // TEMPERATURE
            messageTemp.data.temperature = ms5837_temperature_celsius(&BARsensor);
            messageTemp.data.variance = 0; // 0: Unknown variance
            messageTemp.data.header.stamp = self.get_clock().now().seconds; // Time data type from builtin_interfaces (???)
            messageTemp.data.header.frame_id = "Frame ID Barometer - Temperature"

            // PRESSURE
            messagePress.data.fluid_pressure = ms5837_pressure_pascal(&BARsensor)
            messagePress.data.variance = 0; // 0: Unknown variance
            messagePress.data.header.stamp = self.get_clock().now().seconds;
            messagePress.data.header.frame_id = "Frame ID Barometer - Pressure"

            // DIAGNOSTIC
            auto diagnosticMessage = diagnostic_msgs::msg::DiagnosticArray();

            diagnosticMessage.header.stamp = this->get_clock()->now();
            diagnosticMessage.header.frame_id = "Barometer Diagnostic";

            // PUBLISHING
            tempPublisher_->publish(messageTemp);
            pressPublisher_->publish(messagePress);
            diagnosticPublisher_->publish(diagnosticMessage);
        }

    public:
        PublisherBAR(): Node("bar_publisher")
        {
            tempPublisher_ = this->create_publisher<sensor_msgs::msg::BarData>("barTemp_topic", 10);
            pressPublisher_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("barPress_topic", 10);
            diagnosticPublisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnosticBAR_topic", 10);

            timer_ = this->create_wall_timer(300ms, std::bind(&PublisherBAR::timer_callback, this));

            Wire.begin();

            ms5837_i2c_set_read_fn(&BARsensor, i2c_read);
            ms5837_i2c_set_write_fn(&BARsensor, i2c_write);

            ms5837_reset(&BARsensor);

            delay(20);  // It is necessary to give the barometer time to finish the initialization

            ms5837_read_calibration_data(&BARsensor);
        }
}

//=========================================================================================================

// MAIN FUNCTION

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublisherBAR>());
    rclcpp::shutdown();
    return 0;
}