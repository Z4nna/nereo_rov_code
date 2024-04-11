#include <chrono>
#include <memory>
#include <string>
#include <stdint.h>
#include <stdbool.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include <diagnostic_updater/diagnostic_updater.hpp>
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

        rclcpp::TimerBase::SharedPtr diagnostics_timer_;
        diagnostic_updater::Updater diagnostic_updater_; // Diagnostics updater object

        void checkBarometerStatus(diagnostic_updater::DiagnosticStatusWrapper &stat){
        if (/* FUNZIONE DI VERIFICA */){
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Barometer is functioning normally");
        }
        else{
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Barometer is not responding");
        }

        // You can add additional diagnostic information here
        stat.add("Temperature", std::to_string(ms5837_temperature_celsius(&BARsensor)) + " C");
        stat.add("Pressure", std::to_string(ms5837_pressure_pascal(&BARsensor)) + " Pa");
        }

        void updateDiagnostics(){
            diagnostics_updater.force_update();
        }

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
            messageTemp.data.header.frame_id = "Frame ID Barometer"

            // PRESSURE
            messagePress.data.fluid_pressure = ms5837_pressure_pascal(&BARsensor)
            messagePress.data.variance = 0; // 0: Unknown variance
            messagePress.data.header.stamp = self.get_clock().now().seconds;
            messagePress.data.header.frame_id = //.... (Stringa)

            updateDiagnostics();

            // PUBLISHING
            tempPublisher_->publish(messageTemp);
            pressPublisher_->publish(messagePress);
        }

    public:
        PublisherBAR(): Node("bar_publisher"), diagnostic_updater_(this)
        {
            tempPublisher_ = this->create_publisher<sensor_msgs::msg::BarData>("barTemp_topic", 10);
            pressPublisher_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("barPress_topic", 10);
            statusPublisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("barStatus_topic", 10);

            timer_ = this->create_wall_timer(300ms, std::bind(&PublisherBAR::timer_callback, this));

            Wire.begin();

            ms5837_i2c_set_read_fn(&BARsensor, i2c_read);
            ms5837_i2c_set_write_fn(&BARsensor, i2c_write);

            ms5837_reset(&BARsensor);

            delay(20);  // It is necessary to give the barometer time to finish the initialization

            ms5837_read_calibration_data(&BARsensor);

            // DIAGNOSTIC
            diagnostic_updater_.setHardwareID("BarometerSensor"); // Hardware ID
            diagnostic_updater_.add("Barometer Status", this, &PublisherBAR::checkBarometerStatus);
            diagnostic_timer_ = this->create_wall_timer(1000ms, std::bind(&PublisherBAR::updateDiagnostic, this));

            if(sensor.calibration_loaded)
                RCLCPP_INFO(this->get_logger("Calibration OK"));
            else
                RCLCPP_ERROR(this->get_logger("Calibration failed!"));

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