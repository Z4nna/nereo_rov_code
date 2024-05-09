#ifndef BAR_PUB_H
#define BAR_PUB_H

#include <chrono>
#include <memory>
#include <string>
#include <stdint.h>
#include <stdbool.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "ms5837.h"
#include <wiringPiI2C.h>

// Status values for diagnostic array
enum status {OK, WARN, ERROR, STALE};

// I2C FUNCTIONS
int32_t IICInit(ms5837_t *sensor);
int32_t IICreadBytes(uint8_t, uint8_t, uint8_t *, uint8_t);
int32_t IICwriteBytes(uint8_t, uint8_t, uint8_t *, uint8_t);

// Delay function
void delayFor(int);

class PublisherBAR: public rclcpp::Node
{
    private:
        rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr tempPublisher_;
        rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressPublisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnosticPublisher_;
        ms5837_t BARsensor;
        void timer_callback();
    public:
        PublisherBAR();
};

#endif