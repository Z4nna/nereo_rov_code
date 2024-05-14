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

#include "barometer_libs/driver_ms5837.h"
#include "barometer_libs/driver_ms5837_basic.h"
#include "barometer_libsdriver_ms5837_interface.h"
#include "barometer_libs/iic.h"

// STATUS VALUES FOR DIAGNOSTIC
enum status {OK, WARN, ERROR, STALE};


class PublisherBAR: public rclcpp::Node
{
    private:
        rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr tempPublisher_;
        rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressPublisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnosticPublisher_;

        void timer_callback();

        sensor_msgs::msg::Temperature messageTemp = sensor_msgs::msg::Temperature();
        sensor_msgs::msg::FluidPressure messagePress = sensor_msgs::msg::FluidPressure();
        diagnostic_msgs::msg::DiagnosticArray diagnosticMessage = diagnostic_msgs::msg::DiagnosticArray();
    public:
        PublisherBAR();
};

#endif