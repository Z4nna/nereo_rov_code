#include "nereo_sensors_pkg/barPub.hpp"
using namespace std::chrono_literals;

// BAROMETER DATA VARIABLES
float temperature_c;       // Temperature in Celsius
float pressure_mbar;       // Pressure in mbar
float pressure_zero;       // Pressure at sea level
int res;                   // Result of barometer acquisition
int hasError = 0;     // Flag for error in barometer acquisition

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

    // ACQUISITION
    hasError = ms5837_basic_read(&temperature_c, &pressure_mbar);

    RCLCPP_INFO(this->get_logger(), "Data acquired!");

    // TEMPERATURE
    messageTemp.temperature = temperature_c;
    messageTemp.variance = 0;                                           // 0: Unknown variance
    messageTemp.header.stamp = this->get_clock()->now();                // Time data type from builtin_interfaces (???)
    messageTemp.header.frame_id = "Frame ID Barometer - Temperature";
    RCLCPP_INFO(this->get_logger(), "Temperature message filled!");
    // PRESSURE
    messagePress.fluid_pressure = pressure_mbar;
    messagePress.variance = 0;                                          // 0: Unknown variance
    messagePress.header.stamp = this->get_clock()->now();
    messagePress.header.frame_id = "Frame ID Barometer - Pressure";
    RCLCPP_INFO(this->get_logger(), "Pressure message filled!");

    diagnosticMessage.header.stamp = this->get_clock()->now();
    diagnosticMessage.header.frame_id = "Barometer Diagnostic";

    // DIAGNOSTIC DURING ACQUISITION
    if (hasError)
    {
        diagnostic_msgs::msg::DiagnosticStatus diagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus();
        diagnosticStatus.level = ERROR;
        diagnosticStatus.name = "Barometer acqiuisition error";
        diagnosticStatus.message = "Error while acquiring data from barometer";
        diagnosticMessage.status.push_back(diagnosticStatus);
        RCLCPP_INFO(this->get_logger(), "Error while acquiring data from barometer.");
    }
    else {
        diagnostic_msgs::msg::DiagnosticStatus diagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus();
        diagnosticStatus.level = OK;
        diagnosticStatus.name = "Barometer acquisition";
        diagnosticStatus.message = "Data acquired correctly";
        diagnosticMessage.status.push_back(diagnosticStatus);
        RCLCPP_INFO(this->get_logger(), "Data acquired correctly.");
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

    res = ms5837_basic_init(MS5837_TYPE_30BA26);

    // DIAGNOSTIC DURING INITIALIZATION
    if (res != 0){
        RCLCPP_INFO(this->get_logger(), "Error while initializing barometer");
        diagnostic_msgs::msg::DiagnosticStatus diagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus();
        diagnosticStatus.level = ERROR;
        diagnosticStatus.name = "Barometer initialization error";
        diagnosticStatus.message = "Error while initializing barometer";
        diagnosticMessage.status.push_back(diagnosticStatus);
        RCLCPP_INFO(this->get_logger(), "Barometer initialization error");
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Barometer initialized");
        diagnostic_msgs::msg::DiagnosticStatus diagnostic_status = diagnostic_msgs::msg::DiagnosticStatus();
        diagnostic_status.level = OK;
        diagnostic_status.name = "Barometer initialization";
        diagnostic_status.message = "Barometer initialized correctly";
        diagnosticMessage.status.push_back(diagnostic_status);
        RCLCPP_INFO(this->get_logger(), "Barometer initialized correctly");
    }
}
    // depth = (pressure_mbar-pressure_zero)*100/(9.80665*997.0f);  //997 = density fresh water [FORMULA DA RIVEDERE]