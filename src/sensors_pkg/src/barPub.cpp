#include <chrono>
#include <memory>
#include <string>
#include <stdint.h>
#include <stdbool.h>
#include <i2c.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "../../nereo_interfaces/msg/BarData.hpp" // Not existing file. Write it before sending
#include "../lib/MS5837/ms5837.h"

#define I2C_ADDR 0x18 // I2C slave address of the barometer sensor
#define I2C_BUS 1 // I2C bus number (usually 1 or 0)

# define ERROR_VAL -10000000000

int toPublish = 1;

typedef enum {Temp, Press} sensorBool;

using namespace std::chrono_literals;

// USER-DEFINED STRUCT

typedef struct {
    float temperature;
    float pressure;
} barometer_data;

//=================================================================================================

// USER-DEFINED FUNCTIONS

void init_i2c(void){
    i2c_init();
    i2c_set_slave_address(I2C_ADDR);
}

float dataToFloat(uint8_t *buffer, sensorBool command){
    switch (command)
    {
    case Temp:
        return ((buffer[0] << 8) | buffer[1]) * 0.01;
        break;
    case Press:
        return ((buffer[2] << 8) | buffer[3]) * 0.01;
    default:
        return ERROR_VAL; //In case of an error, I'll return -inf to easily spot it
    }
}

barometer_data read_data(void){
    barometer_data data;
    uint8_t buffer[2]; // Store data from the sensor

    i2c_write(0x01, 0x00, 0x00, 0x00, 0x00); // Temperature

    i2c_write(0x02, 0x00, 0x00, 0x00, 0x00); // Pressure

    i2c_read(0x01, buffer, 2); // Read Temperature

    i2c_read(0x02, buffer, 2); // Read Pressure

    data.temperature = dataToFloat(&buffer, Temp);
    data.pressure = dataToFloat(&buffer, Press);

    return data;
}

//===========================================================================================================

// NODE CLASS

class PublisherBAR: public rclcpp::Node
{
    private:
        size_t count_;
        rclcpp::Publisher<nereo_interfaces::msg::BarData>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        void timer_callback()
        {
            barometer_data data;
            auto message = nereo_interfaces::msg::BarData();

            // Getting data from Barometer
            data = read_data();

            // I use this flag to spot errors while getting data, in order to avoid them from being published
            toPublish = 1;

            if (data.temperature == ERROR_VAL){
                RCLCPP_ERROR(this->get_logger(), "Error while getting TEMPERATURE data");
                toPublish = 0;
            }
            if (data.pressure == ERROR_VAL){
                RCLCPP_ERROR(this->get_logger(), "Error while getting PRESSURE data");
                toPublish = 0;
            }

            // ADD HERE THE TIME COLLECTION

            if (toPublish){
                data.isValid = 1;
                publisher_->publish(data);
            }
            else{
                data.isValid = 0;
                publisher_->publish()
            }
        }

    public:
        PublisherBAR(): Node("bar_publisher"), count_()
        {
            publisher_ = this->create_publisher<nereo_interfaces::msg::BarData>("bar_topic", 10);
            timer_ = this->create_wall_timer(200ms, std::bind(&PublisherBAR::timer_callback, this));
            
            // Barometer init
            ms5837_t BARsensor = { 0 };

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

//========================================================================================================

/* Class implemented in nereo_interfaces (type of the message to publish)
class BarData
{
    private:
        int year;
        int month;
        int day;
        int hours;
        int minutes;
        int seconds;
        uint8_t command;
        float depth;
        float pressure;
        float temperature;
        int isValid;     // I use this to get aware of possible errors in data collection
}
*/