#include "nereo_sensors_pkg/barPub.hpp"
#include <iostream>
#include <thread>
//using namespace std::chrono_literals;

// BAROMETER DATA VARIABLES
float temperature_c;       // Temperature in Celsius
float pressure_mbar;       // Pressure in mbar
float pressure_zero;       // Pressure at sea level
int res;                   // Result of barometer acquisition
bool hasError = false;     // Flag for error in barometer acquisition

//=========================================================================================================

// MAIN FUNCTION
int main(int argc, char const *argv[])
{
    res = ms5837_basic_init(MS5837_TYPE_30BA26);
    // DIAGNOSTIC DURING INITIALIZATION
    if (res != 0) std::cout << "Error initializing the barometer." << std::endl;
    else std::cout << "Barometer initialized correctly." << std::endl;

    for(;;) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    hasError = ms5837_basic_read(&temperature_c, &pressure_mbar);

    // DIAGNOSTIC DURING ACQUISITION
    if (hasError) std::cout << "Error while acquiring data." << std::endl;
    else std::cout << "Data acquired correctly." << std::endl;

    std::cout << "Temperature: " << temperature_c << std::endl;
    std::cout << "Fluid pressure: " << pressure_mbar << std::endl;
    }
    return 0;
}