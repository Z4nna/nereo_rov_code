#include "nereo_sensors_pkg/barPub.hpp"
#include <iostream>
#include <thread>
#include <string>
#include <semaphore.h>
#include <fcntl.h>

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
    sem_t *i2c_semaphore;
    int oflag = O_CREAT;
    mode_t mode = 0644;
    const char *sem_name = "semafero";
    unsigned int value = 1;
    int sts;
    bool sem_created = false;
    i2c_semaphore = sem_open(sem_name, oflag, mode, value);

    res = ms5837_basic_init(MS5837_TYPE_30BA26);
    // DIAGNOSTIC DURING INITIALIZATION
    if (res != 0) std::cout << "Error initializing the barometer." << std::endl;
    else std::cout << "Barometer initialized correctly." << std::endl;
    int sleep_nanoseconds = atoi(argv[1]);
    int number_of_iterations = atoi(argv[2]);

    for(int i = 0; i < number_of_iterations; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_nanoseconds));

        sts = sem_wait(i2c_semaphore);
        hasError = ms5837_basic_read(&temperature_c, &pressure_mbar);
        sem_post(i2c_semaphore);
        

        // DIAGNOSTIC DURING ACQUISITION
        if (hasError) std::cout << "Error while acquiring data." << std::endl;
        else std::cout << "Data acquired correctly." << std::endl;

        std::cout << "Temperature: " << temperature_c << std::endl;
        std::cout << "Fluid pressure: " << pressure_mbar << std::endl;
    }
    sem_close(i2c_semaphore);
    return 0;
}