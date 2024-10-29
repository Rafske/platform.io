#ifndef PH4502C_H
#define PH4502C_H

#include <Arduino.h>
#include <ph4502c_sensor.h>  // Include the PH4502C-Sensor library

namespace GAIT {
    
    class PH4502C {
    public:
        PH4502C(uint8_t pin = PH_SENSOR_PIN);  // Constructor with default pin defined in platformio.ini
        void begin();                          // Method to initialize the sensor
        float getPH();                         // Method to get the pH value

    private:
        uint8_t _pin;                          // Analog pin connected to the sensor
        PH4502C_Sensor _sensor;                // PH4502C-Sensor library object
    };

} // namespace GAIT

#endif
