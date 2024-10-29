#include "PH4502C.h"

namespace GAIT {
    
    PH4502C::PH4502C(uint8_t pin) : _pin(pin), _sensor(pin,35) {
        // Constructor: Initializes the sensor on the specified pin
    }

    void PH4502C::begin() {
        _sensor.init();  // Initialize the sensor library
    }

    float PH4502C::getPH() {
        return _sensor.read_ph_level();  // Return the pH value from the sensor
    }

} // namespace GAIT