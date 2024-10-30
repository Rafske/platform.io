#include "PH4502C.h"

namespace GAIT {

    PH4502C::PH4502C(uint8_t phPin, uint8_t tempPin)
        : ph4502c(phPin, tempPin) {
    }
    float voltageNeutral = 2.5;
    float voltageSlope = 0.18;  

    void PH4502C::setup() {
        // Initialize the sensor
        pinMode(PH4502C_PH_PIN, INPUT);
        
    }

    float PH4502C::getPHLevel() {
        // Read pH value
        int sensorValue = analogRead(PH4502C_PH_PIN);
        float voltage = sensorValue * (3.3 / 4095.0);
        return 7.0 - ((voltage - voltageNeutral) / voltageSlope);
    }

} // namespace GAIT