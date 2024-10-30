#include "GravityTDS1.h"

namespace GAIT {

    GravityTDS::GravityTDS(uint8_t pin, float vcc, uint16_t adcResoluton)
        : pin(pin) {
        gravityTds.setPin(pin);
        gravityTds.setAref(vcc);              // reference voltage on ADC, default 5.0V on Arduino UNO
        gravityTds.setAdcRange(adcResoluton); // 1024 for 10bit ADC;4096 for 12bit ADC
    }

    void GravityTDS::setup() {
        gravityTds.begin(); // initialization
    }

    float GravityTDS::getValue(float temperature) {
        gravityTds.setTemperature(temperature); // set the temperature and execute temperature compensation
        gravityTds.update();                    // sample and calculate
        return gravityTds.getTdsValue();        // then get the value
    }

} // namespace GAIT
