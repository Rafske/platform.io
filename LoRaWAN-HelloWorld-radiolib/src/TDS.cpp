#include "TDS.h"

namespace GAIT {

    // Constructor
    TDS::TDS(uint8_t tdsPin) : _tdsPin(tdsPin) {}

    void TDS::setup() {
        // Initialize the analog pin for TDS sensor reading
        pinMode(_tdsPin, INPUT);
    }

    float TDS::getTDSValue(float temperature) {
        int analogValue = analogRead(_tdsPin); // Read raw analog signal
        float voltage = analogValue * (3.3 / 4095.0); // Convert to voltage for 3.3V system, 12-bit ADC
        return _voltageToTDS(voltage, temperature); // Convert voltage to TDS with compensation
    }

    float TDS::_voltageToTDS(float voltage, float temperature) {
        float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0); // Temp compensation
        float compensationVoltage = voltage / compensationCoefficient;
        return (133.42 * compensationVoltage * compensationVoltage * compensationVoltage 
                - 255.86 * compensationVoltage * compensationVoltage 
                + 857.39 * compensationVoltage); // TDS formula in ppm
    }

} // namespace GAIT
