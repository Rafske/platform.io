#include "TDS.h"

namespace GAIT {

    TDS::TDS(uint8_t tdsPin) : _tdsPin(tdsPin) {}

    void TDS::setup() {
        pinMode(_tdsPin, INPUT); // Initialize the TDS sensor pin as input
    }

    int TDS::getTDSValue() {
        return _readTdsSensor(); // Get the TDS value in ppm
    }

    int customMap(int value, int fromLow, int fromHigh, int toLow, int toHigh) {
    return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
    }

    int TDS::_readTdsSensor() {
    int sensorValue = analogRead(_tdsPin);             // Read analog value from TDS sensor
    int tdsValue = customMap(sensorValue, 0, 4095, 0, 1000); // Convert analog value to TDS in ppm
    return tdsValue;
    }


} // namespace GAIT
