#ifndef TDS_H
#define TDS_H

#include <cstdint>
#include <Wire.h>

namespace GAIT {

    class TDS {
    public:
        TDS(uint8_t tdsPin);

        void setup();
        float getTDSValue(float temperature);  // Reads TDS value, using temperature for compensation

    private:
        uint8_t _tdsPin;
        float _voltageToTDS(float voltage, float temperature);
    };

} // namespace GAIT

#endif // TDS_H
