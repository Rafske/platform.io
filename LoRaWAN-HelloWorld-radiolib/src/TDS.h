#ifndef TDS_H
#define TDS_H

#include <cstdint>
#include <Wire.h>

namespace GAIT {

    class TDS {
    public:
        TDS(uint8_t tdsPin);
        void setup();
        int getTDSValue();

    private:
        uint8_t _tdsPin;
        int _readTdsSensor();
    };

} // namespace GAIT

#endif // TDS_H




