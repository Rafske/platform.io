#include "PH4502C.h"

namespace GAIT {

    PH4502C::PH4502C(uint8_t phPin, uint8_t tempPin)
        : ph4502c(phPin, tempPin) {
    }

    void PH4502C::setup(float calibration) {
        // Initialize the sensor
        ph4502c.init();
        ph4502c.recalibrate(calibration);
    }

    float PH4502C::getPHLevel() {
        // Read pH value
        return ph4502c.read_ph_level();
    }

} // namespace GAIT
