#ifndef LORAWAN_H
#define LORAWAN_H

#include <RadioLib.h>
#include <cstdint>
#include <esp_attr.h>
#include <string>

namespace GAIT {

    // utilities & vars to support ESP32 deep-sleep. The RTC_DATA_ATTR attribute
    // puts these in to the RTC memory which is preserved during deep-sleep
    extern RTC_DATA_ATTR uint16_t bootCountSinceUnsuccessfulJoin;
    extern RTC_DATA_ATTR uint8_t LWsession[];

    template <typename LoRaModule>
    class LoRaWAN {
    public:
        LoRaWAN(const LoRaWANBand_t& region,
                const uint64_t joinEUI,
                const uint64_t devEUI,
                uint8_t appKey[16],
                uint8_t nwkKey[16],
                uint8_t pin1,
                uint8_t pin2,
                uint8_t pin3,
                uint8_t pin4,
                const uint8_t subBand = 0);

        void goToSleep();

        void setup(uint16_t bootCount);
        void loop(uint8_t fPort, std::string& uplinkPayload);

    private:
        int16_t activate(uint16_t bootCount);

        //    RADIOLIB_LORA_MODULE radio;
        LoRaModule radio;
        LoRaWANNode node;
    };

} // namespace GAIT

#endif // LORAWAN_H
