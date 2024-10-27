/*

This demonstrates how to save the join information in to permanent memory
so that if the power fails, batteries run out or are changed, the rejoin
is more efficient & happens sooner due to the way that LoRaWAN secures
the join process - see the wiki for more details.

This is typically useful for devices that need more power than a battery
driven sensor - something like a air quality monitor or GPS based device that
is likely to use up it's power source resulting in loss of the session.

The relevant code is flagged with a ##### comment

Saving the entire session is possible but not demonstrated here - it has
implications for flash wearing and complications with which parts of the
session may have changed after an uplink. So it is assumed that the device
is going in to deep-sleep, as below, between normal uplinks.

Once you understand what happens, feel free to delete the comments and
Serial.prints - we promise the final result isn't that many lines.

*/

#if !defined(ESP32)
#pragma error("This is not the example your device is looking for - ESP32 only")
#endif

// ##### load the ESP32 preferences facilites
#include <Preferences.h>

RTC_DATA_ATTR uint16_t bootCount = 0;

#include "GPS.h"
#include "LoRaWAN.hpp"

static GAIT::LoRaWAN<RADIOLIB_LORA_MODULE> loRaWan(RADIOLIB_LORA_REGION,
                                                   RADIOLIB_LORAWAN_JOIN_EUI,
                                                   RADIOLIB_LORAWAN_DEV_EUI,
                                                   (uint8_t[16]) {RADIOLIB_LORAWAN_APP_KEY},
                                                   (uint8_t[16]) {RADIOLIB_LORAWAN_NWK_KEY},
                                                   RADIOLIB_LORA_MODULE_BITMAP);

static GAIT::GPS gps(2, 9600, SERIAL_8N1, 16, 17);

// abbreviated version from the Arduino-ESP32 package, see
// https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/api/deepsleep.html
// for the complete set of options
void print_wakeup_reason() {
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
        Serial.println(F("Wake from sleep"));
    } else {
        Serial.print(F("Wake not caused by deep sleep: "));
        Serial.println(wakeup_reason);
    }

    Serial.print(F("Boot count: "));
    Serial.println(++bootCount); // increment before printing
}

// put device in to lowest power deep-sleep mode
void gotoSleep(uint32_t seconds) {
    loRaWan.goToSleep();

    esp_sleep_enable_timer_wakeup(seconds * 1000UL * 1000UL); // function uses uS
    esp_deep_sleep_start();

    // if this appears in the serial debug, we didn't go to sleep!
    // so take defensive action so we don't continually uplink
    Serial.println(F("\n\n### Sleep failed, delay of 5 minutes & then restart ###\n"));
    delay(5UL * 60UL * 1000UL);
    ESP.restart();
}

std::string uplinkPayload = RADIOLIB_LORAWAN_PAYLOAD;
uint8_t fPort = 1;
// For application use: 1 ... 223,
// reserved for further use: 224 ... 255,
// reserved for mac commands: 0
// Here 221 (info), 222 (warning), 223 (error) are used

// setup & execute all device functions ...
void setup() {
    Serial.begin(115200);
    while (!Serial)
        ;        // wait for serial to be initalised
    delay(2000); // give time to switch to the serial monitor

    Serial.println(F("\nSetup"));

    print_wakeup_reason();

    loRaWan.setup(bootCount);

    // build uplinkPayload byte array
    Serial.println(F("[APP] Constructing uplink"));

    Serial.println(F("Aquire data"));

    gps.setup();

    if (gps.gpsIsValid()) {
        fPort = 1; // 1 is location
        uplinkPayload = std::to_string(gps.getLatitude()) + "," + std::to_string(gps.getLongitude()) + "," +
                        std::to_string(gps.getAltitude()) + "," + std::to_string(gps.getHdop());
    } else {
        fPort = 222; // 222 is warning. 223 is error, 222 warning, 221 info message
        uplinkPayload = RADIOLIB_LORAWAN_PAYLOAD;
    }
}

void loop() {
    loRaWan.loop(fPort, uplinkPayload);
}

// Does it respond to a UBX-MON-VER request?
// uint8_t ubx_mon_ver[] = { 0xB5,0x62,0x0A,0x04,0x00,0x00,0x0E,0x34 };
