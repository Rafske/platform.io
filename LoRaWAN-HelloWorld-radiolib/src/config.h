#ifndef _CONFIG_H
#define _CONFIG_H

#include <RadioLib.h>

// The default LoRaWAN payload
#ifndef RADIOLIB_LORAWAN_PAYLOAD
#define RADIOLIB_LORAWAN_PAYLOAD = {"RadioLib device: Hello, World!"}
#endif

// How often to send an uplink - consider legal & FUP constraints - see notes
#ifndef RADIOLIB_LORA_UPLINK_INTERVAL_SECONDS
#define RADIOLIB_LORA_UPLINK_INTERVAL_SECONDS (5UL * 50UL) // minutes x seconds
#endif

// JoinEUI - previous versions of LoRaWAN called this AppEUI
// for development purposes you can use all zeros - see wiki for details
#define RADIOLIB_LORAWAN_JOIN_EUI 0x0000000000000000

// Regional choices: EU868, US915, AU915, AS923, IN865, KR920, CN780, CN500
#ifdef RADIOLIB_LORA_REGION
const LoRaWANBand_t Region = RADIOLIB_LORA_REGION;
#else
const LoRaWANBand_t Region = EU868;
#endif

// Subband definition
#ifdef RADIOLIB_LORA_SUBBANDS
const uint8_t subBand = RADIOLIB_LORA_SUBBANDS;
#else
const uint8_t subBand = 0; // For US915, change this to 2, otherwise leave on 0
#endif

// ============================================================================
// Below is to support the sketch - only make changes if the notes say so ...
// Auto select MCU <-> radio connections
// If not detected use a potential platformio configuration
// If you get an error message when compiling, it may be that the
// pinmap could not be determined - see the notes for more info
//
// SX1262 pin order: Module(NSS/CS, DIO1, RESET, BUSY);
// SX1276 pin order: Module(NSS/CS, DIO0, RESET, DIO1);
// SX1278 pin order: Module(NSS/CS, DIO0, RESET, DIO1);
// ============================================================================

// Adafruit
#if defined(ARDUINO_SAMD_FEATHER_M0)
#pragma message("Adafruit Feather M0 with RFM95")
#pragma message("Link required on board")
SX1276 radio = new Module(8, 3, 4, 6);

// LilyGo
#elif defined(ARDUINO_TTGO_LORA32_V1)
#pragma message("TTGO LoRa32 v1 - no Display")
SX1276 radio = new Module(18, 26, 14, 33);

#elif defined(ARDUINO_TTGO_LORA32_V2)
#pragma error("ARDUINO_TTGO_LORA32_V2 awaiting pin map")

#elif defined(ARDUINO_TTGO_LoRa32_v21new) // T3_V1.6.1
#pragma message("Using TTGO LoRa32 v2.1 marked T3_V1.6.1 + Display")
SX1276 radio = new Module(18, 26, 14, 33);

#elif defined(ARDUINO_TBEAM_USE_RADIO_SX1262)
#pragma error("ARDUINO_TBEAM_USE_RADIO_SX1262 awaiting pin map")

#elif defined(ARDUINO_TBEAM_USE_RADIO_SX1276)
#pragma message("Using TTGO LoRa32 v2.1 marked T3_V1.6.1 + Display")
SX1276 radio = new Module(18, 26, 23, 33);

// Heltec
#elif defined(ARDUINO_HELTEC_WIFI_LORA_32)
#pragma error("ARDUINO_HELTEC_WIFI_LORA_32 awaiting pin map")

#elif defined(ARDUINO_heltec_wifi_kit_32_V2)
#pragma message("ARDUINO_heltec_wifi_kit_32_V2 awaiting pin map")
SX1276 radio = new Module(18, 26, 14, 35);

#elif defined(ARDUINO_heltec_wifi_kit_32_V3)
#pragma message("Using Heltec WiFi LoRa32 v3 - Display + USB-C")
SX1262 radio = new Module(8, 14, 12, 13);

#elif defined(ARDUINO_CUBECELL_BOARD)
#pragma message("Using TTGO LoRa32 v2.1 marked T3_V1.6.1 + Display")
SX1262 radio = new Module(RADIOLIB_BUILTIN_MODULE);

#elif defined(ARDUINO_CUBECELL_BOARD_V2)
#pragma error("ARDUINO_CUBECELL_BOARD_V2 awaiting pin map")

// ============================================================================
// Use definition from platformio
// -D RADIOLIB_LORA_MODULE=SX127[6,8]|SX1262
// -D RADIOLIB_LORA_MODULE_BITMAP="5, 2, 14, 4"
// ============================================================================
#elif defined(RADIOLIB_LORA_MODULE)
#pragma message("Using module configured in platformio.ini")
RADIOLIB_LORA_MODULE radio = new Module(RADIOLIB_LORA_MODULE_BITMAP);

// Unknown board
#else
#pragma message("Unknown board - no automagic pinmap available")

#endif

// Copy over the EUI's & keys in to the something that will not compile if
// incorrectly formatted
uint64_t joinEUI = RADIOLIB_LORAWAN_JOIN_EUI;
uint64_t devEUI = RADIOLIB_LORAWAN_DEV_EUI;
uint8_t appKey[] = {RADIOLIB_LORAWAN_APP_KEY};
#ifdef RADIOLIB_LORAWAN_NWK_KEY
uint8_t nwkKey[] = {RADIOLIB_LORAWAN_NWK_KEY};
#else
uint8_t* nwkKey = nullptr;
#endif

// Create the LoRaWAN node
LoRaWANNode node(&radio, &Region, subBand);

// Helper function to display any issues
void debug(bool isFail, const __FlashStringHelper* message, int state, bool Freeze) {
    if (isFail) {
        Serial.print(message);
        Serial.print("(");
        Serial.print(state);
        Serial.println(")");
        while (Freeze)
            ;
    }
}

// Helper function to display a byte array
void arrayDump(uint8_t* buffer, uint16_t len) {
    for (uint16_t c = 0; c < len; c++) {
        Serial.printf("0x%02X ", buffer[c]);
    }
    Serial.print("-> ");

    char str[len + 1];
    str[len] = '\0';

    snprintf(str, len + 1, "%s", buffer);
    Serial.println(str);
}

#endif
