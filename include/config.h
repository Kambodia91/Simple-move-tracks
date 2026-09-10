#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#if !defined(PLATFORMIO)
#define VARIANT_ESP32_S2              // Variant
#define VARIANT_ESP32_LOLIN              // Variant
#define VARIANT_ESP32_WROOM32          // Variant
#define VARIANT_ESP8266               // Variant
//#define VARIANT_ARDUINO_PRO_MINI      // Variant
#endif

#if defined(VARIANT_ESP32_WROOM32) && !defined(LED_BUILTIN)
#define LED_BUILTIN 2
#endif

#endif