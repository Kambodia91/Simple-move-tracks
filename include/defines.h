#ifndef DEFINES_H
#define DEFINES_H

#include <Arduino.h>

//-------------------------BLYNK DEFINES----------------------------//
#define blynk_token             "o6uW3mPLvn_kHlFuyCeDox48s4sCUvxI"  // Token do ESP.
//#define BRIDGE_TOKEN                                                // Włączenie Mostu między Blynkiem.
#define bridge_token            "ieDu65nyAK1b2zeVFdpWcV18I7QeIBqy"  // Token od ESP.
#define blynk_bridge_pin        201                                 // Virtual Pin Bridge Blynk.
#define blynk_server            "home-server.tailf80b3b.ts.net"     //"kambodia.ddns.net"                 // Zewnętrzny Adres Servera Blynk.
//#define blynk_server            "192.168.1.10"                      // Lokalny Adres Servera Blynk.
#define blynk_port              "8080"                              // Port Servera Blynk
#define blynk_time_out          10                                  // Czas gdzy urzadzenie nie ma połączenia z serwerem Blynk nastepuje restat. (Czas w sekundach)
//----------------------HTTP UPDATE DEFINES-------------------------//
#define Http_ota_server         "kambodia.ddns.net"                 // Adres Http Servera OTA z Plikami *.bin
#define Http_ota_port           "80"                                // Port Http Servera OTA
//--------------------------WIFI DEFINES----------------------------//

#define MySsid                  "NETIA"                             // Nazwa Sieci
#define MyPass                  "Nikuda2518"                        // Hasło Sieci
#define terminal_name_device    "[ESP32] "                          // Nazwa Wyświetlana w Terminalu.
#define Name_ESP                "RcLawnMower"                       // Nazwa Hosta w Routerze.

//-----------------------WIFIMANAGER DEFINES------------------------//
#define BRIDGE_TOKEN                                                // Dodanie Do WifiManagera Mozliwość Wpisania Bridge Token.
#define static_ip               "192.168.1.50"                      // Ip Urządzenia
#define static_gw               "192.168.1.1"                       // Ip Routera
#define static_sn               "255.255.255.0"                     // Maska Podsieci

//---------------------------Esp Setings----------------------------//
#define SpeedRate               115200                              // Prędkość Serial.

        // HAL Sensor signal in
#define SENSOR_PIN              1                                   // Input pin.
        // REALY signal out
#define OUTPUT_PIN              16                                  // Output pin.


#define ANALOG_IN_RES           12                                  // 12bit.
#define PWM_FREQ                5000                                // 5000hz.
#define PWM_RES                 10                                  // 10bit.
                           
#define SERWO_THROTTLE          15

#define HALL_PIN                12                                  // [IN] czujnik hall do pomiaru obrotow silnika spalinowego.
#define safetyStopPin           25                                  // [OUT] Stycznik rozłącznik iskry.                         [3]
#define SWITCH_3                33
#define CHARGING_PIN            14                                  // [OUT] Stycznik Załączenie ładowania w Alternatorze.      [1]
#define SWITCH_4                32
//-------------------------Send/Recived-Cmd-------------------------//

#define HOVER_SERIAL_BAUD   115200      // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define START_FRAME         0xABCD     	// [-] Start frme definition for reliable serial communication
#define TIME_SEND           50          // [ms] Sending time interval
#define SERIAL_TIMEOUT      2000         // [-] Serial timeout duration for the received data. 160 ~= 0.8 sec. Calculation: 0.8 sec / 0.005 sec
// #define PRINT_SERIAL_DATA
// #define DEBUG_SERIAL1_RX              // [-] Debug received data. Prints all bytes to serial (comment-out to disable)
// #define DEBUG_SERIAL2_RX              // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

// Leds definitions
#define LED1_SET            (0x01)
#define LED2_SET            (0x02)
#define LED3_SET            (0x04)
#define LED4_SET            (0x08)
#define LED5_SET            (0x10)

//---------------------------VOLTAGE REG----------------------------//
// Kalibracja Napiecia
#define ALT_FILT_COEF           655                                 // battery voltage filter coefficient in fixed-point. coef_fixedPoint = coef_floatingPoint * 2^16. In this case 655 = 0.01 * 2^16
#define BAT_CELLS               10                                  // battery number of cells. Normal Hoverboard battery: 10s  
#define ALT_CALIB_ADC           1495                                // input voltage measured by multimeter (multiplied by 100). In this case 43.00 V * 100 = 4300
#define ALT_CALIB_REAL_VOLTAGE  3947                                // adc-value measured by mainboard (value nr 5 on UART debug output)
// Kalibracja pid
#define PID_KP                  10                                  // 
#define PID_KI                  2                                   // 
#define PID_KD                  5                                   // 
#define PID_LIMIT_MIN           0                                   // 
#define PID_LIMIT_MAX           1023                                // 
#define PID_SET_POINT           2000                                // 

//ADC_MODE(ADC_VCC);                                                  // Pomiar Napięcia ADC.
//-----------------------------STARTER------------------------------//

#define PID_KP                  10                                  // 
#define PID_KI                  2                                   // 
#define PID_KD                  5                                   // 
//----------------------------End File------------------------------//

#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))

#endif