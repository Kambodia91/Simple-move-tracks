#ifndef DEFINES_H
#define DEFINES_H

#include <Arduino.h>


#define MySsid                  "NETIA"                             // Nazwa Sieci
#define MyPass                  "Nikuda2518"                        // Hasło Sieci
#define terminal_name_device    "[ESP32] "                          // Nazwa Wyświetlana w Terminalu.
#define Name_ESP                "RcLawnMower"                       // Nazwa Hosta w Routerze.


//---------------------------Esp Setings----------------------------//

#define SERIAL_RX_1             27                                  // [IN]    Serial 1 RX
#define SERIAL_TX_1             26                                  // [OUT]   Serial 1 TX
#define SERIAL_RX_2             16                                  // [IN]    Serial 2 RX
#define SERIAL_TX_2             17                                  // [OUT]   Serial 2 TX

#define ANALOG_IN_RES           12                                  // 12bit.
#define PWM_FREQ                5000                                // 5000hz.
#define PWM_RES                 10                                  // 10bit.
                           
#define SERWO_THROTTLE          15

#define HALL_PIN                12                                  // [IN]     czujnik hall do pomiaru obrotow silnika spalinowego.
#define SWITCH_1_CHARGING_PIN   14                                  // [OUT]    Stycznik Załączenie ładowania w Alternatorze.           []
#define SWITCH_2_IGNITION       25                                  // [OUT]    Stycznik rozłącznik iskry.                              []
#define SWITCH_3                33                                  // [OUT]    Sterowanie siłownikiem podnoszenia biegun               [+]
#define SWITCH_4                32                                  // [OUT]    Sterowanie siłownikiem podnoszenia biegun               [-]
#define SWITCH_CLUTCH_PIN       22                                  // [OUT]    Stycznik Załączenie sprzęgła.                           []
//-------------------------Send/Recived-Cmd-------------------------//

#define HOVER_SERIAL_BAUD       115200                                  // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define START_FRAME             0xABCD     	                            // [-] Start frme definition for reliable serial communication
#define TIME_SEND               25                                      // [ms] Sending time interval
// #define SERIAL_TIMEOUT          784                                     // [cycle] Serial timeout duration for the received data. 784 ~= 0.8 sec. Calculate (98 cycle / 0,1 sec).
#define SERIAL_TIMEOUT_MS       800                                     // [ms] Timeout for missing valid feedback frame
#define SBUS_TIMEOUT_MS         200                                     // [ms] Timeout for missing SBUS frame
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