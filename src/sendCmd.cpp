//------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------ 
#include <Arduino.h>
#include <ArduinoLogger.h>                         // [Serial / Terminal]
#include "defines.h"
#include "config.h"
#include "setup.h"
#include "moveTracks.h"
#include "sendCmd.h"
#include "sbusRx.h"
#include "starter.h"
#include "prm01.h"
#include "controlServo.h"
#include "temperatureDS18B20.h"
#include "webTerminal.h"
#include "cuttingHeight.h"

#include <stdint.h>

//------------------------------------------------------------------------
// variables const
//------------------------------------------------------------------------ 

//------------------------------------------------------------------------
// objects
//------------------------------------------------------------------------
//TrackSpeeds speeds;                                                       // Speed from moveTrack.cpp
SerialCommand Command;
SerialFeedback Feedback_Serial1,
               NewFeedback_Serial1,
               Feedback_Serial2,
               NewFeedback_Serial2;


//------------------------------------------------------------------------
// variables 
//------------------------------------------------------------------------
uint32_t loop_counter;
unsigned long iTimeSend = 0;
// bool enable_Blynk = 0;                                // from blynk | 0 = OFF | 1 = ON |
byte controlMode_Blynk = 0;                     // from blynk | 0 = OPEN_Mode | 1 = FOC_Voltege | 2 = FOC_Speed | 3 = FOC_Torque | 4 = SIN_CTRL | 5 = COMM_CTRL |
// uint8_t test;
// uint8_t movement;
// bool enable_1 = 0;                         // from moweTrack
// bool enable_2 = 0;                         // from moweTrack

uint8_t idx_Serial1 = 0;                        // Index for new data pointer
byte *p_Serial1;                                // Pointer declaration for the new received data
byte incomingByte_Serial1;
byte incomingBytePrev_Serial1;

uint8_t idx_Serial2 = 0;                        // Index for new data pointer
byte *p_Serial2;                                // Pointer declaration for the new received data
byte incomingByte_Serial2;
byte incomingBytePrev_Serial2;

bool serial1Blynk;
bool serial2Blynk;

uint16_t  timeoutCntSerial_2 = 0;               // Timeout counter for Rx Serial command
uint8_t   timeoutFlgSerial_2 = 0;               // Timeout Flag for Rx Serial command: 0 = OK, 1 = Problem detected (line disconnected or wrong Rx data)
bool      timeoutMsgSerial_2 = 0;

uint16_t  timeoutCntSerial_1 = 0;               // Timeout counter for Rx Serial command
uint8_t   timeoutFlgSerial_1 = 0;               // Timeout Flag for Rx Serial command: 0 = OK, 1 = Problem detected (line disconnected or wrong Rx data)
bool      timeoutMsgSerial_1 = 0;
uint32_t lastValidSerial1 = 0;
uint32_t lastValidSerial2 = 0;
uint16_t    dirLeft = 0;
uint16_t    dirRight = 1;

//------------------------------------------------------------------------
// sending procedure
//------------------------------------------------------------------------ 
void sendSerial(int8_t serialPort, int16_t uEnableMotors, int16_t uControlMode, int16_t uSpeedLeft, int16_t uSpeedRight) {
  
  HardwareSerial* serial;
    
    switch(serialPort) {
      case 0:
        serial = &Serial;
        break;
      case 1:
        serial = &Serial1;
        break;
      case 2:
        serial = &Serial2;
        break;
        // Add more cases if needed for additional serial ports
      default:
        inf << terminal_name_device << "Invalid serial port number " << serialPort << " does not exist."<< endl; // Invalid serial port number
        return;
    }

  // Create command
  Command.start           = (uint16_t)START_FRAME;    // Start Frame  
  Command.enableMotors    = (int16_t)uEnableMotors;   // Enable Motors
  Command.controlMode     = (int16_t)uControlMode;    // Enable Motors
  Command.speedLeft       = (int16_t)uSpeedLeft;      // Speed Left
  Command.speedRight      = (int16_t)uSpeedRight;     // Speed Right
  Command.checksum        = (uint16_t)( Command.start ^ 
                                        Command.enableMotors ^ 
                                        Command.controlMode ^ 
                                        Command.speedLeft ^ 
                                        Command.speedRight);
  #ifdef PRINT_SERIAL_DATA
  //inf << Command.start << " , " << Command.enableMotors << " , " << Command.controlMode  << " , " << Command.speedLeft << " , " << Command.speedRight << " , " << Command.checksum << " , " << serialPort << endl;
  #endif
  // Write to Serial
  serial->write((uint8_t *) &Command, sizeof(Command)); 
}

//------------------------------------------------------------------------
// receiver procedure
//------------------------------------------------------------------------ 

static bool processFeedbackByte(byte incomingByte, SerialFeedback& frame,
                                uint8_t& frameIndex, byte*& framePointer,
                                byte& previousByte) {
  uint16_t startFrame = ((uint16_t)incomingByte << 8) | previousByte;

  if (startFrame == START_FRAME) {
    framePointer = (byte *)&frame;
    *framePointer++ = previousByte;
    *framePointer++ = incomingByte;
    frameIndex = 2;
  } else if (frameIndex >= 2 && frameIndex < sizeof(SerialFeedback)) {
    *framePointer++ = incomingByte;
    frameIndex++;
  }

  previousByte = incomingByte;

  if (frameIndex != sizeof(SerialFeedback)) {
    return false;
  }

  frameIndex = 0;
  uint16_t checksum = (uint16_t)(frame.start ^
                                 frame.cmd1 ^
                                 frame.cmd2 ^
                                 frame.speedR_meas ^
                                 frame.speedL_meas ^
                                 frame.batVoltage ^
                                 frame.boardTempMaster ^
                                 frame.boardTempSlave ^
                                 frame.enableFinMaster ^
                                 frame.enableFinSlave ^
                                 frame.chargeStatus ^
                                 frame.motor_dc_currMaster ^
                                 frame.motor_dc_currSlave ^
                                 frame.cmdLed);

  return frame.start == START_FRAME && checksum == frame.checksum;
}

void Receive_serial_1()
{
  while (Serial1.available() > 0) {
    incomingByte_Serial1 = Serial1.read();

    if (processFeedbackByte(incomingByte_Serial1, NewFeedback_Serial1,
                            idx_Serial1, p_Serial1,
                            incomingBytePrev_Serial1)) {
      memcpy(&Feedback_Serial1, &NewFeedback_Serial1, sizeof(SerialFeedback));
      lastValidSerial1 = millis();
      timeoutCntSerial_1 = 0;
      timeoutFlgSerial_1 = 0;
      timeoutMsgSerial_1 = 0;
    }
  }

  if ((uint32_t)(millis() - lastValidSerial1) >= SERIAL_TIMEOUT_MS) {
    timeoutFlgSerial_1 = 1;
    timeoutCntSerial_1 = SERIAL_TIMEOUT;
  }

  if (timeoutFlgSerial_1 == 1) {
    if (!timeoutMsgSerial_1) {
    inf << "Serial1 RX no connection to the inventer feedback." <<  endl;
    timeoutMsgSerial_1 = 1;
    }
  }
}

void Receive_serial_2()
{
  while (Serial2.available() > 0) {
    incomingByte_Serial2 = Serial2.read();

    if (processFeedbackByte(incomingByte_Serial2, NewFeedback_Serial2,
                            idx_Serial2, p_Serial2,
                            incomingBytePrev_Serial2)) {
      memcpy(&Feedback_Serial2, &NewFeedback_Serial2, sizeof(SerialFeedback));
      lastValidSerial2 = millis();
      timeoutCntSerial_2 = 0;
      timeoutFlgSerial_2 = 0;
      timeoutMsgSerial_2 = 0;
    }
  }

  if ((uint32_t)(millis() - lastValidSerial2) >= SERIAL_TIMEOUT_MS) {
    timeoutFlgSerial_2 = 1;
    timeoutCntSerial_2 = SERIAL_TIMEOUT;
  }

  if (timeoutFlgSerial_2 == 1) {
    if (!timeoutMsgSerial_2) {
    inf << "Serial2 RX no connection to the inventer feedback ." <<  endl;
    timeoutMsgSerial_2 = 1;
    }
  }
}



//------------------------------------------------------------------------
// procedures send command setup
//------------------------------------------------------------------------ 
void setupSendCmd() {
  Serial1.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, 27, 26 );
  Serial2.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, 16, 17 );
  lastValidSerial1 = millis();
  lastValidSerial2 = lastValidSerial1;
}

//------------------------------------------------------------------------
// procedures send command loop
//------------------------------------------------------------------------ 
void loopSendCmd() {
  // Send commands                                                                      ////////////////////////////////
  unsigned long timeNow = millis();                                                     //  Serial_1 Control/FeedBack //
  if (timeNow - iTimeSend >= TIME_SEND) {                                               //  Serial_2         FeedBack //
    iTimeSend = timeNow;                                                                //        ╔═════════╗         //
    // Uart1,     ENNABLE,   MODE,       LEFT SPEED,       RIGHT SPEED                  //   LP ╠═╣    ↑    ╠═╣  PP   //
    sendSerial(1, buttonD, 2, speeds.leftSpeed, speeds.rightSpeed);                     // MASTER ║    ↑    ║  MASTER //
    //                                                                                  //        ║    ↑    ║         //
    // Uart2 //                                                                         //        ║    ↑    ║         //
    // sendSerial(2, buttonD, 2, -speeds.rightSpeed, speeds.rightSpeed);                //   LT ╠═╣    ↑    ╠═╣  PT   //
    //                                         LP                LT                     //  SLAVE ╚═════════╝   SLAVE //
  //                                                                                    //                            //
  //                                                                                    ////////////////////////////////
//  inf << loop_counter << endl;
//   loop_counter = 0;
  }

// if (timeoutFlgSerial_2 && (loop_counter % 1000 == 0)) {
//       Feedback_Serial2.cmdLed ^= LED3_SET;
//       //Feedback_Serial2.cmdLed &= ~LED1_SET & ~LED2_SET & ~LED4_SET & ~LED5_SET;
//     }
// Receive commands
Receive_serial_1();
Receive_serial_2();
//   inf << np << " : L" << dirLeft << " Speed: "<< speeds.leftSpeed << " P: " << dirRight << " Speed: " << speeds.rightSpeed <<  " enable: " << buttonD << " LT: " << endl;
loop_counter++;

}

//------------------------------------------------------------------------
// end file
//------------------------------------------------------------------------ 