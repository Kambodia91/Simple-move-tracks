//------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------ 
#include "defines.h"
#include "config.h"
#include "setup.h"
#include "moveTracks.h"
#include "sendCmd.h"

#include <stdint.h>
#include <ArduinoLogger.h>                         // [Serial / Terminal]

//------------------------------------------------------------------------
// objects
//------------------------------------------------------------------------
TrackSpeeds speeds;                                                       // Speed from moveTrack.cpp
SerialCommand Command;
SerialFeedback Feedback_Serial1,
               NewFeedback_Serial1,
               Feedback_Serial2,
               NewFeedback_Serial2;


//------------------------------------------------------------------------
// variables 
//------------------------------------------------------------------------
unsigned long iTimeSend = 0;
bool enable = 0;                                                          // from blynk
byte controlMode = 0;                                                     // from blynk

uint8_t idx_Serial1 = 0;                        // Index for new data pointer
uint16_t bufStartFrame_Serial1;                 // Buffer Start Frame
byte *p_Serial1;                                // Pointer declaration for the new received data
byte incomingByte_Serial1;
byte incomingBytePrev_Serial1;
bool available_Serial1 = 1;

uint8_t idx_Serial2 = 0;                        // Index for new data pointer
uint16_t bufStartFrame_Serial2;                 // Buffer Start Frame
byte *p_Serial2;                                // Pointer declaration for the new received data
byte incomingByte_Serial2;
byte incomingBytePrev_Serial2;
bool available_Serial2 = 1;

bool serial1Blynk;
bool serial2Blynk;

//------------------------------------------------------------------------
// sending procedure
//------------------------------------------------------------------------ 
void sendSerial(int8_t serialPort, int16_t uEnableMotors, int16_t uControlMode, int16_t uSpeedMaster, int16_t uSpeedSlave) {
  
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
  Command.controlMode     = (int16_t)uControlMode;   // Enable Motors
  Command.speedMaster     = (int16_t)uSpeedMaster;    // Speed Master Board
  Command.speedSlave      = (int16_t)uSpeedSlave;     // Speed Slave Board
  Command.checksum        = (uint16_t)( Command.start ^ 
                                        Command.enableMotors ^ 
                                        Command.controlMode ^ 
                                        Command.speedMaster ^ 
                                        Command.speedSlave);
  #ifdef PRINT_SERIAL_DATA
  //inf << Command.start << " , " << Command.enableMotors << " , " << Command.controlMode  << " , " << Command.speedMaster << " , " << Command.speedSlave << " , " << Command.checksum << " , " << serialPort << endl;
  #endif
  // Write to Serial
  serial->write((uint8_t *) &Command, sizeof(Command)); 
}

//------------------------------------------------------------------------
// receiver procedure
//------------------------------------------------------------------------ 

void Receive_serial_1()
{
  // Serial 1//
    // Check for new data availability in the Serial buffer
    if (Serial1.available()) {
        incomingByte_Serial1 	  = Serial1.read();                                   // Read the incoming byte
        bufStartFrame_Serial1	= ((uint16_t)(incomingByte_Serial1) << 8) | incomingBytePrev_Serial1;       // Construct the start frame
    }
    else {
        return;
    }

  // If DEBUG_RX is defined print all incoming bytes
  #ifdef DEBUG_SERIAL1_RX
        Serial.print(Serial1.read());
        return;
    #endif

    // Copy received data
    if (bufStartFrame_Serial1 == START_FRAME) {	                    // Initialize if new data is detected
        p_Serial1       = (byte *)&NewFeedback_Serial1;
        *p_Serial1++    = incomingBytePrev_Serial1;
        *p_Serial1++    = incomingByte_Serial1;
        idx_Serial1     = 2;	
    } else if (idx_Serial1 >= 2 && idx_Serial1 < sizeof(SerialFeedback)) {  // Save the new received data
        *p_Serial1++    = incomingByte_Serial1; 
        idx_Serial1++;
    }	
    
    // Check if we reached the end of the package
    if (idx_Serial1 == sizeof(SerialFeedback)) {
        uint16_t checksum;
        checksum = (uint16_t)(NewFeedback_Serial1.start ^ 
                              NewFeedback_Serial1.cmd1 ^ 
                              NewFeedback_Serial1.cmd2 ^ 
                              NewFeedback_Serial1.speedR_meas ^ 
                              NewFeedback_Serial1.speedL_meas ^ 
                              NewFeedback_Serial1.batVoltage ^ 
                              NewFeedback_Serial1.boardTempMaster ^ 
                              NewFeedback_Serial1.boardTempSlave ^ 
                              NewFeedback_Serial1.enableFinMaster ^ 
                              NewFeedback_Serial1.enableFinSlave ^ 
                              NewFeedback_Serial1.chargeStatus ^ 
                              NewFeedback_Serial1.cmdLed);

        // Check validity of the new data
        if (NewFeedback_Serial1.start == START_FRAME && checksum == NewFeedback_Serial1.checksum) {
            // Copy the new data
            memcpy(&Feedback_Serial1, &NewFeedback_Serial1, sizeof(SerialFeedback));
            #ifdef PRINT_SERIAL_DATA
            // Print data to built-in Serial
            Serial.print("Serial_1: 1: ");   Serial.print(Feedback_Serial1.cmd1);
            Serial.print(" 2: ");  Serial.print(Feedback_Serial1.cmd2);
            Serial.print(" 3: ");  Serial.print(Feedback_Serial1.speedR_meas);
            Serial.print(" 4: ");  Serial.print(Feedback_Serial1.speedL_meas);
            Serial.print(" 5: ");  Serial.print(Feedback_Serial1.batVoltage);
            Serial.print(" 6: ");  Serial.print(Feedback_Serial1.boardTempMaster);
            Serial.print(" 7: ");  Serial.print(Feedback_Serial1.boardTempSlave);
            Serial.print(" 8: ");  Serial.print(Feedback_Serial1.enableFinMaster);
            Serial.print(" 9: ");  Serial.print(Feedback_Serial1.enableFinSlave);
            Serial.print(" 10: ");  Serial.print(Feedback_Serial1.chargeStatus);
            Serial.print(" 11: ");  Serial.println(Feedback_Serial1.cmdLed);
            #endif
        } else {
          #ifdef PRINT_SERIAL_DATA
          Serial.println("Software Serial_1 Non-valid data skipped");
          #endif
        }
        idx_Serial1 = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
    }

    // Update previous states
    incomingBytePrev_Serial1 = incomingByte_Serial1; 
}

void Receive_serial_2()
{
  // Serial2 //
    // Check for new data availability in the Serial buffer
    if (Serial2.available()) {
        incomingByte_Serial2 	  = Serial2.read();                                   // Read the incoming byte
        bufStartFrame_Serial2	= ((uint16_t)(incomingByte_Serial2) << 8) | incomingBytePrev_Serial2;       // Construct the start frame
    }
    else {
        return;
    }

  // If DEBUG_RX is defined print all incoming bytes
  #ifdef DEBUG_SERIAL2_RX
        Serial.print(Serial2.read());
        return;
    #endif

    // Copy received data
    if (bufStartFrame_Serial2 == START_FRAME) {	                    // Initialize if new data is detected
        p_Serial2       = (byte *)&NewFeedback_Serial2;
        *p_Serial2++    = incomingBytePrev_Serial2;
        *p_Serial2++    = incomingByte_Serial2;
        idx_Serial2     = 2;	
    } else if (idx_Serial2 >= 2 && idx_Serial2 < sizeof(SerialFeedback)) {  // Save the new received data
        *p_Serial2++    = incomingByte_Serial2; 
        idx_Serial2++;
    }	
    
    // Check if we reached the end of the package
    if (idx_Serial2 == sizeof(SerialFeedback)) {
        uint16_t checksum;
        checksum = (uint16_t)(NewFeedback_Serial2.start ^ 
                              NewFeedback_Serial2.cmd1 ^ 
                              NewFeedback_Serial2.cmd2 ^ 
                              NewFeedback_Serial2.speedR_meas ^ 
                              NewFeedback_Serial2.speedL_meas ^ 
                              NewFeedback_Serial2.batVoltage ^ 
                              NewFeedback_Serial2.boardTempMaster ^ 
                              NewFeedback_Serial2.boardTempSlave ^ 
                              NewFeedback_Serial2.enableFinMaster ^ 
                              NewFeedback_Serial2.enableFinSlave ^ 
                              NewFeedback_Serial2.chargeStatus ^ 
                              NewFeedback_Serial2.cmdLed);

        // Check validity of the new data
        if (NewFeedback_Serial2.start == START_FRAME && checksum == NewFeedback_Serial2.checksum) {
            // Copy the new data
            memcpy(&Feedback_Serial2, &NewFeedback_Serial2, sizeof(SerialFeedback));
            #ifdef PRINT_SERIAL_DATA
            // Print data to built-in Serial
            Serial.print("Serial_2: 1: ");   Serial.print(Feedback_Serial2.cmd1);
            Serial.print(" 2: ");  Serial.print(Feedback_Serial2.cmd2);
            Serial.print(" 3: ");  Serial.print(Feedback_Serial2.speedR_meas);
            Serial.print(" 4: ");  Serial.print(Feedback_Serial2.speedL_meas);
            Serial.print(" 5: ");  Serial.print(Feedback_Serial2.batVoltage);
            Serial.print(" 6: ");  Serial.print(Feedback_Serial2.boardTempMaster);
            Serial.print(" 7: ");  Serial.print(Feedback_Serial2.boardTempSlave);
            Serial.print(" 8: ");  Serial.print(Feedback_Serial2.enableFinMaster);
            Serial.print(" 9: ");  Serial.print(Feedback_Serial2.enableFinSlave);
            Serial.print(" 10: ");  Serial.print(Feedback_Serial2.chargeStatus);
            Serial.print(" 11: ");  Serial.println(Feedback_Serial2.cmdLed);
            #endif
        } else {
          #ifdef PRINT_SERIAL_DATA
          Serial.println("Software Serial_2 Non-valid data skipped");
          #endif
        }
        idx_Serial2 = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
    }

    // Update previous states
    incomingBytePrev_Serial2 = incomingByte_Serial2;
}

//------------------------------------------------------------------------
// procedures send command setup
//------------------------------------------------------------------------ 
void setupSendCmd() {
  Serial1.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, 27, 26 );
  delay(100);
  Serial2.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, 16, 17 );
  delay(100);
}

//------------------------------------------------------------------------
// procedures send command loop
//------------------------------------------------------------------------ 
void loopSendCmd() {
  // Send commands 
  unsigned long timeNow = millis();
  if (timeNow - iTimeSend >= TIME_SEND) {
    iTimeSend = timeNow;
    // Uart1//
    sendSerial(1, enable, controlMode, -speeds.leftSpeed, speeds.leftSpeed); 
    //                                         LP                LT    
    // Uart2 //
    sendSerial(2, enable, controlMode, speeds.rightSpeed, -speeds.rightSpeed);
    //                                         PP                PT
  }

// Receive commands
Receive_serial_1();
Receive_serial_2();
}

//------------------------------------------------------------------------
// end file
//------------------------------------------------------------------------ 