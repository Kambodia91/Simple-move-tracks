#ifndef SENDCMD_H
#define SENDCMD_H

#include <stdint.h>
//------------------------------------------------------------------------
// external variables 
//------------------------------------------------------------------------

//------------------------------------------------------------------------
// struct
//------------------------------------------------------------------------ 

typedef struct{
   uint16_t start;
   int16_t  enableMotors;
   int16_t  controlMode;
   int16_t  speedLeft;
   int16_t  speedRight;
   uint16_t checksum;
} SerialCommand;


typedef struct{
   uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   int16_t  speedR_meas;
   int16_t  speedL_meas;
   int16_t  batVoltage;
   int16_t  boardTempMaster;
   int16_t  boardTempSlave;
   int16_t  enableFinMaster;
   int16_t  enableFinSlave;
   int16_t  chargeStatus;
   uint16_t cmdLed;
   int16_t  motor_dc_currMaster;
   int16_t  motor_dc_currSlave;
   uint16_t checksum;
} SerialFeedback;

//------------------------------------------------------------------------
// external objects
//------------------------------------------------------------------------ 

extern SerialFeedback Feedback_Serial1,
                      NewFeedback_Serial1,
                      Feedback_Serial2,
                      NewFeedback_Serial2;

//------------------------------------------------------------------------
// procedures
//------------------------------------------------------------------------ 

void sendSerial(int8_t serialPort, int16_t uEnableMotors, int16_t uControlMode, int16_t uSpeedMaster, int16_t uSpeedSlave);
void Receive_serial_1();
void Receive_serial_2();
void loopSendCmd();
void setupSendCmd();

#endif