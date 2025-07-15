/* This establishes the packets for communication to and from the robotic arm support via the serial port.

   Created 3/1/2021
   by erick nunez
*/

#include <Arduino.h>
#include "RobotControl.h"
#include "SerialPackets.h"
#include "UtilityFunctions.h"
#include "OpenCMNamespace.h"

/* ------------------------------------------------------------------------------------------------------/
/ Serial Packet Contructor  -----------------------------------------------------------------------------/
/-------------------------------------------------------------------------------------------------------*/
SerialPackets::SerialPackets(HardwareSerial  *ptrSer, const int baudrate)
  : _BAUDRATE{baudrate}
{
  c2cPort_M = ptrSer;
}

/* ------------------------------------------------------------------------------------------------------/
/ Serial Packets Setters and  Getters -------------------------------------------------------------------/
/-------------------------------------------------------------------------------------------------------*/
void SerialPackets::InitalizingComm(){
  c2cPort_M->begin(_BAUDRATE);
  for (int i = 0; i < 30; i++){
    if (c2cPort_M->available() > 0){
      if (c2cPort_M->read() == 0x01){
        c2cPort_M->write(0x02);
        digitalWrite(OCM::COMM_LED_PIN, HIGH);
        testingMode_M = false;
        while (c2cPort_M->available() > 0){
          c2cPort_M->read();
        }
        break;
      }
    }
    digitalWrite(OCM::COMM_LED_PIN, HIGH);
    delay(500);
    digitalWrite(OCM::COMM_LED_PIN, LOW); 
    delay(500);
  }
}

/* ------------------------------------------------------------------------------------------------------/
/ Serial Packets Setters and  Getters -------------------------------------------------------------------/
/-------------------------------------------------------------------------------------------------------*/
bool SerialPackets::DataAvailable() {
  if (Serial && testingMode_M){
    return Serial.available();
  }
  return c2cPort_M->available();
}

bool SerialPackets::DataRequested(){
  return dataRequested_M;
}

bool SerialPackets::InTestingMode(){
  return testingMode_M;
}

/* ------------------------------------------------------------------------------------------------------/
/ Serial Packet Writer ----------------------------------------------------------------------------------/
/-------------------------------------------------------------------------------------------------------*/
void SerialPackets::WritePackets(unsigned long &totalTime, RobotControl &Robot, unsigned long &lTime) {
  /* TX Packet Structure: Header: [  0,  1,  2,  3,...
                     elapsedTime:    4,  5,  6,  7,...
                       ForceX(N):    8,  9, 10, 11,...
                       ForceY(N):   12, 13, 14, 15,...
                       ForceZ(N):   16, 17, 18, 19,...
                     TorqueX(Nm):   20, 21, 22, 23,...
                     TorqueY(Nm):   24, 25, 26, 27,...
                     TorqueZ(Nm):   28, 29, 30, 31,...
                     ForceX(cts):   32, 33, 34, 35,...
                     ForceY(cts):   36, 37, 38, 39,...
                     ForceZ(cts):   40, 41, 42, 43,...
                    TorqueX(Cts):   44, 45, 46, 47,...
                    TorqueY(Cts):   48, 49, 50, 51,...
                    TorqueZ(Cts):   52, 53, 54, 55,...

                        loopTime:  144,145,146,147,...
                        CheckSum:  148,149]
  */ 
  byte dataPacket[_TX_PKT_LEN] = {0};
  uint16_t packetSum    = 0;
  int16_t byteLen       = 4;
  byte *packetBuffer;

  // Header Bytes 
  for (int16_t i = 0; i < 4; i++) {
    dataPacket[i] = _TXHEADER[i];
  }

  // Elapsed Time 
  dataPacket[4] = DXL_LOBYTE(DXL_LOWORD(totalTime));
  dataPacket[5] = DXL_HIBYTE(DXL_LOWORD(totalTime));
  dataPacket[6] = DXL_LOBYTE(DXL_HIWORD(totalTime));
  dataPacket[7] = DXL_HIBYTE(DXL_HIWORD(totalTime));

  // Motor's PresQ (Radians)
  packetBuffer = floatArrayToBytes(Robot.GetPresQ());
  for (int16_t i = _TX_presQ_SLOT; i < _TX_presQDOT_SLOT; i++){
    dataPacket[i] = packetBuffer[i - _TX_presQ_SLOT];
  }
  // Motor PresQdot (radians/sec)
  packetBuffer = floatArrayToBytes(Robot.GetPresQDot());
  for (int16_t i = _TX_presQDOT_SLOT; i < _TX_presXYZ_SLOT; i++){
    dataPacket[i] = packetBuffer[i - _TX_presQDOT_SLOT];
  }
  // Robot Present XYZ (meters)
  packetBuffer = floatArrayToBytes(Robot.GetPresXYZ());
  for (int16_t i = _TX_presXYZ_SLOT; i < _TX_presXYZdot_SLOT; i++){
    dataPacket[i] = packetBuffer[i - _TX_presXYZ_SLOT];
  }
  // Robot Present XYZdot (meters/sec)
  packetBuffer = floatArrayToBytes(Robot.GetPresXYZdot());
  for (int16_t i = _TX_presXYZdot_SLOT; i < _TX_goalQ_SLOT; i++){
    dataPacket[i] = packetBuffer[i - _TX_presXYZdot_SLOT];
  }
  // Motor GoalQ (radians)
  packetBuffer = floatArrayToBytes(Robot.GetGoalQ());
  for (int16_t i = _TX_goalQ_SLOT; i < _TX_goalQDOT_SLOT; i++){
    dataPacket[i] = packetBuffer[i - _TX_goalQ_SLOT];
  }
  // Motor GoalQdot (radians/sec)
  packetBuffer = floatArrayToBytes(Robot.GetGoalQDot());
  for (int16_t i = _TX_goalQDOT_SLOT; i < _TX_presQcts_SLOT; i++){
    dataPacket[i] = packetBuffer[i - _TX_goalQDOT_SLOT];
  }

  // Robot Current Torque Mode
  dataPacket[_TX_PKT_LEN - 7] = Robot.GetCurrentTorqueMode();

  // looptime  
  dataPacket[_TX_PKT_LEN - 6] = DXL_LOBYTE(DXL_LOWORD(lTime));
  dataPacket[_TX_PKT_LEN - 5] = DXL_HIBYTE(DXL_LOWORD(lTime));
  dataPacket[_TX_PKT_LEN - 4] = DXL_LOBYTE(DXL_HIWORD(lTime));
  dataPacket[_TX_PKT_LEN - 3] = DXL_HIBYTE(DXL_HIWORD(lTime));

  // check Sum
  for (int16_t i = 0; i < _TX_PKT_LEN - 2; i++) {
    packetSum += dataPacket[i];
  }
  dataPacket[_TX_PKT_LEN - 2] = floor(packetSum / 256);
  dataPacket[_TX_PKT_LEN - 1] = floor(packetSum % 256);

  // Flag Reset
  dataRequested_M = false;
  
  // write data packet
  if (Serial && testingMode_M){
    Serial.write(dataPacket,_TX_PKT_LEN);
    return;
  }

  c2cPort_M->write(dataPacket, _TX_PKT_LEN); 
}

/* ------------------------------------------------------------------------------------------------------/
/ Serial Packet Reader ----------------------------------------------------------------------------------/
/-------------------------------------------------------------------------------------------------------*/
void SerialPackets::ReadPackets() {
  /* RX Packet Structure:     Header: [ 0, 1, 2, 3,...
                               zeros:   4,5,6,7,...
                            CheckSum:  8,9]
  */     
  byte dataPacket[_RX_PKT_LEN];
  byte tempHeader[4];
  int16_t sumCheck;
  int16_t CHECKSUM;
  
  /* Check for instructions */
  unsigned long timeOUtTime = millis();
  if (Serial && testingMode_M){
    while (Serial.available() < _RX_PKT_LEN) {
      if (millis() - timeOUtTime > 5){
        while(Serial.available()) Serial.read();
        return;
      }
    }
  } else {
    while (c2cPort_M->available() < _RX_PKT_LEN) {
      if (millis() - timeOUtTime > 5){
        while(c2cPort_M->available()) c2cPort_M->read();
        return;
      }
    }
  }
  
  
  /* Read Instructions */
  if (Serial && testingMode_M){
    for (int16_t i = 0; i < _RX_PKT_LEN; i++) {
      dataPacket[i] = Serial.read();
    }
  } else {
    for (int16_t i = 0; i < _RX_PKT_LEN; i++) {
      dataPacket[i] = c2cPort_M->read();
    }
  }
  

  /* Verify Packet */
  CHECKSUM = bytesToCounts(dataPacket[_RX_PKT_LEN - 2], dataPacket[_RX_PKT_LEN - 1]);
  sumCheck = 0;
  for (int16_t i = 0; i < _RX_PKT_LEN - 2; i++) {
    sumCheck += dataPacket[i];
  }
  for (int16_t i = 0; i < 4; i++) {
    tempHeader[i] = dataPacket[i];
  }

  /* Escapes */
  if (sumCheck != CHECKSUM) return;
  if (memcmp(_ZEROHEADER, tempHeader, sizeof(_ZEROHEADER)) == 0){
    testingMode_M = true;
    return;
  }
  if (memcmp(_RXHEADER, tempHeader, sizeof(_RXHEADER)) != 0) return;
  

  /* Data Requested */
  dataRequested_M = true;
  return;
  
}
  
