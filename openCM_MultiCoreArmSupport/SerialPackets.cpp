/* This establishes the packets for communication to and from the robotic arm support via the serial port.

   Created 3/1/2021
   by erick nunez
*/

#include <Arduino.h>
#include "RobotControl.h"
#include "SerialPackets.h"
#include "UtilityFunctions.h"

/* ------------------------------------------------------------------------------------------------------/
/ Serial Packet Contructor  -----------------------------------------------------------------------------/
/-------------------------------------------------------------------------------------------------------*/
SerialPackets::SerialPackets(USBSerial *ptrSer, const int baudrate)
  : _BAUDRATE{baudrate}
{
  SerialPort_M = ptrSer;
  SerialPort_M->begin(_BAUDRATE);
}

/* ------------------------------------------------------------------------------------------------------/
/ Serial Packets Setters and  Getters -------------------------------------------------------------------/
/-------------------------------------------------------------------------------------------------------*/
bool SerialPackets::DataAvailable() {
  return SerialPort_M->available();
}

bool SerialPackets::NewGoalAvailable(){
  return newGoal_M;
}

uint8_t SerialPackets::GetNewMode(){
  return mode_M;
}

float * SerialPackets::GetNewGoalQ(){
  return goalQ_M;
}

float * SerialPackets::GetNewGoalQdot(){
  return goalQdot_M;
}

float * SerialPackets::GetNewGoalTorque(){
  return goalTorque_M;
}

void SerialPackets::NewGoalsPulled(){
  newGoal_M = false;
  return;
}

/* ------------------------------------------------------------------------------------------------------/
/ Serial Packet Writer ----------------------------------------------------------------------------------/
/-------------------------------------------------------------------------------------------------------*/
void SerialPackets::WritePackets(unsigned long &totalTime, RobotControl &Robot, unsigned long &loopTime) {
  // TX = [Header, ElapsedTime, Q1, Q2, Q4, Qdot1, Qdot2, Qdot4, T1, T2, T4, _, LoopTime, CheckSum]
  byte dataPacket[_TX_PKT_LEN] = {0};
  uint16_t packetSum    = 0;
  int16_t byteLen       = 4;

  // Header Bytes 
  for (int16_t i = 0; i < 4; i++) {
    dataPacket[i] = _TXHEADER[i];
  }

  // Elapsed Time 
  dataPacket[4] = DXL_LOBYTE(DXL_LOWORD(totalTime));
  dataPacket[5] = DXL_HIBYTE(DXL_LOWORD(totalTime));
  dataPacket[6] = DXL_LOBYTE(DXL_HIWORD(totalTime));
  dataPacket[7] = DXL_HIBYTE(DXL_HIWORD(totalTime));

  // Motor's PresQ, PresQdot, and Torques
  byte *PresQ_bytes = floatArrayToBytes(Robot.GetPresQ());
  for (int16_t i = _Q_SLOT; i < _QDOT_SLOT; i++){
    dataPacket[i] = PresQ_bytes[i - _Q_SLOT];
  }
  byte *PresQDot_bytes = floatArrayToBytes(Robot.GetPresQDot());
  for (int16_t i = _QDOT_SLOT; i < _TORQUE_SLOT; i++){
    dataPacket[i] = PresQDot_bytes[i - _QDOT_SLOT];
  }
  // byte *PresQDot_bytes = floatArrayToBytes(Robot.GetTorques());
  // for (int16_t i = _TORQUE_SLOT; i < _NEW_SLOT1; i++){
  //   dataPacket[i] = PresQDot_bytes[i - _TORQUE_SLOT];
  // }

  // looptime
  dataPacket[_TX_PKT_LEN - 6] = DXL_LOBYTE(DXL_LOWORD(loopTime));
  dataPacket[_TX_PKT_LEN - 5] = DXL_HIBYTE(DXL_LOWORD(loopTime));
  dataPacket[_TX_PKT_LEN - 4] = DXL_LOBYTE(DXL_HIWORD(loopTime));
  dataPacket[_TX_PKT_LEN - 3] = DXL_HIBYTE(DXL_HIWORD(loopTime));

  // check Sum
  for (int16_t i = 0; i < _TX_PKT_LEN - 2; i++) {
    packetSum += dataPacket[i];
  }
  dataPacket[_TX_PKT_LEN - 2] = floor(packetSum / 256);
  dataPacket[_TX_PKT_LEN - 1] = floor(packetSum % 256);

  // write data packet
  if (Serial){
    Serial.write(dataPacket,_TX_PKT_LEN);
  } else {
    SerialPort_M->write(dataPacket,_TX_PKT_LEN);
  }
  
}

/* ------------------------------------------------------------------------------------------------------/
/ Serial Packet Reader ----------------------------------------------------------------------------------/
/-------------------------------------------------------------------------------------------------------*/
void SerialPackets::ReadPackets() {
  // RX = [Header, Mode, Q1, Q2, Q4, Qdot1, Qdot2, Qdot4, T1, T2, T4, _, CheckSum]
  byte dataPacket[_RX_PKT_LEN];
  byte tempHeader[4];
  int16_t SumCheck;
  int16_t CHECKSUM;
  
  unsigned long timeOUtTime = millis();
  while (SerialPort_M->available() < _RX_PKT_LEN) {
    if (millis() - timeOUtTime > 10){
      return;
    }
  }
  
  for (int16_t i = 0; i < _RX_PKT_LEN; i++) {
    dataPacket[i] = SerialPort_M->read();
  }
  
  CHECKSUM = bytesToCounts(dataPacket[_RX_PKT_LEN - 2], dataPacket[_RX_PKT_LEN - 1]);
  SumCheck = 0;
  for (int16_t i = 0; i < _RX_PKT_LEN - 2; i++) {
    SumCheck += dataPacket[i];
  }
  
  for (int16_t i = 0; i < 4; i++) {
    tempHeader[i] = dataPacket[i];
  }

  if (SumCheck != CHECKSUM) return;

  if (memcmp(_RXHEADER, tempHeader, sizeof(_RXHEADER)) != 0) return;
  
  newGoal_M = true;

  /* Drive Mode */
  mode_M = dataPacket[5];

  /* GoalQ slot = 8 */
  goalQ_M[0] = bytesToFloat(dataPacket[8], dataPacket[9], dataPacket[10], dataPacket[11]);
  goalQ_M[1] = bytesToFloat(dataPacket[12], dataPacket[13], dataPacket[14], dataPacket[15]);
  goalQ_M[2] = bytesToFloat(dataPacket[16], dataPacket[17], dataPacket[18], dataPacket[19]);

  /* GoalQdot slot = 20 */
  goalQdot_M[0] = bytesToFloat(dataPacket[20], dataPacket[21], dataPacket[22], dataPacket[23]);
  goalQdot_M[1] = bytesToFloat(dataPacket[24], dataPacket[25], dataPacket[26], dataPacket[27]);
  goalQdot_M[2] = bytesToFloat(dataPacket[28], dataPacket[29], dataPacket[30], dataPacket[31]);

  /* GoalTorque slot = 32 */
  goalTorque_M[0] = bytesToFloat(dataPacket[32], dataPacket[33], dataPacket[34], dataPacket[35]);
  goalTorque_M[1] = bytesToFloat(dataPacket[36], dataPacket[37], dataPacket[38], dataPacket[39]);
  goalTorque_M[2] = bytesToFloat(dataPacket[40], dataPacket[41], dataPacket[42], dataPacket[43]);
}
