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
SerialPackets::SerialPackets(HardwareSerial  *ptrSer, const int baudrate)
  : _BAUDRATE{baudrate}
{
  c2cPort_M = ptrSer;
  c2cPort_M->begin(_BAUDRATE);
}

/* ------------------------------------------------------------------------------------------------------/
/ Serial Packets Setters and  Getters -------------------------------------------------------------------/
/-------------------------------------------------------------------------------------------------------*/
bool SerialPackets::DataAvailable() {
  if (Serial){
    return Serial.available();
  } 
  return c2cPort_M->available();
}

bool SerialPackets::NewGoalAvailable(){
  return newGoal_M;
}

uint8_t SerialPackets::GetNewMode(){
  return torqueMode_M;
}

float * SerialPackets::GetNewGoalQ(){
  return goalXYZ_M;
}

float * SerialPackets::GetNewGoalQdot(){
  return goalXYZdot_M;
}

float * SerialPackets::GetNewGoalCurrent(){
  return goalCurrent_M;
}

void SerialPackets::NewGoalsPulled(){
  newGoal_M = false;
  return;
}

/* ------------------------------------------------------------------------------------------------------/
/ Serial Packet Writer ----------------------------------------------------------------------------------/
/-------------------------------------------------------------------------------------------------------*/
void SerialPackets::WritePackets(unsigned long &totalTime, RobotControl &Robot, unsigned long &loopTime) {
  /* TXpacket = Header: [ 0, 1, 2, 3,...
           elapsedTime:   4, 5, 6, 7,...
                presQ1:   8, 9,10,11,...
                presQ2:  12,13,14,15,...
                presQ4:  16,17,18,19,...
             presQ1dot:  20,21,22,23,...
             presQ2dot:  24,25,26,27,...
             presQ3dot:  28,29,30,31,...
          presCurrent1:  32,33,34,35,...
          presCurrent2:  36,37,38,39,...
          presCurrent3:  40,41,42,43,...
                 presX:  44,45,46,47,...
                 presY:  48,49,50,51,...
                 presZ:  52,53,54,55,...
              presXdot:  56,57,58,59,...
              presYdot:  60,61,62,63,...
              presZdot:  64,65,66,67,...
                     _:  68,69,70,71,72,73,...
              loopTime:  74,75,76,77,...
              CheckSum:  78,79]
  */ 
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
  for (int16_t i = _presQ_SLOT; i < _presQDOT_SLOT; i++){
    dataPacket[i] = PresQ_bytes[i - _presQ_SLOT];
  }
  byte *PresQDot_bytes = floatArrayToBytes(Robot.GetPresQDot());
  for (int16_t i = _presQDOT_SLOT; i < _presCURRENT_SLOT; i++){
    dataPacket[i] = PresQDot_bytes[i - _presQDOT_SLOT];
  }
  byte *PresI_bytes = floatArrayToBytes(Robot.GetPresCurrent());
  for (int16_t i = _presCURRENT_SLOT; i < _presXYZ_SLOT; i++){
    dataPacket[i] = PresI_bytes[i - _presCURRENT_SLOT];
  }
  byte *PresXYZ_bytes = floatArrayToBytes();
  for (int16_t i = _presXYZ_SLOT; i < _presXYZdot_SLOT; i++){
    dataPacket[i] = PresXYZ_bytes[i - _presXYZ_SLOT];
  }
  byte *PresXYZdot_bytes = floatArrayToBytes();
  for (int16_t i = _presXYZdot_SLOT; i < _BLANK_SLOT; i++){
    dataPacket[i] = PresXYZdot_bytes[i - _presXYZdot_SLOT];
  }

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
    return;
  }
  c2cPort_M->write(dataPacket,_TX_PKT_LEN); 
}

/* ------------------------------------------------------------------------------------------------------/
/ Serial Packet Reader ----------------------------------------------------------------------------------/
/-------------------------------------------------------------------------------------------------------*/
void SerialPackets::ReadPackets() {
  /* RXpacket = Header: [ 0, 1, 2, 3,...
                     _:   4,...
                  Mode:   5,...
                     _:   6, 7,...
                 goalX:   8, 9,10,11,...
                 goalY:  12,13,14,15,...
                 goalZ:  16,17,18,19,...
              goalXdot:  20,21,22,23,...
              goalYdot:  24,25,26,27,...
              goalZdot:  28,29,30,31,...
          goalCurrent1:  32,33,34,35,...
          goalCurrent2:  36,37,38,39,...
          goalCurrent3:  40,41,42,43,...
                     _:  44,45,46,47,48,49,50,51,52,53,54,55,56,57,...
              CheckSum:  58,59]
  */ 
  byte dataPacket[_RX_PKT_LEN];
  byte tempHeader[4];
  int16_t sumCheck;
  int16_t CHECKSUM;
  
  /* Check for instructions */
  unsigned long timeOUtTime = millis();
  while (c2cPort_M->available() < _RX_PKT_LEN) {
    if (millis() - timeOUtTime > 10){
      return;
    }
  }
  
  /* Read Instructions */
  for (int16_t i = 0; i < _RX_PKT_LEN; i++) {
    dataPacket[i] = c2cPort_M->read();
  }

  /* verify packet */
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
  if (memcmp(_RXHEADER, tempHeader, sizeof(_RXHEADER)) != 0) return;
  
  newGoal_M = true;

  /* Drive Mode */
  torqueMode_M = dataPacket[5];

  /* GoalQ slot = 8 */
  goalXYZ_M[0] = bytesToFloat(dataPacket[8], dataPacket[9], dataPacket[10], dataPacket[11]);
  goalXYZ_M[1] = bytesToFloat(dataPacket[12], dataPacket[13], dataPacket[14], dataPacket[15]);
  goalXYZ_M[2] = bytesToFloat(dataPacket[16], dataPacket[17], dataPacket[18], dataPacket[19]);

  /* GoalQdot slot = 20 */
  goalXYZdot_M[0] = bytesToFloat(dataPacket[20], dataPacket[21], dataPacket[22], dataPacket[23]);
  goalXYZdot_M[1] = bytesToFloat(dataPacket[24], dataPacket[25], dataPacket[26], dataPacket[27]);
  goalXYZdot_M[2] = bytesToFloat(dataPacket[28], dataPacket[29], dataPacket[30], dataPacket[31]);

  /* GoalCurrent slot = 32 */
  goalCurrent_M[0] = bytesToFloat(dataPacket[32], dataPacket[33], dataPacket[34], dataPacket[35]);
  goalCurrent_M[1] = bytesToFloat(dataPacket[36], dataPacket[37], dataPacket[38], dataPacket[39]);
  goalCurrent_M[2] = bytesToFloat(dataPacket[40], dataPacket[41], dataPacket[42], dataPacket[43]);
}
