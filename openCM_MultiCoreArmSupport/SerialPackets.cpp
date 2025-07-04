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
      Serial.println("a");
      if (c2cPort_M->read() == 0x01){
        Serial.println("b");
        c2cPort_M->write(0x02);
        digitalWrite(OCM::COMM_LED_PIN, HIGH);
        testingMode_M = false;
        while (c2cPort_M->available() > 0){
          c2cPort_M->read();
        }
        break;
      }
    }
    delay(1000);
//    Serial.println(i);
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

bool SerialPackets::NewGoalAvailable(){
  return newGoal_M;
}

float * SerialPackets::GetNewXYZGoal(){
  return goalXYZ_M;
}

float * SerialPackets::GetNewXYZdotGoal(){
  return goalXYZdot_M;
}

float * SerialPackets::GetNewCurrentGoal(){
  return goalCurrent_M;
}

void SerialPackets::NewGoalApplied(){
  newGoal_M = false;
  return;
}

bool SerialPackets::TorqueChanged(){
  return torqueChange_M;
}

uint8_t SerialPackets::ChangeModeTo(){
  return torqueMode_M;
}

void SerialPackets::TorqueChangeApplied(){
  torqueChange_M = false;
  return;
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
                     presQ1(rad):    8,  9, 10, 11,...
                     presQ2(rad):   12, 13, 14, 15,...
                     presQ4(rad):   16, 17, 18, 19,...
              presQ1dot(rad/sec):   20, 21, 22, 23,...
              presQ2dot(rad/sec):   24, 25, 26, 27,...
              presQ3dot(rad/sec):   28, 29, 30, 31,...
                        presX(m):   32, 33, 34, 35,...
                        presY(m):   36, 37, 38, 39,...
                        presZ(m):   40, 41, 42, 43,...
                   presXdot(m/s):   44, 45, 46, 47,...
                   presYdot(m/s):   48, 49, 50, 51,...
                   presZdot(m/s):   52, 53, 54, 55,...
                     goalQ1(rad):   56, 57, 58, 59,...
                     goalQ2(rad):   60, 61, 62, 63,...
                     goalQ4(rad):   64, 65, 66, 67,...
              goalQ1dot(rad/sec):   68, 69, 70, 71,...
              goalQ2dot(rad/sec):   72, 73, 74, 75,...
              goalQ4dot(rad/sec):   76, 77, 78, 79,...
                     presQ1(cts):   80, 81, 82, 83,...
                     presQ2(cts):   84, 85, 86, 87,...
                     presQ4(cts):   88, 89, 90, 91,...
                  presQ1dot(cts):   92, 93, 94, 95,...
                  presQ2dot(cts):   96, 97, 98, 99,...
                  presQ3dot(cts):  100,101,102,103,...
                     goalQ1(cts):  104,105,106,107,...
                     goalQ2(cts):  108,109,110,111,...
                     goalQ4(cts):  112,113,114,115,...
                  goalQ1dot(cts):  116,117,118,119,...
                  goalQ2dot(cts):  120,121,122,123,...
                  goalQ4dot(cts):  124,125,126,127,...
                                :  128-142,...
                currentDriveMode:  143,...
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

  // Motor Preset Q (COUNTS)
  packetBuffer = int32ArrayToBytes(Robot.GetPresQCts());
  for (int16_t i = _TX_presQcts_SLOT; i < _TX_presQDOTcts_SLOT; i++){
    dataPacket[i] = packetBuffer[i - _TX_presQcts_SLOT];
  }
  // Motor PresQdot (COUNTS)
  packetBuffer = int32ArrayToBytes(Robot.GetPresQDotCts());
  for (int16_t i = _TX_presQDOTcts_SLOT; i < _TX_goalQcts_SLOT; i++){
    dataPacket[i] = packetBuffer[i - _TX_presQDOTcts_SLOT];
  }
  // Motor GoalQ (COUNTS)
  packetBuffer = int32ArrayToBytes(Robot.GetGoalQCts());
  for (int16_t i = _TX_goalQcts_SLOT; i < _TX_goalQDOTcts_SLOT; i++){
    dataPacket[i] = packetBuffer[i - _TX_goalQcts_SLOT];
  }
  // Motor GoalQdot (COUNTS)
  packetBuffer = int32ArrayToBytes(Robot.GetGoalQDotCts());
  for (int16_t i = _TX_goalQDOTcts_SLOT; i < _TX_presCURRENT_SLOT; i++){
    dataPacket[i] = packetBuffer[i - _TX_goalQDOTcts_SLOT];
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
  c2cPort_M->write(dataPacket,_TX_PKT_LEN); 
  for (int i = 0; i < _TX_PKT_LEN; i++){
    Serial.print(dataPacket[i]); Serial.print(" ");
  }
  Serial.println("");
}

/* ------------------------------------------------------------------------------------------------------/
/ Serial Packet Reader ----------------------------------------------------------------------------------/
/-------------------------------------------------------------------------------------------------------*/
void SerialPackets::ReadPackets() {
  /* RX Packet Structure:     Header: [ 0, 1, 2, 3,...
                                   _:   4,...
    Packet Type          Packet Type:   5,...
    --------------                 _:   6,...
    No.| T | G | R              Mode:   7,...
    --------------             goalX:   8, 9,10,11,...
    0 || 0 | 0 | 0             goalY:  12,13,14,15,...
    1 || 1 | 0 | 0             goalZ:  16,17,18,19,...
    2 || 0 | 1 | 0          goalXdot:  20,21,22,23,...
    3 || 1 | 1 | 0          goalYdot:  24,25,26,27,...
    4 || 0 | 0 | 1          goalZdot:  28,29,30,31,...
    5 || 1 | 0 | 1      goalCurrent1:  32,33,34,35,...
    6 || 0 | 1 | 1      goalCurrent2:  36,37,38,39,...
    7 || 1 | 1 | 1      goalCurrent3:  40,41,42,43,...
                                   _:  44,45,46,47,48,49,50,51,52,53,54,55,56,57,...
                            CheckSum:  58,59]
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
  uint8_t packetType = dataPacket[5];
  if (packetType > 7) return;

  /* Find Packet Type */
  uint8_t mask = 1;
  uint8_t bitArray[3];
  for (int16_t i = 0; i < 3; i++) {
      bitArray[i] = (packetType & (mask << i)) != 0;
  }

  /* Torque Change Packet (1,3,5,7) *****/
  if (bitArray[0] == 1){
    torqueChange_M = true;
    torqueMode_M = dataPacket[7];
  }
  /* New XYZ Goal Packet (2,3,6,7) *****/
  if (bitArray[1] == 1){
    newGoal_M = true;
    /* XYZ Goal slot = 8 */
    goalXYZ_M[0] = bytesToFloat(dataPacket[8], dataPacket[9], dataPacket[10], dataPacket[11]);
    goalXYZ_M[1] = bytesToFloat(dataPacket[12], dataPacket[13], dataPacket[14], dataPacket[15]);
    goalXYZ_M[2] = bytesToFloat(dataPacket[16], dataPacket[17], dataPacket[18], dataPacket[19]);

    /* XYZdot Goal slot = 20 */
    goalXYZdot_M[0] = bytesToFloat(dataPacket[20], dataPacket[21], dataPacket[22], dataPacket[23]);
    goalXYZdot_M[1] = bytesToFloat(dataPacket[24], dataPacket[25], dataPacket[26], dataPacket[27]);
    goalXYZdot_M[2] = bytesToFloat(dataPacket[28], dataPacket[29], dataPacket[30], dataPacket[31]);

    /* Current Goal slot = 32 */
    goalCurrent_M[0] = bytesToFloat(dataPacket[32], dataPacket[33], dataPacket[34], dataPacket[35]);
    goalCurrent_M[1] = bytesToFloat(dataPacket[36], dataPacket[37], dataPacket[38], dataPacket[39]);
    goalCurrent_M[2] = bytesToFloat(dataPacket[40], dataPacket[41], dataPacket[42], dataPacket[43]);
  }
  /* Data Requested Packet (4,5,6,7) *****/
  if (bitArray[2] == 1){
    dataRequested_M = true;
  }
}
  
