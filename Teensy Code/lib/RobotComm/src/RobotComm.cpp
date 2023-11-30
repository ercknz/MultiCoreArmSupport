/* Class controls the arm support robot
   It includes the fKine and iKine as well as the
   functions needed to write and read to the motors
   Refer to github-Dynamixel for more information on dynamixel library.

   Class arrays are used as follows:
   xyz[3]     = {x, y, x}; 
   xyzDot[3]  = {xDot, yDot, zDot};
   q[3]       = {q1(shoulder), q2(elevation), q4(elbow)};
   qDot[3]    = {q1Dot(shoulder), q2Dot(elevation), q4Dot(elbow)}

   Created 10/28/2020
   Script by Erick Nunez
*/

#include <Arduino.h>
#include "RobotComm.h"
#include "UtilityFunctions.h"

/* ---------------------------------------------------------------------------------------/
/ Arm Support Constructor ----------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
RobotComm::RobotComm(){
  //scalingFactor_M = SPRING_FORCE_SCALING_FACTOR;
}

/* ---------------------------------------------------------------------------------------/
/ Arm Support Get Member functions -------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
float* RobotComm::GetPresQ(){
  return qPres_M;
}

float* RobotComm::GetPresQDot(){
  return qDotPres_M;
}

float* RobotComm::GetPresPos(){ 
  return xyzPres_M;
}

float* RobotComm::GetPresVel(){
  return xyzDotPres_M;
}

float* RobotComm::GetGoalPos(){ 
  return xyzGoal_M;
}

float* RobotComm::GetGoalVel(){
  return xyzDotGoal_M;
}

float RobotComm::GetSpringForce(){
  return springF_M;
}

/* ---------------------------------------------------------------------------------------/
/ Arm Support Robot Reading --------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
void RobotComm::ReadRobot(){
    /* Incoming Packet Structure: Header: [ 0, 1, 2, 3,...
                             elapsedTime:   4, 5, 6, 7,...
                                  presQ1:   8, 9,10,11,...
                                  presQ2:  12,13,14,15,...
                                  presQ4:  16,17,18,19,...
                               presQ1dot:  20,21,22,23,...
                               presQ2dot:  24,25,26,27,...
                               presQ3dot:  28,29,30,31,...
                               presQ1cts:  32,33,34,35,...
                               presQ2cts:  36,37,38,39,...
                               presQ4cts:  40,41,42,43,...
                                   presX:  44,45,46,47,...
                                   presY:  48,49,50,51,...
                                   presZ:  52,53,54,55,...
                                presXdot:  56,57,58,59,...
                                presYdot:  60,61,62,63,...
                                presZdot:  64,65,66,67,...
                                  goalQ1:  68,69,70,71,...
                                  goalQ2:  72,73,74,75,...
                                  goalQ4:  76,77,78,79,...
                               goalQ1cts:  80,81,82,83,...
                               goalQ2cts:  84,85,86,87,...
                               goalQ4cts:  88,89,90,91,...
                                        :  92,...
                        currentDriveMode:  93,...
                                loopTime:  94,95,96,97,...
                                CheckSum:  98,99]
  */
  byte dataPacket[_RX_PKT_LEN];
  byte tempHeader[4];
  int16_t sumCheck;
  int16_t CHECKSUM;
  
  /* Check for instructions */
  unsigned long timeOUtTime = millis();
  while (robotPort_M->available() < _RX_PKT_LEN) {
    if (millis() - timeOUtTime > 10){
      return;
    }
  }
  
  /* Read Instructions */
  for (int16_t i = 0; i < _RX_PKT_LEN; i++) {
    dataPacket[i] = robotPort_M->read();
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
  if (memcmp(_READ_HEADER, tempHeader, sizeof(_READ_HEADER)) != 0) return;
  
  /* Pres Q slot = 8 */
  qPres_M[0] = bytesToFloat(dataPacket[8], dataPacket[9], dataPacket[10], dataPacket[11]);
  qPres_M[1] = bytesToFloat(dataPacket[12], dataPacket[13], dataPacket[14], dataPacket[15]);
  qPres_M[2] = bytesToFloat(dataPacket[16], dataPacket[17], dataPacket[18], dataPacket[19]);

  /* Pres Qdot slot = 20 */
  qDotPres_M[0] = bytesToFloat(dataPacket[20], dataPacket[21], dataPacket[22], dataPacket[23]);
  qDotPres_M[1] = bytesToFloat(dataPacket[24], dataPacket[25], dataPacket[26], dataPacket[27]);
  qDotPres_M[2] = bytesToFloat(dataPacket[28], dataPacket[29], dataPacket[30], dataPacket[31]);

  /* Pres Current slot = 32 */
  presCurrent_M[0] = bytesToFloat(dataPacket[32], dataPacket[33], dataPacket[34], dataPacket[35]);
  presCurrent_M[1] = bytesToFloat(dataPacket[36], dataPacket[37], dataPacket[38], dataPacket[39]);
  presCurrent_M[2] = bytesToFloat(dataPacket[40], dataPacket[41], dataPacket[42], dataPacket[43]);

  /* Qdot Goal slot = 20 */
  xyzPres_M[0] = bytesToFloat(dataPacket[44], dataPacket[45], dataPacket[46], dataPacket[47]);
  xyzPres_M[1] = bytesToFloat(dataPacket[48], dataPacket[49], dataPacket[50], dataPacket[51]);
  xyzPres_M[2] = bytesToFloat(dataPacket[52], dataPacket[53], dataPacket[54], dataPacket[55]);

  /* Current Goal slot = 32 */
  xyzDotPres_M[0] = bytesToFloat(dataPacket[56], dataPacket[57], dataPacket[58], dataPacket[59]);
  xyzDotPres_M[1] = bytesToFloat(dataPacket[60], dataPacket[61], dataPacket[62], dataPacket[63]);
  xyzDotPres_M[2] = bytesToFloat(dataPacket[64], dataPacket[65], dataPacket[66], dataPacket[67]);

  /* Torque State */
  torqueState_M = dataPacket[73];
}

/* ---------------------------------------------------------------------------------------/
/ Arm Support Robot Writing --------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
void RobotComm::WriteToRobot(uint8_t packetType, float *goalXYZ, float * goalXYZdot, uint8_t torqueMode){
  /* Outgoing Packet Structure:       Header: [ 0, 1, 2, 3,...
                                           _:   4,...
    Packet Type                  Packet Type:   5,...
    --------------                         _:   6,...
    No.| T | G | R                      Mode:   7,...
    --------------                     goalX:   8, 9,10,11,...
    0 || 0 | 0 | 0                     goalY:  12,13,14,15,...
    1 || 1 | 0 | 0                     goalZ:  16,17,18,19,...
    2 || 0 | 1 | 0                  goalXdot:  20,21,22,23,...
    3 || 1 | 1 | 0                  goalYdot:  24,25,26,27,...
    4 || 0 | 0 | 1                  goalZdot:  28,29,30,31,...
    5 || 1 | 0 | 1              goalCurrent1:  32,33,34,35,...
    6 || 0 | 1 | 1              goalCurrent2:  36,37,38,39,...
    7 || 1 | 1 | 1              goalCurrent3:  40,41,42,43,...
                                           _:  44,45,46,47,48,49,50,51,52,53,54,55,56,57,...
                                    CheckSum:  58,59]
  */ 
  for (int16_t i = 0; i < 3; i++){
    xyzGoal_M[i] = goalXYZ[i];
    xyzDotGoal_M[i] = goalXYZdot[i];
  }

  byte dataPacket[_TX_PKT_LEN] = {0};
  uint16_t packetSum    = 0;
  int16_t byteLen       = 4;

  // Header Bytes 
  for (int16_t i = 0; i < 4; i++) {
    dataPacket[i] = _WRITE_HEADER[i];
  }

  /* Filling in Parameters*/
  dataPacket[_TX_PKT_TYPE_SLOT] = packetType;
  dataPacket[_TX_TORQUE_CHANGE_SLOT] = torqueMode;

  // Motor's PresQ, PresQdot, and Torques
  byte *goalXYZ_bytes = floatArrayToBytes(xyzGoal_M);
  for (int16_t i = _TX_GOAL_XYZ_SLOT; i < _TX_GOAL_XYZDOT_SLOT; i++){
    dataPacket[i] = goalXYZ_bytes[i - _TX_GOAL_XYZ_SLOT];
  }
  byte *goalXYZdot_bytes = floatArrayToBytes(xyzDotGoal_M);
  for (int16_t i = _TX_GOAL_XYZDOT_SLOT; i < _TX_GOAL_CURRENT_SLOT; i++){
    dataPacket[i] = goalXYZdot_bytes[i - _TX_GOAL_XYZDOT_SLOT];
  }

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
  robotPort_M->write(dataPacket,_TX_PKT_LEN); 
}

/* ---------------------------------------------------------------------------------------/
/ Arm Support Spring Force Member Functions ----------------------------------------------/
/----------------------------------------------------------------------------------------*/
void RobotComm::SetScalingFactor(float newScalingFactor){
  scalingFactor_M = newScalingFactor;
}