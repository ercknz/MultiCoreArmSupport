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
RobotComm::RobotComm(HardwareSerial *ptrSer, const int baudrate)
  : _BAUDRATE{baudrate}
{
  robotPort_M = ptrSer;
  scalingFactor_M = 1.0f;
  springF_M = 0.0f;
  torqueState_M = 0;
  
  // Initialize arrays to zero
  for(int i = 0; i < 3; i++) {
    qPres_M[i] = 0.0f;
    qDotPres_M[i] = 0.0f;
    qPresCts_M[i] = 0;
    qDotPresCts_M[i] = 0;
    xyzPres_M[i] = 0.0f;
    xyzDotPres_M[i] = 0.0f;
    qGoal_M[i] = 0.0f;
    qDotGoal_M[i] = 0.0f;
    qGoalCts_M[i] = 0;
    qDotGoalCts_M[i] = 0;
    xyzGoal_M[i] = 0.0f;
    xyzDotGoal_M[i] = 0.0f;
  }

}

int RobotComm::Connect2Robot(uint8_t LEDpin){
  pinMode(LEDpin, OUTPUT);
  robotPort_M->begin(_BAUDRATE);
  robotPort_M->addMemoryForRead(_serialRXbuffer_M, _RX_BUFFER_SIZE);
  for (int i = 0; i < 30; i++)
  {
    robotPort_M->write(0x01);
    if (robotPort_M->available() > 0)
    {
      if (robotPort_M->read() == 0x02)
      {
        // CONNECTION SUCCESSFUL
        while (robotPort_M->available() > 0)
        {
          // Clear buffer
          robotPort_M->read();
        }
        // Blink LED to signal connection
        for (int j = 0; j < 2; j++)
        {
          digitalWrite(LEDpin, HIGH);
        }
        connected2Robot_M = true;
        return 1;
      }
    }
    digitalWrite(LEDpin, HIGH);
    delay(500);
    digitalWrite(LEDpin, LOW);
    delay(500);
  }
  return 0;
}

/* ---------------------------------------------------------------------------------------/
/ Arm Support Get Member functions -------------------------------------------------------/
/----------------------------------------------------------------------------------------*/

int RobotComm::BytesAvailable(){
  return robotPort_M->available();
}

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

int32_t* RobotComm::GetPresQCts(){
  return qPresCts_M;
}

int32_t* RobotComm::GetPresQDotCts(){
  return qDotPresCts_M;
}

int32_t* RobotComm::GetGoalQCts(){
  return qGoalCts_M;
}

int32_t* RobotComm::GetGoalQDotCts(){
  return qDotGoalCts_M;
}

float*   RobotComm::GetGoalQ(){
  return qGoal_M;
}

float*   RobotComm::GetGoalQDot(){
  return qDotGoal_M;
}

uint8_t RobotComm::GetTorqueState(){
  return torqueState_M;
}

bool RobotComm::IsConnected(){
  return connected2Robot_M;
}

/* ---------------------------------------------------------------------------------------/
/ Arm Support Robot Reading --------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
void RobotComm::ReadRobot(){
    /* Incoming Packet Structure: Header: [  0,  1,  2,  3,...
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

  /* presQ(rad)         slot = 8   */
  qPres_M[0] = bytesToFloat(dataPacket[8], dataPacket[9], dataPacket[10], dataPacket[11]);
  qPres_M[1] = bytesToFloat(dataPacket[12], dataPacket[13], dataPacket[14], dataPacket[15]);
  qPres_M[2] = bytesToFloat(dataPacket[16], dataPacket[17], dataPacket[18], dataPacket[19]);

  /* presQdot(rad/sec)  slot = 20  */
  qDotPres_M[0] = bytesToFloat(dataPacket[20], dataPacket[21], dataPacket[22], dataPacket[23]);
  qDotPres_M[1] = bytesToFloat(dataPacket[24], dataPacket[25], dataPacket[26], dataPacket[27]);
  qDotPres_M[2] = bytesToFloat(dataPacket[28], dataPacket[29], dataPacket[30], dataPacket[31]);

  /* presXYZ(m)         slot = 32  */
  xyzPres_M[0] = bytesToFloat(dataPacket[32], dataPacket[33], dataPacket[34], dataPacket[35]);
  xyzPres_M[1] = bytesToFloat(dataPacket[36], dataPacket[37], dataPacket[38], dataPacket[39]);
  xyzPres_M[2] = bytesToFloat(dataPacket[40], dataPacket[41], dataPacket[42], dataPacket[43]);

  /* presXYZdot(m/s)    slot = 44  */
  xyzDotPres_M[0] = bytesToFloat(dataPacket[44], dataPacket[45], dataPacket[46], dataPacket[47]);
  xyzDotPres_M[1] = bytesToFloat(dataPacket[48], dataPacket[49], dataPacket[50], dataPacket[51]);
  xyzDotPres_M[2] = bytesToFloat(dataPacket[52], dataPacket[53], dataPacket[54], dataPacket[55]);
  
  /* goalQ(rad)         slot = 56  */
  qGoal_M[0] = bytesToFloat(dataPacket[56], dataPacket[57], dataPacket[58], dataPacket[59]);
  qGoal_M[1] = bytesToFloat(dataPacket[60], dataPacket[61], dataPacket[62], dataPacket[63]);
  qGoal_M[2] = bytesToFloat(dataPacket[64], dataPacket[65], dataPacket[66], dataPacket[67]);

  /* goalQdot(rad/sec)  slot = 68  */
  qDotGoal_M[0] = bytesToFloat(dataPacket[68], dataPacket[69], dataPacket[70], dataPacket[71]);
  qDotGoal_M[1] = bytesToFloat(dataPacket[72], dataPacket[73], dataPacket[74], dataPacket[75]);
  qDotGoal_M[2] = bytesToFloat(dataPacket[76], dataPacket[77], dataPacket[78], dataPacket[79]);

  /* presQ(cts)         slot = 80  */
  qPresCts_M[0] = bytesToInt32(dataPacket[80], dataPacket[81], dataPacket[82], dataPacket[83]);
  qPresCts_M[1] = bytesToInt32(dataPacket[84], dataPacket[85], dataPacket[86], dataPacket[87]);
  qPresCts_M[2] = bytesToInt32(dataPacket[88], dataPacket[89], dataPacket[90], dataPacket[91]);

  /* presQdot(cts)      slot = 92  */
  qDotPresCts_M[0] = bytesToInt32(dataPacket[92], dataPacket[93], dataPacket[94], dataPacket[95]);
  qDotPresCts_M[1] = bytesToInt32(dataPacket[96], dataPacket[97], dataPacket[98], dataPacket[99]);
  qDotPresCts_M[2] = bytesToInt32(dataPacket[100], dataPacket[101], dataPacket[102], dataPacket[103]);

  /* goalQ(cts)         slot = 104 */
  qGoalCts_M[0] = bytesToInt32(dataPacket[104], dataPacket[105], dataPacket[106], dataPacket[107]);
  qGoalCts_M[1] = bytesToInt32(dataPacket[108], dataPacket[109], dataPacket[110], dataPacket[111]);
  qGoalCts_M[2] = bytesToInt32(dataPacket[112], dataPacket[113], dataPacket[114], dataPacket[115]);

  /* goalQdot(cts)      slot = 116 */
  qDotGoalCts_M[0] = bytesToInt32(dataPacket[116], dataPacket[117], dataPacket[118], dataPacket[119]);
  qDotGoalCts_M[1] = bytesToInt32(dataPacket[120], dataPacket[121], dataPacket[122], dataPacket[123]);
  qDotGoalCts_M[2] = bytesToInt32(dataPacket[124], dataPacket[125], dataPacket[126], dataPacket[127]);

  /* Torque State */
  torqueState_M = dataPacket[143];
}

/* ---------------------------------------------------------------------------------------/
/ Arm Support Robot Writing --------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
void RobotComm::RequestDataOnly(){
  //RobotComm::Send only Request for Data packet to robot.
  float blankData[3] = {0.0f, 0.0f, 0.0f};
  uint8_t blankTorque = 0;
  WriteToRobot(4, blankData, blankData, blankTorque);
}

void RobotComm::ChangeTorqueOnly(uint8_t newTorqueValue){
  //RobotComm::Only sends a torque mode change to robot. 
  float blankData[3] = {0.0f, 0.0f, 0.0f};
  WriteToRobot(1, blankData, blankData, newTorqueValue);
}

void RobotComm::SendNewGoalOnly(float *newXYZGoal, float * newXYZdotGoal){
  //RobotComm::Only sends new goal position and velocity to robot.
  uint8_t blankTorque = 0;
  WriteToRobot(2, newXYZGoal, newXYZdotGoal, blankTorque);
}

void RobotComm::SendZeroes(){
  byte dataPacket[_TX_PKT_LEN] = {0};
  // write data packet
  robotPort_M->write(dataPacket,_TX_PKT_LEN); 
}

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
  robotPort_M->write(dataPacket,_TX_PKT_LEN); 
}

/* ---------------------------------------------------------------------------------------/
/ Arm Support Spring Force Member Functions ----------------------------------------------/
/----------------------------------------------------------------------------------------*/
void RobotComm::SetScalingFactor(float newScalingFactor){
  scalingFactor_M = newScalingFactor;
}