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

#include <DynamixelSDK.h>
#include <Arduino.h>
#include "RobotControl.h"
#include "OpenCMNamespace.h"

/* ---------------------------------------------------------------------------------------/
/ Arm Support Constructor ----------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
RobotControl::RobotControl(){
  // Initalize RobotControl Class
}

/* ---------------------------------------------------------------------------------------/
/ Arm Support Get Member functions -------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
int32_t* RobotControl::GetPresQCts(){ 
  return qPresCts_M;
} 

int32_t* RobotControl::GetPresQDotCts(){
  return qDotPresCts_M;
}

float* RobotControl::GetPresQ(){
  return qPres_M;
}

float* RobotControl::GetPresQDot(){
  return qDotPres_M;
}

int32_t* RobotControl::GetGoalQCts(){ 
  return qCts_M;
}

int32_t* RobotControl::GetGoalQDotCts(){
  return qDotCts_M;
}

float* RobotControl::GetGoalQ(){
  return q_M;
}

float* RobotControl::GetGoalQDot(){
  return qDot_M;
}

/* ---------------------------------------------------------------------------------------/
/ Arm Support DXL Torque Enabling Member Function ----------------------------------------/
/----------------------------------------------------------------------------------------*/
void  RobotControl::EnableTorque(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler  *packetHandler, uint8_t state) {
  /* Modes to Select from:
   *  [5]:  Full Admittance Control (Shoulder, Elbow, and Elevation ENABLED)
   *  [10]: Planar Admittance Control ONLY (Shoulder and  Elbow ENABLED, Elevation DISABLED)
   *  [15]: Vertical Admittance Control ONLY (Elevation ENABLED, Shoulder and Elbow DISABLED)
   *  [20]: Fully Passive (Shoulder, Elbow, and Elevation DISABLED)
   */
  using namespace OCM;
  int dxlCommResult;
  if ((state == 5)||(state == 10)){
    dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_SHOULDER,     ADDRESS_TORQUE_ENABLE, ENABLE, &dxl_error);
    dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_ELBOW,        ADDRESS_TORQUE_ENABLE, ENABLE, &dxl_error);
  }
  if ((state == 5)||(state == 15)){
    dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_ELEVATION,    ADDRESS_TORQUE_ENABLE, ENABLE, &dxl_error);
  }
  if ((state == 20)||(state == 10)){
    dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_ELEVATION,    ADDRESS_TORQUE_ENABLE, DISABLE, &dxl_error);
  }
  if ((state == 20)||(state == 15)){
    dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_SHOULDER,     ADDRESS_TORQUE_ENABLE, DISABLE, &dxl_error);
    dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_ELBOW,        ADDRESS_TORQUE_ENABLE, DISABLE, &dxl_error);
  }
}

/* ---------------------------------------------------------------------------------------/
/ Arm Support DXL Configuration Member Function ------------------------------------------/
/----------------------------------------------------------------------------------------*/
void  RobotControl::MotorConfig(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler  *packetHandler) {
  using namespace OCM;
  int dxlCommResult;
  /* Enable LED for visual indication */
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_SHOULDER,     ADDRESS_LED, ENABLE, &dxl_error);
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_ELBOW,        ADDRESS_LED, ENABLE, &dxl_error);
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_ELEVATION,    ADDRESS_LED, ENABLE, &dxl_error);
  /* Sets control mode */
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_SHOULDER,     ADDRESS_OPERATING_MODE, POSITION_CONTROL, &dxl_error);
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_ELEVATION,    ADDRESS_OPERATING_MODE, POSITION_CONTROL, &dxl_error);
  dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_ELBOW,        ADDRESS_OPERATING_MODE, POSITION_CONTROL, &dxl_error);
  /* Set Velocity Limits */
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_SHOULDER,     ADDRESS_VELOCITY_LIMIT, VEL_MAX_LIMIT,   &dxl_error);
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_ELEVATION,    ADDRESS_VELOCITY_LIMIT, VEL_MAX_LIMIT,   &dxl_error);
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_ELBOW,        ADDRESS_VELOCITY_LIMIT, VEL_MAX_LIMIT,   &dxl_error);
  /* Sets Position Limits */
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_SHOULDER,     ADDRESS_MIN_POSITION,   SHOULDER_MIN_POS, &dxl_error);
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_SHOULDER,     ADDRESS_MAX_POSITION,   SHOULDER_MAX_POS, &dxl_error);
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_ELEVATION,    ADDRESS_MIN_POSITION,   ELEVATION_MIN_POS, &dxl_error);
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_ELEVATION,    ADDRESS_MAX_POSITION,   ELEVATION_MAX_POS, &dxl_error);
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_ELBOW,        ADDRESS_MIN_POSITION,   ELBOW_MIN_POS,    &dxl_error);
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_ELBOW,        ADDRESS_MAX_POSITION,   ELBOW_MAX_POS,    &dxl_error);
  /* Sets Velocity Profiles */
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_SHOULDER,     ADDRESS_PROFILE_VELOCITY, VEL_BASED_PROFILE, &dxl_error);
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_ELEVATION,    ADDRESS_PROFILE_VELOCITY, VEL_BASED_PROFILE, &dxl_error);
  dxlCommResult = packetHandler->write4ByteTxRx(portHandler, ID_ELBOW,        ADDRESS_PROFILE_VELOCITY, VEL_BASED_PROFILE, &dxl_error);
}

/* ---------------------------------------------------------------------------------------/
/ Arm Support DXL Read Member Function ---------------------------------------------------/
/----------------------------------------------------------------------------------------*/
void  RobotControl::ReadMotors(dynamixel::GroupSyncRead  &syncReadPacket) {
  /* Read Position and Velocity */
  int dxlCommResult = syncReadPacket.txRxPacket();
  qPresCts_M[0]    = syncReadPacket.getData(OCM::ID_SHOULDER,  OCM::ADDRESS_PRESENT_POSITION, OCM::LEN_PRESENT_POSITION);
  qPresCts_M[1]    = syncReadPacket.getData(OCM::ID_ELEVATION, OCM::ADDRESS_PRESENT_POSITION, OCM::LEN_PRESENT_POSITION);
  qPresCts_M[2]    = syncReadPacket.getData(OCM::ID_ELBOW,     OCM::ADDRESS_PRESENT_POSITION, OCM::LEN_PRESENT_POSITION);
  qDotPresCts_M[0] = syncReadPacket.getData(OCM::ID_SHOULDER,  OCM::ADDRESS_PRESENT_VELOCITY, OCM::LEN_PRESENT_VELOCITY);
  qDotPresCts_M[1] = syncReadPacket.getData(OCM::ID_ELEVATION, OCM::ADDRESS_PRESENT_VELOCITY, OCM::LEN_PRESENT_VELOCITY);
  qDotPresCts_M[2] = syncReadPacket.getData(OCM::ID_ELBOW,     OCM::ADDRESS_PRESENT_VELOCITY, OCM::LEN_PRESENT_VELOCITY);
}

/* ---------------------------------------------------------------------------------------/
/ Arm Support DXL Write Member Function --------------------------------------------------/
/----------------------------------------------------------------------------------------*/
int  RobotControl::WriteToMotors(bool &addParamResult, dynamixel::GroupSyncWrite &syncWritePacket) {
  int dxlCommResult;
  uint8_t elbowParam[4], shoulderParam[4], elevateParam[4];

  /* Shoulder Goal Position Packet */
  shoulderParam[0] = DXL_LOBYTE(DXL_LOWORD(qCts_M[0]));
  shoulderParam[1] = DXL_HIBYTE(DXL_LOWORD(qCts_M[0]));
  shoulderParam[2] = DXL_LOBYTE(DXL_HIWORD(qCts_M[0]));
  shoulderParam[3] = DXL_HIBYTE(DXL_HIWORD(qCts_M[0]));

  /* Elevation Goal Position Packet */
  elevateParam[0] = DXL_LOBYTE(DXL_LOWORD(qCts_M[1]));
  elevateParam[1] = DXL_HIBYTE(DXL_LOWORD(qCts_M[1]));
  elevateParam[2] = DXL_LOBYTE(DXL_HIWORD(qCts_M[1]));
  elevateParam[3] = DXL_HIBYTE(DXL_HIWORD(qCts_M[1]));

  /* Elbow Goal Position Packet */
  elbowParam[0] = DXL_LOBYTE(DXL_LOWORD(qCts_M[2]));
  elbowParam[1] = DXL_HIBYTE(DXL_LOWORD(qCts_M[2]));
  elbowParam[2] = DXL_LOBYTE(DXL_HIWORD(qCts_M[2]));
  elbowParam[3] = DXL_HIBYTE(DXL_HIWORD(qCts_M[2]));

  /* Writes Packets */
  addParamResult = syncWritePacket.addParam(OCM::ID_SHOULDER,  shoulderParam);
  addParamResult = syncWritePacket.addParam(OCM::ID_ELEVATION, elevateParam);
  addParamResult = syncWritePacket.addParam(OCM::ID_ELBOW,     elbowParam);
  dxlCommResult = syncWritePacket.txPacket();
  syncWritePacket.clearParam();

  return dxlCommResult;
}
