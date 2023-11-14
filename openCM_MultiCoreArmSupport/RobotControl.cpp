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
RobotControl::RobotControl(const float A1, const float L1, const float A2, const float L2, const float A3, const float Offset)
  :_A1A2{A1 + A2},
  _L1{L1},
  _L2{L2},
  _A3{A3},
  _OFFSET{Offset},
  _PHI{atan(Offset / L2)},
  _H_OF_L2{sqrt(pow(Offset, 2) + pow(L2, 2))},
  _Q1_MIN{OCM::SHOULDER_MIN_POS * OCM::DEGREES_PER_COUNT * (PI / 180.0)},
  _Q1_MAX{OCM::SHOULDER_MAX_POS * OCM::DEGREES_PER_COUNT * (PI / 180.0)},
  _Q2_LIMIT{abs((OCM::ELEVATION_MAX_POS - OCM::ELEVATION_CENTER) * OCM::DEGREES_PER_COUNT * (PI / 180.0) * (1/OCM::ELEVATION_RATIO))},
  _Q4_MIN{(OCM::ELBOW_MIN_POS - OCM::ELBOW_MIN_POS) * OCM::DEGREES_PER_COUNT * (PI / 180.0) + OCM::SHOULDER_OFFSET},
  _Q4_MAX{(OCM::ELBOW_MAX_POS - OCM::ELBOW_MIN_POS) * OCM::DEGREES_PER_COUNT * (PI / 180.0) + OCM::SHOULDER_OFFSET},
  _INNER_R{A1 + L1 + A2 - L2},
  _Z_LIMIT{abs(L1 * sin((OCM::ELEVATION_MAX_POS - OCM::ELEVATION_CENTER) * OCM::DEGREES_PER_COUNT * (PI / 180.0) * (1/OCM::ELEVATION_RATIO)))},
  _SPRING_Li{sqrt(pow(OCM::SPRING_SIDE_A,2) + pow(OCM::SPRING_SIDE_B,2) + 2 * OCM::SPRING_SIDE_A * OCM::SPRING_SIDE_B * OCM::COS_SIN_45)},
  _BETAi{asin((OCM::SPRING_SIDE_A/_SPRING_Li) * - OCM::COS_SIN_45)},
  _SPRING_Fi{OCM::SPRING_KS * (OCM::SPRING_XI - OCM::SPRING_X0) * sin(_BETAi + OCM::DEG_TO_RAD_45)}
{
  // Initalize RobotControl Class
  scalingFactor_M = OCM::SPRING_FORCE_SCALING_FACTOR;
}

/* ---------------------------------------------------------------------------------------/
/ Arm Support Get Member functions -------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
uint8_t RobotControl::GetCurrentTorqueMode(){
  return currentTorqueMode_M;
}

float* RobotControl::GetPresQ(){
  return qPres_M;
}

float* RobotControl::GetPresQDot(){
  return qDotPres_M;
}

int32_t* RobotControl::GetPresQCts(){ 
  return qPresCts_M;
} 

int32_t* RobotControl::GetPresQDotCts(){
  return qDotPresCts_M;
}

float* RobotControl::GetPresXYZ(){ 
  return xyzPres_M;
}

float* RobotControl::GetPresXYZdot(){
  return xyzDotPres_M;
}

float* RobotControl::GetGoalQ(){
  return q_M;
}

float* RobotControl::GetGoalQDot(){
  return qDot_M;
}

int32_t* RobotControl::GetGoalQCts(){ 
  return qCts_M;
}

int32_t* RobotControl::GetGoalQDotCts(){
  return qDotCts_M;
}

float* RobotControl::GetGoalXYZ(){ 
  return xyz_M;
}

float* RobotControl::GetGoalXYZdot(){
  return xyzDot_M;
}

float* RobotControl::GetPresCurrent(){
  return iPres_M;
}

int16_t* RobotControl::GetPresCurrentCts(){ 
  return iPresCts_M;
}

float RobotControl::GetSpringForce(){
  return springF_M;
} 

void RobotControl::InitializeGoals(){
  for (int i = 0; i < 3; i++){
    q_M[i] = qPres_M[i];
  }
}

/* ---------------------------------------------------------------------------------------/
/ ROBOT LEVEL FUNCTIONS ------------------------------------------------------------------/
/-----------------------------------------------------------------------------------------/
/-------------------------------------------------------------------------------/
/ Arm Support Motors Reading/Writing -------------------------------------------/
/------------------------------------------------------------------------------*/
void RobotControl::ReadRobot(dynamixel::GroupSyncRead &syncReadPacket){
  ReadMotors(syncReadPacket);
  fKine();
}

void RobotControl::WriteToRobot(float *xyz, float *xyzDot, bool &addParamResult, dynamixel::GroupSyncWrite &syncWritePacket){
  iKine(xyz, xyzDot);
  int returnInt = WriteToMotors(addParamResult, syncWritePacket);
}

/* -----------------------------------------------------------------------------/
/ Arm Support Inverse Kinematics Member function -------------------------------/
/------------------------------------------------------------------------------*/
void RobotControl::iKine(float *goalXYZ, float *goalXYZDot) {
  /*  NOTE:
              xyz_M[3] = {x, y, z}
              q_M[3] = {q1, q2, q4} = {shoulder, elevation, elbow} 
  */
  float L1_XY, OUTER_R, R, alpha, presR, presAlpha, beta, gamma, detJ;
  for (int i=0; i<3; i++){
    xyz_M[i]    = goalXYZ[i];
    xyzDot_M[i] = goalXYZDot[i];
  }

  /* Check Z limits */
  if (xyz_M[2] >  _Z_LIMIT) xyz_M[2] =  _Z_LIMIT;
  if (xyz_M[2] < -_Z_LIMIT) xyz_M[2] = -_Z_LIMIT;

  /* Find variables based on Z */
  q_M[1]  = asin((xyz_M[2] - _A3) / _L1);
  L1_XY   = sqrt(pow(_L1, 2) - pow((xyz_M[2] - _A3), 2));

   /* Checks if X and Y are both 0 */
  if ((abs(xyz_M[0]) < 0.001) && (abs(xyz_M[1]) < 0.001)) {
    q_M[0] = qPres_M[0];
    q_M[2] = _Q4_MAX;
    xyz_M[0] = _A1A2 * cos(q_M[0]) + _L1 * cos(q_M[0]) * cos(q_M[1]) + _OFFSET * sin(q_M[0] + q_M[2]) + _L2 * cos(q_M[0] + q_M[2]);
    xyz_M[1] = _A1A2 * sin(q_M[0]) + _L1 * sin(q_M[0]) * cos(q_M[1]) - _OFFSET * cos(q_M[0] + q_M[2]) + _L2 * sin(q_M[0] + q_M[2]);
  }

  /* R and Alpha */
  R       = sqrt(pow(xyz_M[0], 2) + pow(xyz_M[1], 2));
  alpha   = atan2(xyz_M[1], xyz_M[0]);
  if (alpha < 0.0f) alpha += 2 * PI;
  presR       = sqrt(pow(xyzPres_M[0], 2) + pow(xyzPres_M[1], 2));
  presAlpha   = atan2(xyzPres_M[1], xyzPres_M[0]);
  if (presAlpha < 0.0f) presAlpha += 2 * PI;

  /* Checks walls */
  OUTER_R = _A1A2 + _H_OF_L2 + L1_XY;
  if (R < _INNER_R) {
    R         = _INNER_R;
    xyz_M[0]  = _INNER_R * cos(alpha);
    xyz_M[1]  = _INNER_R * sin(alpha);
  }
  if (R > OUTER_R) {
    R         = OUTER_R;
    xyz_M[0]  = OUTER_R * cos(alpha);
    xyz_M[1]  = OUTER_R * sin(alpha);
  } 
  
  /* Finds and checks Elbow Angle */
  if ((abs(R-presR) < 0.01) && (alpha < presAlpha)){
    q_M[2] = qPres_M[2];
    gamma = PI - q_M[2];
  } else {
    gamma = acos((pow((_A1A2 + L1_XY), 2) + pow(_H_OF_L2, 2) - pow(xyz_M[0], 2) - pow(xyz_M[1], 2)) / (2 * _H_OF_L2 * (_A1A2 + L1_XY)));
    q_M[2] = PI - gamma + _PHI;
  }

  /* Finds and checks shoulder angle */
  beta = asin((_H_OF_L2 * sin(gamma)) / R);
  q_M[0] = alpha - beta;// + OCM::SHOULDER_OFFSET;
  if (q_M[0] < 0.0f) q_M[0] += 2 * PI;

  /* Check for nans */
  if (q_M[0] != q_M[0]) q_M[0] = qPres_M[0];
  if (q_M[2] != q_M[2]) q_M[2] = qPres_M[2];

  /* Check Jointspace Limits *
  if (q_M[2] < _Q4_MIN) q_M[2] = _Q4_MIN;
  if (q_M[2] > _Q4_MAX) q_M[2] = _Q4_MAX;
  if (q_M[0] < _Q1_MIN) q_M[0] = _Q1_MIN;
  if (q_M[0] > _Q1_MAX) q_M[0] = _Q1_MAX;*/

  /* Solve for joint angular velocities (psuedo inverse Jacobian) */
  detJ      = (-_L1 * sin(q_M[0]) - _L2 * sin(q_M[0] + q_M[2])) * (_L2 * cos(q_M[0] + q_M[2])) - (-_L2 * sin(q_M[0] + q_M[2])) * (_L1 * cos(q_M[0]) + _L2 * cos(q_M[0] + q_M[2]));
  qDot_M[0] = (xyzDot_M[0] * (_L2 * cos(q_M[0] + q_M[2])) + xyzDot_M[1] * (_L2 * sin(q_M[0] + q_M[2]))) / detJ;
  qDot_M[1] = xyzDot_M[2] / (_L1 * sqrt(1 - pow((xyz_M[2] / _L1), 2)));
  qDot_M[2] = (xyzDot_M[0] * (-_L1 * cos(q_M[0]) - _L2 * cos(q_M[0] + q_M[2])) + xyzDot_M[1] * (-_L1 * sin(q_M[0]) - _L2 * sin(q_M[0] + q_M[2]))) / detJ;
}

/* -----------------------------------------------------------------------------/
/ Arm Support Forward Kinematics Member Function -------------------------------/
/------------------------------------------------------------------------------*/
void  RobotControl::fKine() {
  /* Calculates the Taskspace Position */
  xyzPres_M[0] = _A1A2 * cos(qPres_M[0]) + _L1 * cos(qPres_M[0]) * cos(qPres_M[1]) + _OFFSET * sin(qPres_M[0] + qPres_M[2]) + _L2 * cos(qPres_M[0] + qPres_M[2]);
  xyzPres_M[1] = _A1A2 * sin(qPres_M[0]) + _L1 * sin(qPres_M[0]) * cos(qPres_M[1]) - _OFFSET * cos(qPres_M[0] + qPres_M[2]) + _L2 * sin(qPres_M[0] + qPres_M[2]);
  xyzPres_M[2] =   _L1 * sin(qPres_M[1]);

  /* Calculates Jacobian Matrix */
  J_M[0][0] = - _A1A2   * sin(qPres_M[0]) - _L1 * sin(qPres_M[0]) * cos(qPres_M[1]) + _OFFSET * cos(qPres_M[0] + qPres_M[2]) - _L2 * sin(qPres_M[0] + qPres_M[2]);
  J_M[0][1] = - _L1     * cos(qPres_M[0]) * sin(qPres_M[1]);
  J_M[0][2] =   _OFFSET * cos(qPres_M[0] + qPres_M[2]) - _L2 * sin(qPres_M[0] + qPres_M[2]);
  J_M[1][0] =   _A1A2   * cos(qPres_M[0]) + _L1 * cos(qPres_M[0]) * cos(qPres_M[1]) + _OFFSET * sin(qPres_M[0] + qPres_M[2]) + _L2 * cos(qPres_M[0] + qPres_M[2]);
  J_M[1][1] = - _L1     * sin(qPres_M[0]) * sin(qPres_M[1]);
  J_M[1][2] =   _OFFSET * sin(qPres_M[0] + qPres_M[2]) + _L2 * cos(qPres_M[0] + qPres_M[2]);
  J_M[2][1] =   _L1     * cos(qPres_M[1]);  // J31 = J33 = 0.0

  /* Calcaultes Taskspace Velociies */
  xyzDotPres_M[0] = qDotPres_M[0] * J_M[0][0] + qDotPres_M[1] * J_M[0][1] + qDotPres_M[2] * J_M[0][2];
  xyzDotPres_M[1] = qDotPres_M[0] * J_M[1][0] + qDotPres_M[1] * J_M[1][1] + qDotPres_M[2] * J_M[1][2];
  xyzDotPres_M[2] = qDotPres_M[0] * J_M[2][0] + qDotPres_M[1] * J_M[2][1] + qDotPres_M[2] * J_M[2][2];
}

/* -----------------------------------------------------------------------------/
/ Arm Support Spring Force Member Functions ------------------------------------/
/------------------------------------------------------------------------------*/
void RobotControl::CalculateSpringForce(float *forces){
  /* Calculated Variables */
  float maxCompensation = 0.5;
  float phaseCompensation;
  if (forces[2] < 0.0){
    phaseCompensation = maxCompensation * 0.40;
  } else {
    phaseCompensation = 0.0;
  }
  float alpha = 90.0 - qPres_M[1];
  float springLength = sqrt(pow(OCM::SPRING_SIDE_A,2) + pow(OCM::SPRING_SIDE_B,2) - 2 * OCM::SPRING_SIDE_A * OCM::SPRING_SIDE_B * cos(alpha));
  float beta = asin((OCM::SPRING_SIDE_A/springLength) * sin(alpha));
  springF_M = scalingFactor_M * phaseCompensation *(OCM::SPRING_KS * (springLength - _SPRING_Li + OCM::SPRING_XI - OCM::SPRING_X0) * sin(beta - qPres_M[1]) - _SPRING_Fi);
}

void RobotControl::SetScalingFactor(float newScalingFactor){
  scalingFactor_M = newScalingFactor;
}

/* ---------------------------------------------------------------------------------------/
/ MOTOR LEVEL FUNCTIONS ------------------------------------------------------------------/
/-----------------------------------------------------------------------------------------/
/-------------------------------------------------------------------------------/
/ Arm Support DXL Torque Enabling Member Function ------------------------------/
/------------------------------------------------------------------------------*/
void  RobotControl::EnableTorque(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler  *packetHandler, uint8_t state) {
  /* Modes to Select from:
   *  [5]:  Full Admittance Control (Shoulder, Elbow, and Elevation TORQUE ENABLED)
   *  [10]: Planar Admittance Control ONLY (Shoulder and  Elbow TORQUE ENABLED, Elevation TORQUE DISABLED)
   *  [15]: Vertical Admittance Control ONLY (Elevation TORQUE ENABLED, Shoulder and Elbow TORQUE DISABLED)
   *  [20]: Fully Passive (Shoulder, Elbow, and Elevation TORQUE DISABLED)
   */
  using namespace OCM;
  currentTorqueMode_M = state;
  int dxlCommResult;
//  if ((state == 5)||(state == 10)){
//    dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_SHOULDER,     ADDRESS_TORQUE_ENABLE, ENABLE, &dxl_error);
//    dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_ELBOW,        ADDRESS_TORQUE_ENABLE, ENABLE, &dxl_error);
//  }
//  if ((state == 5)||(state == 15)){
//    dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_ELEVATION,    ADDRESS_TORQUE_ENABLE, ENABLE, &dxl_error);
//  }
  if ((state == 20)||(state == 10)){
    dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_ELEVATION,    ADDRESS_TORQUE_ENABLE, DISABLE, &dxl_error);
  }
  if ((state == 20)||(state == 15)){
    dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_SHOULDER,     ADDRESS_TORQUE_ENABLE, DISABLE, &dxl_error);
    dxlCommResult = packetHandler->write1ByteTxRx(portHandler, ID_ELBOW,        ADDRESS_TORQUE_ENABLE, DISABLE, &dxl_error);
  }
}

/* -----------------------------------------------------------------------------/
/ Arm Support DXL Configuration Member Function --------------------------------/
/------------------------------------------------------------------------------*/
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

/* -----------------------------------------------------------------------------/
/ Arm Support DXL Read Member Function -----------------------------------------/
/------------------------------------------------------------------------------*/
void  RobotControl::ReadMotors(dynamixel::GroupSyncRead  &syncReadPacket) {
  /* Read Position, Velocity,  */
  int dxlCommResult = syncReadPacket.txRxPacket();
  qPresCts_M[0]    = syncReadPacket.getData(OCM::ID_SHOULDER,  OCM::ADDRESS_PRESENT_POSITION, OCM::LEN_PRESENT_POSITION);
  qPresCts_M[1]    = syncReadPacket.getData(OCM::ID_ELEVATION, OCM::ADDRESS_PRESENT_POSITION, OCM::LEN_PRESENT_POSITION);
  qPresCts_M[2]    = syncReadPacket.getData(OCM::ID_ELBOW,     OCM::ADDRESS_PRESENT_POSITION, OCM::LEN_PRESENT_POSITION);
  qDotPresCts_M[0] = syncReadPacket.getData(OCM::ID_SHOULDER,  OCM::ADDRESS_PRESENT_VELOCITY, OCM::LEN_PRESENT_VELOCITY);
  qDotPresCts_M[1] = syncReadPacket.getData(OCM::ID_ELEVATION, OCM::ADDRESS_PRESENT_VELOCITY, OCM::LEN_PRESENT_VELOCITY);
  qDotPresCts_M[2] = syncReadPacket.getData(OCM::ID_ELBOW,     OCM::ADDRESS_PRESENT_VELOCITY, OCM::LEN_PRESENT_VELOCITY);
  iPresCts_M[0]    = syncReadPacket.getData(OCM::ID_SHOULDER,  OCM::ADDRESS_PRESENT_CURRENT,  OCM::LEN_PRESENT_CURRENT); 
  iPresCts_M[1]    = syncReadPacket.getData(OCM::ID_ELEVATION, OCM::ADDRESS_PRESENT_CURRENT,  OCM::LEN_PRESENT_CURRENT); 
  iPresCts_M[2]    = syncReadPacket.getData(OCM::ID_ELBOW,     OCM::ADDRESS_PRESENT_CURRENT,  OCM::LEN_PRESENT_CURRENT); 

  /* Convert from Motor Counts */
  qPres_M[0]      =  (qPresCts_M[0]) * OCM::DEGREES_PER_COUNT * (PI / 180.0) + OCM::SHOULDER_OFFSET;
  qPres_M[1]      = -(qPresCts_M[1] - OCM::ELEVATION_CENTER) * OCM::DEGREES_PER_COUNT * (PI / 180.0) * (1/OCM::ELEVATION_RATIO);
  qPres_M[2]      =  (qPresCts_M[2] - OCM::ELBOW_MIN_POS) * OCM::DEGREES_PER_COUNT * (PI / 180.0);
  qDotPres_M[0]   = qDotPresCts_M[0] * OCM::RPM_PER_COUNT * (2.0 * PI / 60.0);
  qDotPres_M[1]   = qDotPresCts_M[1] * OCM::RPM_PER_COUNT * (2.0 * PI / 60.0) * (1/OCM::ELEVATION_RATIO);
  qDotPres_M[2]   = qDotPresCts_M[2] * OCM::RPM_PER_COUNT * (2.0 * PI / 60.0);
  iPres_M[0]      = iPresCts_M[0] * OCM::CURRENT_PER_COUNT; 
  iPres_M[1]      = iPresCts_M[1] * OCM::CURRENT_PER_COUNT;
  iPres_M[2]      = iPresCts_M[2] * OCM::CURRENT_PER_COUNT;
}

/* -----------------------------------------------------------------------------/
/ Arm Support DXL Write Member Function ----------------------------------------/
/------------------------------------------------------------------------------*/
int  RobotControl::WriteToMotors(bool &addParamResult, dynamixel::GroupSyncWrite &syncWritePacket) {
  int dxlCommResult;
  uint8_t elbowParam[4], shoulderParam[4], elevateParam[4];

  /* Convert to Motor Counts */
  qCts_M[0]    = (q_M[0] - OCM::SHOULDER_OFFSET) * (180.0 / PI) / OCM::DEGREES_PER_COUNT;
  qCts_M[1]    = OCM::ELEVATION_CENTER - (q_M[1] * OCM::ELEVATION_RATIO * (180.0 / PI) / OCM::DEGREES_PER_COUNT);
  qCts_M[2]    = OCM::ELBOW_MIN_POS + q_M[2] * (180.0 / PI) / OCM::DEGREES_PER_COUNT;
  qDotCts_M[0] = abs(qDot_M[0] * (60.0 / (2.0 * PI)) / OCM::RPM_PER_COUNT);
  qDotCts_M[1] = abs(qDot_M[1] * (60.0 / (2.0 * PI)) / OCM::RPM_PER_COUNT) * OCM::ELEVATION_RATIO;
  qDotCts_M[2] = abs(qDot_M[2] * (60.0 / (2.0 * PI)) / OCM::RPM_PER_COUNT);

  /* Check versus hard limits */
  if (qCts_M[0] < OCM::SHOULDER_MIN_POS) qCts_M[0] = OCM::SHOULDER_MIN_POS;
  if (qCts_M[0] > OCM::SHOULDER_MAX_POS) qCts_M[0] = OCM::SHOULDER_MAX_POS;
  if (qCts_M[1] < OCM::ELEVATION_MIN_POS) qCts_M[1] = OCM::ELEVATION_MIN_POS;
  if (qCts_M[1] > OCM::ELEVATION_MAX_POS) qCts_M[1] = OCM::ELEVATION_MAX_POS;
  if (qCts_M[2] < OCM::ELBOW_MIN_POS) qCts_M[2] = OCM::ELBOW_MIN_POS;
  if (qCts_M[2] > OCM::ELBOW_MAX_POS) qCts_M[2] = OCM::ELBOW_MAX_POS;

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
