/* This establishes the packets for communication to and from the robotic arm support via the serial port.

   Created 3/1/2021
   by erick nunez
*/

#include <Arduino.h>
#include "RobotComm.h"
#include "PCComm.h"
#include "ForceSensor.h"
#include "AdmittanceModel.h"
#include "UtilityFunctions.h"

/* ------------------------------------------------------------------------------------------------------/
/ Serial Packet Contructor  -----------------------------------------------------------------------------/
/-------------------------------------------------------------------------------------------------------*/
PCComm::PCComm(usb_serial_class  *ptrSer, const int baudrate)
  : _BAUDRATE{baudrate}
{
  pcPort_M = ptrSer;
  pcPort_M->begin(_BAUDRATE);
}

/* ------------------------------------------------------------------------------------------------------/
/ Serial Packets Setters and  Getters -------------------------------------------------------------------/
/-------------------------------------------------------------------------------------------------------*/
bool PCComm::DataAvailable() {
  return pcPort_M->available();
}

bool PCComm::ModifyMassXY() {
  return _NEW_MASS_XY;
}

bool PCComm::ModifyMassZ() {
  return _NEW_MASS_Z;
}

bool PCComm::ModifyDampingXY() {
  return _NEW_DAMPING_XY;
}

bool PCComm::ModifyDampingZ() {
  return _NEW_DAMPING_Z;
}

bool PCComm::ModifyScalingFactor() {
  return _NEW_SCALING_FACTOR;
}

bool PCComm::ModifyFilter() {
  return _NEW_FILTER;
}

bool PCComm::NewExternalForces() {
  return _NEW_EXT_FORCES;
}

float PCComm::GetNewMassXY() {
  _NEW_MASS_XY = false;
  return newMassXY_M;
}

float PCComm::GetNewMassZ() {
  _NEW_MASS_Z = false;
  return newMassZ_M;
}

float PCComm::GetNewDampingXY() {
  _NEW_DAMPING_XY = false;
  return newDampingXY_M;
}

float PCComm::GetNewDampingZ() {
  _NEW_DAMPING_Z = false;
  return newDampingZ_M;
}

float PCComm::GetNewScalingFactor() {
  _NEW_SCALING_FACTOR = false;
  return newScalingFactor_M;
}

float * PCComm::GetExternalForces(){
  _NEW_EXT_FORCES = false;
  return ExtForces_M;
}

float PCComm::GetNewFilter() {
  _NEW_FILTER = false;
  return newFilter_M;
}

bool PCComm::NewGoalQAvailable(){
  return newGoalQ_M;
}

bool PCComm::NewGoalQdotAvailable(){
  return newGoalQdot_M;
}

bool PCComm::NewGoalCurrentAvailable(){
  return newGoalCurrent_M;
}

float * PCComm::GetNewGoalQ(){
  newGoalQ_M = false;
  return goalQ_M;
}

float * PCComm::GetNewGoalQdot(){
  newGoalQdot_M = false;
  return goalQdot_M;
}

float * PCComm::GetNewCurrent(){
  newGoalCurrent_M = false;
  return goalCurrent_M;
}

bool PCComm::NewGoalXYZAvailable(){
  return newGoalXYZ_M;
}

bool PCComm::NewGoalXYZdotAvailable(){
  return newGoalXYZdot_M;
}

float * PCComm::GetNewGoalXYZ(){
  newGoalXYZ_M = false;
  return goalXYZ_M;
}

float * PCComm::GetNewGoalXYZdot(){
  newGoalXYZdot_M = false;
  return goalXYZdot_M;
}

bool PCComm::SetTorqueMode(){
  return newTorqueMode_M;
}

uint8_t PCComm::GetNewMode(){
  newTorqueMode_M = false;
  return torqueMode_M;
}

bool PCComm::DataRequested() {
  return dataRequested_M;
}

/* ------------------------------------------------------------------------------------------------------/
/ Serial Packet Writer ----------------------------------------------------------------------------------/
/ - Streams data back to the PC for logging or inferfacing ----------------------------------------------/
/-------------------------------------------------------------------------------------------------------*/
void PCComm::WritePackets(unsigned long &totalTime, ForceSensor &Sensor, AdmittanceModel &Model, RobotComm &Robot, unsigned long &loopTime) {
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
                                          :   80-81,...
                                RawFx(cts):   82, 83,...
                                RawFy(cts):   84, 85,...
                                RawFz(cts):   86, 87,...
                                RawTx(cts):   88, 89,...
                                RawTy(cts):   90, 91,...
                                RawTz(cts):   92, 93,...
                               GlobalFx(N):   94, 95, 96, 97,...
                               GlobalFy(N):   98, 99,100,101,...
                               GlobalFz(N):  102,103,104,105,...
                              GlobalTx(Nm):  106,107,108,109,...
                              GlobalTy(Nm):  110,111,112,113,...
                              GlobalTz(Nm):  114,115,116,117,...
                                 Modelx(m):  118,119,120,121,...
                                 Modely(m):  122,123,124,125,...
                                 Modelz(m):  126,127,128,129,...
                            Modelxdot(m/s):  130,131,132,133,...
                            Modelydot(m/s):  134,135,136,137,...
                            Modelzdot(m/s):  138,139,140,141,...
                                          :  142,143,...
                                MassXY(kg):  144,145,146,147,...
                                 MassZ(kg):  148,149,150,151,...
                           DampingXY(Ns/m):  152,153,154,155,...
                            DampingZ(Ns/m):  156,157,158,159,...
                                          :  160,161,162,...
                          currentDriveMode:  163,...
                                  loopTime:  164,165,166,167,...
                                  CheckSum:  168,169]
  */
  byte RxPacket[_TX_PKT_LEN] = {0};
  uint16_t packetSum    = 0;
  byte *packetBuffer    = nullptr;

  // Header Bytes 
  for (int16_t i = 0; i < 4; i++) {
    RxPacket[i] = _WRITEHEADER[i];
  }

  // Elapsed Time 
  RxPacket[4] = DXL_LOBYTE(DXL_LOWORD(totalTime));
  RxPacket[5] = DXL_HIBYTE(DXL_LOWORD(totalTime));
  RxPacket[6] = DXL_LOBYTE(DXL_HIWORD(totalTime));
  RxPacket[7] = DXL_HIBYTE(DXL_HIWORD(totalTime));

  // Robot State Variables
  // Motor's PresQ (Radians)
  packetBuffer = floatArrayToBytes(Robot.GetPresQ());
  for (int16_t i = _TX_presQ_SLOT; i < _TX_presQDOT_SLOT; i++){
    RxPacket[i] = packetBuffer[i - _TX_presQ_SLOT];
  }
  // Motor PresQdot (radians/sec)
  packetBuffer = floatArrayToBytes(Robot.GetPresQDot());
  for (int16_t i = _TX_presQDOT_SLOT; i < _TX_presXYZ_SLOT; i++){
    RxPacket[i] = packetBuffer[i - _TX_presQDOT_SLOT];
  }
  // Robot Present XYZ (meters)
  packetBuffer = floatArrayToBytes(Robot.GetPresPos());
  for (int16_t i = _TX_presXYZ_SLOT; i < _TX_presXYZdot_SLOT; i++){
    RxPacket[i] = packetBuffer[i - _TX_presXYZ_SLOT];
  }
  // Robot Present XYZdot (meters/sec)
  packetBuffer = floatArrayToBytes(Robot.GetPresVel());
  for (int16_t i = _TX_presXYZdot_SLOT; i < _TX_goalQ_SLOT; i++){
    RxPacket[i] = packetBuffer[i - _TX_presXYZdot_SLOT];
  }
  // Motor GoalQ (radians)
  packetBuffer = floatArrayToBytes(Robot.GetGoalQ());
  for (int16_t i = _TX_goalQ_SLOT; i < _TX_goalQDOT_SLOT; i++){
    RxPacket[i] = packetBuffer[i - _TX_goalQ_SLOT];
  }
  // Motor GoalQdot (radians/sec)
  packetBuffer = floatArrayToBytes(Robot.GetGoalQDot());
  for (int16_t i = _TX_goalQDOT_SLOT; i < _TX_1stBlank_SLOT; i++){
    RxPacket[i] = packetBuffer[i - _TX_goalQDOT_SLOT];
  }

  // Model State Variables
  // Raw Forces and Torques (counts)
  // packetBuffer = uint16ArrayToBytes(Sensor.GetRawCtsFT());
  // for (int16_t i = _TX_rawFTcts_SLOT; i < _TX_globalF_SLOT; i++){
  //   RxPacket[i] = packetBuffer[i - _TX_rawFTcts_SLOT];
  // }
  // Raw Forces (Newtons) for testing
  packetBuffer = floatArrayToBytes(Sensor.GetFilteredForces());
  for (int16_t i = _TX_rawFTcts_SLOT; i < _TX_globalF_SLOT; i++){
    RxPacket[i] = packetBuffer[i - _TX_rawFTcts_SLOT];
  }
  // Global Forces (Newtons)
  packetBuffer = floatArrayToBytes(Sensor.GetGlobalForces());
  for (int16_t i = _TX_globalF_SLOT; i < _TX_globalT_SLOT; i++){
    RxPacket[i] = packetBuffer[i - _TX_globalF_SLOT];
  }
  // Global Torques (Newton-meters)
  packetBuffer = floatArrayToBytes(Sensor.GetGlobalTorques());
  for (int16_t i = _TX_globalT_SLOT; i < _TX_modelXYZ_SLOT; i++){
    RxPacket[i] = packetBuffer[i - _TX_globalT_SLOT];
  }
  // Model Position (meters)
  packetBuffer = floatArrayToBytes(Model.GetGoalPos());
  for (int16_t i = _TX_modelXYZ_SLOT; i < _TX_modelXYZdot_SLOT; i++){
    RxPacket[i] = packetBuffer[i - _TX_modelXYZ_SLOT];
  }
  // Model Velocity (meters/sec)
  packetBuffer = floatArrayToBytes(Model.GetGoalVel());
  for (int16_t i = _TX_modelXYZdot_SLOT; i < _TX_2ndBlank_SLOT; i++){
    RxPacket[i] = packetBuffer[i - _TX_modelXYZdot_SLOT];
  }

  // Model Mass (kg)
  packetBuffer = floatArrayToBytes(Model.GetMass());
  for (int16_t i = _TX_mass_SLOT; i < _TX_mass_SLOT + 4; i++){
    RxPacket[i] = packetBuffer[i - _TX_mass_SLOT];
    RxPacket[i+4] = packetBuffer[i - _TX_mass_SLOT + 8];
  }
  // Model Damping (Ns/m)
  packetBuffer = floatArrayToBytes(Model.GetDamping());
  for (int16_t i = _TX_damping_SLOT; i < _TX_damping_SLOT + 4; i++){
    RxPacket[i] = packetBuffer[i - _TX_damping_SLOT];
    RxPacket[i+4] = packetBuffer[i - _TX_damping_SLOT + 8];
  }

  // Robot Drive Mode
  RxPacket[_TX_PKT_LEN - 7] = Robot.GetTorqueState();

  // looptime
  RxPacket[_TX_PKT_LEN - 6] = DXL_LOBYTE(DXL_LOWORD(loopTime));
  RxPacket[_TX_PKT_LEN - 5] = DXL_HIBYTE(DXL_LOWORD(loopTime));
  RxPacket[_TX_PKT_LEN - 4] = DXL_LOBYTE(DXL_HIWORD(loopTime));
  RxPacket[_TX_PKT_LEN - 3] = DXL_HIBYTE(DXL_HIWORD(loopTime));

  // check Sum
  for (int16_t i = 0; i < _TX_PKT_LEN - 2; i++) {
    packetSum += RxPacket[i];
  }
  RxPacket[_TX_PKT_LEN - 2] = floor(packetSum / 256);
  RxPacket[_TX_PKT_LEN - 1] = floor(packetSum % 256);

  // Reset the data requested flag
  dataRequested_M = false;

  // write data packet
  pcPort_M->write(RxPacket,_TX_PKT_LEN);

}

/* ------------------------------------------------------------------------------------------------------/
/ Serial Packet Reader ----------------------------------------------------------------------------------/
/-------------------------------------------------------------------------------------------------------*/
void PCComm::ReadPackets() {
  /* Rx Packet Structure : Header: [ 0, 1, 2, 3,...
                          Actions:   4, 5, 6,...
                             Data:   7 - 57,...
                         CheckSum:  58, 59]
  */
  byte RXPacket[_RX_PKT_LEN];
  byte tempHeader[4];
  int16_t SumCheck;
  int16_t CHECKSUM;

  // Check for instructions
  unsigned long timeOutTime = millis();
  while (pcPort_M->available() < _RX_PKT_LEN) {
    if (millis() - timeOutTime > 5){
        while(pcPort_M->available()) pcPort_M->read();
        return;
    }
  }
  
  // Read Instructions
  for (int16_t i = 0; i < _RX_PKT_LEN; i++) {
    RXPacket[i] = pcPort_M->read();
  }

  // Verify Packet
  CHECKSUM = bytesToCounts(RXPacket[_RX_PKT_LEN - 2], RXPacket[_RX_PKT_LEN - 1]);
  SumCheck = 0;
  for (int16_t i = 0; i < _RX_PKT_LEN - 2; i++) {
    SumCheck += RXPacket[i];
  }
  if (SumCheck != CHECKSUM) return;

  // Perform action based on type of Packet
  for (int16_t i = 0; i < 4; i++) {
    tempHeader[i] = RXPacket[i];
  }
  if(memcmp(_REQUESTHEADER, tempHeader, sizeof(_REQUESTHEADER)) == 0){
    // Frame requested
    dataRequested_M = true;
    return;
  }
  if (memcmp(_CONFIGHEADER, tempHeader, sizeof(_CONFIGHEADER)) == 0) {
      // Found Configuration Packet
      ConfigPacketRX(RXPacket);
      return;
  }
  if (memcmp(_MODHEADER, tempHeader, sizeof(_MODHEADER)) == 0) {
    // Found Parameter Modifier Packet
    ModifierPacketRX(RXPacket);
    return;
  }
  if (memcmp(_CTRLHEADER, tempHeader, sizeof(_CTRLHEADER)) == 0) {
    // Found Robot Control Packet
    ControlPacketRX(RXPacket);
    return;
  }
}

/* ---------------------------------------------------------------------------------------/
/ Configuration RX Packet ----------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
void PCComm::ConfigPacketRX(byte * RxPacket) {
  /* Return Packet Configurator Structure:  Config Flags: 7-24,...
            (See below) 
  */
  SendFlagResets();
  if (RxPacket[7])  _SEND_RAWF          = true;
  if (RxPacket[8])  _SEND_XYZGOAL       = true;
  if (RxPacket[9])  _SEND_XYZDOTGOAL    = true;
  if (RxPacket[10]) _SEND_XYZBOTGOAL    = true;
  if (RxPacket[11]) _SEND_XYZDOTBOTGOAL = true;
  if (RxPacket[12]) _SEND_PRESQCTS      = true;
  if (RxPacket[13]) _SEND_PRESQDOTCTS   = true;
  if (RxPacket[14]) _SEND_PRESQ         = true;
  if (RxPacket[15]) _SEND_PRESQDOT      = true;
  if (RxPacket[16]) _SEND_GOALQCTS      = true;
  if (RxPacket[17]) _SEND_GOALQDOTCTS   = true;
  if (RxPacket[18]) _SEND_GOALQ         = true;
  if (RxPacket[19]) _SEND_GOALQDOT      = true;
  if (RxPacket[20]) _SEND_MASS          = true;
  if (RxPacket[21]) _SEND_DAMPING       = true;
  if (RxPacket[22]) _SEND_SPRING_F      = true;
  if (RxPacket[23]) _SEND_TOTAL_FORCES  = true;
  if (RxPacket[24]) _SEND_FORCE_FILTER  = true;
}

void PCComm::SendFlagResets() {
  _SEND_RAWF          = false;
  _SEND_XYZGOAL       = false;
  _SEND_XYZDOTGOAL    = false;
  _SEND_XYZBOTGOAL    = false;
  _SEND_XYZDOTBOTGOAL = false;
  _SEND_PRESQCTS      = false;
  _SEND_PRESQDOTCTS   = false;
  _SEND_PRESQ         = false;
  _SEND_PRESQDOT      = false;
  _SEND_GOALQCTS      = false;
  _SEND_GOALQDOTCTS   = false;
  _SEND_GOALQ         = false;
  _SEND_GOALQDOT      = false;
  _SEND_MASS          = false;
  _SEND_DAMPING       = false;
  _SEND_SPRING_F      = false;
  _SEND_TOTAL_FORCES  = false;
  _SEND_FORCE_FILTER  = false;
}

/* ---------------------------------------------------------------------------------------/
/ Modifier RX Packet ---------------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
void PCComm::ModifierPacketRX(byte * RxPacket) {
  /* Admittance Model Parameter Modifier Packet Data Structure: 
                                          Modifier Byte 1:  4,...
                                          Modifier Byte 2:  5,...
    Modifier Byte 1                                     _:  6,...
    [1]:MassXY                                 New MassXY:  7, 8, 9,10,... 
    [2]:MassZ                                   New MassZ: 11,12,13,14,... 
    [4]:DampingXY                           New DampingXY: 15,16,17,18,...
    [8]:DampingZ                             New DampingZ: 19,20,21,22,...
    [16]:eForces                          New External Fx: 23,24,25,26,...
                                          New External Fy: 27,28,29,30,...
                                          New External Fz: 31,32,33,34,...
                                       New Scaling Factor: 35,...
    Modifier Byte 2               New Force Sensor Filter: 36,...
    [1]: Scaling Factor                                  : 37-47,...                                                  
    [2]: Force Filter Value                      CheckSum: 58,59] 
  */
  byte mask = 1;
  byte bitArrayLarge[7];
  byte bitArraySmall[2];
  for (int16_t i = 0; i < 7; i++) {
    bitArrayLarge[i] = (RxPacket[4] & (mask << i)) != 0;
  }
  for (int16_t i = 0; i < 2; i++){
    bitArraySmall[i] = (RxPacket[5] & (mask << i)) != 0;
  }
  if (bitArrayLarge[0] == 1) {
    _NEW_MASS_XY = true;
    newMassXY_M = bytesToFloat(RxPacket[7], RxPacket[8], RxPacket[9], RxPacket[10]);
  }
  if (bitArrayLarge[1] == 1) {
    _NEW_MASS_Z = true;
    newMassZ_M = bytesToFloat(RxPacket[11], RxPacket[12], RxPacket[13], RxPacket[14]);
  }
  if (bitArrayLarge[2] == 1) {
    _NEW_DAMPING_XY = true;
    newDampingXY_M = bytesToFloat(RxPacket[15], RxPacket[16], RxPacket[17], RxPacket[18]);
  }
  if (bitArrayLarge[3] == 1) {
    _NEW_DAMPING_Z = true;
    newDampingZ_M = bytesToFloat(RxPacket[19], RxPacket[20], RxPacket[21], RxPacket[22]);
  }
  if (bitArrayLarge[4] == 1) {
    _NEW_EXT_FORCES = true;
    ExtForces_M[0] = bytesToFloat(RxPacket[23], RxPacket[24], RxPacket[25], RxPacket[26]);
    ExtForces_M[1] = bytesToFloat(RxPacket[27], RxPacket[28], RxPacket[29], RxPacket[30]);
    ExtForces_M[2] = bytesToFloat(RxPacket[31], RxPacket[32], RxPacket[33], RxPacket[34]);
  }
  if (bitArraySmall[0] == 1) {
    _NEW_SCALING_FACTOR = true;
    newScalingFactor_M = (float)(0.01 * RxPacket[35]);
  }
  if (bitArraySmall[1] == 1) {
    _NEW_FILTER = true;
    newFilter_M = (float)(0.01 * RxPacket[36]);
  }
}

/* ---------------------------------------------------------------------------------------/
/ Control RX Packet ----------------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
void PCComm::ControlPacketRX(byte * RxPacket) {
  /* Robot Control Packet Data Structure: New Robot Goal Check:  4,...
                                          New Model Goal Check:  5,...
                                                New Drive mode:  6,...
                                                 Goal Q1 Value:  7, 8, 9,10,...
                                                 Goal Q2 Value: 11,12,13,14,...
                                                 Goal Q4 Value: 15,16,17,18,...
                                              Goal Qdot1 Value: 19,20,21,22,...
                                              Goal Qdot2 Value: 23,24,25,26,...
                                              Goal Qdot4 Value: 27,28,29,30,...
                                         Goal Model PosX Value: 31,32,33,34,...
                                         Goal Model PosY Value: 35,36,37,38,...
                                         Goal Model PosZ Value: 39,40,41,42,...
                                         Goal Model VelX Value: 43,44,45,46,...
                                         Goal Model VelY Value: 47,48,49,50,...
                                         Goal Model VelZ Value: 51,52,53,54,...
                                                             _: 55,...
                                                   Torque Mode: 56,...
                                                             _: 57,...
                                                      CheckSum: 58,59] 
  */

  // New Q Goals
  if (RxPacket[4]) {
    newGoalQ_M = true;
    goalQ_M[0] = bytesToFloat(RxPacket[7], RxPacket[8], RxPacket[9], RxPacket[10]);
    goalQ_M[1] = bytesToFloat(RxPacket[11], RxPacket[12], RxPacket[13], RxPacket[14]);
    goalQ_M[2] = bytesToFloat(RxPacket[15], RxPacket[16], RxPacket[17], RxPacket[18]);
  }

  // New Model XYZ Goals
  if (RxPacket[5]) {
    newGoalXYZ_M = true;
    goalXYZ_M[0] = bytesToFloat(RxPacket[31], RxPacket[32], RxPacket[33], RxPacket[34]);
    goalXYZ_M[1] = bytesToFloat(RxPacket[35], RxPacket[36], RxPacket[37], RxPacket[38]);
    goalXYZ_M[2] = bytesToFloat(RxPacket[39], RxPacket[40], RxPacket[41], RxPacket[42]);
  }

  // New Drive Mode
  if (RxPacket[6] && RxPacket[56] != torqueMode_M) {
    newTorqueMode_M = true;
    torqueMode_M = RxPacket[56];
  }
}
