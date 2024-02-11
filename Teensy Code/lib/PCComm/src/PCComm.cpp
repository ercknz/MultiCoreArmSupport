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
  if (~_NEW_EXT_FORCE_X) {
    ExtForces_M[0] = 0.0f;
  }
  if (~_NEW_EXT_FORCE_Y) {
    ExtForces_M[1] = 0.0f;
  }
  if (~_NEW_EXT_FORCE_Z) {
    ExtForces_M[2] = 0.0f;
  }
  _NEW_EXT_FORCE_X = false;
  _NEW_EXT_FORCE_Y = false;
  _NEW_EXT_FORCE_Z = false;
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

bool PCComm::ChangeTorqueMode(){
  return newTorqueMode_M;
}

uint8_t PCComm::GetNewMode(){
  newTorqueMode_M = false;
  return torqueMode_M;
}

/* ------------------------------------------------------------------------------------------------------/
/ Serial Packet Writer ----------------------------------------------------------------------------------/
/ - Streams data back to the PC for logging or inferfacing ----------------------------------------------/
/-------------------------------------------------------------------------------------------------------*/
void PCComm::WritePackets(unsigned long &totalTime, ForceSensor &Sensor, AdmittanceModel &Model, RobotComm &Robot, unsigned long &loopTime) {
  byte RxPacket[_TX_PKT_LEN] = {0};
  int16_t slotsFilled   = 0;
  int16_t dataPosition  = 44;
  uint16_t packetSum    = 0;
  int16_t byteLen       = 4;

  // Header Bytes 
  for (int16_t i = 0; i < 4; i++) {
    RxPacket[i] = _WRITEHEADER[i];
  }

  // Elapsed Time 
  RxPacket[4] = DXL_LOBYTE(DXL_LOWORD(totalTime));
  RxPacket[5] = DXL_HIBYTE(DXL_LOWORD(totalTime));
  RxPacket[6] = DXL_LOBYTE(DXL_HIWORD(totalTime));
  RxPacket[7] = DXL_HIBYTE(DXL_HIWORD(totalTime));

  // Global Forces, Positions, and Velocities
  byte * GlobalF_bytes = floatArrayToBytes(Sensor.GetGlobalF());
  for (int16_t i = 8; i < 20; i++) {
    RxPacket[i] = GlobalF_bytes[i - 8];
  }
  byte * PresPos_bytes = floatArrayToBytes(Robot.GetPresPos());
  for (int16_t i = 20; i < 32; i++) {
    RxPacket[i] = PresPos_bytes[i - 20];
  }
  byte * PresVel_bytes = floatArrayToBytes(Robot.GetPresVel());
  for (int16_t i = 32; i < 44; i++) {
    RxPacket[i] = PresVel_bytes[i - 32];
  }

  // Optional Data Slots
  if (_SEND_RAWF && slotsFilled < _MAX_TX_DATA_SLOTS) {
    byte * RawF_bytes = floatArrayToBytes(Sensor.GetRawF());
    for (int16_t i = dataPosition; i < dataPosition + (3 * byteLen); i++) {
      RxPacket[i] = RawF_bytes[i - dataPosition];
    }
    slotsFilled += 3;
    dataPosition += (3 * byteLen);
  }
  if (_SEND_XYZGOAL && slotsFilled < _MAX_TX_DATA_SLOTS) {
    byte * xyzGoal_bytes = floatArrayToBytes(Model.GetGoalPos());
    for (int16_t i = dataPosition; i < dataPosition + (3 * byteLen); i++) {
      RxPacket[i] = xyzGoal_bytes[i - dataPosition];
    }
    slotsFilled += 3;
    dataPosition += (3 * byteLen);
  }
  if (_SEND_XYZDOTGOAL && slotsFilled < _MAX_TX_DATA_SLOTS) {
    byte * xyzDotGoal_bytes = floatArrayToBytes(Model.GetGoalVel());
    for (int16_t i = dataPosition; i < dataPosition + (3 * byteLen); i++) {
      RxPacket[i] = xyzDotGoal_bytes[i - dataPosition];
    }
    slotsFilled += 3;
    dataPosition += (3 * byteLen);
  }
  if (_SEND_XYZBOTGOAL && slotsFilled < _MAX_TX_DATA_SLOTS) {
    byte * xyzBotGoal_bytes = floatArrayToBytes(Robot.GetGoalPos());
    for (int16_t i = dataPosition; i < dataPosition + (3 * byteLen); i++) {
      RxPacket[i] = xyzBotGoal_bytes[i - dataPosition];
    }
    slotsFilled += 3;
    dataPosition += (3 * byteLen);
  }
  if (_SEND_XYZDOTBOTGOAL && slotsFilled < _MAX_TX_DATA_SLOTS) {
    byte * xyzDotBotGoal_bytes = floatArrayToBytes(Robot.GetGoalVel());
    for (int16_t i = dataPosition; i < dataPosition + (3 * byteLen); i++) {
      RxPacket[i] = xyzDotBotGoal_bytes[i - dataPosition];
    }
    slotsFilled += 3;
    dataPosition += (3 * byteLen);
  }
  if (_SEND_PRESQCTS && slotsFilled < _MAX_TX_DATA_SLOTS) {
    byte * PresQCts_bytes = int32ArrayToBytes(Robot.GetPresQCts());
    for (int16_t i = dataPosition; i < dataPosition + (3 * byteLen); i++) {
      RxPacket[i] = PresQCts_bytes[i - dataPosition];
    }
    slotsFilled += 3;
    dataPosition += (3 * byteLen);
  }
  if (_SEND_PRESQDOTCTS && slotsFilled < _MAX_TX_DATA_SLOTS) {
    byte * PresQDotCts_bytes = int32ArrayToBytes(Robot.GetPresQDotCts());
    for (int16_t i = dataPosition; i < dataPosition + (3 * byteLen); i++) {
      RxPacket[i] = PresQDotCts_bytes[i - dataPosition];
    }
    slotsFilled += 3;
    dataPosition += (3 * byteLen);
  }
  if (_SEND_PRESQ && slotsFilled < _MAX_TX_DATA_SLOTS) {
    byte * PresQ_bytes = floatArrayToBytes(Robot.GetPresQ());
    for (int16_t i = dataPosition; i < dataPosition + (3 * byteLen); i++) {
      RxPacket[i] = PresQ_bytes[i - dataPosition];
    }
    slotsFilled += 3;
    dataPosition += (3 * byteLen);
  }
  if (_SEND_PRESQDOT && slotsFilled < _MAX_TX_DATA_SLOTS) {
    byte * PresQDot_bytes = floatArrayToBytes(Robot.GetPresQDot());
    for (int16_t i = dataPosition; i < dataPosition + (3 * byteLen); i++) {
      RxPacket[i] = PresQDot_bytes[i - dataPosition];
    }
    slotsFilled += 3;
    dataPosition += (3 * byteLen);
  }
  if (_SEND_GOALQCTS && slotsFilled < _MAX_TX_DATA_SLOTS) {
    byte * GoalQCts_bytes = int32ArrayToBytes(Robot.GetGoalQCts());
    for (int16_t i = dataPosition; i < dataPosition + (3 * byteLen); i++) {
      RxPacket[i] = GoalQCts_bytes[i - dataPosition];
    }
    slotsFilled += 3;
    dataPosition += (3 * byteLen);
  }
  if (_SEND_GOALQDOTCTS && slotsFilled < _MAX_TX_DATA_SLOTS) {
    byte * GoalQDotCts_bytes = int32ArrayToBytes(Robot.GetGoalQDotCts());
    for (int16_t i = dataPosition; i < dataPosition + (3 * byteLen); i++) {
      RxPacket[i] = GoalQDotCts_bytes[i - dataPosition];
    }
    slotsFilled += 3;
    dataPosition += (3 * byteLen);
  }
  if (_SEND_GOALQ && slotsFilled < _MAX_TX_DATA_SLOTS) {
    byte * GoalQ_bytes = floatArrayToBytes(Robot.GetGoalQ());
    for (int16_t i = dataPosition; i < dataPosition + (3 * byteLen); i++) {
      RxPacket[i] = GoalQ_bytes[i - dataPosition];
    }
    slotsFilled += 3;
    dataPosition += (3 * byteLen);
  }
  if (_SEND_GOALQDOT && slotsFilled < _MAX_TX_DATA_SLOTS) {
    byte * GoalQDot_bytes = floatArrayToBytes(Robot.GetGoalQDot());
    for (int16_t i = dataPosition; i < dataPosition + (3 * byteLen); i++) {
      RxPacket[i] = GoalQDot_bytes[i - dataPosition];
    }
    slotsFilled += 3;
    dataPosition += (3 * byteLen);
  }
  if (_SEND_MASS && slotsFilled < _MAX_TX_DATA_SLOTS) {
    byte * Mass_bytes = floatArrayToBytes(Model.GetMass());
    for (int16_t i = dataPosition; i < dataPosition + (2 * byteLen); i++) {
      RxPacket[i] = Mass_bytes[i - dataPosition];
    }
    slotsFilled += 2;
    dataPosition += (2 * byteLen);
  }
  if (_SEND_DAMPING && slotsFilled < _MAX_TX_DATA_SLOTS) {
    byte * Damping_bytes = floatArrayToBytes(Model.GetDamping());
    for (int16_t i = dataPosition; i < dataPosition + (2 * byteLen); i++) {
      RxPacket[i] = Damping_bytes[i - dataPosition];
    }
    slotsFilled += 2;
    dataPosition += (2 * byteLen);
  }
  if (_SEND_SPRING_F && slotsFilled < _MAX_TX_DATA_SLOTS) {
    byte * springF_bytes = floatToBytes(Robot.GetSpringForce());
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      RxPacket[i] = springF_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }
  if (_SEND_TOTAL_FORCES && slotsFilled < _MAX_TX_DATA_SLOTS) {
    byte * totalF_bytes = floatArrayToBytes(Model.GetTotalForces());
    for (int16_t i = dataPosition; i < dataPosition + (3 * byteLen); i++) {
      RxPacket[i] = totalF_bytes[i - dataPosition];
    }
    slotsFilled += 3;
    dataPosition += (3 * byteLen);
  }
  if (_SEND_FORCE_FILTER && slotsFilled < _MAX_TX_DATA_SLOTS) {
    byte * forceFilter_bytes = floatToBytes(Sensor.GetForceFilter());
    for (int16_t i = dataPosition; i < dataPosition + byteLen; i++) {
      RxPacket[i] = forceFilter_bytes[i - dataPosition];
    }
    slotsFilled += 1;
    dataPosition += byteLen;
  }

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

  // write data packet
  pcPort_M->write(RxPacket,_TX_PKT_LEN);
}

/* ------------------------------------------------------------------------------------------------------/
/ Serial Packet Reader ----------------------------------------------------------------------------------/
/-------------------------------------------------------------------------------------------------------*/
void PCComm::ReadPackets() {
  /* Rx Packet Structure : Header: [ 0, 1, 2, 3,...
                          Actions:   4, 5, 6,...
                             Data:   7 - 47,...
                         CheckSum:  48, 49]
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
    [3]:DampingXY                           New DampingXY: 15,16,17,18,...
    [4]:DampingZ                             New DampingZ: 19,20,21,22,...
    [5]:eFx                               New External Fx: 23,24,25,26,...
    [6]:eFy                               New External Fy: 27,28,29,30,...
    [7]:eFz                               New External Fz: 31,32,33,34,...
                                       New Scaling Factor: 35,...
    Modifier Byte 2               New Force Sensor Filter: 36,...
    [1]: Scaling FActor                               
    [2]: Force Filter Value
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
    _NEW_EXT_FORCE_X = true;
    ExtForces_M[0] = bytesToFloat(RxPacket[23], RxPacket[24], RxPacket[25], RxPacket[26]);
  }
  if (bitArrayLarge[5] == 1) {
    _NEW_EXT_FORCE_Y = true;
    ExtForces_M[1] = bytesToFloat(RxPacket[27], RxPacket[28], RxPacket[29], RxPacket[30]);
  }
  if (bitArrayLarge[6] == 1) {
    _NEW_EXT_FORCE_Z = true;
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
  /* Robot Control Packet Data Structure:     New Q Goal Check:  4,...
                                           New Qdot Goal Check:  5,...
                                        New Current Goal Check:  6,...
                                                 Goal Q1 Value:  7, 8, 9,10,...
                                                 Goal Q2 Value: 11,12,13,14,...
                                                 Goal Q4 Value: 15,16,17,18,...
                                              Goal Qdot1 Value: 19,20,21,22,...
                                              Goal Qdot2 Value: 23,24,25,26,...
                                              Goal Qdot4 Value: 27,28,29,30,...
                                           Goal Current1 Value: 31,32,33,34,...
                                           Goal Current2 Value: 35,36,37,38,...
                                           Goal Current4 Value: 39,40,41,42,...
                                                             _: 43,44,...
                                                   Torque Mode: 45,...
                                                             _: 46,47,...
  */

  // New Q Goals
  if (RxPacket[4]) {
    newGoalQ_M = true;
    goalQ_M[0] = bytesToFloat(RxPacket[7], RxPacket[8], RxPacket[9], RxPacket[10]);
    goalQ_M[1] = bytesToFloat(RxPacket[11], RxPacket[12], RxPacket[13], RxPacket[14]);
    goalQ_M[2] = bytesToFloat(RxPacket[15], RxPacket[16], RxPacket[17], RxPacket[18]);
  }

  // New Qdot Goals
  if (RxPacket[5]) {
    newGoalQdot_M = true;
    goalQdot_M[0] = bytesToFloat(RxPacket[19], RxPacket[20], RxPacket[21], RxPacket[22]);
    goalQdot_M[1] = bytesToFloat(RxPacket[23], RxPacket[24], RxPacket[25], RxPacket[26]);
    goalQdot_M[2] = bytesToFloat(RxPacket[27], RxPacket[28], RxPacket[29], RxPacket[30]);
  }

  // New Motor Current Goals
  if (RxPacket[6]) {
    newGoalCurrent_M = true;
    goalCurrent_M[0] = bytesToFloat(RxPacket[31], RxPacket[32], RxPacket[33], RxPacket[34]);
    goalCurrent_M[1] = bytesToFloat(RxPacket[35], RxPacket[36], RxPacket[37], RxPacket[38]);
    goalCurrent_M[2] = bytesToFloat(RxPacket[39], RxPacket[40], RxPacket[41], RxPacket[42]);
  }

  // New Drive Mode
  if (RxPacket[45] != torqueMode_M){
    _NEW_MODE = true;
    torqueMode_M = RxPacket[5];
  }
}
