/* This establishes the packets for communication to and from the robotic arm support via the serial port.

   Created 3/1/2021
   by erick nunez
*/

#ifndef PC_COMM_H
#define PC_COMM_H

#include <Arduino.h>
#include "RobotComm.h"
#include "ForceSensor.h"
#include "AdmittanceModel.h"

class PCComm {
  public:
         PCComm(usb_serial_class  *ptrSer, const int baudrate);
    bool DataAvailable();
    void ReadPackets();
    void WritePackets(unsigned long &totalTime, ForceSensor &Sensor, AdmittanceModel &Model, RobotComm &Robot, unsigned long &loopTime);
    bool ModifyMassXY();
    bool ModifyMassZ();
    bool ModifyDampingXY();
    bool ModifyDampingZ();
    bool ModifyScalingFactor();
    bool ModifyFilter();
    bool NewExternalForces();
    float GetNewMassXY();
    float GetNewMassZ();
    float GetNewDampingXY();
    float GetNewDampingZ();
    float GetNewScalingFactor();
    float * GetExternalForces();
    float GetNewFilter();
    bool NewGoalQAvailable();
    bool NewGoalQdotAvailable();
    bool NewGoalCurrentAvailable();
    float * GetNewGoalQ();
    float * GetNewGoalQdot();
    float * GetNewCurrent();
    bool NewGoalXYZAvailable();
    bool NewGoalXYZdotAvailable();
    float * GetNewGoalXYZ();
    float * GetNewGoalXYZdot();
    bool SetTorqueMode();
    uint8_t GetNewMode();
    bool DataRequested();

  protected:
    const int   _BAUDRATE;
    usb_serial_class *pcPort_M;
    const byte  _CONFIGHEADER[4]  = {150, 0, 69, 8};
    const byte  _MODHEADER[4]     = {150, 10, 10, 96};
    const byte  _CTRLHEADER[4]    = {150, 50, 50, 175};
    const byte _REQUESTHEADER[4] = {151, 125, 125, 251};
    const int16_t _RX_PKT_LEN = 60;
    const byte  _WRITEHEADER[4]   = {170, 8, 69, 0};
    const int16_t _TX_PKT_LEN = 150;
    const int16_t _TX_presQ_SLOT        = 8;
    const int16_t _TX_presQDOT_SLOT     = 20;
    const int16_t _TX_presXYZ_SLOT      = 32;
    const int16_t _TX_presXYZdot_SLOT   = 44;
    const int16_t _TX_goalQ_SLOT        = 56;
    const int16_t _TX_goalQDOT_SLOT     = 68;
    const int16_t _TX_1stBlank_SLOT     = 80;
    const int16_t _TX_rawFTcts_SLOT     = 82;
    const int16_t _TX_globalF_SLOT      = 94;
    const int16_t _TX_globalT_SLOT      = 106;
    const int16_t _TX_modelXYZ_SLOT     = 118;
    const int16_t _TX_modelXYZdot_SLOT  = 130;
    const int16_t _TX_2ndBlank_SLOT     = 142;

    void ConfigPacketRX(byte * RxPacket);
    void ModifierPacketRX(byte * RxPacket);
    void ControlPacketRX(byte * RxPacket);
    void SendFlagResets();

    // Config Parameters
    bool _SEND_RAWF           = true;
    bool _SEND_XYZGOAL        = false;
    bool _SEND_XYZDOTGOAL     = false;
    bool _SEND_XYZBOTGOAL     = false;
    bool _SEND_XYZDOTBOTGOAL  = false;
    bool _SEND_PRESQCTS       = false;
    bool _SEND_PRESQDOTCTS    = false;
    bool _SEND_PRESQ          = true;
    bool _SEND_PRESQDOT       = true;
    bool _SEND_GOALQCTS       = false;
    bool _SEND_GOALQDOTCTS    = false;
    bool _SEND_GOALQ          = true;
    bool _SEND_GOALQDOT       = true;
    bool _SEND_MASS           = true;
    bool _SEND_DAMPING        = true;
    bool _SEND_SPRING_F       = true;
    bool _SEND_TOTAL_FORCES   = false;
    bool _SEND_FORCE_FILTER   = true;

    // Mod Parameters
    bool _NEW_MASS_XY         = false;
    bool _NEW_MASS_Z          = false;
    bool _NEW_DAMPING_XY      = false;
    bool _NEW_DAMPING_Z       = false;
    bool _NEW_SCALING_FACTOR  = false;
    bool _NEW_EXT_FORCES     = false;
    bool _NEW_FILTER          = false;
    float newMassXY_M,    newMassZ_M;
    float newDampingXY_M, newDampingZ_M;
    float newScalingFactor_M;
    float ExtForces_M[3] = {0.0f};
    float newFilter_M;
    bool dataRequested_M = false;

    // Ctrl Parameters
    bool newTorqueMode_M      = false;
    uint8_t torqueMode_M;
    bool    newGoalQ_M        = false;
    bool    newGoalQdot_M     = false;
    bool    newGoalCurrent_M  = false;
    float   goalQ_M[3]        = {0.0f};
    float   goalQdot_M[3]     = {0.0f};
    float   goalCurrent_M[3]  = {0.0f};
    bool    newGoalXYZ_M      = false;
    bool    newGoalXYZdot_M   = false;
    float   goalXYZ_M[3]      = {0.0f};
    float   goalXYZdot_M[3]   = {0.0f};
};

#endif // PC_COMM_H
