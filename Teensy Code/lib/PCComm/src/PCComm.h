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
    bool ModifyMode();
    bool ModifyFilter();
    float GetNewMassXY();
    float GetNewMassZ();
    float GetNewDampingXY();
    float GetNewDampingZ();
    float GetNewScalingFactor();
    uint8_t GetNewMode();
    float * GetExternalForces();
    float GetNewFilter();

  protected:
    const int   _BAUDRATE;
    usb_serial_class *pcPort_M;
    const byte  _CONFIGHEADER[4]  = {150, 0, 69, 8};
    const byte  _MODHEADER[4]     = {150, 10, 10, 96};
    const int16_t _RX_PKT_LEN = 39;
    const byte  _WRITEHEADER[4]   = {170, 8, 69, 0};
    const int16_t _TX_PKT_LEN = 146;
    const int16_t _MAX_TX_DATA_SLOTS = 24;

    void ConfigPacketRX(byte * RxPacket);
    void ModifierPacketRX(byte * RxPacket);
    void SendFlagResets();
    void ModeSelection(byte modeNumber);

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

    bool _NEW_MASS_XY         = false;
    bool _NEW_MASS_Z          = false;
    bool _NEW_DAMPING_XY      = false;
    bool _NEW_DAMPING_Z       = false;
    bool _NEW_SCALING_FACTOR  = false;
    bool _NEW_MODE            = false;
    bool _NEW_EXT_FORCE_X     = false;
    bool _NEW_EXT_FORCE_Y     = false;
    bool _NEW_EXT_FORCE_Z     = false;
    bool _NEW_FILTER          = false;
    float newMassXY_M,    newMassZ_M;
    float newDampingXY_M, newDampingZ_M;
    float newScalingFactor_M;
    uint8_t newMode_M;
    float ExtForces_M[3] = {0.0f};
    float newFilter_M;

    const int16_t _TX_PKT_LEN = 60;
    const int16_t _RX_PKT_LEN = 60;
    const byte  _RXHEADER[4]  = {150, 10, 1, 101};
    const byte  _TXHEADER[4]  = {170, 6, 9, 69};
    const int16_t _Q_SLOT = 8;
    const int16_t _QDOT_SLOT = 20;
    const int16_t _CURRENT_SLOT = 32;
    const int16_t _NEW_SLOT1 = 44;

    bool newGoal_M  = false;

    uint8_t mode_M;
    float goalQ_M[3] = {0.0f};
    float goalQdot_M[3] = {0.0f};
    float goalCurrent_M[3] = {0.0f};
};

#endif // PC_COMM_H
