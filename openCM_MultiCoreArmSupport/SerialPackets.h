/* This establishes the packets for communication to and from the robotic arm support via the serial port.

   Created 3/1/2021
   by erick nunez
*/

#ifndef SERIAL_PACKETS_H
#define SERIAL_PACKETS_H

#include <Arduino.h>
#include "RobotControl.h"

class SerialPackets {
  public:
         SerialPackets(HardwareSerial *ptrSer, const int baudrate);

    bool DataAvailable();
    bool NewGoalAvailable();
    float * GetNewXYZGoal();
    float * GetNewXYZdotGoal();
    float * GetNewCurrentGoal();
    void NewGoalApplied();
    bool TorqueChanged();
    uint8_t ChangeModeTo();
    void TorqueChangeApplied();
    bool DataRequested();
    void InitalizingComm();
    bool InTestingMode();
    
    void WritePackets(unsigned long &totalTime, RobotControl &Robot, unsigned long &lTime);
    void ReadPackets();

  protected:
    const int   _BAUDRATE;
    HardwareSerial *c2cPort_M;
    const int16_t _TX_PKT_LEN = 150;
    const int16_t _RX_PKT_LEN = 60;
    const byte  _RXHEADER[4]  = {150, 10, 1, 101};  //Header for all incoming packets
    const byte  _TXHEADER[4]  = {170, 6, 9, 69};    //Header for outgoing packets
    const byte  _ZEROHEADER[4] = {0};
    const int16_t _TX_presQ_SLOT        = 8;
    const int16_t _TX_presQDOT_SLOT     = 20;
    const int16_t _TX_presXYZ_SLOT      = 32;
    const int16_t _TX_presXYZdot_SLOT   = 44;
    const int16_t _TX_goalQ_SLOT        = 56;
    const int16_t _TX_goalQDOT_SLOT     = 68;
    const int16_t _TX_presQcts_SLOT     = 80;
    const int16_t _TX_presQDOTcts_SLOT  = 92;
    const int16_t _TX_goalQcts_SLOT     = 104;
    const int16_t _TX_goalQDOTcts_SLOT  = 116;
    const int16_t _TX_presCURRENT_SLOT  = 128;
    const int16_t _TX_BLANK_SLOT        = 140;

    bool testingMode_M = true;

    bool newGoal_M        = false;
    bool torqueChange_M   = false;
    bool dataRequested_M  = false;
    uint8_t torqueMode_M;

    float goalXYZ_M[3] = {0.0f};
    float goalXYZdot_M[3] = {0.0f};
    float goalCurrent_M[3] = {0.0f};
};

#endif // SERIAL_PACKETS_H
