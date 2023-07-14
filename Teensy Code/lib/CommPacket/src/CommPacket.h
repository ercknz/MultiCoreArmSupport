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
         SerialPackets(HardwareSerial  *ptrSer, const int baudrate);

    bool DataAvailable();
    bool NewGoalAvailable();
    uint8_t GetNewMode();
    float * GetNewGoalQ();
    float * GetNewGoalQdot();
    float * GetNewGoalCurrent();
    void NewGoalsPulled();
    
    void WritePackets(unsigned long &totalTime, RobotControl &Robot, unsigned long &loopTime);
    void ReadPackets();

  protected:
    const int   _BAUDRATE;
    HardwareSerial *c2cPort_M;
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

#endif // SERIAL_PACKETS_H
