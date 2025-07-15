/* This establishes the packets for communication to and from the robotic arm support via the serial port.

   Created 3/1/2021
   by erick nunez
*/

#ifndef SERIAL_PACKETS_H
#define SERIAL_PACKETS_H

#include <Arduino.h>
#include "ForceSensor.h"

class SerialPackets {
  public:
         SerialPackets(HardwareSerial *ptrSer, const int baudrate);

    bool DataAvailable();
    bool DataRequested();
    void InitalizingComm();
    bool InTestingMode();

    void WritePackets(unsigned long &totalTime, ForceSensor &ati, unsigned long &lTime);
    void ReadPackets();

  protected:
    const int   _BAUDRATE;
    HardwareSerial *c2cPort_M;
    const int16_t _TX_PKT_LEN = 150;
    const int16_t _RX_PKT_LEN = 10;
    const byte  _RXHEADER[4]  = {150, 10, 1, 101};  //Header for all incoming packets
    const byte  _TXHEADER[4]  = {170, 6, 9, 69};    //Header for outgoing packets
    const byte  _ZEROHEADER[4] = {0};

    bool testingMode_M = true;
    bool dataRequested_M  = false;

};

#endif // SERIAL_PACKETS_H
