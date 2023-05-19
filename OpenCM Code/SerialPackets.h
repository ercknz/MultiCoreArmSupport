/* This establishes the packets for communication to and from the robotic arm support via the serial port.

   Created 3/1/2021
   by erick nunez
*/

#ifndef SERIAL_PACKETS_H
#define SERIAL_PACKETS_H

#include <Arduino.h>
#include "ForceSensor.h"
#include "AdmittanceModel.h"
#include "RobotControl.h"

class SerialPackets {
  public:
         SerialPackets(USBSerial *ptrSer, const int baudrate);
    bool DataAvailable();
    void ReadPackets();
    void WritePackets(unsigned long &totalTime, RobotControl &Robot, unsigned long &loopTime);
    bool ModifyMode();
    uint8_t GetNewMode();

  protected:
    const int   _BAUDRATE;
    USBSerial * SerialPort_M;
    const int16_t _TX_PKT_LEN = 146;
    const int16_t _RX_PKT_LEN = 39;
    const int16_t _MAX_TX_DATA_SLOTS = 24;
    const byte  _CONFIGHEADER[4]  = {150, 0, 69, 8};
    const byte  _MODHEADER[4]     = {150, 10, 10, 96};
    const byte  _WRITEHEADER[4]   = {170, 8, 69, 0};

    void ConfigPacketRX(byte * RxPacket);
    void ModifierPacketRX(byte * RxPacket);
    void SendFlagResets();
    void ModeSelection(byte modeNumber);

    bool _NEW_MODE            = false;
    uint8_t newMode_M;
};

#endif // SERIAL_PACKETS_H
