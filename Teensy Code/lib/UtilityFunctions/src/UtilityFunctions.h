/* Utility functions used with the arm support robot.

   Created 10/28/2020
   by Erick Nunez
*/

/* Macro Control Table Values from DynamixelSDK/packet_handler.h */
#define DXL_LOWORD(l)       ((uint16_t)(((uint64_t)(l)) & 0xffff))
#define DXL_HIWORD(l)       ((uint16_t)((((uint64_t)(l)) >> 16) & 0xffff))
#define DXL_LOBYTE(w)       ((uint8_t)(((uint64_t)(w)) & 0xff))
#define DXL_HIBYTE(w)       ((uint8_t)((((uint64_t)(w)) >> 8) & 0xff))

#ifndef UTILITY_FUNCTIONS_H
#define UTILITY_FUNCTIONS_H

int16_t bytesToCounts(byte hByte, byte lByte);

float bytesToFloat(byte byte1, byte byte2, byte byte3, byte byte4);

byte * floatArrayToBytes(float * floatValues);

byte * floatToBytes(float floatValue);

byte * int32ArrayToBytes(int32_t * int32Values);

#endif // UTILITY_FUNCTIONS_H