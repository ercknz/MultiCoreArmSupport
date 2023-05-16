/* This establishes the force sensor class.
   Created 02/15/2023
   by erick nunez
*/

#ifndef FORCE_SENSOR_H
#define FORCE_SENSOR_H

#include <Arduino.h>

class ForceSensor {
  public:
          ForceSensor(const uint8_t fxPin, const uint8_t fyPin, const uint8_t fzPin, 
                      const uint8_t txPin, const uint8_t tyPin, const uint8_t tzPin, 
                      const float filterWeight);
    void  CalibrateSensor();
    void  CalculateGlobalForces(float *q);
    int* GetRawFT();
    float* GetGlobalFT();
    float  GetFilterWeight();
    void   SetFilterWeight(float newFilterValue);
    void  ReadForceSensor();
    
  protected:
   
    void  FilterFT();

    const float _READ_DT = 1;  // milliseconds
    float       _xyzCALIBRATION[6] = {0.0f}; // [fx, fy, fz, tx, ty, tz]
    int16_t     _SAMPLECOUNTER;
    int16_t     _SENSORSTATUS;
    //const float _xyzSENSITIVITY[3]; 
    const uint8_t _Fx_PIN;
    const uint8_t _Fy_PIN;
    const uint8_t _Fz_PIN;
    const uint8_t _Tx_PIN;
    const uint8_t _Ty_PIN;
    const uint8_t _Tz_PIN;
    float FilterWeight_M;
    int ftxyzRaw_M[6]     = {0}; // [fx, fy, fz, tx, ty, tz]
    int ftxyzLastRaw_M[6] = {0}; // [fx, fy, fz, tx, ty, tz]
    float ftxyzFilt_M[6]    = {0.0f}; // [fx, fy, fz, tx, ty, tz]
    float ftxyzLastFilt_M[6]= {0.0f}; // [fx, fy, fz, tx, ty, tz]
    float ftxyzGlobal_M[6]  = {0.0f}; // [fx, fy, fz, tx, ty, tz]
    bool calibratingFlag_M = false;
};

#endif // FORCE_SENSOR_H