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
    int* GetRawCtsFT();
    float* GetRawFT();
    float* GetFilteredFT();
    float* GetGlobalFT();
    float  GetFilterWeight();
    void   SetFilterWeight(float newFilterValue);
    void  ReadForceSensor();
    
  protected:
   
    void  FilterFT();

    float       _xyzCALIBRATION[6] = {0.0f}; // [fx, fy, fz, tx, ty, tz]
    int16_t     _SAMPLECOUNTER;
    int16_t     _SENSORSTATUS;
    const uint8_t _Fx_PIN;
    const uint8_t _Fy_PIN;
    const uint8_t _Fz_PIN;
    const uint8_t _Tx_PIN;
    const uint8_t _Ty_PIN;
    const uint8_t _Tz_PIN;

    float AFxy_M, BFxy_M; //const
    float AFz_M, BFz_M; // const
    float ATxyz_M, BTxyz_M; //const 

    float FilterWeight_M;

    int ftxyzRawCts_M[6]     = {0}; // [fxcts, fycts, fzcts, txcts, tycts, tzcts]
    int ftxyzLastRawCts_M[6] = {0}; // [fxcts, fycts, fzcts, txcts, tycts, tzcts]
    float ftxyzRaw_M[6]     = {0}; // [fx, fy, fz, tx, ty, tz]
    float ftxyzLastRaw_M[6] = {0}; // [fx, fy, fz, tx, ty, tz]
    float ftxyzFilt_M[6]    = {0.0f}; // [fx, fy, fz, tx, ty, tz]
    float ftxyzLastFilt_M[6]= {0.0f}; // [fx, fy, fz, tx, ty, tz]
    float ftxyzGlobal_M[6]  = {0.0f}; // [fx, fy, fz, tx, ty, tz]

};

#endif // FORCE_SENSOR_H