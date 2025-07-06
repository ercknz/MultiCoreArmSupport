/* This establishes the force sensor class.
   Created 02/15/2023
   by erick nunez

   checked code (8/2/2024)
*/

#ifndef FORCE_SENSOR_H
#define FORCE_SENSOR_H

#include <Arduino.h>

class ForceSensor {
  public:
          ForceSensor(const float filterWeight);
    void  CalibrateSensor();
    u_int16_t * GetRawCtsFT();
    float * GetRawFT();
    float * GetFilteredFT();
    float * GetGlobalFT();
    float  GetFilterWeight();
    void   SetFilterWeight(float newFilterValue);
    void   ReadForceSensor();
    void   CalculateGlobalForces(float * q);
    
  protected:
   
    void  FilterFT();

    float       _xyzCALIBRATION[6] = {0.0f}; // [fx, fy, fz, tx, ty, tz]
    int16_t     _SAMPLECOUNTER;
    int16_t     _SENSORSTATUS;
    const uint8_t _FX_PIN = 6;
    const uint8_t _FY_PIN = 5;
    const uint8_t _FZ_PIN = 4;
    const uint8_t _TX_PIN = 2;
    const uint8_t _TY_PIN = 1;
    const uint8_t _TZ_PIN = 0;

    const float _FXY_MAX    = 290;          // Max Force in X and Y (N)
    const float _FZ_MAX     = 580;          // Max Force in Z (N)
    const float _TXYZ_MAX   = 10;           // Max Torque in X, Y, Z (Nm)
    const float _VF_MAX     = 10;           // Max Voltage Force (V)
    const float _AIN_TOL    = 3.3;          // Analog Input Tolerance (V)
    const float _R1         = 2200;         // Resistor 1 Value (Ohms)
    const float _R3         = 330;          // Resistor 3 Value (Ohms)
    const float _AIN_RES    = pow(2, 13);   // Analog Input Resolution (13 bits)
    const float _V2         = 11.66;        // OpAmp V2 Voltage Value (V)

    const float _AFXY_M, _BFXY_M;   //Const Coefficients
    const float _AFZ_M, _BFZ_M;     //Const Coefficients
    const float _ATXYZ_M, _BTXYZ_M; //Const Coefficients

    float FilterWeight_M;

    // Force Torque Array: [Fx, Fy, Fz, Tx, Ty, Tz]
    uint16_t ftxyzRawCts_M[6]     = {0};
    uint16_t ftxyzLastRawCts_M[6] = {0};
    float ftxyzRaw_M[6]     = {0};
    float ftxyzLastRaw_M[6] = {0};
    float ftxyzFilt_M[6]    = {0.0f};
    float ftxyzLastFilt_M[6]= {0.0f};
    float ftxyzGlobal_M[6]  = {0.0f};

};

#endif // FORCE_SENSOR_H