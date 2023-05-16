/* This establishes the force sensor class.
   The class reads an analog signal and stores the data before being sent off.
   The force sensor is a Mini45 ATI Force/Torque Sensor, SI-290-10.
   It was configured in MATLAB prior to data collection with the following parameters:
   -Sampling frequency: 2500Hz
   -Analog Voltage: +-10V
   -Running average filter: 8 frames

   Class arrays use the following:
   ForceTorque[6] = {Fx, Fy, Fz, Tx, Ty, Tz};
   Exponential filter used:
   y(n) = w*x(n) + (1-w)*y(n-1)
   where: y(n) is the output value
          w is the filter weight between
          x(n) is the input value
          y(n-1) is the last output
   Created 02/15/2023
   by erick nunez
*/

#include <Arduino.h>
#include "ForceSensor.h"

/* ---------------------------------------------------------------------------------------/
/ Force Sensor Constructor ---------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
ForceSensor::ForceSensor(const uint8_t fxPin, const uint8_t fyPin, const uint8_t fzPin, 
                         const uint8_t txPin, const uint8_t tyPin, const uint8_t tzPin, 
                         const float filterWeight)
  : _Fx_PIN{fxPin},
    _Fy_PIN{fyPin},
    _Fz_PIN{fzPin},
    _Tx_PIN{txPin},
    _Ty_PIN{tyPin},
    _Tz_PIN{tzPin}
{
  FilterWeight_M = filterWeight;
  analogReference(EXTERNAL);
}

/* ---------------------------------------------------------------------------------------/
/ Force Sensor Memeber Getter and Setter Functions ---------------------------------------/
/----------------------------------------------------------------------------------------*/
int* ForceSensor::GetRawFT() {
  return ftxyzRaw_M;
}

float* ForceSensor::GetGlobalFT() {
  return ftxyzGlobal_M;
} 

float ForceSensor::GetFilterWeight(){
  return FilterWeight_M;
}

void ForceSensor::SetFilterWeight(float newFilterValue){
  if (newFilterValue > 0.99) {
    FilterWeight_M = 0.99;
    return;
  }
  if (newFilterValue < 0.1) {
    FilterWeight_M = 0.1;
    return;
  }
  FilterWeight_M = newFilterValue;
}

/* ---------------------------------------------------------------------------------------/
/ Force Sensor Calibration ---------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
void ForceSensor::CalibrateSensor() {
  calibratingFlag_M = true;
  float newXYZcal[3] = {0.0};
  for (int i = 0; i < 3; i++) {
    _xyzCALIBRATION[i] = 0.0f;
  }
  float samples = 2000.0;
  for (int i = 0; i < samples; i++) {
    ReadForceSensor();
    for (int j = 0; j < 3; j++) {
      newXYZcal[j] += ftxyzRaw_M[j] / samples;
    }
  }
  for (int i = 0; i < 3; i++) {
    _xyzCALIBRATION[i] = newXYZcal[i];
  }
  calibratingFlag_M = false;
}

/* ---------------------------------------------------------------------------------------/
/ Force Sensor Reading -------------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
void ForceSensor::ReadForceSensor() {
  // Saves last frames
  for (int i = 0; i < 6; i++) {
    ftxyzLastRaw_M[i] = ftxyzRaw_M[i];
  }
  
  // Short Delay
  unsigned long readTime = millis();
  //(millis() - readTime < _READ_DT){}

  // Reads Analog Signals
  ftxyzRaw_M[0] = analogRead(_Fx_PIN);
  ftxyzRaw_M[1] = analogRead(_Fy_PIN);
  ftxyzRaw_M[2] = analogRead(_Fz_PIN);
  ftxyzRaw_M[3] = analogRead(_Tx_PIN);
  ftxyzRaw_M[4] = analogRead(_Ty_PIN);
  ftxyzRaw_M[5] = analogRead(_Tz_PIN);

  //if (!calibratingFlag_M) FilterFT();
}

/* ---------------------------------------------------------------------------------------/
/ Force Sensor Filter --------------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
void ForceSensor::FilterFT() {
  for (int i = 0; i < 6; i++) {
    // Filter Forces
    ftxyzLastFilt_M[i] = ftxyzFilt_M[i];
    ftxyzFilt_M[i] = FilterWeight_M * (ftxyzRaw_M[i] - _xyzCALIBRATION[i]) + (1.0 - FilterWeight_M) * ftxyzLastFilt_M[i];
  }
}

/* ---------------------------------------------------------------------------------------/
/ Force Sensor Global Forces -------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
void ForceSensor::CalculateGlobalForces(float *q) {
  // q[] = [q1, q2, q4]
  ReadForceSensor();
  ftxyzGlobal_M[0] = ftxyzFilt_M[0] * ( sin(q[0] + q[2])) + ftxyzFilt_M[1] * (-cos(q[0] + q[2]));
  ftxyzGlobal_M[1] = ftxyzFilt_M[0] * (-cos(q[0] + q[2])) + ftxyzFilt_M[1] * (-sin(q[0] + q[2]));
  ftxyzGlobal_M[2] = -ftxyzFilt_M[2];
}