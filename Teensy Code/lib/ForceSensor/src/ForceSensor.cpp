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

   checked code (8/2/2024)
*/

#include <Arduino.h>
#include "ForceSensor.h"

/* ---------------------------------------------------------------------------------------/
/ Force Sensor Constructor ---------------------------------------------------------------/
/ / / / Sets up coefficients to convert from voltage to force for each axis.
/ / / / Only input weight for filter.
/----------------------------------------------------------------------------------------*/
ForceSensor::ForceSensor(const float filterWeight)
    : _AFXY_M{(_V2 * _FXY_MAX)/ _VF_MAX},
      _BFXY_M{(_AIN_TOL * _R1 * _FXY_MAX) / (_AIN_RES * _R3 * _VF_MAX)},
      _AFZ_M{(_V2 * _FZ_MAX) / _VF_MAX},
      _BFZ_M{(_AIN_TOL * _R1 * _FZ_MAX) / (_AIN_RES * _R3 * _VF_MAX)},
      _ATXYZ_M{(_V2 * _TXYZ_MAX) / _VF_MAX},
      _BTXYZ_M{(_AIN_TOL * _R1 * _TXYZ_MAX) / (_AIN_RES * _R3 * _VF_MAX)}
{
  SetFilterWeight(filterWeight);
  analogReference(EXTERNAL);
}

/* ---------------------------------------------------------------------------------------/
/ Force Sensor Memeber Getter and Setter Functions ---------------------------------------/
/----------------------------------------------------------------------------------------*/
uint16_t * ForceSensor::GetRawCtsFT(){
  return ftxyzRawCts_M;
}

float * ForceSensor::GetRawFT(){
  return ftxyzRaw_M;
}

float * ForceSensor::GetFilteredFT(){
  return ftxyzFilt_M;
}

float * ForceSensor::GetGlobalForces(){
  return FxyzGlobal_M;
}

float * ForceSensor::GetGlobalTorques(){
  return TxyzGlobal_M;
}

float ForceSensor::GetFilterWeight(){
  return FilterWeight_M;
}

void ForceSensor::SetFilterWeight(float newFilterValue){
  if (newFilterValue > 0.99){
    FilterWeight_M = 0.99;
    return;
  }
  if (newFilterValue < 0.1){
    FilterWeight_M = 0.1;
    return;
  }
  FilterWeight_M = newFilterValue;
}

/* ---------------------------------------------------------------------------------------/
/ Force Sensor Calibration ---------------------------------------------------------------/
/ / / / Runs for 20 seconds collecting samples to average every 10 milliseconds.
/----------------------------------------------------------------------------------------*/
void ForceSensor::CalibrateSensor(){
  float newXYZcal[6] = {0.0};
  for (int i = 0; i < 6; i++){
    _xyzCALIBRATION[i] = 0.0f;
  }

  int samples = 0;
  int samples4Cal = 2000;
  unsigned long previousTime, currentTime;
  previousTime = millis();
  const float loopDT = 10;

  while (samples < samples4Cal){
    currentTime = millis();
    if (currentTime - previousTime >= loopDT){
      previousTime = currentTime;
      ReadForceSensor();
      for (int i = 0; i < 6; i++){
          newXYZcal[i] += ftxyzRaw_M[i] / samples4Cal;
      }
      samples += 1;
    }
  }
  for (int i = 0; i < 6; i++){
    _xyzCALIBRATION[i] = newXYZcal[i];
  }
}

  /* ---------------------------------------------------------------------------------------/
  / Force Sensor Reading -------------------------------------------------------------------/
  /----------------------------------------------------------------------------------------*/
  void ForceSensor::ReadForceSensor(){
    // Move data from last frame
    for (int i = 0; i < 6; i++){
      ftxyzLastRawCts_M[i] = ftxyzRawCts_M[i];
      ftxyzLastRaw_M[i] = ftxyzRaw_M[i];
    }

    // Reads Analog Signals
    ftxyzRawCts_M[0] = analogRead(_FX_PIN);
    ftxyzRawCts_M[1] = analogRead(_FY_PIN);
    ftxyzRawCts_M[2] = analogRead(_FZ_PIN);
    ftxyzRawCts_M[3] = analogRead(_TX_PIN);
    ftxyzRawCts_M[4] = analogRead(_TY_PIN);
    ftxyzRawCts_M[5] = analogRead(_TZ_PIN);

    // Convert to Force/Torque Values
    ftxyzRaw_M[0] = _AFXY_M - _BFXY_M * ftxyzRawCts_M[0];
    ftxyzRaw_M[1] = _AFXY_M - _BFXY_M * ftxyzRawCts_M[1];
    ftxyzRaw_M[2] = _AFZ_M  - _BFZ_M  * ftxyzRawCts_M[2];
    ftxyzRaw_M[3] = _ATXYZ_M - _BTXYZ_M * ftxyzRawCts_M[3];
    ftxyzRaw_M[4] = _ATXYZ_M - _BTXYZ_M * ftxyzRawCts_M[4];
    ftxyzRaw_M[5] = _ATXYZ_M - _BTXYZ_M * ftxyzRawCts_M[5];

    // Filter Values
    FilterFT();
  }

  /* ---------------------------------------------------------------------------------------/
  / Force Sensor Filter --------------------------------------------------------------------/
  /----------------------------------------------------------------------------------------*/
  void ForceSensor::FilterFT(){
    for (int i = 0; i < 6; i++){
      // Move data from last frame
      ftxyzLastFilt_M[i] = ftxyzFilt_M[i];
      // Filter current frame
      ftxyzFilt_M[i] = FilterWeight_M * (ftxyzRaw_M[i] - _xyzCALIBRATION[i]) + (1.0 - FilterWeight_M) * ftxyzLastFilt_M[i];
    }
  }

  /* ---------------------------------------------------------------------------------------/
/ Force Sensor Global Forces -------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
void ForceSensor::CalculateGlobalForces(float *q) {
  // q[] = [q1, q2, q4] corresponding to the angles of the arm joints.
  // q[0] = q1, q[1] = q2, q[2] = q4
  ReadForceSensor();
  FxyzGlobal_M[0] = ftxyzFilt_M[0] * ( sin(q[0] + q[2])) + ftxyzFilt_M[1] * (-cos(q[0] + q[2]));
  FxyzGlobal_M[1] = ftxyzFilt_M[0] * (-cos(q[0] + q[2])) + ftxyzFilt_M[1] * (-sin(q[0] + q[2]));
  FxyzGlobal_M[2] = -ftxyzFilt_M[2];
}