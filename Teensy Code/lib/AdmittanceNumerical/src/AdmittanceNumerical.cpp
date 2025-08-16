/* This class is the admittance control model.
   It takes a XYZ force input and output XYZ position and velocity based on initial conditions.
   This uses a Numerical Backward Euler integration method to determine the new postions. 

   Class arrays use the following:
   xyz[3]     = {x, y, z};
   xyzDot[3]  = {xDot, yDot, zDot};

   X Direction *****************************************************************
   2nd order eqn:       M*x" + B*x' = Fx
   Approximations:      x" = (x - 2*x_1 + x_2) / dt^2
                        x' = (x - x_1) / dt
   Plug-in:             (M/dt^2 + B/dt)*x - (2*M/dt^2 + B/dt)*x_1  + (M/dt^2)*x_2 = Fx
   Coefficents:         a0 = M/dt^2 + B/dt
                        a1 = 2*M/dt^2 + B/dt
                        a2 = M/dt^2
   Solution:            x = (a1/a0)*x_1 - (a2/a0)*x_2 + (1/a0)*Fx

   Y Direction *****************************************************************
   2nd order eqn:       M*y" + B*y' = Fy
   Approximations:      y" = (y - 2*y_1 + y_2) / dt^2
                        y' = (y - y_1) / dt
   Plug-in:             (M/dt^2 + B/dt)*y - (2*M/dt^2 + B/dt)*y_1  + (M/dt^2)*y_2 = Fy
   Coefficents:         a0 = M/dt^2 + B/dt
                        a1 = 2*M/dt^2 + B/dt
                        a2 = M/dt^2
   Solution:            y = (a1/a0)*y_1 - (a2/a0)*y_2 + (1/a0)*Fy
   Z Direction *****************************************************************
   2nd order eqn:       M*z" + B*z' = Fz
   Approximations:      z" = (z - 2*z_1 + z_2) / dt^2
                        z' = (z - z_1) / dt
   Plug-in:             (M/dt^2 + B/dt)*z - (2*M/dt^2 + B/dt)*z_1  + (M/dt^2)*z_2 = Fz
   Coefficents:         a0 = M/dt^2 + B/dt
                        a1 = 2*M/dt^2 + B/dt
                        a2 = M/dt^2
   Solution:            z = (a1/a0)*z_1 - (a2/a0)*z_2 + (1/a0)*Fz

   Created 10/27/2020
   by erick nunez
*/

#include <Arduino.h>
#include "AdmittanceNumerical.h"
#include "ArmSupportNamespace.h"

/* ---------------------------------------------------------------------------------------/
/ Admittance Model Constructor -----------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
AdmittanceNumerical::AdmittanceNumerical(float Mxy, float Mz, float Bxy, float Bz)
  : _GRAVITY{ASR::GRAVITY},
    _DELTA_T{ASR::MODEL_DT},
    _ELEVATION_CENTER{(ASR::ELEVATION_MAX_POS + ASR::ELEVATION_MIN_POS) / 2},
    _MODEL_Z_LIMIT{abs(ASR::L1_LINK * sinf((ASR::ELEVATION_MAX_POS - _ELEVATION_CENTER) * ASR::DEGREES_PER_COUNT * (PI / 180.0) * (1/ASR::ELEVATION_RATIO)))},
    _H_OF_L2{sqrtf((pow(ASR::LINK_OFFSET, 2) + pow(ASR::L2_LINK, 2)))},
    _A1A2{ASR::A1_LINK + ASR::A2_LINK},
    _INNER_R_LIMIT{ASR::A1_LINK + ASR::L1_LINK + ASR::A2_LINK - ASR::L2_LINK},
    _VEL_LIMIT{ASR::VEL_XYZ_LIMIT},
    _MAX_STEP{ASR::VEL_XYZ_LIMIT * ASR::MODEL_DT}
{
  mass_M[0] = Mxy;
  mass_M[1] = Mxy;
  mass_M[2] = Mz;
  damping_M[0] = Bxy;
  damping_M[1] = Bxy;
  damping_M[2] = Bz;
  CalculateParameters();
}

/* ---------------------------------------------------------------------------------------/
/ Admittance Model Initalizer ------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
void AdmittanceNumerical::SetPosition(float *newXYZ) {
  /* Check TaskSpace Limits */
  float clampedZ = newXYZ[2];
  if (clampedZ >  _MODEL_Z_LIMIT) clampedZ =  _MODEL_Z_LIMIT;
  if (clampedZ < -_MODEL_Z_LIMIT) clampedZ = -_MODEL_Z_LIMIT;
  float L1_XY = sqrt(pow(ASR::L1_LINK, 2) - pow(xyzGoal_M[2] - ASR::A3_LINK, 2));
  float outerRLimit = _A1A2 + _H_OF_L2 + L1_XY;
  xyzGoal_M[2] = clampedZ;
  float Rxy = sqrt(pow(newXYZ[0],2) + pow(newXYZ[1],2));
  float alpha   = atan2(newXYZ[1], newXYZ[0]);
  if (alpha < 0.0f) alpha += 2 * PI;
  if (Rxy < _INNER_R_LIMIT) {
    Rxy           = _INNER_R_LIMIT;
    xyzGoal_M[0]  = _INNER_R_LIMIT * cos(alpha);
    xyzGoal_M[1]  = _INNER_R_LIMIT * sin(alpha);
    return;
  }
  if (Rxy > outerRLimit) {
    Rxy           = outerRLimit;
    xyzGoal_M[0]  = outerRLimit * cos(alpha);
    xyzGoal_M[1]  = outerRLimit * sin(alpha);
    return;
  } 
  xyzGoal_M[0] = newXYZ[0];
  xyzGoal_M[1] = newXYZ[1];
  if (xyzGoal_M[0] < _MODEL_X_LIMIT) xyzGoal_M[0] = _MODEL_X_LIMIT;
  if (xyzGoal_M[1] > _MODEL_Y_LIMIT) xyzGoal_M[1] = _MODEL_Y_LIMIT;
  for (int i = 0; i < 3; i++) {
    xyz_1_M[i]      = xyzGoal_M[i];
    xyz_2_M[i]      = xyzGoal_M[i];
    xyzDot_1_M[i]   = 0.0f;
    xyzDotGoal_M[i] = 0.0f;
    totalForces_M[i]= 0.0f;
  }
}

/* ---------------------------------------------------------------------------------------/
/ Admittance Model Updater ---------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
void AdmittanceNumerical::UpdateModel(float *forceXYZ, float *externalFxyz) {
  for (int i = 0; i < 3; i++) {
    xyz_2_M[i]    = xyz_1_M[i];
    xyz_1_M[i]    = xyzGoal_M[i];
    xyzDot_1_M[i] = xyzDotGoal_M[i];
  }

  /* Solution for XYZ-Directions */
  for (int i = 0; i < 3; i++){
    totalForces_M[i] = forceXYZ[i] + externalFxyz[i];
    xyzGoal_M[i]    = (a1_M[i]/a0_M[i])*xyz_1_M[i] - (a2_M[i]/a0_M[i])*xyz_2_M[i] + (1/a0_M[i])*totalForces_M[i];
    xyzDotGoal_M[i] = (xyzGoal_M[i] - xyz_1_M[i]) / _DELTA_T;
  }
  
  // Clamp the XYZ Goal Position and Velocity
  for (int i = 0; i < 3; i++) {
    xyzDotGoal_M[i] = constrain(xyzDotGoal_M[i], -_VEL_LIMIT, _VEL_LIMIT);
    if(fabs(xyzGoal_M[i] - xyz_1_M[i]) > _MAX_STEP) {
      xyzGoal_M[i] = xyz_1_M[i] + copysign(_MAX_STEP, xyzGoal_M[i] - xyz_1_M[i]);
    }
  }

  /* Check TaskSpace Limits */
  if (xyzGoal_M[2] >  _MODEL_Z_LIMIT) {
    xyzGoal_M[2] =  _MODEL_Z_LIMIT;
    xyzDotGoal_M[2] = 0.0f;
  }
  if (xyzGoal_M[2] < -_MODEL_Z_LIMIT) {
    xyzGoal_M[2] = -_MODEL_Z_LIMIT;
    xyzDotGoal_M[2] = 0.0f;
  }
  float L1_XY = sqrt(pow(ASR::L1_LINK, 2) - pow((xyzGoal_M[2] - ASR::A3_LINK), 2));
  float outerRLimit = _A1A2 + _H_OF_L2 + L1_XY;
  float Rxy = sqrt(pow(xyzGoal_M[0],2) + pow(xyzGoal_M[1],2));
  float alpha   = atan2(xyzGoal_M[1], xyzGoal_M[0]);
  if (alpha < 0.0f) alpha += 2 * PI;
  if (Rxy < _INNER_R_LIMIT) {
    xyzGoal_M[0]  = _INNER_R_LIMIT * cos(alpha);
    xyzGoal_M[1]  = _INNER_R_LIMIT * sin(alpha);
  }
  if (Rxy > outerRLimit) {
    xyzGoal_M[0]  = outerRLimit * cos(alpha);
    xyzGoal_M[1]  = outerRLimit * sin(alpha);
  } 
  if (xyzGoal_M[0] < _MODEL_X_LIMIT) {
    xyzGoal_M[0] = _MODEL_X_LIMIT;
    xyzDotGoal_M[0] = 0.0f;
  }
  if (xyzGoal_M[1] > _MODEL_Y_LIMIT) {
    xyzGoal_M[1] = _MODEL_Y_LIMIT;
    xyzDotGoal_M[1] = 0.0f;
  }
}

/* ---------------------------------------------------------------------------------------/
/ Admittance Model Get Functions ---------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
float* AdmittanceNumerical::GetGoalPos() {
  return xyzGoal_M;
}

float* AdmittanceNumerical::GetGoalVel() {
  return xyzDotGoal_M;
}

float*  AdmittanceNumerical::GetMass(){
  return mass_M;
}

float*  AdmittanceNumerical::GetDamping(){
  return damping_M;
}

float* AdmittanceNumerical::GetTotalForces(){
  return totalForces_M;
}

/* ---------------------------------------------------------------------------------------/
/ Admittance Model Setter Functions ------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
void AdmittanceNumerical::SetMassXY(float newMxy){
  if (newMxy > 0.1){
    mass_M[0] = newMxy;
    mass_M[1] = newMxy;
  } else {
    mass_M[0] = 0.1;
    mass_M[1] = 0.1;
  }
  CalculateParameters();
}

void AdmittanceNumerical::SetMassZ(float newMz){
  if (newMz > 0.1){
    mass_M[2] = newMz;
  } else {
    mass_M[2] = 0.1;
  }
  CalculateParameters();
}

void AdmittanceNumerical::SetDampingXY(float newBxy){
  if (newBxy > 0.1){
    damping_M[0] = newBxy;
    damping_M[1] = newBxy;
  } else {
    damping_M[0] = 0.1;
    damping_M[1] = 0.1;
  }
  CalculateParameters();
}

void AdmittanceNumerical::SetDampingZ(float newBz){
  if (newBz > 0.1){
    damping_M[2] = newBz;
  } else {
    damping_M[2] = 0.1;
  }
  CalculateParameters();
}

void AdmittanceNumerical::CalculateParameters(){
  for (int i = 0; i < 3; i++) {
    a0_M[i] = mass_M[i] / (_DELTA_T * _DELTA_T) + damping_M[i] / _DELTA_T;
    a1_M[i] = 2 * mass_M[i] / (_DELTA_T * _DELTA_T) + damping_M[i] / _DELTA_T;
    a2_M[i] = mass_M[i] / (_DELTA_T * _DELTA_T);
  }
}
