/* This class is the admittance control model.
   It takes a XYZ force input and output XYZ position and velocity based on initial conditions.

   Class arrays use the following:
   xyz[3]     = {x, y, z};
   xyzDot[3]  = {xDot, yDot, zDot};

   X Direction *****************************************************************
   2nd order eqn:       M*x" = Fx - B*x'
   general solution:    xg(t) = Cx1*exp(-(B/M)*t) + Cx2
   particular solution: xp(t) = (Fx/B)*t
   coefficents:         Cx1 = ((Fx/B) - xPresentVelocity)*(M/B)
                        Cx2 = xPresentPosition - Cx1
   solutions:           x(t) = Cx1*exp(-(B/M)*t) + (Fx/B)*t + Cx2
                        x'(t) = -(B/M)*Cx1*exp(-(B/M)*t) + (Fx/B)
   Y Direction *****************************************************************
   2nd order eqn:       M*y" = Fy - B*y'
   general solution:    yg(t) = Cy1*exp(-(B/M)*t) + Cy2
   particular solution: yp(t) = (Fy/B)*t
   coefficents:         Cy1 = ((Fy/B) - yPresentVelocity)*(M/B)
                        Cy2 = yPresentPosition - Cy1
   solutions:           y(t) = Cy1*exp(-(B/M)*t) + (Fy/B)*t + Cy2
                        y'(t) = -(B/M)*Cy1*exp(-(B/M)*t) + (Fy/B)
   Z Direction *****************************************************************
   2nd order eqn:       M*z" = Fz - B*z' - Mg
   general solution:    zg(t) = Cz1*exp(-(B/M)*t) + Cz2
   particular solution: zp(t) = (1/B)*(Fz-g*M)*t
   coefficents:         Cz1 = ((1/B)*(Fz-g*M) - zPresentVelocity)*(M/B)
                        Cz2 = zPresentPosition - Cz1
   solutions:           z(t) = Cz1*exp(-(B/M)*t) + (1/B)*(Fz-g*M)*t + Cz2
                        z'(t) = -(B/M)*Cz1*exp(-(B/M)*t) + (1/B)*(Fz-g*M)

   Created 10/27/2020
   by erick nunez
*/

#include <Arduino.h>
#include "AdmittanceModel.h"
#include "ArmSupportNamespace.h"

/* ---------------------------------------------------------------------------------------/
/ Admittance Model Constructor -----------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
AdmittanceModel::AdmittanceModel(float Mxy, float Mz, float Bxy, float Bz)
  : _GRAVITY{ASR::GRAVITY},
    _DELTA_T{ASR::MODEL_DT},
    _ELEVATION_CENTER{(ASR::ELEVATION_MAX_POS + ASR::ELEVATION_MIN_POS) / 2},
    _Z_LIMIT{abs(static_cast<float>(ASR::L1_LINK * sin((ASR::ELEVATION_MAX_POS - _ELEVATION_CENTER) * ASR::DEGREES_PER_COUNT * (PI / 180.0) * (1/ASR::ELEVATION_RATIO))))},
    _H_OF_L2{static_cast<float>((pow(ASR::LINK_OFFSET, 2) + pow(ASR::L2_LINK, 2)))},
    _A1A2{ASR::A1_LINK + ASR::A2_LINK},
    _INNER_R_LIMIT{ASR::A1_LINK + ASR::L1_LINK + ASR::A2_LINK - ASR::L2_LINK}
{
  mass_M[0] = Mxy;
  mass_M[1] = Mxy;
  mass_M[2] = Mz;
  damping_M[0] = Bxy;
  damping_M[1] = Bxy;
  damping_M[2] = Bz;
}

/* ---------------------------------------------------------------------------------------/
/ Admittance Model Initalizer ------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
void AdmittanceModel::SetPosition(float *newXYZ) {
  /* Check TaskSpace Limits */
  float clampedZ = newXYZ[2];
  if (clampedZ >  _Z_LIMIT) clampedZ =  _Z_LIMIT;
  if (clampedZ < -_Z_LIMIT) clampedZ = -_Z_LIMIT;
  float outerRLimit = _A1A2 + _H_OF_L2 + sqrt(pow(ASR::L1_LINK, 2) - pow(clampedZ, 2));
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
}

/* ---------------------------------------------------------------------------------------/
/ Admittance Model Updater ---------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
void AdmittanceModel::UpdateModel(float *forceXYZ, float *externalFxyz) {
  for (int i = 0; i < 3; i++) {
    // xyzInit_M[i] = 0.0f;
    xyzInit_M[i]    = xyzGoal_M[i];
    xyzDotInit_M[i] = xyzDotGoal_M[i];
  }

  /* Coefficents and Solution for X-Direction */
  totalForces_M[0] = forceXYZ[0] + externalFxyz[0];
  float Cx1 = ((totalForces_M[0] / damping_M[0]) - xyzDotInit_M[0]) * (mass_M[0] / damping_M[0]);
  float Cx2 = xyzInit_M[0] - Cx1;
  xyzGoal_M[0]    = Cx1 * exp(-(damping_M[0] / mass_M[0]) * _DELTA_T) + (totalForces_M[0] / damping_M[0]) * _DELTA_T + Cx2;
  xyzDotGoal_M[0] = (totalForces_M[0] / damping_M[0]) - (damping_M[0] / mass_M[0]) * Cx1 * exp(-(damping_M[0] / mass_M[0]) * _DELTA_T);

  /* Coefficents and Solution for Y-Direction */
  totalForces_M[1] = forceXYZ[1] + externalFxyz[1];
  float Cy1 = ((totalForces_M[1] / damping_M[1]) - xyzDotInit_M[1]) * (mass_M[1] / damping_M[1]);
  float Cy2 = xyzInit_M[1] - Cy1;
  xyzGoal_M[1]    = Cy1 * exp(-(damping_M[1] / mass_M[1]) * _DELTA_T) + (totalForces_M[1] / damping_M[1]) * _DELTA_T + Cy2;
  xyzDotGoal_M[1] = (totalForces_M[1] / damping_M[1]) - (damping_M[1] / mass_M[1]) * Cy1 * exp(-(damping_M[1] / mass_M[1]) * _DELTA_T);

  /* Coefficents and Solution for Z-Direction */
  totalForces_M[2] = forceXYZ[2] + externalFxyz[2]; // Testing without gravity
  float Cz1 = ((totalForces_M[2]  / damping_M[2]) - xyzDotInit_M[2]) * (mass_M[2] / damping_M[2]);
  float Cz2 = xyzInit_M[2] - Cz1;
  xyzGoal_M[2]    = Cz1 * exp(-(damping_M[2] / mass_M[2]) * _DELTA_T) + (totalForces_M[2] / damping_M[2]) * _DELTA_T + Cz2;
  xyzDotGoal_M[2] = (totalForces_M[2] / damping_M[2]) - (damping_M[2] / mass_M[2]) * Cz1 * exp(-(damping_M[2] / mass_M[2]) * _DELTA_T);

  /* Check TaskSpace Limits */
  if (xyzGoal_M[2] >  _Z_LIMIT) {
    xyzGoal_M[2] =  _Z_LIMIT;
    xyzDotGoal_M[2] = 0.0f;
  }
  if (xyzGoal_M[2] < -_Z_LIMIT) {
    xyzGoal_M[2] = -_Z_LIMIT;
    xyzDotGoal_M[2] = 0.0f;
  }
  float outerRLimit = _A1A2 + _H_OF_L2 + sqrt(pow(ASR::L1_LINK, 2) - pow(xyzGoal_M[2], 2));
  float Rxy = sqrt(pow(xyzGoal_M[0],2) + pow(xyzGoal_M[1],2));
  float alpha   = atan2(xyzGoal_M[1], xyzGoal_M[0]);
  if (alpha < 0.0f) alpha += 2 * PI;
  if (Rxy < _INNER_R_LIMIT) {
    Rxy           = _INNER_R_LIMIT;
    xyzGoal_M[0]  = _INNER_R_LIMIT * cos(alpha);
    xyzGoal_M[1]  = _INNER_R_LIMIT * sin(alpha);
  }
  if (Rxy > outerRLimit) {
    Rxy           = outerRLimit;
    xyzGoal_M[0]  = outerRLimit * cos(alpha);
    xyzGoal_M[1]  = outerRLimit * sin(alpha);
  } 
}

/* ---------------------------------------------------------------------------------------/
/ Admittance Model Get Functions ---------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
float* AdmittanceModel::GetGoalPos() {
  return xyzGoal_M;
}

float* AdmittanceModel::GetGoalVel() {
  return xyzDotGoal_M;
}

float*  AdmittanceModel::GetMass(){
  return mass_M;
}

float*  AdmittanceModel::GetDamping(){
  return damping_M;
}

float* AdmittanceModel::GetTotalForces(){
  return totalForces_M;
}

/* ---------------------------------------------------------------------------------------/
/ Admittance Model Setter Functions ------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
void AdmittanceModel::SetMassXY(float newMxy){
  if (newMxy > 0.1){
    mass_M[0] = newMxy;
    mass_M[1] = newMxy;
  } else {
    mass_M[0] = 0.1;
    mass_M[1] = 0.1;
  }
}

void AdmittanceModel::SetMassZ(float newMz){
  if (newMz > 0.1){
    mass_M[2] = newMz;
  } else {
    mass_M[2] = 0.1;
  }
}

void AdmittanceModel::SetDampingXY(float newBxy){
  if (newBxy > 0.1){
    damping_M[0] = newBxy;
    damping_M[1] = newBxy;
  } else {
    damping_M[0] = 0.1;
    damping_M[1] = 0.1;
  }
}

void AdmittanceModel::SetDampingZ(float newBz){
  if (newBz > 0.1){
    damping_M[2] = newBz;
  } else {
    damping_M[2] = 0.1;
  }
}
