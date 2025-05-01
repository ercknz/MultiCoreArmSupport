/* This class is the admittance control model.
   It takes a XYZ force input and output XYZ position and velocity based on initial conditions.

   Created 10/27/2020
   by erick nunez
*/

#include <Arduino.h>

#ifndef ADMITTANCE_MODEL_H
#define ADMITTANCE_MODEL_H

class AdmittanceModel {
  public:
           AdmittanceModel(float Mxy, float Mz, float Bxy, float Bz);
    void   SetPosition(float *newXYZ);
    void   UpdateModel(float *forceXYZ, float *externalFxyz);
    float* GetGoalPos();
    float* GetGoalVel();
    float* GetMass();
    float* GetDamping();
    float* GetTotalForces();
    void   SetMassXY(float newMxy);
    void   SetMassZ(float newMz);
    void   SetDampingXY(float newBxy);
    void   SetDampingZ(float newBz);

  protected:
    // Model constants
    const float _GRAVITY;
    const float _DELTA_T;
    const float _ELEVATION_CENTER;
    const float _Z_LIMIT;
    const float _H_OF_L2;      
    const float _A1A2;
    const float _INNER_R_LIMIT;

    float mass_M[3];        // [xy, xy, z]
    float damping_M[3];     // [xy, xy, z]
    float xyzGoal_M[3]      = {0.0f};
    float xyzDotGoal_M[3]   = {0.0f};
    float xyzInit_M[3]      = {0.0f};
    float xyzDotInit_M[3]   = {0.0f};
    float totalForces_M[3]  = {0.0f};
};

#endif // ADMITTANCE_MODEL_H
