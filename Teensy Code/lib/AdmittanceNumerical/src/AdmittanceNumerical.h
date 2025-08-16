/* This class is the admittance control model.
   It takes a XYZ force input and output XYZ position and velocity based on initial conditions and model parameters.
   The model is using a numerical Euler integration method to update the position and velocity.

   Created 8/16/2025
   by erick nunez
*/

#ifndef ADMITTANCE_NUMERICAL_H
#define ADMITTANCE_NUMERICAL_H

#include <Arduino.h>

class AdmittanceNumerical {
  public:
           AdmittanceNumerical(float Mxy, float Mz, float Bxy, float Bz);
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
    void   CalculateParameters();

    // Model constants
    const float _GRAVITY;
    const float _DELTA_T;
    const float _ELEVATION_CENTER;
    const float _MODEL_Z_LIMIT;
    const float _H_OF_L2;      
    const float _A1A2;
    const float _INNER_R_LIMIT;
    const float _MODEL_X_LIMIT = 0.0f;
    const float _MODEL_Y_LIMIT = 0.5f;
    const float _VEL_LIMIT;
    const float _MAX_STEP;

    float mass_M[3];        // [x, y, z]
    float damping_M[3];     // [x, y, z]
    float a0_M[3]           = {0.0f};
    float a1_M[3]           = {0.0f};
    float a2_M[3]           = {0.0f};
    float xyzGoal_M[3]      = {0.0f};
    float xyzDotGoal_M[3]   = {0.0f};
    float xyz_1_M[3]        = {0.0f};
    float xyz_2_M[3]        = {0.0f};
    float xyzDot_1_M[3]     = {0.0f};
    float totalForces_M[3]  = {0.0f};
};

#endif // ADMITTANCE_NUMERICAL_H
