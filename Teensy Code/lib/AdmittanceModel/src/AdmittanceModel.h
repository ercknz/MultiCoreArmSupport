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
           AdmittanceModel(float Mxy, float Mz, float Bxy, float Bz, const float G, const float T);
    void   SetPosition(float *newXYZ);
    void   UpdateModel(float *forceXYZ, float springFz, float *externalFxyz);
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
    const float _GRAVITY,       _DELTA_T;
    // Task Space limits
    const float A1_LINK = 0.073;
    const float L1_LINK = 0.419;
    const float A2_LINK = 0.082;
    const float L2_LINK = 0.520;
    const float ELEVATION_MAX_POS = 3487;
    const float ELEVATION_MIN_POS = 1137;
    const float ELEVATION_CENTER = (ELEVATION_MAX_POS + ELEVATION_MIN_POS) / 2;
    const float DEGREES_PER_COUNT = 0.088;
    const float ELEVATION_RATIO = 2.2978;
    const float LINK_OFFSET = 0.035;
    const float _H_OF_L2 = sqrt(pow(LINK_OFFSET, 2) + pow(L2_LINK, 2));      
    const float _A1A2 = A1_LINK + A2_LINK;
    const float _INNER_R_LIMIT = A1_LINK + L1_LINK + A2_LINK - L2_LINK;
    const float _Z_LIMIT = abs(L1_LINK * sin((ELEVATION_MAX_POS - ELEVATION_CENTER) * DEGREES_PER_COUNT * (PI / 180.0) * (1/ELEVATION_RATIO)));

    float mass_M[3];        // [xy, xy, z]
    float damping_M[3];     // [xy, xy, z]
    float xyzGoal_M[3]      = {0.0f};
    float xyzDotGoal_M[3]   = {0.0f};
    float xyzInit_M[3]      = {0.0f};
    float xyzDotInit_M[3]   = {0.0f};
    float totalForces_M[3]  = {0.0f};
};

#endif // ADMITTANCE_MODEL_H
