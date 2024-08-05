/* Namespace containing DXL parameters

   Created 10/29/2020
   by Erick Nunez
*/

#include <Arduino.h>
#include "armSupportNamespace.h"

namespace ASR
{
/* ---------------------------------------------------------------------------------------/
/ Constants ------------------------------------------------------------------------------/
/----------------------------------------------------------------------------------------*/
// Dynamixel Communication Parameters
const float   PROTOCOL_VERSION      = 2.0;
const int     CONTROLLER_BAUDRATE   = 1000000;
const int     SERIAL_BAUDRATE   = 115200;
const int     MOTOR_BAUDRATE    = 1000000;
const char   *DEVICEPORT        = "3";
const uint8_t CAL_BUTTON_PIN    = 23;
const uint8_t TORQUE_SWITCH_PIN = 1;

// Force Sensor Constants
const float xyzSens[3] = {0.0496, 0.0494, 0.6231};
const float SENSOR_FILTER_WEIGHT = 0.05;

// Admitance Control Constants and initial Values
const float LOOP_DT        = 8;        // Milliseconds
const float MODEL_DT       = 0.008;    // Secs
const float GRAVITY        = 9.80665;  // m/sec^2
float initMassXY    = 1.5f;     // kg
float initDampingXY = 5.0f;     // N*(sec/m)
float initMassZ     = 1.5f;     // kg
float initDampingZ  = 4.5f;     // N*(sec/m)

// Kinematic Constants for Task Space Limits
const float A1_LINK     = 0.073;      // Shoulder to 4bar linkage
const float L1_LINK     = 0.419;      // length of 4bar linkage
const float A2_LINK     = 0.082;      // 4bar linkage to elbow
const float L2_LINK     = 0.520;      // elbow to sensor
const float LINK_OFFSET = 0.035;      // elbow to sensor offset
const float ELEVATION_MAX_POS = 3487;
const float ELEVATION_MIN_POS = 1137;
const float DEGREES_PER_COUNT = 0.088;
const float ELEVATION_RATIO   = 2.2978;

// Spring Dynamics
const int    SPRING_KS      = 8231;     // N/m
const float  SPRING_X0      = 0.127;    // unstretched spring length in m (5")
const float  SPRING_XI      = 0.16002;  // initial stretch lenght in m (6.3")
const float  SPRING_SIDE_A  = 0.066;    // in m
const float  SPRING_SIDE_B  = 0.0365;   // in m
const float  COS_SIN_45     = 0.7071;   // Cos Sin of 45 degrees
const float  DEG_TO_RAD_45  = 0.7853;   // 45 Degrees to radians
float SPRING_FORCE_SCALING_FACTOR = 0.5;
}