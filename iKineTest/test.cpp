// Online C++ compiler to run C++ program online
#include <iostream>
#include <math.h>

// Dynamixel Motor Limits
const int ELBOW_MIN_POS = 1200;
const int ELBOW_MAX_POS = 3130;
const int SHOULDER_MIN_POS = 776;
const int SHOULDER_MAX_POS = 3677;
const int ELEVATION_MIN_POS = 720;
const int ELEVATION_MAX_POS = 3040;
const int ELEVATION_CENTER = (ELEVATION_MAX_POS + ELEVATION_MIN_POS) / 2;
const float ELEVATION_RATIO = 2.2978;
const int VEL_MAX_LIMIT = 100;
const float SHOULDER_OFFSET = 0.6219;

// Kinematic Constants
const float A1_LINK = 0.073;	 // Shoulder to 4bar linkage
const float L1_LINK = 0.419;	 // length of 4bar linkage
const float A2_LINK = 0.082;	 // 4bar linkage to elbow
const float L2_LINK = 0.520;	 // elbow to sensor
const float A3_LINK = 0.072;	 // Vertical Offset
const float LINK_OFFSET = 0.035; // elbow to sensor offset

// Dynamixel Parameters for calculations
const float DEGREES_PER_COUNT = 0.088; // degrees
const float RPM_PER_COUNT = 0.229;	   // Rev/min
const float CURRENT_PER_COUNT = 2.69;  // mA
const float PI = 3.14159;

const int SPRING_KS = 8231;			// N/m
const float SPRING_X0 = 0.127;		// unstretched spring length in m (5")
const float SPRING_XI = 0.16002;	// initial stretch lenght in m (6.3")
const float SPRING_SIDE_A = 0.066;	// in m
const float SPRING_SIDE_B = 0.0365; // in m
const float COS_SIN_45 = 0.7071;	// Cos Sin of 45 degrees
const float DEG_TO_RAD_45 = 0.7853; // 45 Degrees to radians
float SPRING_FORCE_SCALING_FACTOR = 0.5;

// Other Constants
const float _A1A2 = A1_LINK + A2_LINK;
const float _L1 = L1_LINK;
const float _L2 = L2_LINK;
const float _OFFSET = LINK_OFFSET;
const float _PHI = atan(_OFFSET / _L2);
const float _H_OF_L2 = sqrt(pow(_OFFSET, 2) + pow(_L2, 2));
const float _Q1_MIN = SHOULDER_MIN_POS * DEGREES_PER_COUNT * (PI / 180.0);
const float _Q1_MAX = SHOULDER_MAX_POS * DEGREES_PER_COUNT * (PI / 180.0);
const float _Q2_LIMIT = abs((ELEVATION_MAX_POS - ELEVATION_CENTER) * DEGREES_PER_COUNT * (PI / 180.0) * (1 / ELEVATION_RATIO));
const float _Q4_MIN = (ELBOW_MIN_POS - ELBOW_MIN_POS) * DEGREES_PER_COUNT * (PI / 180.0) + SHOULDER_OFFSET;
const float _Q4_MAX = (ELBOW_MAX_POS - ELBOW_MIN_POS) * DEGREES_PER_COUNT * (PI / 180.0) + SHOULDER_OFFSET;
const float _INNER_R = A1_LINK + L1_LINK + A2_LINK - L2_LINK;
const float _Z_LIMIT = abs(L1_LINK * sin((ELEVATION_MAX_POS - ELEVATION_CENTER) * DEGREES_PER_COUNT * (PI / 180.0) * (1 / ELEVATION_RATIO)));
const float _SPRING_Li = sqrt(pow(SPRING_SIDE_A, 2) + pow(SPRING_SIDE_B, 2) + 2 * SPRING_SIDE_A * SPRING_SIDE_B * COS_SIN_45);
const float _BETAi = asin((SPRING_SIDE_A / _SPRING_Li) * -COS_SIN_45);
const float _SPRING_Fi = SPRING_KS * (SPRING_XI - SPRING_X0) * sin(_BETAi + DEG_TO_RAD_45);

float goal1[3] = {0.75,  0.3,  0.25};
float goal2[3] = {0.75, -0.3,  0.25};
float goal3[3] = {0.55,  0.0,  0.0};
float goal4[3] = {0.65,  0.3, -0.2};
float goal5[3] = {0.65, -0.3, -0.2};

float   q_M[3];

void iKine(float *xyz_M)
{
	/*  NOTE:
				xyz_M[3] = {x, y, z}
				q_M[3] = {q1, q2, q4} = {shoulder, elevation, elbow}
	*/
	float L1_XY, OUTER_R, R, alpha, presR, presAlpha, beta, gamma, detJ;

	/* Check Z limits */
	if (xyz_M[2] > _Z_LIMIT)
		xyz_M[2] = _Z_LIMIT;
	if (xyz_M[2] < -_Z_LIMIT)
		xyz_M[2] = -_Z_LIMIT;

	/* Find variables based on Z */
	q_M[1] = asin((xyz_M[2]-A3_LINK) / _L1);
	L1_XY = sqrt(pow(_L1, 2) - pow((xyz_M[2]-A3_LINK), 2));

	/* Checks if X and Y are both 0 */
	if ((abs(xyz_M[0]) < 0.001) && (abs(xyz_M[1]) < 0.001))
	{
		q_M[0] = 69;
		q_M[2] = _Q4_MAX;
		xyz_M[0] = _A1A2 * cos(q_M[0]) + _L1 * cos(q_M[0]) * cos(q_M[1]) + _OFFSET * sin(q_M[0] + q_M[2]) + _L2 * cos(q_M[0] + q_M[2]);
		xyz_M[1] = _A1A2 * sin(q_M[0]) + _L1 * sin(q_M[0]) * cos(q_M[1]) - _OFFSET * cos(q_M[0] + q_M[2]) + _L2 * sin(q_M[0] + q_M[2]);
	}

	/* R and Alpha */
	R = sqrt(pow(xyz_M[0], 2) + pow(xyz_M[1], 2));
	alpha = atan2(xyz_M[1], xyz_M[0]);
	if (alpha < 0.0f)
		alpha += 2 * PI;
		printf("%f\n", alpha);
	/*presR = sqrt(pow(xyzPres_M[0], 2) + pow(xyzPres_M[1], 2));
	presAlpha = atan2(xyzPres_M[1], xyzPres_M[0]);
	if (presAlpha < 1.0f)
		presAlpha += 2 * PI;

	/* Checks walls */
	OUTER_R = _A1A2 + _H_OF_L2 + L1_XY;
	if (R < _INNER_R)
	{
		R = _INNER_R;
		xyz_M[0] = _INNER_R * cos(alpha);
		xyz_M[1] = _INNER_R * sin(alpha);
	}
	if (R > OUTER_R)
	{
		R = OUTER_R;
		xyz_M[0] = OUTER_R * cos(alpha);
		xyz_M[1] = OUTER_R * sin(alpha);                                   
	}

	/* Finds and checks Elbow Angle *
	if ((abs(R - presR) < 0.01) && (alpha < presAlpha))
	{
		q_M[2] = qPres_M[2];
		gamma = PI - q_M[2];
	}
	else
	{*/
		gamma = acos((pow((_A1A2 + L1_XY), 2) + pow(_H_OF_L2, 2) - pow(xyz_M[0], 2) - pow(xyz_M[1], 2)) / (2 * _H_OF_L2 * (_A1A2 + L1_XY)));
		printf("%f\n", gamma);
		q_M[2] = PI - gamma + _PHI;
	//}

	/* Finds and checks shoulder angle */
	beta = asin((_H_OF_L2 * sin(gamma)) / R);
	printf("%f\n", beta);
	q_M[0] = alpha - beta;// + SHOULDER_OFFSET;

	/* Check for nans *
	if (q_M[0] != q_M[0])
		q_M[0] = qPres_M[0];
	if (q_M[2] != q_M[2])
		q_M[2] = qPres_M[2]; */

	/* Check Jointspace Limits *
	if (q_M[2] < _Q4_MIN)
		q_M[2] = _Q4_MIN;
	if (q_M[2] > _Q4_MAX)
		q_M[2] = _Q4_MAX;
	if (q_M[0] < _Q1_MIN)
		q_M[0] = _Q1_MIN;
	if (q_M[0] > _Q1_MAX)
		q_M[0] = _Q1_MAX;*/

	/* Solve for joint angular velocities (psuedo inverse Jacobian) *
	detJ = (-_L1 * sin(q_M[0]) - _L2 * sin(q_M[0] + q_M[2])) * (_L2 * cos(q_M[0] + q_M[2])) - (-_L2 * sin(q_M[0] + q_M[2])) * (_L1 * cos(q_M[0]) + _L2 * cos(q_M[0] + q_M[2]));
	qDot_M[0] = (xyzDot_M[0] * (_L2 * cos(q_M[0] + q_M[2])) + xyzDot_M[1] * (_L2 * sin(q_M[0] + q_M[2]))) / detJ;
	qDot_M[1] = xyzDot_M[2] / (_L1 * sqrt(1 - pow((xyz_M[2] / _L1), 2)));
	qDot_M[2] = (xyzDot_M[0] * (-_L1 * cos(q_M[0]) - _L2 * cos(q_M[0] + q_M[2])) + xyzDot_M[1] * (-_L1 * sin(q_M[0]) - _L2 * sin(q_M[0] + q_M[2]))) / detJ;
	*/
}

int main()
{
	/*printf("A1A2 = %f\n", _A1A2);
	printf("Phi = %f\n", _PHI);
	printf("HofL2 = %f\n", _H_OF_L2);
	printf("Q1Max = %f\n", _Q1_MAX);
	printf("Q1Min = %f\n", _Q1_MIN);
	printf("Q2Limit = %f\n", _Q2_LIMIT);
	printf("Q4max = %f\n", _Q4_MAX);
	printf("Q4min = %f\n", _Q4_MIN);
	printf("innerR = %f\n", _INNER_R);
	printf("zLimit = %f\n", _Z_LIMIT);
	printf("BetaI = %f\n", _BETAi);*/

	iKine(goal1);
	printf("%f\t%f\t%f\n", q_M[0], q_M[1], q_M[2]);

	iKine(goal2);
	printf("%f\t%f\t%f\n", q_M[0], q_M[1], q_M[2]);

	iKine(goal3);
	printf("%f\t%f\t%f\n", q_M[0], q_M[1], q_M[2]);

	iKine(goal4);
	printf("%f\t%f\t%f\n", q_M[0], q_M[1], q_M[2]);

	iKine(goal5);
	printf("%f\t%f\t%f\n", q_M[0], q_M[1], q_M[2]);

	return 0;
}