%% Arm Support Kinematics
% This function is used to calculate the position and velocity of the end
% effector of the robot in the workpace given the joint angles and
% velocities.
%
% Script by erick nunez

%% Constants
% physical robot dimensions
Link1 = 0.419;
Link2 = 0.520;
AOS1 = 0.073;
AOS2 = 0.082;
AOS3 = 0.035;

%% Matrix Functions
% Rotation about X and Z axis matrixs and Transformation Matrix
rotateX = @(a)  [1,0,0;
                 0,cos(a),-sin(a);
                 0,sin(a),cos(a)];
rotateZ = @(c)  [cos(c),-sin(c),0;
                 sin(c),cos(c),0;
                 0,0,1];

matrixT = @(R,P)[R, P; 
                 0,0,0,1];
             
matrixTi = @(alpha,a,theta,d)   [cos(theta),            -sin(theta),            0,              a;
                                 sin(theta)*cos(alpha), cos(theta)*cos(alpha),  -sin(alpha),    -sin(alpha)*d;
                                 sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),     cos(alpha)*d;
                                 0,                     0,                      0,              1];
             
%% Frames
% Transformation Matrices for each point along robot. Each transformation
% matrix is made up of rotations about X times the rotate about Z and the
% position vector of the translate from the last frame.
T01 = @(q1) matrixT(rotateZ(q1),[0,0,0]');
T12 = @(q2,A1) matrixT(rotateX(sym(pi/2))*rotateZ(q2),[A1,0,0]');
T23 = @(q2,L1) matrixT(rotateZ(-q2),[L1,0,0]');
T34 = @(q4,A2) matrixT(rotateX(sym(-pi/2))*rotateZ(q4),[A2,0,0]');
T45 = @(A3) matrixT(rotateZ(0),[0,0,A3]');
T56 = @(A4) matrixT(rotateZ(0),[0,-A4,0]');
T67 = @(L2) matrixT(rotateZ(0),[L2,0,0]');

T02 = @(q1,q2,A1) T01(q1)*T12(q2,A1);
T03 = @(q1,q2,A1,L1) T01(q1)*T12(q2,A1)*T23(q2,L1);
T04 = @(q1,q2,A1,L1,q4,A2) T01(q1)*T12(q2,A1)*T23(q2,L1)*T34(q4,A2);
T05 = @(q1,q2,A1,L1,q4,A2,A3) T01(q1)*T12(q2,A1)*T23(q2,L1)*T34(q4,A2)*T45(A3);
T06 = @(q1,q2,A1,L1,q4,A2,A3,A4) T01(q1)*T12(q2,A1)*T23(q2,L1)*T34(q4,A2)*T45(A3)*T56(A4);
T07 = @(q1,q2,A1,L1,q4,A2,A3,A4,L2) T01(q1)*T12(q2,A1)*T23(q2,L1)*T34(q4,A2)*T45(A3)*T56(A4)*T67(L2);

%% Forward equations
% Transformation matrix from base frame to end-effector. Matrix can be
% broken down to [N,S,A,P;0,0,0,1] where N is the normal vector of
% end-effector. S is the sliding vector of the end-effector. A is the
% approach vector of the end-effector.
syms q1 q2 q4 L1 L2 A1 A2 A3 A4 real
output = T07(q1,q2,A1,L1,q4,A2,A3,A4,L2);
n = simplify(output(1:3,1))
s = simplify(output(1:3,2))
a = simplify(output(1:3,3))
p = simplify(output(1:3,4))

%% Inverse Transformation matrix
% Inverse Transformation matrix of robot. Goes from end-effector frame to
% base frame. 
Ti = [output(1:3,1:3)',[-n'*p,-s'*p,-a'*p]';0,0,0,1]

R = Ti(1:3,1:3)*[1;1;1]

%% Inverse Kinematics from C++ code
A1A2 = A1 + A2;
HofL2 = sqrt(A4^2 + L2^2);
% Check z is within limits, if not set to limits.
% Check if X and Y are near zero, if so find new position.
Q2 = @(z) asin((z-A3) / L1);
L1xy = @(z) sqrt(L1^2 - (z-A3)^2);
R = @(x,y) sqrt(x^2 + y^2);
Alpha = @(x,y) atan2(y, x);
% Check if Alpha is negative, if so add 2*pi.
% Check if R is outside radius limits, if so, find new position w/r to Alpha.
Gamma = @(x,y,L1xy) acos(((A1A2 + L1xy)^2 + HofL2^2 - x^2 - y^2) / (2 * HofL2 * (A1A2 + L1xy)));
Q4 = @(gamma) pi - gamma + deg2rad(ELBW_LIMIT(1));
Beta = @(gamma,R) asin((HofL2 * sin(gamma)) / R);
Q1 = @(alpha,beta) alpha - beta;

%% Velocity forward kinematics
% Robot is underactuated. We will only consider the linear velocity
% components. 
X = p(1);
Y = p(2);
Z = p(3);
dXdQ1 = diff(X,q1);
dXdQ2 = diff(X,q2);
dXdQ4 = diff(X,q4);

dYdQ1 = diff(Y,q1);
dYdQ2 = diff(Y,q2);
dYdQ4 = diff(Y,q4);

dZdQ1 = diff(Z,q1);
dZdQ2 = diff(Z,q2);
dZdQ4 = diff(Z,q4);

Jv = [dXdQ1,dXdQ2,dXdQ4; dYdQ1,dYdQ2,dYdQ4; dZdQ1,dZdQ2,dZdQ4]

invJ = simplify(inv(Jv))