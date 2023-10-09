function [Q1,Q2,Q4] = armSupportIKine(x,y,z)
%% Arm Support Inverse Kinematics
% This function is used to calculate the joint angles and angular
% velocities of the robot given the position and velocity of the object in
% the workspace.
%
% Script by erick nunez

A1A2 = A1 + A2;
HofL2 = sqrt(A4^2 + L2^2);
Q2 = @(z) asin((z-A3) / L1);
L1xy = @(z) sqrt(L1^2 - (z-A3)^2);
R = @(x,y) sqrt(x^2 + y^2);
Alpha = @(x,y) atan2(y, x);
Gamma = @(x,y,L1xy) acos(((A1A2 + L1xy)^2 + HofL2^2 - x^2 - y^2) / (2 * HofL2 * (A1A2 + L1xy)));
Q4 = @(gamma) pi - gamma + deg2rad(ELBW_LIMIT(1));
Beta = @(gamma,R) asin((HofL2 * sin(gamma)) / R);
Q1 = @(alpha,beta) alpha - beta;

end