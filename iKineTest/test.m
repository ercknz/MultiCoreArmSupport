clear; clc; 

%% Constants and limits
L1 = 0.419;
L2 = 0.520;
A1 = 0.073;
A2 = 0.082;
A3 = 0.072;
offset = 0.035;

SHDR_LIMIT = [0,270];
ELVN_LIMIT = [-45,45];
ELBW_LIMIT = [atand(offset/L2),180];

testGoals = [0.750,  0.300,  0.250;
             0.750, -0.300,  0.250;
             0.550,  0.000,  0.000;
             0.650,  0.300, -0.200;
             0.650, -0.300, -0.200];
         
Q = [0, 0, 0];
         
%% Inverse Kinematics
A1A2 = A1 + A2;
HofL2 = sqrt(offset^2 + L2^2);
Q2 = @(z) asin((z-A3) / L1);
L1xy = @(z) sqrt(L1^2 - (z-A3)^2);
R = @(x,y) sqrt(x^2 + y^2);
Alpha = @(y,x) atan2(y, x);
Gamma = @(x,y,L1xy) acos(((A1A2 + L1xy)^2 + HofL2^2 - x^2 - y^2) / (2 * HofL2 * (A1A2 + L1xy)));
Q4 = @(gamma) pi - gamma + deg2rad(ELBW_LIMIT(1));
Beta = @(gamma,R) asin((HofL2 * sin(gamma)) / R);
Q1 = @(alpha,beta) alpha - beta;

%% Print results

for i = 1:5
    x = testGoals(i,1);
    y = testGoals(i,2);
    z = testGoals(i,3);
    Q(2) = Q2(z);
    tempL1xy = L1xy(z);
    tempR = R(x,y);
    tempAlpha = Alpha(y,x);
    if tempAlpha < 0
        tempAlpha = tempAlpha + 2*pi;
    end
    fprintf('%f\n', tempAlpha)
    tempGamma = Gamma(x,y,tempL1xy);
    fprintf('%f\n', tempGamma)
    Q(3) = Q4(tempGamma);
    tempBeta = Beta(tempGamma,tempR);
    fprintf('%f\n', tempBeta)
    Q(1) = Q1(tempAlpha,tempBeta);
    fprintf('%f\t%f\t%f\n', Q(1), Q(2), Q(3))
end

