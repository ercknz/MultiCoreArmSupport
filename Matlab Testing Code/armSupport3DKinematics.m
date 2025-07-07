function armSupport3DKinematics()
%% Arm support 3D kinematics
% code by Erick Nunez

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
idx = 1;
velocityXYZ= 0.3;

%% Rotational Matrix Definition
rotateX = @(a)  [1,0,0;
                 0,cos(a),-sin(a);
                 0,sin(a),cos(a)];
rotateZ = @(c)  [cos(c),-sin(c),0;
                 sin(c),cos(c),0;
                 0,0,1];

matrixT = @(R,P)[R, P; 
                 0,0,0,1];
             
%% Forward Kinematics
T01 = @(q1) matrixT(rotateZ(q1),[0,0,0]');
T12 = @(q2) matrixT(rotateX(pi/2)*rotateZ(q2),[A1,0,0]');
T23 = @(q2) matrixT(rotateZ(-q2),[L1,0,0]');
T34 = @(q4) matrixT(rotateX(-pi/2)*rotateZ(q4),[A2,0,0]');
T45 = matrixT(rotateZ(0),[0,0,A3]');
T56 = matrixT(rotateZ(0),[0,-offset,0]');
T67 = matrixT(rotateZ(0),[L2,0,0]');

T02 = @(q1,q2) T01(q1)*T12(q2);
T03 = @(q1,q2) T01(q1)*T12(q2)*T23(q2);
T04 = @(q1,q2,q4) T01(q1)*T12(q2)*T23(q2)*T34(q4);
T05 = @(q1,q2,q4) T01(q1)*T12(q2)*T23(q2)*T34(q4)*T45;
T06 = @(q1,q2,q4) T01(q1)*T12(q2)*T23(q2)*T34(q4)*T45*T56;
T07 = @(q1,q2,q4) T01(q1)*T12(q2)*T23(q2)*T34(q4)*T45*T56*T67;

t1o = T01(0);
t2o = T02(0,0);
t3o = T03(0,0);
t4o = T04(0,0,ELBW_LIMIT(1));
t5o = T05(0,0,ELBW_LIMIT(1));
t6o = T06(0,0,ELBW_LIMIT(1));
t7o = T07(0,0,ELBW_LIMIT(1));

%% Inverse Kinematics
A1A2 = A1 + A2;
HofL2 = sqrt(offset^2 + L2^2);
Q2 = @(z) asin((z-A3) / L1);
L1xy = @(z) sqrt(L1^2 - (z-A3)^2);
R = @(x,y) sqrt(x^2 + y^2);
function Alpha = findAlpha(x,y) 
    Alpha = atan2(y, x);
    if Alpha < 0
        Alpha = Alpha + 2*pi;
    end
end
Gamma = @(x,y,L1xy) acos(((A1A2 + L1xy)^2 + HofL2^2 - x^2 - y^2) / (2 * HofL2 * (A1A2 + L1xy)));
Q4 = @(gamma) pi - gamma + deg2rad(ELBW_LIMIT(1));
Beta = @(gamma,R) asin((HofL2 * sin(gamma)) / R);
Q1 = @(alpha,beta) alpha - beta;



%% Serial Comm
robotSerialPort = 'COM4';
robotBaud = 115200;
botSerial = [];
fullRobot = false;

%% GUI
% Initial Values for lines
t0o = [0,0,0];
t1o = T01(0);
t2o = T02(0,0);
t3o = T03(0,0);
t4o = T04(0,0,deg2rad(ELBW_LIMIT(1)));
t5o = T05(0,0,deg2rad(ELBW_LIMIT(1)));
t6o = T06(0,0,deg2rad(ELBW_LIMIT(1)));
t7o = T07(0,0,deg2rad(ELBW_LIMIT(1)));
Hfigure = figure(100);
set(Hfigure,'Units','normalized','Position',[0.05 0.05 0.90 0.85])
set(Hfigure,'UserData',false);
% set(Hfigure,'CloseRequestFcn',@closereq);
% Top(XY) plot creation (top right) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
plotXYHandle = subplot(2,2,1);
grid on; hold on;
% Robot
baseLink_XY = plot([t0o(1) t1o(1,4)], [t0o(2) t1o(2,4)],'LineWidth',2,'Color','#0072BD');
link1_XY = plot([t1o(1,4) t2o(1,4)], [t1o(2,4) t2o(2,4)],'LineWidth',2,'Color','#0072BD');
link2_XY = plot([t2o(1,4) t3o(1,4)], [t2o(2,4) t3o(2,4)],'LineWidth',2,'Color','#D95319');
link3_XY = plot([t3o(1,4) t4o(1,4)], [t3o(2,4) t4o(2,4)],'LineWidth',2,'Color','#0072BD');
link4_XY = plot([t4o(1,4) t5o(1,4)], [t4o(2,4) t5o(2,4)],'LineWidth',2,'Color','#0072BD');
link5_XY = plot([t5o(1,4) t6o(1,4)], [t5o(2,4) t6o(2,4)],'LineWidth',2,'Color','#0072BD');
link6_XY = plot([t6o(1,4) t7o(1,4)], [t6o(2,4) t7o(2,4)],'LineWidth',2,'Color','#D95319');
end_XY = plot(t7o(1,4),t7o(2,4),'g*','MarkerSize',12);
% Goal
g_baseLink_XY = plot([t0o(1) t1o(1,4)], [t0o(2) t1o(2,4)],'LineWidth',2,'Color',[0 0 0 0.25]);
g_link1_XY = plot([t1o(1,4) t2o(1,4)], [t1o(2,4) t2o(2,4)],'LineWidth',2,'Color',[0 0 0 0.25]);
g_link2_XY = plot([t2o(1,4) t3o(1,4)], [t2o(2,4) t3o(2,4)],'LineWidth',2,'Color',[1 0 1 0.25]);
g_link3_XY = plot([t3o(1,4) t4o(1,4)], [t3o(2,4) t4o(2,4)],'LineWidth',2,'Color',[0 0 0 0.25]);
g_link4_XY = plot([t4o(1,4) t5o(1,4)], [t4o(2,4) t5o(2,4)],'LineWidth',2,'Color',[0 0 0 0.25]);
g_link5_XY = plot([t5o(1,4) t6o(1,4)], [t5o(2,4) t6o(2,4)],'LineWidth',2,'Color',[0 0 0 0.25]);
g_link6_XY = plot([t6o(1,4) t7o(1,4)], [t6o(2,4) t7o(2,4)],'LineWidth',2,'Color',[1 0 1 0.25]);
g_end_XY = plot(t7o(1,4),t7o(2,4),'*','MarkerSize',12,'Color',[0.6350 0.0780 0.1840 0.2]);
% Goal Setting
s_baseLink_XY = plot([t0o(1) t1o(1,4)], [t0o(2) t1o(2,4)],'LineWidth',2,'Color',[1 0 0 0.25]);
s_link1_XY = plot([t1o(1,4) t2o(1,4)], [t1o(2,4) t2o(2,4)],'LineWidth',2,'Color',[1 0 0 0.25]);
s_link2_XY = plot([t2o(1,4) t3o(1,4)], [t2o(2,4) t3o(2,4)],'LineWidth',2,'Color',[0 1 0 0.25]);
s_link3_XY = plot([t3o(1,4) t4o(1,4)], [t3o(2,4) t4o(2,4)],'LineWidth',2,'Color',[1 0 0 0.25]);
s_link4_XY = plot([t4o(1,4) t5o(1,4)], [t4o(2,4) t5o(2,4)],'LineWidth',2,'Color',[1 0 0 0.25]);
s_link5_XY = plot([t5o(1,4) t6o(1,4)], [t5o(2,4) t6o(2,4)],'LineWidth',2,'Color',[1 0 0 0.25]);
s_link6_XY = plot([t6o(1,4) t7o(1,4)], [t6o(2,4) t7o(2,4)],'LineWidth',2,'Color',[0 1 0 0.25]);
% Model XY
modelXY = plot(t7o(1,4),t7o(2,4),'o','MarkerSize',12,'Color',[0.47 0.25 0.80 0.5]);
set(plotXYHandle,'Position',[0.05 0.575 0.3 0.4])
axis([-1.2 1.2 -1.2 1.2]);
xlabel('X');ylabel('Y');
% 3D plot creation (top left) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
plot3DHandle = subplot(2,2,2);
grid on; hold on;
% Robot
baseLink_3d = plot3([t0o(1) t1o(1,4)], [t0o(2) t1o(2,4)], [t0o(3) t1o(3,4)],'LineWidth',2,'Color','#0072BD');
link1_3d = plot3([t1o(1,4) t2o(1,4)], [t1o(2,4) t2o(2,4)], [t1o(3,4) t2o(3,4)],'LineWidth',2,'Color','#0072BD');
link2_3d = plot3([t2o(1,4) t3o(1,4)], [t2o(2,4) t3o(2,4)], [t2o(3,4) t3o(3,4)],'LineWidth',2,'Color','#D95319');
link3_3d = plot3([t3o(1,4) t4o(1,4)], [t3o(2,4) t4o(2,4)], [t3o(3,4) t4o(3,4)],'LineWidth',2,'Color','#0072BD');
link4_3d = plot3([t4o(1,4) t5o(1,4)], [t4o(2,4) t5o(2,4)], [t4o(3,4) t5o(3,4)],'LineWidth',2,'Color','#0072BD');
link5_3d = plot3([t5o(1,4) t6o(1,4)], [t5o(2,4) t6o(2,4)], [t5o(3,4) t6o(3,4)],'LineWidth',2,'Color','#0072BD');
link6_3d = plot3([t6o(1,4) t7o(1,4)], [t6o(2,4) t7o(2,4)], [t6o(3,4) t7o(3,4)],'LineWidth',2,'Color','#D95319');
end_3d = plot3(t7o(1,4),t7o(2,4),t7o(3,4),'g*','MarkerSize',12);
% Goal
g_baseLink_3d = plot3([t0o(1) t1o(1,4)], [t0o(2) t1o(2,4)], [t0o(3) t1o(3,4)],'LineWidth',2,'Color',[0 0 0 0.1]);
g_link1_3d = plot3([t1o(1,4) t2o(1,4)], [t1o(2,4) t2o(2,4)], [t1o(3,4) t2o(3,4)],'LineWidth',2,'Color',[0 0 0 0.25]);
g_link2_3d = plot3([t2o(1,4) t3o(1,4)], [t2o(2,4) t3o(2,4)], [t2o(3,4) t3o(3,4)],'LineWidth',2,'Color',[1 0 1 0.25]);
g_link3_3d = plot3([t3o(1,4) t4o(1,4)], [t3o(2,4) t4o(2,4)], [t3o(3,4) t4o(3,4)],'LineWidth',2,'Color',[0 0 0 0.25]);
g_link4_3d = plot3([t4o(1,4) t5o(1,4)], [t4o(2,4) t5o(2,4)], [t4o(3,4) t5o(3,4)],'LineWidth',2,'Color',[0 0 0 0.25]);
g_link5_3d = plot3([t5o(1,4) t6o(1,4)], [t5o(2,4) t6o(2,4)], [t5o(3,4) t6o(3,4)],'LineWidth',2,'Color',[0 0 0 0.25]);
g_link6_3d = plot3([t6o(1,4) t7o(1,4)], [t6o(2,4) t7o(2,4)], [t6o(3,4) t7o(3,4)],'LineWidth',2,'Color',[1 0 1 0.25]);
g_end_3d = plot3(t7o(1,4),t7o(2,4),t7o(3,4),'*','MarkerSize',12,'Color',[0.6350 0.0780 0.1840 0.2]);
% Goal Setting
s_baseLink_3d = plot3([t0o(1) t1o(1,4)], [t0o(2) t1o(2,4)], [t0o(3) t1o(3,4)],'LineWidth',2,'Color',[1 0 0 0.1]);
s_link1_3d = plot3([t1o(1,4) t2o(1,4)], [t1o(2,4) t2o(2,4)], [t1o(3,4) t2o(3,4)],'LineWidth',2,'Color',[1 0 0 0.25]);
s_link2_3d = plot3([t2o(1,4) t3o(1,4)], [t2o(2,4) t3o(2,4)], [t2o(3,4) t3o(3,4)],'LineWidth',2,'Color',[0 1 0 0.25]);
s_link3_3d = plot3([t3o(1,4) t4o(1,4)], [t3o(2,4) t4o(2,4)], [t3o(3,4) t4o(3,4)],'LineWidth',2,'Color',[1 0 0 0.25]);
s_link4_3d = plot3([t4o(1,4) t5o(1,4)], [t4o(2,4) t5o(2,4)], [t4o(3,4) t5o(3,4)],'LineWidth',2,'Color',[1 0 0 0.25]);
s_link5_3d = plot3([t5o(1,4) t6o(1,4)], [t5o(2,4) t6o(2,4)], [t5o(3,4) t6o(3,4)],'LineWidth',2,'Color',[1 0 0 0.25]);
s_link6_3d = plot3([t6o(1,4) t7o(1,4)], [t6o(2,4) t7o(2,4)], [t6o(3,4) t7o(3,4)],'LineWidth',2,'Color',[0 1 0 0.25]);
% ModelXYZ
modelXYZ = plot3(t7o(1,4),t7o(2,4),t7o(3,4),'o','MarkerSize',12,'Color',[0.47 0.25 0.80 0.5]);
set(plot3DHandle,'Position',[0.4 0.575 0.3 0.4])
set(plot3DHandle,'View',[45,45])
axis([-1.2 1.2 -1.2 1.2 -1.2 1.2]);
legend([link2_3d, end_3d, g_link2_3d, g_end_3d, s_link2_3d],'presR','presXYZ','rGoal','rGoalXYZ','goalSet','Location','northwest');
xlabel('X');ylabel('Y');zlabel('Z');
% Front(XZ) plot creation (bottom left) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
plotXZHandle = subplot(2,2,3);
grid on; hold on;
% Robot
baseLink_XZ = plot([t0o(1) t1o(1,4)], [t0o(3) t1o(3,4)],'LineWidth',2,'Color','#0072BD');
link1_XZ = plot([t1o(1,4) t2o(1,4)], [t1o(3,4) t2o(3,4)],'LineWidth',2,'Color','#0072BD');
link2_XZ = plot([t2o(1,4) t3o(1,4)], [t2o(3,4) t3o(3,4)],'LineWidth',2,'Color','#D95319');
link3_XZ = plot([t3o(1,4) t4o(1,4)], [t3o(3,4) t4o(3,4)],'LineWidth',2,'Color','#0072BD');
link4_XZ = plot([t4o(1,4) t5o(1,4)], [t4o(3,4) t5o(3,4)],'LineWidth',2,'Color','#0072BD');
link5_XZ = plot([t5o(1,4) t6o(1,4)], [t5o(3,4) t6o(3,4)],'LineWidth',2,'Color','#0072BD');
link6_XZ = plot([t6o(1,4) t7o(1,4)], [t6o(3,4) t7o(3,4)],'LineWidth',2,'Color','#D95319');
end_XZ = plot(t7o(1,4),t7o(3,4),'g*','MarkerSize',12);
% Goal
g_baseLink_XZ = plot([t0o(1) t1o(1,4)], [t0o(3) t1o(3,4)],'LineWidth',2,'Color',[0 0 0 0.25]);
g_link1_XZ = plot([t1o(1,4) t2o(1,4)], [t1o(3,4) t2o(3,4)],'LineWidth',2,'Color',[0 0 0 0.25]);
g_link2_XZ = plot([t2o(1,4) t3o(1,4)], [t2o(3,4) t3o(3,4)],'LineWidth',2,'Color',[1 0 1 0.25]);
g_link3_XZ = plot([t3o(1,4) t4o(1,4)], [t3o(3,4) t4o(3,4)],'LineWidth',2,'Color',[0 0 0 0.25]);
g_link4_XZ = plot([t4o(1,4) t5o(1,4)], [t4o(3,4) t5o(3,4)],'LineWidth',2,'Color',[0 0 0 0.25]);
g_link5_XZ = plot([t5o(1,4) t6o(1,4)], [t5o(3,4) t6o(3,4)],'LineWidth',2,'Color',[0 0 0 0.25]);
g_link6_XZ = plot([t6o(1,4) t7o(1,4)], [t6o(3,4) t7o(3,4)],'LineWidth',2,'Color',[1 0 1 0.25]);
g_end_XZ = plot(t7o(1,4),t7o(3,4),'*','MarkerSize',12,'Color',[0.6350 0.0780 0.1840 0.2]);
% Goal Setting
s_baseLink_XZ = plot([t0o(1) t1o(1,4)], [t0o(3) t1o(3,4)],'LineWidth',2,'Color',[1 0 0 0.25]);
s_link1_XZ = plot([t1o(1,4) t2o(1,4)], [t1o(3,4) t2o(3,4)],'LineWidth',2,'Color',[1 0 0 0.25]);
s_link2_XZ = plot([t2o(1,4) t3o(1,4)], [t2o(3,4) t3o(3,4)],'LineWidth',2,'Color',[0 1 0 0.25]);
s_link3_XZ = plot([t3o(1,4) t4o(1,4)], [t3o(3,4) t4o(3,4)],'LineWidth',2,'Color',[1 0 0 0.25]);
s_link4_XZ = plot([t4o(1,4) t5o(1,4)], [t4o(3,4) t5o(3,4)],'LineWidth',2,'Color',[1 0 0 0.25]);
s_link5_XZ = plot([t5o(1,4) t6o(1,4)], [t5o(3,4) t6o(3,4)],'LineWidth',2,'Color',[1 0 0 0.25]);
s_link6_XZ = plot([t6o(1,4) t7o(1,4)], [t6o(3,4) t7o(3,4)],'LineWidth',2,'Color',[0 1 0 0.25]);
% Model XZ
modelXZ = plot(t7o(1,4),t7o(3,4),'o','MarkerSize',12,'Color',[0.47 0.25 0.80 0.5]);
set(plotXZHandle,'Position',[0.05 0.125 0.3 0.4])
axis([-1.2 1.2 -1.2 1.2]);
xlabel('X');ylabel('Z');
% Right(YZ) plot creation (bottom right) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
plotYZHandle = subplot(2,2,4);
grid on; hold on;
% Robot
baseLink_YZ = plot([t0o(2) t1o(2,4)], [t0o(3) t1o(3,4)],'LineWidth',2,'Color','#0072BD');
link1_YZ = plot([t1o(2,4) t2o(2,4)], [t1o(3,4) t2o(3,4)],'LineWidth',2,'Color','#0072BD');
link2_YZ = plot([t2o(2,4) t3o(2,4)], [t2o(3,4) t3o(3,4)],'LineWidth',2,'Color','#D95319');
link3_YZ = plot([t3o(2,4) t4o(2,4)], [t3o(3,4) t4o(3,4)],'LineWidth',2,'Color','#0072BD');
link4_YZ = plot([t4o(2,4) t5o(2,4)], [t4o(3,4) t5o(3,4)],'LineWidth',2,'Color','#0072BD');
link5_YZ = plot([t5o(2,4) t6o(2,4)], [t5o(3,4) t6o(3,4)],'LineWidth',2,'Color','#0072BD');
link6_YZ = plot([t6o(2,4) t7o(2,4)], [t6o(3,4) t7o(3,4)],'LineWidth',2,'Color','#D95319');
end_YZ = plot(t7o(2,4),t7o(3,4),'g*','MarkerSize',12);
% Goal
g_baseLink_YZ = plot([t0o(2) t1o(2,4)], [t0o(3) t1o(3,4)],'LineWidth',2,'Color',[0 0 0 0.25]);
g_link1_YZ = plot([t1o(2,4) t2o(2,4)], [t1o(3,4) t2o(3,4)],'LineWidth',2,'Color',[0 0 0 0.25]);
g_link2_YZ = plot([t2o(2,4) t3o(2,4)], [t2o(3,4) t3o(3,4)],'LineWidth',2,'Color',[1 0 1 0.25]);
g_link3_YZ = plot([t3o(2,4) t4o(2,4)], [t3o(3,4) t4o(3,4)],'LineWidth',2,'Color',[0 0 0 0.25]);
g_link4_YZ = plot([t4o(2,4) t5o(2,4)], [t4o(3,4) t5o(3,4)],'LineWidth',2,'Color',[0 0 0 0.25]);
g_link5_YZ = plot([t5o(2,4) t6o(2,4)], [t5o(3,4) t6o(3,4)],'LineWidth',2,'Color',[0 0 0 0.25]);
g_link6_YZ = plot([t6o(2,4) t7o(2,4)], [t6o(3,4) t7o(3,4)],'LineWidth',2,'Color',[1 0 1 0.25]);
g_end_YZ = plot(t7o(2,4),t7o(3,4),'*','MarkerSize',12,'Color',[0.6350 0.0780 0.1840 0.2]);
% Goal Setting
s_baseLink_YZ = plot([t0o(2) t1o(2,4)], [t0o(3) t1o(3,4)],'LineWidth',2,'Color',[1 0 0 0.25]);
s_link1_YZ = plot([t1o(2,4) t2o(2,4)], [t1o(3,4) t2o(3,4)],'LineWidth',2,'Color',[1 0 0 0.25]);
s_link2_YZ = plot([t2o(2,4) t3o(2,4)], [t2o(3,4) t3o(3,4)],'LineWidth',2,'Color',[0 1 0 0.25]);
s_link3_YZ = plot([t3o(2,4) t4o(2,4)], [t3o(3,4) t4o(3,4)],'LineWidth',2,'Color',[1 0 0 0.25]);
s_link4_YZ = plot([t4o(2,4) t5o(2,4)], [t4o(3,4) t5o(3,4)],'LineWidth',2,'Color',[1 0 0 0.25]);
s_link5_YZ = plot([t5o(2,4) t6o(2,4)], [t5o(3,4) t6o(3,4)],'LineWidth',2,'Color',[1 0 0 0.25]);
s_link6_YZ = plot([t6o(2,4) t7o(2,4)], [t6o(3,4) t7o(3,4)],'LineWidth',2,'Color',[0 1 0 0.25]);
% Model YZ
modelYZ = plot(t7o(2,4),t7o(3,4),'o','MarkerSize',12,'Color',[0.47 0.25 0.80 0.5]);
set(plotYZHandle,'Position',[0.4 0.125 0.3 0.4])
axis([-1.2 1.2 -1.2 1.2 -1.2 1.2]);
xlabel('Y');ylabel('Z');

% UI Controls %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% buttons -----------------------------------------------------------------
HbttnOpenCMStart = uicontrol('Style','pushbutton');
set(HbttnOpenCMStart,'Units','normalized','Position',[0.70 0.95 0.1 0.05],...
    'String','Start OpenCM','Callback',@startOpenCM);
HbttnOpenCMStop = uicontrol('Style','pushbutton');
set(HbttnOpenCMStop,'Units','normalized','Position',[0.80 0.95 0.1 0.05],...
    'String','Stop OpenCM','Callback',@stopOpenCM);
HbttnTeensyStart = uicontrol('Style','pushbutton');
set(HbttnTeensyStart,'Units','normalized','Position',[0.70 0.9 0.1 0.05],...
    'String','Start Teensy','Callback',@startTeensy);
HbttnTeensyStop = uicontrol('Style','pushbutton');
set(HbttnTeensyStop,'Units','normalized','Position',[0.80 0.9 0.1 0.05],...
    'String','Stop Teensy','Callback',@stopTeensy);
HbttnTestGoal = uicontrol('Style','pushbutton');
set(HbttnTestGoal,'Units','normalized','Position',[0.90 0.9 0.05, 0.1],...
    'String','<html>Send<br />Goal</html>','Callback',@sendGoal);

% time --------------------------------------------------------------------
HtimeLabel = uicontrol('Style','text');
set(HtimeLabel,'Units','normalized','Position',[0.70 0.85 0.066 0.05],...
    'String','elapsedT:','Fontsize',18);
HtimeValue = uicontrol('Style','text');
set(HtimeValue,'Units','normalized','Position',[0.766 0.85 0.066 0.05],...
    'String','0','Fontsize',20);

HloopLabel = uicontrol('Style','text');
set(HloopLabel,'Units','normalized','Position',[0.70 0.8 0.066 0.05],...
    'String','loopT:','Fontsize',20);
HloopValue = uicontrol('Style','text');
set(HloopValue,'Units','normalized','Position',[0.766 0.8 0.066 0.05],...
    'String','0','Fontsize',20);

% Drive Mode --------------------------------------------------------------
HdriveModeLabel = uicontrol('Style','text');
set(HdriveModeLabel,'Units','normalized','Position',[0.898 0.85 0.066 0.05],...
    'String','DriveMode','Fontsize',16);
HdriveModeValue = uicontrol('Style','text');
set(HdriveModeValue,'Units','normalized','Position',[0.898 0.8 0.066 0.05],...
    'String','0','Fontsize',20);

% Present Q1, Q2, Q4 ------------------------------------------------------
HalphaLabel = uicontrol('Style','text');
set(HalphaLabel,'Units','normalized','Position',[0.7 0.75 0.066 0.05],...
    'String','rQ1:','Fontsize',20);
Hq1Value = uicontrol('Style','text');
set(Hq1Value,'Units','normalized','Position',[0.766 0.75 0.066 0.05],...
    'String','0','Fontsize',20);
Hq1CtsValue = uicontrol('Style','text');
set(Hq1CtsValue,'Units','normalized','Position',[0.832 0.75 0.066 0.05],...
    'String','0','Fontsize',20,'ForegroundColor',"#7E2F8E");

HbetaLabel = uicontrol('Style','text');
set(HbetaLabel,'Units','normalized','Position',[0.7 0.7 0.066 0.05],...
    'String','rQ2:','Fontsize',20);
Hq2Value = uicontrol('Style','text');
set(Hq2Value,'Units','normalized','Position',[0.766 0.7 0.066 0.05],...
    'String','0','Fontsize',20);
Hq2CtsValue = uicontrol('Style','text');
set(Hq2CtsValue,'Units','normalized','Position',[0.832 0.7 0.066 0.05],...
    'String','0','Fontsize',20,'ForegroundColor',"#7E2F8E");

HthetaLabel = uicontrol('Style','text');
set(HthetaLabel,'Units','normalized','Position',[0.7 0.65 0.066 0.05],...
    'String','rQ4:','Fontsize',20);
Hq4Value = uicontrol('Style','text');
set(Hq4Value,'Units','normalized','Position',[0.766 0.65 0.066 0.05],...
    'String','0','Fontsize',20);
Hq4CtsValue = uicontrol('Style','text');
set(Hq4CtsValue,'Units','normalized','Position',[0.832 0.65 0.066 0.05],...
    'String','0','Fontsize',20,'ForegroundColor',"#7E2F8E");

% Present X, Y, Z ---------------------------------------------------------
HRxLabel = uicontrol('Style','text');
set(HRxLabel,'Units','normalized','Position',[0.7 0.6 0.066 0.05],...
    'String','rX:','Fontsize',20);
HRxValue = uicontrol('Style','text');
set(HRxValue,'Units','normalized','Position',[0.766 0.6 0.066 0.05],...
    'String','0','Fontsize',20);

HRyLabel = uicontrol('Style','text');
set(HRyLabel,'Units','normalized','Position',[0.7 0.55 0.066 0.05],...
    'String','rY:','Fontsize',20);
HRyValue = uicontrol('Style','text');
set(HRyValue,'Units','normalized','Position',[0.766 0.55 0.066 0.05],...
    'String','0','Fontsize',20);

HRzLabel = uicontrol('Style','text');
set(HRzLabel,'Units','normalized','Position',[0.7 0.5 0.066 0.05],...
    'String','rZ:','Fontsize',20);
HRzValue = uicontrol('Style','text');
set(HRzValue,'Units','normalized','Position',[0.766 0.5 0.066 0.05],...
    'String','0','Fontsize',20);

% Goal X, Y, Z ---------------------------------------------------------
HGxLabel = uicontrol('Style','text');
set(HGxLabel,'Units','normalized','Position',[0.7 0.45 0.066 0.05],...
    'String','gX:','Fontsize',20);
HGxValue = uicontrol('Style','text');
set(HGxValue,'Units','normalized','Position',[0.766 0.45 0.066 0.05],...
    'String','0','Fontsize',20);
HSxValue = uicontrol('Style','text');
set(HSxValue,'Units','normalized','Position',[0.832 0.45 0.066 0.05],...
    'String','0','Fontsize',20,'ForegroundColor','#A2142F');


HGyLabel = uicontrol('Style','text');
set(HGyLabel,'Units','normalized','Position',[0.7 0.4 0.066 0.05],...
    'String','gY:','Fontsize',20);
HGyValue = uicontrol('Style','text');
set(HGyValue,'Units','normalized','Position',[0.766 0.4 0.066 0.05],...
    'String','0','Fontsize',20);
HSyValue = uicontrol('Style','text');
set(HSyValue,'Units','normalized','Position',[0.832 0.4 0.066 0.05],...
    'String','0','Fontsize',20,'ForegroundColor','#A2142F');

HGzLabel = uicontrol('Style','text');
set(HGzLabel,'Units','normalized','Position',[0.7 0.35 0.066 0.05],...
    'String','gZ:','Fontsize',20);
HGzValue = uicontrol('Style','text');
set(HGzValue,'Units','normalized','Position',[0.766 0.35 0.066 0.05],...
    'String','0','Fontsize',20);
HSzValue = uicontrol('Style','text');
set(HSzValue,'Units','normalized','Position',[0.832 0.35 0.066 0.05],...
    'String','0','Fontsize',20,'ForegroundColor','#A2142F');

% Goal Q1, Q2, Q4 ------------------------------------------------------
HgoalQ1Label = uicontrol('Style','text');
set(HgoalQ1Label,'Units','normalized','Position',[0.7 0.3 0.066 0.05],...
    'String','gQ1:','Fontsize',20);
HgoalQ1Value = uicontrol('Style','text');
set(HgoalQ1Value,'Units','normalized','Position',[0.766 0.3 0.066 0.05],...
    'String','0','Fontsize',20);
HsQ1Value = uicontrol('Style','text');
set(HsQ1Value,'Units','normalized','Position',[0.832 0.3 0.066 0.05],...
    'String','0','Fontsize',20,'ForegroundColor','#A2142F');
HgoalQ1ctsValue = uicontrol('Style','text');
set(HgoalQ1ctsValue,'Units','normalized','Position',[0.898 0.3 0.066 0.05],...
    'String','0','Fontsize',20,'ForegroundColor','#7E2F8E');

HgoalQ2Label = uicontrol('Style','text');
set(HgoalQ2Label,'Units','normalized','Position',[0.7 0.25 0.066 0.05],...
    'String','gQ2:','Fontsize',20);
HgoalQ2Value = uicontrol('Style','text');
set(HgoalQ2Value,'Units','normalized','Position',[0.766 0.25 0.066 0.05],...
    'String','0','Fontsize',20);
HsQ2Value = uicontrol('Style','text');
set(HsQ2Value,'Units','normalized','Position',[0.832 0.25 0.066 0.05],...
    'String','0','Fontsize',20,'ForegroundColor','#A2142F');
HgoalQ2ctsValue = uicontrol('Style','text');
set(HgoalQ2ctsValue,'Units','normalized','Position',[0.898 0.25 0.066 0.05],...
    'String','0','Fontsize',20,'ForegroundColor','#7E2F8E');

HgoalQ4Label = uicontrol('Style','text');
set(HgoalQ4Label,'Units','normalized','Position',[0.7 0.2 0.066 0.05],...
    'String','gQ4:','Fontsize',20);
HgoalQ4Value = uicontrol('Style','text');
set(HgoalQ4Value,'Units','normalized','Position',[0.766 0.2 0.066 0.05],...
    'String','0','Fontsize',20);
HsQ4Value = uicontrol('Style','text');
set(HsQ4Value,'Units','normalized','Position',[0.832 0.2 0.066 0.05],...
    'String','0','Fontsize',20,'ForegroundColor','#A2142F');
HgoalQ4ctsValue = uicontrol('Style','text');
set(HgoalQ4ctsValue,'Units','normalized','Position',[0.898 0.2 0.066 0.05],...
    'String','0','Fontsize',20,'ForegroundColor','#7E2F8E');

%% Callback Functions

    function startOpenCM(source,eventdata)
        testRunning = get(Hfigure,'UserData');
        if  ~testRunning
            botSerial = CommOpenCM(robotSerialPort, robotBaud);
            fullRobot = false;
            botSerial.Start();
            set(Hfigure,'UserData',true);
            animatedata();
        end
    end

    function stopOpenCM(source,eventdata)
        testRunning = get(Hfigure,'UserData');
        if  testRunning
            set(Hfigure,'UserData',false);
            botSerial.Stop();
            fullRobot = false;
        end
    end

    function startTeensy(source,eventdata)
        testRunning = get(Hfigure,'UserData');
        if  ~testRunning
            botSerial = ArmSupportRobot(robotSerialPort, robotBaud);
            fullRobot = true;
            botSerial.Start();
            set(Hfigure,'UserData',true);
            animatedata();
        end
    end

    function stopTeensy(source,eventdata)
        testRunning = get(Hfigure,'UserData');
        if  testRunning
           set(Hfigure,'UserData',false);
           botSerial.Stop();
           fullRobot = false;
        end
    end

    function sendGoal(source,eventdata)
        test = randi(5);
        while test == idx
            test = randi(5);
        end
        idx = test;
        botSerial.SendGoal(testGoals(idx,1), testGoals(idx,2), testGoals(idx,3), velocityXYZ, velocityXYZ, velocityXYZ);
        set(HSxValue,'String',num2str(testGoals(idx,1)));
        set(HSyValue,'String',num2str(testGoals(idx,2)));
        set(HSzValue,'String',num2str(testGoals(idx,3)));
        set(g_end_XY,'XData',testGoals(idx,1),'YData',testGoals(idx,2));
        set(g_end_3d,'XData',testGoals(idx,1),'YData',testGoals(idx,2),'ZData',testGoals(idx,3));
        set(g_end_XZ,'XData',testGoals(idx,1),'YData',testGoals(idx,3));
        set(g_end_YZ,'XData',testGoals(idx,2),'YData',testGoals(idx,3));
        goalQ = iKine(testGoals(idx,1), testGoals(idx,2),testGoals(idx,3));
        t0_g = [0,0,0];
        t1_g = T01(goalQ(1));
        t2_g = T02(goalQ(1),goalQ(2));
        t3_g = T03(goalQ(1),goalQ(2));
        t4_g = T04(goalQ(1),goalQ(2),goalQ(3));
        t5_g = T05(goalQ(1),goalQ(2),goalQ(3));
        t6_g = T06(goalQ(1),goalQ(2),goalQ(3));
        t7_g = T07(goalQ(1),goalQ(2),goalQ(3));
        % XY Plot update
        set(s_baseLink_XY,'XData',[t0_g(1) t1_g(1,4)],'YData',[t0_g(2) t1_g(2,4)])
        set(s_link1_XY,'XData',[t1_g(1,4) t2_g(1,4)],'YData',[t1_g(2,4) t2_g(2,4)])
        set(s_link2_XY,'XData',[t2_g(1,4) t3_g(1,4)],'YData',[t2_g(2,4) t3_g(2,4)])
        set(s_link3_XY,'XData',[t3_g(1,4) t4_g(1,4)],'YData',[t3_g(2,4) t4_g(2,4)])
        set(s_link4_XY,'XData',[t4_g(1,4) t5_g(1,4)],'YData',[t4_g(2,4) t5_g(2,4)])
        set(s_link5_XY,'XData',[t5_g(1,4) t6_g(1,4)],'YData',[t5_g(2,4) t6_g(2,4)])
        set(s_link6_XY,'XData',[t6_g(1,4) t7_g(1,4)],'YData',[t6_g(2,4) t7_g(2,4)])
        % 3D Plot update
        set(s_baseLink_3d,'XData',[t0_g(1) t1_g(1,4)],'YData',[t0_g(2) t1_g(2,4)],'ZData',[t0_g(3) t1_g(3,4)])
        set(s_link1_3d,'XData',[t1_g(1,4) t2_g(1,4)],'YData',[t1_g(2,4) t2_g(2,4)],'ZData',[t1_g(3,4) t2_g(3,4)])
        set(s_link2_3d,'XData',[t2_g(1,4) t3_g(1,4)],'YData',[t2_g(2,4) t3_g(2,4)],'ZData',[t2_g(3,4) t3_g(3,4)])
        set(s_link3_3d,'XData',[t3_g(1,4) t4_g(1,4)],'YData',[t3_g(2,4) t4_g(2,4)],'ZData',[t3_g(3,4) t4_g(3,4)])
        set(s_link4_3d,'XData',[t4_g(1,4) t5_g(1,4)],'YData',[t4_g(2,4) t5_g(2,4)],'ZData',[t4_g(3,4) t5_g(3,4)])
        set(s_link5_3d,'XData',[t5_g(1,4) t6_g(1,4)],'YData',[t5_g(2,4) t6_g(2,4)],'ZData',[t5_g(3,4) t6_g(3,4)])
        set(s_link6_3d,'XData',[t6_g(1,4) t7_g(1,4)],'YData',[t6_g(2,4) t7_g(2,4)],'ZData',[t6_g(3,4) t7_g(3,4)])
        % XZ Plot Update
        set(s_baseLink_XZ,'XData',[t0_g(1) t1_g(1,4)],'YData',[t0_g(3) t1_g(3,4)])
        set(s_link1_XZ,'XData',[t1_g(1,4) t2_g(1,4)],'YData',[t1_g(3,4) t2_g(3,4)])
        set(s_link2_XZ,'XData',[t2_g(1,4) t3_g(1,4)],'YData',[t2_g(3,4) t3_g(3,4)])
        set(s_link3_XZ,'XData',[t3_g(1,4) t4_g(1,4)],'YData',[t3_g(3,4) t4_g(3,4)])
        set(s_link4_XZ,'XData',[t4_g(1,4) t5_g(1,4)],'YData',[t4_g(3,4) t5_g(3,4)])
        set(s_link5_XZ,'XData',[t5_g(1,4) t6_g(1,4)],'YData',[t5_g(3,4) t6_g(3,4)])
        set(s_link6_XZ,'XData',[t6_g(1,4) t7_g(1,4)],'YData',[t6_g(3,4) t7_g(3,4)])
        % YZ Plot Update
        set(s_baseLink_YZ,'XData',[t0_g(2) t1_g(2,4)],'YData',[t0_g(3) t1_g(3,4)])
        set(s_link1_YZ,'XData',[t1_g(2,4) t2_g(2,4)],'YData',[t1_g(3,4) t2_g(3,4)])
        set(s_link2_YZ,'XData',[t2_g(2,4) t3_g(2,4)],'YData',[t2_g(3,4) t3_g(3,4)])
        set(s_link3_YZ,'XData',[t3_g(2,4) t4_g(2,4)],'YData',[t3_g(3,4) t4_g(3,4)])
        set(s_link4_YZ,'XData',[t4_g(2,4) t5_g(2,4)],'YData',[t4_g(3,4) t5_g(3,4)])
        set(s_link5_YZ,'XData',[t5_g(2,4) t6_g(2,4)],'YData',[t5_g(3,4) t6_g(3,4)])
        set(s_link6_YZ,'XData',[t6_g(2,4) t7_g(2,4)],'YData',[t6_g(3,4) t7_g(3,4)])
        set(HsQ1Value,'String',num2str(goalQ(1)));
        set(HsQ2Value,'String',num2str(goalQ(2)));
        set(HsQ4Value,'String',num2str(goalQ(3)));
    end

%% Other Functions
    function animatedata()
        while true
            drawnow();
            testRunning = get(Hfigure,'UserData');
            if ~isempty(testRunning) && ~testRunning
                break;
            end
            % Data Request
            botSerial.SendRequest();
            while botSerial.BytesAvailable < botSerial.rxPacketLen
            end
            if botSerial.BytesAvailable >= botSerial.rxPacketLen
                ReadFrame(botSerial);
            end
            q1 = botSerial.frameData(2);% + 0.6219; % offset 35.63deg
            q2 = botSerial.frameData(3);
            q4 = botSerial.frameData(4);
            x = botSerial.frameData(8);
            y = botSerial.frameData(9);
            z = botSerial.frameData(10);
            goalQ1 = botSerial.frameData(14);
            goalQ2 = botSerial.frameData(15);
            goalQ4 = botSerial.frameData(16);
            if fullRobot

            else
                q1Cts = botSerial.frameData(20);
                q2Cts = botSerial.frameData(21);
                q4Cts = botSerial.frameData(22);
                gQ1cts = botSerial.frameData(26);
                gQ2cts = botSerial.frameData(27);
                gQ4cts = botSerial.frameData(28);
            end
            
            % ROBOT Kinematics Calculations %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            t0_r = [0,0,0];
            t1_r = T01(q1);
            t2_r = T02(q1,q2);
            t3_r = T03(q1,q2);
            t4_r = T04(q1,q2,q4);
            t5_r = T05(q1,q2,q4);
            t6_r = T06(q1,q2,q4);
            t7_r = T07(q1,q2,q4);
            % XY Plot update
            set(baseLink_XY,'XData',[t0_r(1) t1_r(1,4)],'YData',[t0_r(2) t1_r(2,4)])
            set(link1_XY,'XData',[t1_r(1,4) t2_r(1,4)],'YData',[t1_r(2,4) t2_r(2,4)])
            set(link2_XY,'XData',[t2_r(1,4) t3_r(1,4)],'YData',[t2_r(2,4) t3_r(2,4)])
            set(link3_XY,'XData',[t3_r(1,4) t4_r(1,4)],'YData',[t3_r(2,4) t4_r(2,4)])
            set(link4_XY,'XData',[t4_r(1,4) t5_r(1,4)],'YData',[t4_r(2,4) t5_r(2,4)])
            set(link5_XY,'XData',[t5_r(1,4) t6_r(1,4)],'YData',[t5_r(2,4) t6_r(2,4)])
            set(link6_XY,'XData',[t6_r(1,4) t7_r(1,4)],'YData',[t6_r(2,4) t7_r(2,4)])
            set(end_XY,'XData',x,'YData',y);
            % 3D Plot update
            set(baseLink_3d,'XData',[t0_r(1) t1_r(1,4)],'YData',[t0_r(2) t1_r(2,4)],'ZData',[t0_r(3) t1_r(3,4)])
            set(link1_3d,'XData',[t1_r(1,4) t2_r(1,4)],'YData',[t1_r(2,4) t2_r(2,4)],'ZData',[t1_r(3,4) t2_r(3,4)])
            set(link2_3d,'XData',[t2_r(1,4) t3_r(1,4)],'YData',[t2_r(2,4) t3_r(2,4)],'ZData',[t2_r(3,4) t3_r(3,4)])
            set(link3_3d,'XData',[t3_r(1,4) t4_r(1,4)],'YData',[t3_r(2,4) t4_r(2,4)],'ZData',[t3_r(3,4) t4_r(3,4)])
            set(link4_3d,'XData',[t4_r(1,4) t5_r(1,4)],'YData',[t4_r(2,4) t5_r(2,4)],'ZData',[t4_r(3,4) t5_r(3,4)])
            set(link5_3d,'XData',[t5_r(1,4) t6_r(1,4)],'YData',[t5_r(2,4) t6_r(2,4)],'ZData',[t5_r(3,4) t6_r(3,4)])
            set(link6_3d,'XData',[t6_r(1,4) t7_r(1,4)],'YData',[t6_r(2,4) t7_r(2,4)],'ZData',[t6_r(3,4) t7_r(3,4)])
            set(end_3d,'XData',x,'YData',y,'ZData',z);
            % XZ Plot Update
            set(baseLink_XZ,'XData',[t0_r(1) t1_r(1,4)],'YData',[t0_r(3) t1_r(3,4)])
            set(link1_XZ,'XData',[t1_r(1,4) t2_r(1,4)],'YData',[t1_r(3,4) t2_r(3,4)])
            set(link2_XZ,'XData',[t2_r(1,4) t3_r(1,4)],'YData',[t2_r(3,4) t3_r(3,4)])
            set(link3_XZ,'XData',[t3_r(1,4) t4_r(1,4)],'YData',[t3_r(3,4) t4_r(3,4)])
            set(link4_XZ,'XData',[t4_r(1,4) t5_r(1,4)],'YData',[t4_r(3,4) t5_r(3,4)])
            set(link5_XZ,'XData',[t5_r(1,4) t6_r(1,4)],'YData',[t5_r(3,4) t6_r(3,4)])
            set(link6_XZ,'XData',[t6_r(1,4) t7_r(1,4)],'YData',[t6_r(3,4) t7_r(3,4)])
            set(end_XZ,'XData',x,'YData',z);
            % YZ Plot Update
            set(baseLink_YZ,'XData',[t0_r(2) t1_r(2,4)],'YData',[t0_r(3) t1_r(3,4)])
            set(link1_YZ,'XData',[t1_r(2,4) t2_r(2,4)],'YData',[t1_r(3,4) t2_r(3,4)])
            set(link2_YZ,'XData',[t2_r(2,4) t3_r(2,4)],'YData',[t2_r(3,4) t3_r(3,4)])
            set(link3_YZ,'XData',[t3_r(2,4) t4_r(2,4)],'YData',[t3_r(3,4) t4_r(3,4)])
            set(link4_YZ,'XData',[t4_r(2,4) t5_r(2,4)],'YData',[t4_r(3,4) t5_r(3,4)])
            set(link5_YZ,'XData',[t5_r(2,4) t6_r(2,4)],'YData',[t5_r(3,4) t6_r(3,4)])
            set(link6_YZ,'XData',[t6_r(2,4) t7_r(2,4)],'YData',[t6_r(3,4) t7_r(3,4)])
            set(end_YZ,'XData',y,'YData',z);
            % Values update
            set(Hq1Value,'String',num2str(q1));
            set(Hq2Value,'String',num2str(q2));
            set(Hq4Value,'String',num2str(q4));
            set(HRxValue,'String',num2str(x));
            set(HRyValue,'String',num2str(y));
            set(HRzValue,'String',num2str(z));
            % GOAL Kinematics Calculations %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            t0_g = [0,0,0];
            t1_g = T01(goalQ1);
            t2_g = T02(goalQ1, goalQ2);
            t3_g = T03(goalQ1, goalQ2);
            t4_g = T04(goalQ1, goalQ2, goalQ4);
            t5_g = T05(goalQ1, goalQ2, goalQ4);
            t6_g = T06(goalQ1, goalQ2, goalQ4);
            t7_g = T07(goalQ1, goalQ2, goalQ4);
            % XY Plot update
            set(g_baseLink_XY,'XData',[t0_g(1) t1_g(1,4)],'YData',[t0_g(2) t1_g(2,4)])
            set(g_link1_XY,'XData',[t1_g(1,4) t2_g(1,4)],'YData',[t1_g(2,4) t2_g(2,4)])
            set(g_link2_XY,'XData',[t2_g(1,4) t3_g(1,4)],'YData',[t2_g(2,4) t3_g(2,4)])
            set(g_link3_XY,'XData',[t3_g(1,4) t4_g(1,4)],'YData',[t3_g(2,4) t4_g(2,4)])
            set(g_link4_XY,'XData',[t4_g(1,4) t5_g(1,4)],'YData',[t4_g(2,4) t5_g(2,4)])
            set(g_link5_XY,'XData',[t5_g(1,4) t6_g(1,4)],'YData',[t5_g(2,4) t6_g(2,4)])
            set(g_link6_XY,'XData',[t6_g(1,4) t7_g(1,4)],'YData',[t6_g(2,4) t7_g(2,4)])
            % 3D Plot update
            set(g_baseLink_3d,'XData',[t0_g(1) t1_g(1,4)],'YData',[t0_g(2) t1_g(2,4)],'ZData',[t0_g(3) t1_g(3,4)])
            set(g_link1_3d,'XData',[t1_g(1,4) t2_g(1,4)],'YData',[t1_g(2,4) t2_g(2,4)],'ZData',[t1_g(3,4) t2_g(3,4)])
            set(g_link2_3d,'XData',[t2_g(1,4) t3_g(1,4)],'YData',[t2_g(2,4) t3_g(2,4)],'ZData',[t2_g(3,4) t3_g(3,4)])
            set(g_link3_3d,'XData',[t3_g(1,4) t4_g(1,4)],'YData',[t3_g(2,4) t4_g(2,4)],'ZData',[t3_g(3,4) t4_g(3,4)])
            set(g_link4_3d,'XData',[t4_g(1,4) t5_g(1,4)],'YData',[t4_g(2,4) t5_g(2,4)],'ZData',[t4_g(3,4) t5_g(3,4)])
            set(g_link5_3d,'XData',[t5_g(1,4) t6_g(1,4)],'YData',[t5_g(2,4) t6_g(2,4)],'ZData',[t5_g(3,4) t6_g(3,4)])
            set(g_link6_3d,'XData',[t6_g(1,4) t7_g(1,4)],'YData',[t6_g(2,4) t7_g(2,4)],'ZData',[t6_g(3,4) t7_g(3,4)])
            % XZ Plot Update
            set(g_baseLink_XZ,'XData',[t0_g(1) t1_g(1,4)],'YData',[t0_g(3) t1_g(3,4)])
            set(g_link1_XZ,'XData',[t1_g(1,4) t2_g(1,4)],'YData',[t1_g(3,4) t2_g(3,4)])
            set(g_link2_XZ,'XData',[t2_g(1,4) t3_g(1,4)],'YData',[t2_g(3,4) t3_g(3,4)])
            set(g_link3_XZ,'XData',[t3_g(1,4) t4_g(1,4)],'YData',[t3_g(3,4) t4_g(3,4)])
            set(g_link4_XZ,'XData',[t4_g(1,4) t5_g(1,4)],'YData',[t4_g(3,4) t5_g(3,4)])
            set(g_link5_XZ,'XData',[t5_g(1,4) t6_g(1,4)],'YData',[t5_g(3,4) t6_g(3,4)])
            set(g_link6_XZ,'XData',[t6_g(1,4) t7_g(1,4)],'YData',[t6_g(3,4) t7_g(3,4)])
            % YZ Plot Update
            set(g_baseLink_YZ,'XData',[t0_g(2) t1_g(2,4)],'YData',[t0_g(3) t1_g(3,4)])
            set(g_link1_YZ,'XData',[t1_g(2,4) t2_g(2,4)],'YData',[t1_g(3,4) t2_g(3,4)])
            set(g_link2_YZ,'XData',[t2_g(2,4) t3_g(2,4)],'YData',[t2_g(3,4) t3_g(3,4)])
            set(g_link3_YZ,'XData',[t3_g(2,4) t4_g(2,4)],'YData',[t3_g(3,4) t4_g(3,4)])
            set(g_link4_YZ,'XData',[t4_g(2,4) t5_g(2,4)],'YData',[t4_g(3,4) t5_g(3,4)])
            set(g_link5_YZ,'XData',[t5_g(2,4) t6_g(2,4)],'YData',[t5_g(3,4) t6_g(3,4)])
            set(g_link6_YZ,'XData',[t6_g(2,4) t7_g(2,4)],'YData',[t6_g(3,4) t7_g(3,4)])
            % Values update
            set(HgoalQ1Value,'String',num2str(goalQ1));
            set(HgoalQ2Value,'String',num2str(goalQ2));
            set(HgoalQ4Value,'String',num2str(goalQ4));
            set(HGxValue,'String',num2str(t7_g(1,4)));
            set(HGyValue,'String',num2str(t7_g(2,4)));
            set(HGzValue,'String',num2str(t7_g(3,4)));
            % Time parameters
            set(HtimeValue,'String',num2str(botSerial.frameData(1)));
            set(HloopValue,'String',num2str(botSerial.frameData(24)));
            set(HdriveModeValue,'String',num2str(botSerial.frameData(23)));
            % Differences between packets from each MCU.
            if fullRobot

            else
                set(HgoalQ1ctsValue,'String',num2str(gQ1cts));
                set(HgoalQ2ctsValue,'String',num2str(gQ2cts));
                set(HgoalQ4ctsValue,'String',num2str(gQ4cts));
                set(Hq1CtsValue,'String',num2str(q1Cts));
                set(Hq2CtsValue,'String',num2str(q2Cts));
                set(Hq4CtsValue,'String',num2str(q4Cts));
            end
        end
    end

    function Q = iKine(x,y,z)
        Q = [0,0,0];
        Q(2) = Q2(z);
        tempL1xy = L1xy(z);
        tempR = R(x,y);
        tempAlpha = findAlpha(x,y);
        tempGamma = Gamma(x,y,tempL1xy);
        Q(3) = Q4(tempGamma);
        tempBeta = Beta(tempGamma,tempR);
        Q(1) = Q1(tempAlpha,tempBeta);
        if Q(1) < 0
            Q(1) = Q(1) + 2*pi;
        end
    end

end