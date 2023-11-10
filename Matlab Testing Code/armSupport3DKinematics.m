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
idx = 5;

%% Calculate
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

t1 = T01(0);
t2 = T02(0,0);
t3 = T03(0,0);
t4 = T04(0,0,ELBW_LIMIT(1));
t5 = T05(0,0,ELBW_LIMIT(1));
t6 = T06(0,0,ELBW_LIMIT(1));
t7 = T07(0,0,ELBW_LIMIT(1));

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

Q = [0,0,0];

%% Serial Comm
robotSerialPort = 'COM7';
robotBaud = 115200;
botSerial = CommOpenCM(robotSerialPort, robotBaud);
packetLen = 90;

%% GUI
% Initial Values for lines
t0 = [0,0,0];
t1 = T01(0);
t2 = T02(0,0);
t3 = T03(0,0);
t4 = T04(0,0,deg2rad(ELBW_LIMIT(1)));
t5 = T05(0,0,deg2rad(ELBW_LIMIT(1)));
t6 = T06(0,0,deg2rad(ELBW_LIMIT(1)));
t7 = T07(0,0,deg2rad(ELBW_LIMIT(1)));
Hfigure = figure(100);
set(Hfigure,'Units','normalized','Position',[0 0 1 1])
set(Hfigure,'UserData',false);
% set(Hfigure,'CloseRequestFcn',@closereq);
% Top(XY) plot creation (top right) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
plotXYHandle = subplot(2,2,1);
grid on; hold on;
% Robot
baseLink_XY = plot([t0(1) t1(1,4)], [t0(2) t1(2,4)],'LineWidth',2,'Color','#0072BD');
link1_XY = plot([t1(1,4) t2(1,4)], [t1(2,4) t2(2,4)],'LineWidth',2,'Color','#0072BD');
link2_XY = plot([t2(1,4) t3(1,4)], [t2(2,4) t3(2,4)],'LineWidth',2,'Color','#D95319');
link3_XY = plot([t3(1,4) t4(1,4)], [t3(2,4) t4(2,4)],'LineWidth',2,'Color','#0072BD');
link4_XY = plot([t4(1,4) t5(1,4)], [t4(2,4) t5(2,4)],'LineWidth',2,'Color','#0072BD');
link5_XY = plot([t5(1,4) t6(1,4)], [t5(2,4) t6(2,4)],'LineWidth',2,'Color','#0072BD');
link6_XY = plot([t6(1,4) t7(1,4)], [t6(2,4) t7(2,4)],'LineWidth',2,'Color','#D95319');
end_XY = plot(t7(1,4),t7(2,4),'g*','MarkerSize',12);
% Goal
g_baseLink_XY = plot([t0(1) t1(1,4)], [t0(2) t1(2,4)],'LineWidth',2,'Color',[0 0 0 0.1]);
g_link1_XY = plot([t1(1,4) t2(1,4)], [t1(2,4) t2(2,4)],'LineWidth',2,'Color',[0 0 0 0.1]);
g_link2_XY = plot([t2(1,4) t3(1,4)], [t2(2,4) t3(2,4)],'LineWidth',2,'Color',[1 0 1 0.1]);
g_link3_XY = plot([t3(1,4) t4(1,4)], [t3(2,4) t4(2,4)],'LineWidth',2,'Color',[0 0 0 0.1]);
g_link4_XY = plot([t4(1,4) t5(1,4)], [t4(2,4) t5(2,4)],'LineWidth',2,'Color',[0 0 0 0.1]);
g_link5_XY = plot([t5(1,4) t6(1,4)], [t5(2,4) t6(2,4)],'LineWidth',2,'Color',[0 0 0 0.1]);
g_link6_XY = plot([t6(1,4) t7(1,4)], [t6(2,4) t7(2,4)],'LineWidth',2,'Color',[1 0 1 0.1]);
g_end_XY = plot(t7(1,4),t7(2,4),'*','MarkerSize',12,'Color',[0.6350 0.0780 0.1840 0.2]);
set(plotXYHandle,'Position',[0.05 0.575 0.3 0.4])
axis([-1.2 1.2 -1.2 1.2]);
xlabel('X');ylabel('Y');
% 3D plot creation (top left) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
plot3DHandle = subplot(2,2,2);
grid on; hold on;
% Robot
baseLink_3d = plot3([t0(1) t1(1,4)], [t0(2) t1(2,4)], [t0(3) t1(3,4)],'LineWidth',2,'Color','#0072BD');
link1_3d = plot3([t1(1,4) t2(1,4)], [t1(2,4) t2(2,4)], [t1(3,4) t2(3,4)],'LineWidth',2,'Color','#0072BD');
link2_3d = plot3([t2(1,4) t3(1,4)], [t2(2,4) t3(2,4)], [t2(3,4) t3(3,4)],'LineWidth',2,'Color','#D95319');
link3_3d = plot3([t3(1,4) t4(1,4)], [t3(2,4) t4(2,4)], [t3(3,4) t4(3,4)],'LineWidth',2,'Color','#0072BD');
link4_3d = plot3([t4(1,4) t5(1,4)], [t4(2,4) t5(2,4)], [t4(3,4) t5(3,4)],'LineWidth',2,'Color','#0072BD');
link5_3d = plot3([t5(1,4) t6(1,4)], [t5(2,4) t6(2,4)], [t5(3,4) t6(3,4)],'LineWidth',2,'Color','#0072BD');
link6_3d = plot3([t6(1,4) t7(1,4)], [t6(2,4) t7(2,4)], [t6(3,4) t7(3,4)],'LineWidth',2,'Color','#D95319');
end_3d = plot3(t7(1,4),t7(2,4),t7(3,4),'g*','MarkerSize',12);
% Goal
g_baseLink_3d = plot3([t0(1) t1(1,4)], [t0(2) t1(2,4)], [t0(3) t1(3,4)],'LineWidth',2,'Color',[0 0 0 0.1]);
g_link1_3d = plot3([t1(1,4) t2(1,4)], [t1(2,4) t2(2,4)], [t1(3,4) t2(3,4)],'LineWidth',2,'Color',[0 0 0 0.1]);
g_link2_3d = plot3([t2(1,4) t3(1,4)], [t2(2,4) t3(2,4)], [t2(3,4) t3(3,4)],'LineWidth',2,'Color',[1 0 1 0.1]);
g_link3_3d = plot3([t3(1,4) t4(1,4)], [t3(2,4) t4(2,4)], [t3(3,4) t4(3,4)],'LineWidth',2,'Color',[0 0 0 0.1]);
g_link4_3d = plot3([t4(1,4) t5(1,4)], [t4(2,4) t5(2,4)], [t4(3,4) t5(3,4)],'LineWidth',2,'Color',[0 0 0 0.1]);
g_link5_3d = plot3([t5(1,4) t6(1,4)], [t5(2,4) t6(2,4)], [t5(3,4) t6(3,4)],'LineWidth',2,'Color',[0 0 0 0.1]);
g_link6_3d = plot3([t6(1,4) t7(1,4)], [t6(2,4) t7(2,4)], [t6(3,4) t7(3,4)],'LineWidth',2,'Color',[1 0 1 0.1]);
g_end_3d = plot3(t7(1,4),t7(2,4),t7(3,4),'*','MarkerSize',12,'Color',[0.6350 0.0780 0.1840 0.2]);
set(plot3DHandle,'Position',[0.4 0.575 0.3 0.4])
set(plot3DHandle,'View',[45,45])
axis([-1.2 1.2 -1.2 1.2 -1.2 1.2]);
xlabel('X');ylabel('Y');zlabel('Z');
% Front(XZ) plot creation (bottom left) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
plotXZHandle = subplot(2,2,3);
grid on; hold on;
% Robot
baseLink_XZ = plot([t0(1) t1(1,4)], [t0(3) t1(3,4)],'LineWidth',2,'Color','#0072BD');
link1_XZ = plot([t1(1,4) t2(1,4)], [t1(3,4) t2(3,4)],'LineWidth',2,'Color','#0072BD');
link2_XZ = plot([t2(1,4) t3(1,4)], [t2(3,4) t3(3,4)],'LineWidth',2,'Color','#D95319');
link3_XZ = plot([t3(1,4) t4(1,4)], [t3(3,4) t4(3,4)],'LineWidth',2,'Color','#0072BD');
link4_XZ = plot([t4(1,4) t5(1,4)], [t4(3,4) t5(3,4)],'LineWidth',2,'Color','#0072BD');
link5_XZ = plot([t5(1,4) t6(1,4)], [t5(3,4) t6(3,4)],'LineWidth',2,'Color','#0072BD');
link6_XZ = plot([t6(1,4) t7(1,4)], [t6(3,4) t7(3,4)],'LineWidth',2,'Color','#D95319');
end_XZ = plot(t7(1,4),t7(3,4),'g*','MarkerSize',12);
% Goal
g_baseLink_XZ = plot([t0(1) t1(1,4)], [t0(3) t1(3,4)],'LineWidth',2,'Color',[0 0 0 0.1]);
g_link1_XZ = plot([t1(1,4) t2(1,4)], [t1(3,4) t2(3,4)],'LineWidth',2,'Color',[0 0 0 0.1]);
g_link2_XZ = plot([t2(1,4) t3(1,4)], [t2(3,4) t3(3,4)],'LineWidth',2,'Color',[1 0 1 0.1]);
g_link3_XZ = plot([t3(1,4) t4(1,4)], [t3(3,4) t4(3,4)],'LineWidth',2,'Color',[0 0 0 0.1]);
g_link4_XZ = plot([t4(1,4) t5(1,4)], [t4(3,4) t5(3,4)],'LineWidth',2,'Color',[0 0 0 0.1]);
g_link5_XZ = plot([t5(1,4) t6(1,4)], [t5(3,4) t6(3,4)],'LineWidth',2,'Color',[0 0 0 0.1]);
g_link6_XZ = plot([t6(1,4) t7(1,4)], [t6(3,4) t7(3,4)],'LineWidth',2,'Color',[1 0 1 0.1]);
g_end_XZ = plot(t7(1,4),t7(3,4),'*','MarkerSize',12,'Color',[0.6350 0.0780 0.1840 0.2]);
set(plotXZHandle,'Position',[0.05 0.125 0.3 0.4])
axis([-1.2 1.2 -1.2 1.2]);
xlabel('X');ylabel('Z');
% Right(YZ) plot creation (bottom right) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
plotYZHandle = subplot(2,2,4);
grid on; hold on;
% Robot
baseLink_YZ = plot([t0(2) t1(2,4)], [t0(3) t1(3,4)],'LineWidth',2,'Color','#0072BD');
link1_YZ = plot([t1(2,4) t2(2,4)], [t1(3,4) t2(3,4)],'LineWidth',2,'Color','#0072BD');
link2_YZ = plot([t2(2,4) t3(2,4)], [t2(3,4) t3(3,4)],'LineWidth',2,'Color','#D95319');
link3_YZ = plot([t3(2,4) t4(2,4)], [t3(3,4) t4(3,4)],'LineWidth',2,'Color','#0072BD');
link4_YZ = plot([t4(2,4) t5(2,4)], [t4(3,4) t5(3,4)],'LineWidth',2,'Color','#0072BD');
link5_YZ = plot([t5(2,4) t6(2,4)], [t5(3,4) t6(3,4)],'LineWidth',2,'Color','#0072BD');
link6_YZ = plot([t6(2,4) t7(2,4)], [t6(3,4) t7(3,4)],'LineWidth',2,'Color','#D95319');
end_YZ = plot(t7(2,4),t7(3,4),'g*','MarkerSize',12);
% Goal
g_baseLink_YZ = plot([t0(2) t1(2,4)], [t0(3) t1(3,4)],'LineWidth',2,'Color',[0 0 0 0.1]);
g_link1_YZ = plot([t1(2,4) t2(2,4)], [t1(3,4) t2(3,4)],'LineWidth',2,'Color',[0 0 0 0.1]);
g_link2_YZ = plot([t2(2,4) t3(2,4)], [t2(3,4) t3(3,4)],'LineWidth',2,'Color',[1 0 1 0.1]);
g_link3_YZ = plot([t3(2,4) t4(2,4)], [t3(3,4) t4(3,4)],'LineWidth',2,'Color',[0 0 0 0.1]);
g_link4_YZ = plot([t4(2,4) t5(2,4)], [t4(3,4) t5(3,4)],'LineWidth',2,'Color',[0 0 0 0.1]);
g_link5_YZ = plot([t5(2,4) t6(2,4)], [t5(3,4) t6(3,4)],'LineWidth',2,'Color',[0 0 0 0.1]);
g_link6_YZ = plot([t6(2,4) t7(2,4)], [t6(3,4) t7(3,4)],'LineWidth',2,'Color',[1 0 1 0.1]);
g_end_YZ = plot(t7(2,4),t7(3,4),'*','MarkerSize',12,'Color',[0.6350 0.0780 0.1840 0.2]);
set(plotYZHandle,'Position',[0.4 0.125 0.3 0.4])
axis([-1.2 1.2 -1.2 1.2 -1.2 1.2]);
xlabel('Y');ylabel('Z');
iKine(t7(1,4),t7(2,4),t7(3,4));

% UI Controls %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% buttons -----------------------------------------------------------------
HbttnOpenCMStart = uicontrol('Style','pushbutton');
set(HbttnOpenCMStart,'Units','normalized','Position',[0.75 0.95 0.1 0.05],...
    'String','Start OpenCM','Callback',@startOpenCM);
HbttnOpenCMStop = uicontrol('Style','pushbutton');
set(HbttnOpenCMStop,'Units','normalized','Position',[0.85 0.95 0.1 0.05],...
    'String','Stop OpenCM','Callback',@stopOpenCM);
HbttnTeensyStart = uicontrol('Style','pushbutton');
set(HbttnTeensyStart,'Units','normalized','Position',[0.75 0.9 0.1 0.05],...
    'String','Start Teensy','Callback',@startTeensy);
HbttnTeensyStop = uicontrol('Style','pushbutton');
set(HbttnTeensyStop,'Units','normalized','Position',[0.85 0.9 0.1 0.05],...
    'String','Stop Teensy','Callback',@stopTeensy);
HbttnTestGoal = uicontrol('Style','pushbutton');
set(HbttnTestGoal,'Units','normalized','Position',[0.96 0.9 0.03, 0.1],...
    'String','<html>Send<br />Goal</html>','Callback',@sendGoal);

% time --------------------------------------------------------------------
HtimeLabel = uicontrol('Style','text');
set(HtimeLabel,'Units','normalized','Position',[0.75 0.85 0.1 0.05],...
    'String','elapsedT:','Fontsize',20);
HtimeValue = uicontrol('Style','text');
set(HtimeValue,'Units','normalized','Position',[0.85 0.85 0.1 0.05],...
    'String','0','Fontsize',20);

HloopLabel = uicontrol('Style','text');
set(HloopLabel,'Units','normalized','Position',[0.75 0.8 0.1 0.05],...
    'String','loopT:','Fontsize',20);
HloopValue = uicontrol('Style','text');
set(HloopValue,'Units','normalized','Position',[0.85 0.8 0.1 0.05],...
    'String','0','Fontsize',20);

% Present Q1, Q2, Q4 ------------------------------------------------------
HalphaLabel = uicontrol('Style','text');
set(HalphaLabel,'Units','normalized','Position',[0.75 0.75 0.1 0.05],...
    'String','rQ1:','Fontsize',20);
Hq1Value = uicontrol('Style','text');
set(Hq1Value,'Units','normalized','Position',[0.85 0.75 0.1 0.05],...
    'String','0','Fontsize',20);

HbetaLabel = uicontrol('Style','text');
set(HbetaLabel,'Units','normalized','Position',[0.75 0.7 0.1 0.05],...
    'String','rQ2:','Fontsize',20);
Hq2Value = uicontrol('Style','text');
set(Hq2Value,'Units','normalized','Position',[0.85 0.7 0.1 0.05],...
    'String','0','Fontsize',20);

HthetaLabel = uicontrol('Style','text');
set(HthetaLabel,'Units','normalized','Position',[0.75 0.65 0.1 0.05],...
    'String','rQ4:','Fontsize',20);
Hq4Value = uicontrol('Style','text');
set(Hq4Value,'Units','normalized','Position',[0.85 0.65 0.1 0.05],...
    'String','0','Fontsize',20);

% Present X, Y, Z ---------------------------------------------------------
HRxLabel = uicontrol('Style','text');
set(HRxLabel,'Units','normalized','Position',[0.75 0.6 0.1 0.05],...
    'String','rX:','Fontsize',20);
HRxValue = uicontrol('Style','text');
set(HRxValue,'Units','normalized','Position',[0.85 0.6 0.1 0.05],...
    'String','0','Fontsize',20);

HRyLabel = uicontrol('Style','text');
set(HRyLabel,'Units','normalized','Position',[0.75 0.55 0.1 0.05],...
    'String','rY:','Fontsize',20);
HRyValue = uicontrol('Style','text');
set(HRyValue,'Units','normalized','Position',[0.85 0.55 0.1 0.05],...
    'String','0','Fontsize',20);

HRzLabel = uicontrol('Style','text');
set(HRzLabel,'Units','normalized','Position',[0.75 0.5 0.1 0.05],...
    'String','rZ:','Fontsize',20);
HRzValue = uicontrol('Style','text');
set(HRzValue,'Units','normalized','Position',[0.85 0.5 0.1 0.05],...
    'String','0','Fontsize',20);

% Goal X, Y, Z ---------------------------------------------------------
HGxLabel = uicontrol('Style','text');
set(HGxLabel,'Units','normalized','Position',[0.75 0.45 0.066 0.05],...
    'String','gX:','Fontsize',20);
HGxValue = uicontrol('Style','text');
set(HGxValue,'Units','normalized','Position',[0.816 0.45 0.066 0.05],...
    'String','0','Fontsize',20);
HSxValue = uicontrol('Style','text');
set(HSxValue,'Units','normalized','Position',[0.882 0.45 0.066 0.05],...
    'String','0','Fontsize',20,'ForegroundColor','#A2142F');


HGyLabel = uicontrol('Style','text');
set(HGyLabel,'Units','normalized','Position',[0.75 0.4 0.066 0.05],...
    'String','gY:','Fontsize',20);
HGyValue = uicontrol('Style','text');
set(HGyValue,'Units','normalized','Position',[0.816 0.4 0.066 0.05],...
    'String','0','Fontsize',20);
HSyValue = uicontrol('Style','text');
set(HSyValue,'Units','normalized','Position',[0.882 0.4 0.066 0.05],...
    'String','0','Fontsize',20,'ForegroundColor','#A2142F');

HGzLabel = uicontrol('Style','text');
set(HGzLabel,'Units','normalized','Position',[0.75 0.35 0.066 0.05],...
    'String','gZ:','Fontsize',20);
HGzValue = uicontrol('Style','text');
set(HGzValue,'Units','normalized','Position',[0.816 0.35 0.066 0.05],...
    'String','0','Fontsize',20);
HSzValue = uicontrol('Style','text');
set(HSzValue,'Units','normalized','Position',[0.882 0.35 0.066 0.05],...
    'String','0','Fontsize',20,'ForegroundColor','#A2142F');

% Goal Q1, Q2, Q4 ------------------------------------------------------
HgoalQ1Label = uicontrol('Style','text');
set(HgoalQ1Label,'Units','normalized','Position',[0.75 0.3 0.1 0.05],...
    'String','gQ1:','Fontsize',20);
HgoalQ1Value = uicontrol('Style','text');
set(HgoalQ1Value,'Units','normalized','Position',[0.85 0.3 0.1 0.05],...
    'String','0','Fontsize',20);

HgoalQ2Label = uicontrol('Style','text');
set(HgoalQ2Label,'Units','normalized','Position',[0.75 0.25 0.1 0.05],...
    'String','gQ2:','Fontsize',20);
HgoalQ2Value = uicontrol('Style','text');
set(HgoalQ2Value,'Units','normalized','Position',[0.85 0.25 0.1 0.05],...
    'String','0','Fontsize',20);

HgoalQ4Label = uicontrol('Style','text');
set(HgoalQ4Label,'Units','normalized','Position',[0.75 0.2 0.1 0.05],...
    'String','gQ4:','Fontsize',20);
HgoalQ4Value = uicontrol('Style','text');
set(HgoalQ4Value,'Units','normalized','Position',[0.85 0.2 0.1 0.05],...
    'String','0','Fontsize',20);

%% Callback Functions
%     function closereq(source,eventdata)
%         testRunning = get(Hfigure,'UserData');
%         if  testRunning
%             botSerial.Stop();
%         end
%     end

    function startOpenCM(source,eventdata)
        testRunning = get(Hfigure,'UserData');
        if  ~testRunning
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
        end
    end

    function startTeensy(source,eventdata)
        testRunning = get(Hfigure,'UserData');
        if  ~testRunning
            
        end
    end

    function stopTeensy(source,eventdata)
        testRunning = get(Hfigure,'UserData');
        if  testRunning
           
        end
    end

    function sendGoal(source,eventdata)
        if idx >= 5
            idx = 1;
        else
            idx = idx + 1;
        end
        botSerial.SendGoal(testGoals(idx,1), testGoals(idx,2), testGoals(idx,3), 0, 0, 0);
        set(HSxValue,'String',num2str(testGoals(idx,1)));
        set(HSyValue,'String',num2str(testGoals(idx,2)));
        set(HSzValue,'String',num2str(testGoals(idx,3)));
        set(g_end_XY,'XData',testGoals(idx,1),'YData',testGoals(idx,2));
        set(g_end_3d,'XData',testGoals(idx,1),'YData',testGoals(idx,2),'ZData',testGoals(idx,3));
        set(g_end_XZ,'XData',testGoals(idx,1),'YData',testGoals(idx,3));
        set(g_end_YZ,'XData',testGoals(idx,2),'YData',testGoals(idx,3));
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
            while botSerial.BytesAvailable < packetLen
            end
            if botSerial.BytesAvailable >= packetLen
                ReadFrame(botSerial);
            end
            q1 = botSerial.frameData(2);% + 0.6219; % offset 35.63deg
            q2 = botSerial.frameData(3);
            q4 = botSerial.frameData(4);
            x = botSerial.frameData(11);
            y = botSerial.frameData(12);
            z = botSerial.frameData(13);
            goalQ1 = botSerial.frameData(17);
            goalQ2 = botSerial.frameData(18);
            goalQ4 = botSerial.frameData(19);
            % ROBOT Kinematics Calculations %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            t0 = [0,0,0];
            t1 = T01(q1);
            t2 = T02(q1,q2);
            t3 = T03(q1,q2);
            t4 = T04(q1,q2,q4);
            t5 = T05(q1,q2,q4);
            t6 = T06(q1,q2,q4);
            t7 = T07(q1,q2,q4);
            % XY Plot update
            set(baseLink_XY,'XData',[t0(1) t1(1,4)],'YData',[t0(2) t1(2,4)])
            set(link1_XY,'XData',[t1(1,4) t2(1,4)],'YData',[t1(2,4) t2(2,4)])
            set(link2_XY,'XData',[t2(1,4) t3(1,4)],'YData',[t2(2,4) t3(2,4)])
            set(link3_XY,'XData',[t3(1,4) t4(1,4)],'YData',[t3(2,4) t4(2,4)])
            set(link4_XY,'XData',[t4(1,4) t5(1,4)],'YData',[t4(2,4) t5(2,4)])
            set(link5_XY,'XData',[t5(1,4) t6(1,4)],'YData',[t5(2,4) t6(2,4)])
            set(link6_XY,'XData',[t6(1,4) t7(1,4)],'YData',[t6(2,4) t7(2,4)])
            set(end_XY,'XData',x,'YData',y);
            % 3D Plot update
            set(baseLink_3d,'XData',[t0(1) t1(1,4)],'YData',[t0(2) t1(2,4)],'ZData',[t0(3) t1(3,4)])
            set(link1_3d,'XData',[t1(1,4) t2(1,4)],'YData',[t1(2,4) t2(2,4)],'ZData',[t1(3,4) t2(3,4)])
            set(link2_3d,'XData',[t2(1,4) t3(1,4)],'YData',[t2(2,4) t3(2,4)],'ZData',[t2(3,4) t3(3,4)])
            set(link3_3d,'XData',[t3(1,4) t4(1,4)],'YData',[t3(2,4) t4(2,4)],'ZData',[t3(3,4) t4(3,4)])
            set(link4_3d,'XData',[t4(1,4) t5(1,4)],'YData',[t4(2,4) t5(2,4)],'ZData',[t4(3,4) t5(3,4)])
            set(link5_3d,'XData',[t5(1,4) t6(1,4)],'YData',[t5(2,4) t6(2,4)],'ZData',[t5(3,4) t6(3,4)])
            set(link6_3d,'XData',[t6(1,4) t7(1,4)],'YData',[t6(2,4) t7(2,4)],'ZData',[t6(3,4) t7(3,4)])
            set(end_3d,'XData',x,'YData',y,'ZData',z);
            % XZ Plot Update
            set(baseLink_XZ,'XData',[t0(1) t1(1,4)],'YData',[t0(3) t1(3,4)])
            set(link1_XZ,'XData',[t1(1,4) t2(1,4)],'YData',[t1(3,4) t2(3,4)])
            set(link2_XZ,'XData',[t2(1,4) t3(1,4)],'YData',[t2(3,4) t3(3,4)])
            set(link3_XZ,'XData',[t3(1,4) t4(1,4)],'YData',[t3(3,4) t4(3,4)])
            set(link4_XZ,'XData',[t4(1,4) t5(1,4)],'YData',[t4(3,4) t5(3,4)])
            set(link5_XZ,'XData',[t5(1,4) t6(1,4)],'YData',[t5(3,4) t6(3,4)])
            set(link6_XZ,'XData',[t6(1,4) t7(1,4)],'YData',[t6(3,4) t7(3,4)])
            set(end_XZ,'XData',x,'YData',z);
            % YZ Plot Update
            set(baseLink_YZ,'XData',[t0(2) t1(2,4)],'YData',[t0(3) t1(3,4)])
            set(link1_YZ,'XData',[t1(2,4) t2(2,4)],'YData',[t1(3,4) t2(3,4)])
            set(link2_YZ,'XData',[t2(2,4) t3(2,4)],'YData',[t2(3,4) t3(3,4)])
            set(link3_YZ,'XData',[t3(2,4) t4(2,4)],'YData',[t3(3,4) t4(3,4)])
            set(link4_YZ,'XData',[t4(2,4) t5(2,4)],'YData',[t4(3,4) t5(3,4)])
            set(link5_YZ,'XData',[t5(2,4) t6(2,4)],'YData',[t5(3,4) t6(3,4)])
            set(link6_YZ,'XData',[t6(2,4) t7(2,4)],'YData',[t6(3,4) t7(3,4)])
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
            set(HloopValue,'String',num2str(botSerial.frameData(21)));
        end
    end

    function iKine(x,y,z)
        Q(2) = Q2(z);
        tempL1xy = L1xy(z);
        tempR = R(x,y);
        tempAlpha = findAlpha(x,y);
        tempGamma = Gamma(x,y,tempL1xy);
        Q(3) = Q4(tempGamma);
        tempBeta = Beta(tempGamma,tempR);
        Q(1) = Q1(tempAlpha,tempBeta);
    end

end