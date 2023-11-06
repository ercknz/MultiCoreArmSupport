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
packetLen = 80;

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
% Top(XY) plot creation (top right) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
plotXYHandle = subplot(2,2,1);
grid on; hold on;
baseLink_XY = plot([t0(1) t1(1,4)], [t0(2) t1(2,4)],'LineWidth',2,'Color','#0072BD');
link1_XY = plot([t1(1,4) t2(1,4)], [t1(2,4) t2(2,4)],'LineWidth',2,'Color','#0072BD');
link2_XY = plot([t2(1,4) t3(1,4)], [t2(2,4) t3(2,4)],'LineWidth',2,'Color','#D95319');
link3_XY = plot([t3(1,4) t4(1,4)], [t3(2,4) t4(2,4)],'LineWidth',2,'Color','#0072BD');
link4_XY = plot([t4(1,4) t5(1,4)], [t4(2,4) t5(2,4)],'LineWidth',2,'Color','#0072BD');
link5_XY = plot([t5(1,4) t6(1,4)], [t5(2,4) t6(2,4)],'LineWidth',2,'Color','#0072BD');
link6_XY = plot([t6(1,4) t7(1,4)], [t6(2,4) t7(2,4)],'LineWidth',2,'Color','#D95319');
set(plotXYHandle,'Position',[0.05 0.575 0.3 0.4])
axis([-1.2 1.2 -1.2 1.2]);
xlabel('X');ylabel('Y');
% 3D plot creation (top left) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
plot3DHandle = subplot(2,2,2);
grid on; hold on;
baseLink_3d = plot3([t0(1) t1(1,4)], [t0(2) t1(2,4)], [t0(3) t1(3,4)],'LineWidth',2,'Color','#0072BD');
link1_3d = plot3([t1(1,4) t2(1,4)], [t1(2,4) t2(2,4)], [t1(3,4) t2(3,4)],'LineWidth',2,'Color','#0072BD');
link2_3d = plot3([t2(1,4) t3(1,4)], [t2(2,4) t3(2,4)], [t2(3,4) t3(3,4)],'LineWidth',2,'Color','#D95319');
link3_3d = plot3([t3(1,4) t4(1,4)], [t3(2,4) t4(2,4)], [t3(3,4) t4(3,4)],'LineWidth',2,'Color','#0072BD');
link4_3d = plot3([t4(1,4) t5(1,4)], [t4(2,4) t5(2,4)], [t4(3,4) t5(3,4)],'LineWidth',2,'Color','#0072BD');
link5_3d = plot3([t5(1,4) t6(1,4)], [t5(2,4) t6(2,4)], [t5(3,4) t6(3,4)],'LineWidth',2,'Color','#0072BD');
link6_3d = plot3([t6(1,4) t7(1,4)], [t6(2,4) t7(2,4)], [t6(3,4) t7(3,4)],'LineWidth',2,'Color','#D95319');
set(plot3DHandle,'Position',[0.4 0.575 0.3 0.4])
set(plot3DHandle,'View',[45,45])
axis([-1.2 1.2 -1.2 1.2 -1.2 1.2]);
xlabel('X');ylabel('Y');zlabel('Z');
% Front(XZ) plot creation (bottom left) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
plotXZHandle = subplot(2,2,3);
grid on; hold on;
baseLink_XZ = plot([t0(1) t1(1,4)], [t0(3) t1(3,4)],'LineWidth',2,'Color','#0072BD');
link1_XZ = plot([t1(1,4) t2(1,4)], [t1(3,4) t2(3,4)],'LineWidth',2,'Color','#0072BD');
link2_XZ = plot([t2(1,4) t3(1,4)], [t2(3,4) t3(3,4)],'LineWidth',2,'Color','#D95319');
link3_XZ = plot([t3(1,4) t4(1,4)], [t3(3,4) t4(3,4)],'LineWidth',2,'Color','#0072BD');
link4_XZ = plot([t4(1,4) t5(1,4)], [t4(3,4) t5(3,4)],'LineWidth',2,'Color','#0072BD');
link5_XZ = plot([t5(1,4) t6(1,4)], [t5(3,4) t6(3,4)],'LineWidth',2,'Color','#0072BD');
link6_XZ = plot([t6(1,4) t7(1,4)], [t6(3,4) t7(3,4)],'LineWidth',2,'Color','#D95319');
set(plotXZHandle,'Position',[0.05 0.125 0.3 0.4])
axis([-1.2 1.2 -1.2 1.2]);
xlabel('X');ylabel('Z');
% Right(YZ) plot creation (bottom right) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
plotYZHandle = subplot(2,2,4);
grid on; hold on;
baseLink_YZ = plot([t0(2) t1(2,4)], [t0(3) t1(3,4)],'LineWidth',2,'Color','#0072BD');
link1_YZ = plot([t1(2,4) t2(2,4)], [t1(3,4) t2(3,4)],'LineWidth',2,'Color','#0072BD');
link2_YZ = plot([t2(2,4) t3(2,4)], [t2(3,4) t3(3,4)],'LineWidth',2,'Color','#D95319');
link3_YZ = plot([t3(2,4) t4(2,4)], [t3(3,4) t4(3,4)],'LineWidth',2,'Color','#0072BD');
link4_YZ = plot([t4(2,4) t5(2,4)], [t4(3,4) t5(3,4)],'LineWidth',2,'Color','#0072BD');
link5_YZ = plot([t5(2,4) t6(2,4)], [t5(3,4) t6(3,4)],'LineWidth',2,'Color','#0072BD');
link6_YZ = plot([t6(2,4) t7(2,4)], [t6(3,4) t7(3,4)],'LineWidth',2,'Color','#D95319');
set(plotYZHandle,'Position',[0.4 0.125 0.3 0.4])
axis([-1.2 1.2 -1.2 1.2 -1.2 1.2]);
xlabel('Y');ylabel('Z');
iKine(t7(1,4),t7(2,4),t7(3,4));

% UI Controls %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% buttons -----------------------------------------------------------------
HbttnSerialStart = uicontrol('Style','pushbutton');
set(HbttnSerialStart,'Units','normalized','Position',[0.75 0.85 0.1 0.05],...
    'String','Start','Callback',@startSerial);
HbttnSerialStop = uicontrol('Style','pushbutton');
set(HbttnSerialStop,'Units','normalized','Position',[0.85 0.85 0.1 0.05],...
    'String','Stop','Callback',@stopSerial);

% time --------------------------------------------------------------------
HtimeLabel = uicontrol('Style','text');
set(HtimeLabel,'Units','normalized','Position',[0.75 0.95 0.1 0.05],...
    'String','elapsedT:','Fontsize',20);
HloopLabel = uicontrol('Style','text');
set(HloopLabel,'Units','normalized','Position',[0.75 0.9 0.1 0.05],...
    'String','loopT:','Fontsize',20);

HtimeValue = uicontrol('Style','text');
set(HtimeValue,'Units','normalized','Position',[0.85 0.95 0.1 0.05],...
    'String','0','Fontsize',20);
HloopValue = uicontrol('Style','text');
set(HloopValue,'Units','normalized','Position',[0.85 0.9 0.1 0.05],...
    'String','0','Fontsize',20);

% Q1, Q2, Q4 --------------------------------------------------------------
HalphaLabel = uicontrol('Style','text');
set(HalphaLabel,'Units','normalized','Position',[0.75 0.8 0.1 0.05],...
    'String','rQ1:','Fontsize',20);
HbetaLabel = uicontrol('Style','text');
set(HbetaLabel,'Units','normalized','Position',[0.75 0.75 0.1 0.05],...
    'String','rQ2:','Fontsize',20);
HthetaLabel = uicontrol('Style','text');
set(HthetaLabel,'Units','normalized','Position',[0.75 0.7 0.1 0.05],...
    'String','rQ4:','Fontsize',20);

Hq1Value = uicontrol('Style','text');
set(Hq1Value,'Units','normalized','Position',[0.85 0.8 0.1 0.05],...
    'String','0','Fontsize',20);
Hq2Value = uicontrol('Style','text');
set(Hq2Value,'Units','normalized','Position',[0.85 0.75 0.1 0.05],...
    'String','0','Fontsize',20);
Hq4Value = uicontrol('Style','text');
set(Hq4Value,'Units','normalized','Position',[0.85 0.7 0.1 0.05],...
    'String','0','Fontsize',20);

% X, Y, Z -----------------------------------------------------------------
HxLabel = uicontrol('Style','text');
set(HxLabel,'Units','normalized','Position',[0.75 0.65 0.1 0.05],...
    'String','rX:','Fontsize',20);
HyLabel = uicontrol('Style','text');
set(HyLabel,'Units','normalized','Position',[0.75 0.6 0.1 0.05],...
    'String','rY:','Fontsize',20);
HzLabel = uicontrol('Style','text');
set(HzLabel,'Units','normalized','Position',[0.75 0.55 0.1 0.05],...
    'String','rZ:','Fontsize',20);

HxValue = uicontrol('Style','text');
set(HxValue,'Units','normalized','Position',[0.85 0.65 0.1 0.05],...
    'String','0','Fontsize',20);
HyValue = uicontrol('Style','text');
set(HyValue,'Units','normalized','Position',[0.85 0.6 0.1 0.05],...
    'String','0','Fontsize',20);
HzValue = uicontrol('Style','text');
set(HzValue,'Units','normalized','Position',[0.85 0.55 0.1 0.05],...
    'String','0','Fontsize',20);

    function startSerial(source,eventdata)
        botSerial.Start();
        set(Hfigure,'UserData',true);
        animatedata();
    end

    function stopSerial(source,eventdata)
        set(Hfigure,'UserData',false);
        botSerial.Stop();
    end

%% Animation of data
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
            q1 = botSerial.frameData(2);
            q2 = botSerial.frameData(3);
            q4 = botSerial.frameData(4);
            x = botSerial.frameData(11);
            y = botSerial.frameData(12);
            z = botSerial.frameData(13);
            % Kinematics Calculations
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
            % 3D Plot update
            set(baseLink_3d,'XData',[t0(1) t1(1,4)],'YData',[t0(2) t1(2,4)],'ZData',[t0(3) t1(3,4)])
            set(link1_3d,'XData',[t1(1,4) t2(1,4)],'YData',[t1(2,4) t2(2,4)],'ZData',[t1(3,4) t2(3,4)])
            set(link2_3d,'XData',[t2(1,4) t3(1,4)],'YData',[t2(2,4) t3(2,4)],'ZData',[t2(3,4) t3(3,4)])
            set(link3_3d,'XData',[t3(1,4) t4(1,4)],'YData',[t3(2,4) t4(2,4)],'ZData',[t3(3,4) t4(3,4)])
            set(link4_3d,'XData',[t4(1,4) t5(1,4)],'YData',[t4(2,4) t5(2,4)],'ZData',[t4(3,4) t5(3,4)])
            set(link5_3d,'XData',[t5(1,4) t6(1,4)],'YData',[t5(2,4) t6(2,4)],'ZData',[t5(3,4) t6(3,4)])
            set(link6_3d,'XData',[t6(1,4) t7(1,4)],'YData',[t6(2,4) t7(2,4)],'ZData',[t6(3,4) t7(3,4)])
            % XZ Plot Update
            set(baseLink_XZ,'XData',[t0(1) t1(1,4)],'YData',[t0(3) t1(3,4)])
            set(link1_XZ,'XData',[t1(1,4) t2(1,4)],'YData',[t1(3,4) t2(3,4)])
            set(link2_XZ,'XData',[t2(1,4) t3(1,4)],'YData',[t2(3,4) t3(3,4)])
            set(link3_XZ,'XData',[t3(1,4) t4(1,4)],'YData',[t3(3,4) t4(3,4)])
            set(link4_XZ,'XData',[t4(1,4) t5(1,4)],'YData',[t4(3,4) t5(3,4)])
            set(link5_XZ,'XData',[t5(1,4) t6(1,4)],'YData',[t5(3,4) t6(3,4)])
            set(link6_XZ,'XData',[t6(1,4) t7(1,4)],'YData',[t6(3,4) t7(3,4)])
            % YZ Plot Update
            set(baseLink_YZ,'XData',[t0(2) t1(2,4)],'YData',[t0(3) t1(3,4)])
            set(link1_YZ,'XData',[t1(2,4) t2(2,4)],'YData',[t1(3,4) t2(3,4)])
            set(link2_YZ,'XData',[t2(2,4) t3(2,4)],'YData',[t2(3,4) t3(3,4)])
            set(link3_YZ,'XData',[t3(2,4) t4(2,4)],'YData',[t3(3,4) t4(3,4)])
            set(link4_YZ,'XData',[t4(2,4) t5(2,4)],'YData',[t4(3,4) t5(3,4)])
            set(link5_YZ,'XData',[t5(2,4) t6(2,4)],'YData',[t5(3,4) t6(3,4)])
            set(link6_YZ,'XData',[t6(2,4) t7(2,4)],'YData',[t6(3,4) t7(3,4)])
            % Values update
            set(Hq1Value,'String',num2str(q1));
            set(Hq2Value,'String',num2str(q2));
            set(Hq4Value,'String',num2str(q4));
            set(HxValue,'String',num2str(x));
            set(HyValue,'String',num2str(y));
            set(HzValue,'String',num2str(z));
            set(HtimeValue,'String',num2str(botSerial.frameData(1)));
            set(HloopValue,'String',num2str(botSerial.frameData(17)));
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