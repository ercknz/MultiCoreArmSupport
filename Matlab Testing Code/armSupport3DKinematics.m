function armSupport3DKinematics()
%% Arm support 3D kinematics
% code by Erick Nunez

%% ========================================================================
%  CONSTANTS AND KINEMATICS SETUP
%  ========================================================================

%% Robot Parameters
L1 = 0.419;
L2 = 0.520;
A1 = 0.073;
A2 = 0.082;
A3 = 0.072;
offset = 0.035;

SHDR_LIMIT = [0,270];
ELVN_LIMIT = [-45,45];
ELBW_LIMIT = [atand(offset/L2),180];

%% Test Goals and Communication Setup
testGoals = [0.750,  0.300,  0.250;
             0.750, -0.300,  0.250;
             0.550,  0.000,  0.000;
             0.650,  0.300, -0.200;
             0.650, -0.300, -0.200];
idx = 1;
velocityXYZ = 0.5;

% Communication setup with port selection
robotBaud = 1000000;
botSerial = [];
fullRobot = false;
torqueToggle = false;
availablePorts = serialportlist;
robotSerialPort = availablePorts(end);

%% Transformation Matrix Functions
rotateX = @(a)  [1,0,0;
                 0,cos(a),-sin(a);
                 0,sin(a),cos(a)];
rotateZ = @(c)  [cos(c),-sin(c),0;
                 sin(c),cos(c),0;
                 0,0,1];
matrixT = @(R,P)[R, P; 0,0,0,1];

%% Forward Kinematics Functions
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

%% Inverse Kinematics Setup
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

%% ========================================================================
%  GUI SETUP AND INITIALIZATION
%  ========================================================================

%% Figure Setup
Hfigure = figure(100);
set(Hfigure,'Units','normalized','Position',[0.05 0.05 0.90 0.85]);
set(Hfigure,'UserData',false);
set(Hfigure, 'Renderer', 'opengl');  % Hardware acceleration
set(Hfigure, 'DoubleBuffer', 'on');  % Reduce flicker

%% Plot Configuration
% Plot order: XY, XZ, YZ, XYZ
plotPositions = [0.05 0.575 0.3 0.4; 0.4 0.575 0.3 0.4; 0.05 0.125 0.3 0.4; 0.4 0.125 0.3 0.4];
plotViews = {[], [], [], [45,45]};  % 3D view is last
axisLabels = {{'X','Y'}, {'X','Z'}, {'Y','Z'}, {'X','Y','Z'}};
axisLimits = {[-1.2 1.2 -1.2 1.2], [-1.2 1.2 -1.2 1.2], [-1.2 1.2 -1.2 1.2 -1.2 1.2], [-1.2 1.2 -1.2 1.2 -1.2 1.2]};

% Robot link colors
robotColors = {'#0072BD', '#0072BD', '#D95319', '#0072BD', '#0072BD', '#0072BD', '#D95319'};
goalColors = {[0 0 0 0.25], [0 0 0 0.25], [1 0 1 0.25], [0 0 0 0.25], [0 0 0 0.25], [0 0 0 0.25], [1 0 1 0.25]};
goalSetColors = {[1 0 0 0.25], [1 0 0 0.25], [0 1 0 0.25], [1 0 0 0.25], [1 0 0 0.25], [1 0 0 0.25], [0 1 0 0.25]};

%% Create AnimatedLine Objects
plotHandles = cell(1,4);
robotLinks = cell(4,7);      % 4 plots x 7 links
goalLinks = cell(4,7);       % Goal robot links
goalSetLinks = cell(4,7);    % Goal setting robot links
endEffectors = cell(4,1);    % End effector markers
goalEndEffectors = cell(4,1); % Goal end effectors
models = cell(4,1);          % Model markers

% Create all plots and animatedlines using loops
for plotIdx = 1:4
    plotHandles{plotIdx} = subplot(2,2,plotIdx);
    grid on; hold on;
    
    % Create robot links
    for linkIdx = 1:7
        robotLinks{plotIdx,linkIdx} = animatedline('LineWidth',2,'Color',robotColors{linkIdx});
        goalLinks{plotIdx,linkIdx} = animatedline('LineWidth',2,'Color',goalColors{linkIdx});
        goalSetLinks{plotIdx,linkIdx} = animatedline('LineWidth',2,'Color',goalSetColors{linkIdx});
    end
    
    % Create end effectors and models
    endEffectors{plotIdx} = animatedline('Marker','*','MarkerSize',12,'Color','g','LineStyle','none');
    goalEndEffectors{plotIdx} = animatedline('Marker','*','MarkerSize',12,'Color',[0.6350 0.0780 0.1840 0.2],'LineStyle','none');
    models{plotIdx} = animatedline('Marker','o','MarkerSize',12,'Color',[0.47 0.25 0.80 0.5],'LineWidth',2,'LineStyle','none');
    
    % Set plot properties
    set(plotHandles{plotIdx},'Position',plotPositions(plotIdx,:));
    if ~isempty(plotViews{plotIdx})
        set(plotHandles{plotIdx},'View',plotViews{plotIdx});
    end
    axis(axisLimits{plotIdx});
    
    % Set labels
    if length(axisLabels{plotIdx}) == 2
        xlabel(axisLabels{plotIdx}{1}); ylabel(axisLabels{plotIdx}{2});
    else
        xlabel(axisLabels{plotIdx}{1}); ylabel(axisLabels{plotIdx}{2}); zlabel(axisLabels{plotIdx}{3});
    end
end

% Add legend to 3D plot (plot 4)
legend([robotLinks{4,3}, endEffectors{4}, goalLinks{4,3}, goalEndEffectors{4}, goalSetLinks{4,3}, models{4}],...
       'presR','presXYZ','rGoal','rGoalXYZ','goalSet','model','Location','northwest');

%% ========================================================================
%  USER INTERFACE CONTROLS
%  ========================================================================

%% Control Buttons
HbttnOpenCMStart = uicontrol('Style','pushbutton',...
    'Units','normalized','Position',[0.70 0.95 0.05 0.05],...
    'String','<html>Start<br />OpenCM</html>','Callback',@startOpenCM);
HbttnOpenCMStop = uicontrol('Style','pushbutton',...
    'Units','normalized','Position',[0.75 0.95 0.05 0.05],...
    'String','<html>Stop<br />OpenCM</html>','Callback',@stopOpenCM);
HbttnTeensyStart = uicontrol('Style','pushbutton',...
    'Units','normalized','Position',[0.80 0.95 0.05 0.05],...
    'String','<html>Start<br />Teensy</html>','Callback',@startTeensy);
HbttnTeensyStop = uicontrol('Style','pushbutton',...
    'Units','normalized','Position',[0.85 0.95 0.05 0.05],...
    'String','<html>Stop<br />Teensy</html>','Callback',@stopTeensy);
HbttnTorque = uicontrol('Style','pushbutton',...
    'Units','normalized','Position',[0.90 0.95 0.05, 0.05],...
    'String','Torque','Callback',@changeTorque);
HbttnTestGoal = uicontrol('Style','pushbutton',...
    'Units','normalized','Position',[0.95 0.95 0.05, 0.05],...
    'String','<html>Send<br />Goal</html>','Callback',@sendGoal);

%% COM Port Selection
HportLabel = uicontrol('Style','text',...
    'Units','normalized','Position',[0 0 0.08 0.05],...
    'String','COM Port:','Fontsize',12,'HorizontalAlignment','left');
HportDropdown = uicontrol('Style','popupmenu',...
    'Units','normalized','Position',[0.05 0 0.08 0.05],...
    'String',availablePorts,'Value',length(availablePorts),...
    'Callback',@selectPort,'FontSize',10);

%% Status Display Elements
% Time displays
HtimeLabel = uicontrol('Style','text','Units','normalized','Position',[0.933 0.9 0.066 0.05],...
    'String','elapsedT:','Fontsize',18);
HtimeValue = uicontrol('Style','text','Units','normalized','Position',[0.933 0.85 0.066 0.05],...
    'String','0','Fontsize',20);
HloopLabel = uicontrol('Style','text','Units','normalized','Position',[0.933 0.8 0.066 0.05],...
    'String','loopT:','Fontsize',20);
HloopValue = uicontrol('Style','text','Units','normalized','Position',[0.933 0.75 0.066 0.05],...
    'String','0','Fontsize',20);
HdriveModeLabel = uicontrol('Style','text','Units','normalized','Position',[0.933 0.7 0.066 0.05],...
    'String','DriveMode:','Fontsize',16);
HdriveModeValue = uicontrol('Style','text','Units','normalized','Position',[0.933 0.65 0.066 0.05],...
    'String','0','Fontsize',20);

%% Joint Position Displays
% Present Q1, Q2, Q4
HalphaLabel = uicontrol('Style','text','Units','normalized','Position',[0.7 0.9 0.066 0.05],...
    'String','rQ1:','Fontsize',20);
Hq1Value = uicontrol('Style','text','Units','normalized','Position',[0.766 0.9 0.066 0.05],...
    'String','0','Fontsize',20);
Hq1CtsValue = uicontrol('Style','text','Units','normalized','Position',[0.832 0.9 0.066 0.05],...
    'String','0','Fontsize',20,'ForegroundColor',"#7E2F8E");

HbetaLabel = uicontrol('Style','text','Units','normalized','Position',[0.7 0.85 0.066 0.05],...
    'String','rQ2:','Fontsize',20);
Hq2Value = uicontrol('Style','text','Units','normalized','Position',[0.766 0.85 0.066 0.05],...
    'String','0','Fontsize',20);
Hq2CtsValue = uicontrol('Style','text','Units','normalized','Position',[0.832 0.85 0.066 0.05],...
    'String','0','Fontsize',20,'ForegroundColor',"#7E2F8E");

HthetaLabel = uicontrol('Style','text','Units','normalized','Position',[0.7 0.8 0.066 0.05],...
    'String','rQ4:','Fontsize',20);
Hq4Value = uicontrol('Style','text','Units','normalized','Position',[0.766 0.8 0.066 0.05],...
    'String','0','Fontsize',20);
Hq4CtsValue = uicontrol('Style','text','Units','normalized','Position',[0.832 0.8 0.066 0.05],...
    'String','0','Fontsize',20,'ForegroundColor',"#7E2F8E");

%% Cartesian Position Displays
% Present X, Y, Z
HRxLabel = uicontrol('Style','text','Units','normalized','Position',[0.7 0.75 0.066 0.05],...
    'String','rX:','Fontsize',20);
HRxValue = uicontrol('Style','text','Units','normalized','Position',[0.766 0.75 0.066 0.05],...
    'String','0','Fontsize',20);
HRyLabel = uicontrol('Style','text','Units','normalized','Position',[0.7 0.7 0.066 0.05],...
    'String','rY:','Fontsize',20);
HRyValue = uicontrol('Style','text','Units','normalized','Position',[0.766 0.7 0.066 0.05],...
    'String','0','Fontsize',20);
HRzLabel = uicontrol('Style','text','Units','normalized','Position',[0.7 0.65 0.066 0.05],...
    'String','rZ:','Fontsize',20);
HRzValue = uicontrol('Style','text','Units','normalized','Position',[0.766 0.65 0.066 0.05],...
    'String','0','Fontsize',20);

% Goal X, Y, Z
HGxLabel = uicontrol('Style','text','Units','normalized','Position',[0.7 0.6 0.066 0.05],...
    'String','gX:','Fontsize',20);
HGxValue = uicontrol('Style','text','Units','normalized','Position',[0.766 0.6 0.066 0.05],...
    'String','0','Fontsize',20);
HSxValue = uicontrol('Style','text','Units','normalized','Position',[0.832 0.6 0.066 0.05],...
    'String','0','Fontsize',20,'ForegroundColor','#A2142F');
HGyLabel = uicontrol('Style','text','Units','normalized','Position',[0.7 0.55 0.066 0.05],...
    'String','gY:','Fontsize',20);
HGyValue = uicontrol('Style','text','Units','normalized','Position',[0.766 0.55 0.066 0.05],...
    'String','0','Fontsize',20);
HSyValue = uicontrol('Style','text','Units','normalized','Position',[0.832 0.55 0.066 0.05],...
    'String','0','Fontsize',20,'ForegroundColor','#A2142F');
HGzLabel = uicontrol('Style','text','Units','normalized','Position',[0.7 0.5 0.066 0.05],...
    'String','gZ:','Fontsize',20);
HGzValue = uicontrol('Style','text','Units','normalized','Position',[0.766 0.5 0.066 0.05],...
    'String','0','Fontsize',20);
HSzValue = uicontrol('Style','text','Units','normalized','Position',[0.832 0.5 0.066 0.05],...
    'String','0','Fontsize',20,'ForegroundColor','#A2142F');

%% Goal Joint Positions
HgoalQ1Label = uicontrol('Style','text','Units','normalized','Position',[0.7 0.45 0.066 0.05],...
    'String','gQ1:','Fontsize',20);
HgoalQ1Value = uicontrol('Style','text','Units','normalized','Position',[0.766 0.45 0.066 0.05],...
    'String','0','Fontsize',20);
HsQ1Value = uicontrol('Style','text','Units','normalized','Position',[0.832 0.45 0.066 0.05],...
    'String','0','Fontsize',20,'ForegroundColor','#A2142F');
HgoalQ1ctsValue = uicontrol('Style','text','Units','normalized','Position',[0.898 0.45 0.066 0.05],...
    'String','0','Fontsize',20,'ForegroundColor','#7E2F8E');

HgoalQ2Label = uicontrol('Style','text','Units','normalized','Position',[0.7 0.4 0.066 0.05],...
    'String','gQ2:','Fontsize',20);
HgoalQ2Value = uicontrol('Style','text','Units','normalized','Position',[0.766 0.4 0.066 0.05],...
    'String','0','Fontsize',20);
HsQ2Value = uicontrol('Style','text','Units','normalized','Position',[0.832 0.4 0.066 0.05],...
    'String','0','Fontsize',20,'ForegroundColor','#A2142F');
HgoalQ2ctsValue = uicontrol('Style','text','Units','normalized','Position',[0.898 0.4 0.066 0.05],...
    'String','0','Fontsize',20,'ForegroundColor','#7E2F8E');

HgoalQ4Label = uicontrol('Style','text','Units','normalized','Position',[0.7 0.35 0.066 0.05],...
    'String','gQ4:','Fontsize',20);
HgoalQ4Value = uicontrol('Style','text','Units','normalized','Position',[0.766 0.35 0.066 0.05],...
    'String','0','Fontsize',20);
HsQ4Value = uicontrol('Style','text','Units','normalized','Position',[0.832 0.35 0.066 0.05],...
    'String','0','Fontsize',20,'ForegroundColor','#A2142F');
HgoalQ4ctsValue = uicontrol('Style','text','Units','normalized','Position',[0.898 0.35 0.066 0.05],...
    'String','0','Fontsize',20,'ForegroundColor','#7E2F8E');

%% Model and Force Displays
% Model X, Y, Z
HMxLabel = uicontrol('Style','text','Units','normalized','Position',[0.7 0.3 0.066 0.05],...
    'String','mX:','Fontsize',20);
HMxValue = uicontrol('Style','text','Units','normalized','Position',[0.766 0.3 0.066 0.05],...
    'String','0','Fontsize',20);
HMyLabel = uicontrol('Style','text','Units','normalized','Position',[0.7 0.25 0.066 0.05],...
    'String','mY:','Fontsize',20);
HMyValue = uicontrol('Style','text','Units','normalized','Position',[0.766 0.25 0.066 0.05],...
    'String','0','Fontsize',20);
HMzLabel = uicontrol('Style','text','Units','normalized','Position',[0.7 0.2 0.066 0.05],...
    'String','mZ:','Fontsize',20);
HMzValue = uicontrol('Style','text','Units','normalized','Position',[0.766 0.2 0.066 0.05],...
    'String','0','Fontsize',20);

% Force X, Y, Z
HFxLabel = uicontrol('Style','text','Units','normalized','Position',[0.7 0.15 0.066 0.05],...
    'String','Fx:','Fontsize',20);
HFxValue = uicontrol('Style','text','Units','normalized','Position',[0.766 0.15 0.066 0.05],...
    'String','0','Fontsize',20);
HFxCtsValue = uicontrol('Style','text','Units','normalized','Position',[0.832 0.15 0.066 0.05],...
    'String','0','Fontsize',20,'ForegroundColor','#A2142F');
HFyLabel = uicontrol('Style','text','Units','normalized','Position',[0.7 0.1 0.066 0.05],...
    'String','Fy:','Fontsize',20);
HFyValue = uicontrol('Style','text','Units','normalized','Position',[0.766 0.1 0.066 0.05],...
    'String','0','Fontsize',20);
HFyCtsValue = uicontrol('Style','text','Units','normalized','Position',[0.832 0.1 0.066 0.05],...
    'String','0','Fontsize',20,'ForegroundColor','#A2142F');
HFzLabel = uicontrol('Style','text','Units','normalized','Position',[0.7 0.05 0.066 0.05],...
    'String','Fz:','Fontsize',20);
HFzValue = uicontrol('Style','text','Units','normalized','Position',[0.766 0.05 0.066 0.05],...
    'String','0','Fontsize',20);
HFzCtsValue = uicontrol('Style','text','Units','normalized','Position',[0.832 0.05 0.066 0.05],...
    'String','0','Fontsize',20,'ForegroundColor','#A2142F');

%% Initialize Display
initializeAnimatedLines();

%% ========================================================================
%  CALLBACK FUNCTIONS
%  ========================================================================
    function selectPort(~,~)
        selectedPortIdx = get(HportDropdown,'Value');
        robotSerialPort = availablePorts(selectedPortIdx);
    end

    function startOpenCM(~,~)
        testRunning = get(Hfigure,'UserData');
        if ~testRunning
            botSerial = CommOpenCM(robotSerialPort, robotBaud);
            fullRobot = false;
            resetModelPositions();
            botSerial.Start();
            set(Hfigure,'UserData',true);
            animatedata();
        end
    end

    function stopOpenCM(~,~)
        testRunning = get(Hfigure,'UserData');
        if testRunning
            set(Hfigure,'UserData',false);
            botSerial.Stop();
            fullRobot = false;
        end
    end

    function startTeensy(~,~)
        testRunning = get(Hfigure,'UserData');
        if ~testRunning
            botSerial = armSupportRobot(robotSerialPort, robotBaud);
            fullRobot = true;
            botSerial.Start();
            set(Hfigure,'UserData',true);
            animatedata();
        end
    end

    function stopTeensy(~,~)
        testRunning = get(Hfigure,'UserData');
        if testRunning
           set(Hfigure,'UserData',false);
           botSerial.Stop();
           fullRobot = false;
        end
    end

    function changeTorque(~,~)
        testRunning = get(Hfigure,'UserData');
        if testRunning
            if torqueToggle
                botSerial.SendTorqueChange(20);
                torqueToggle = false;
                set(HbttnTorque,'String','<html>torque<br/>OFF</html>','ForegroundColor','red');
            else
                botSerial.SendTorqueChange(5);
                torqueToggle = true;
                set(HbttnTorque,'String','<html>torque<br/>ON</html>','ForegroundColor','green');
            end
        end
    end

    function sendGoal(~,~)
        test = randi(5);
        while test == idx
            test = randi(5);
        end
        idx = test;
        
        if fullRobot
            botSerial.SendNewModelGoals(testGoals(idx,1), testGoals(idx,2), testGoals(idx,3));
        else
            botSerial.SendGoal(testGoals(idx,1), testGoals(idx,2), testGoals(idx,3), velocityXYZ, velocityXYZ, velocityXYZ);
        end
        
        % Update text displays
        set(HSxValue,'String',num2str(testGoals(idx,1)));
        set(HSyValue,'String',num2str(testGoals(idx,2)));
        set(HSzValue,'String',num2str(testGoals(idx,3)));
        
        % Update goal end effector position
        updateGoalEndEffector(testGoals(idx,1), testGoals(idx,2), testGoals(idx,3));
        
        % Calculate and update goal robot configuration
        goalQ = iKine(testGoals(idx,1), testGoals(idx,2), testGoals(idx,3));
        updateGoalSettingDisplay(goalQ);
        
        % Update goal joint text displays
        set(HsQ1Value,'String',num2str(goalQ(1)));
        set(HsQ2Value,'String',num2str(goalQ(2)));
        set(HsQ4Value,'String',num2str(goalQ(3)));
    end

%% ========================================================================
%  ANIMATION AND DATA PROCESSING
%  ========================================================================

    function animatedata()
        % Performance optimization variables
        targetFPS = 30;
        frameInterval = 1/targetFPS;
        lastDrawTime = tic;
        frameCount = 0;
        
        % Previous values for comparison
        persistent prevX prevY prevZ prevQ1 prevQ2 prevQ4;
        if isempty(prevX)
            prevX = 0; prevY = 0; prevZ = 0;
            prevQ1 = 0; prevQ2 = 0; prevQ4 = 0;
        end
        
        while true
            frameStart = tic;
            
            testRunning = get(Hfigure,'UserData');
            if ~isempty(testRunning) && ~testRunning
                break;
            end
            
            % Data Request for OpenCM/Teensy
            botSerial.SendRequest();
            
            % Non-blocking wait with timeout
            if waitForSerialData()
                [robotData, goalData, modelData, statusData] = extractFrameData();
                
                % Update robot display
                updateRobotDisplay(calculateRobotTransforms(robotData));
                updateEndEffector(robotData.x, robotData.y, robotData.z);
                [prevX, prevY, prevZ, prevQ1, prevQ2, prevQ4] = updatePreviousValues(robotData);

                
                % Update goal display
                updateGoalDisplay(goalData);
                
                % Update model (for Teensy mode)
                if fullRobot
                    updateModelPosition(modelData.x, modelData.y, modelData.z);
                end
                
                % Update text displays
                updateAllTextDisplays(robotData, goalData, modelData, statusData);
            end
            
            frameCount = frameCount + 1;
            
            % Controlled frame rate update
            if toc(lastDrawTime) >= frameInterval
                drawnow limitrate;
                lastDrawTime = tic;
                
                % Performance monitoring (optional)
                if mod(frameCount, 150) == 0
                    frameTime = toc(frameStart);
                    fprintf('Frame %d: %.1f ms processing time\n', frameCount, frameTime*1000);
                end
            end
            
            pause(0.001);
        end
    end

%% ========================================================================
%  HELPER FUNCTIONS - INITIALIZATION
%  ========================================================================

    function initializeAnimatedLines()
        % Initialize all animatedlines with default robot position
        t0_init = [0,0,0];
        t1_init = T01(0);
        t2_init = T02(0,0);
        t3_init = T03(0,0);
        t4_init = T04(0,0,deg2rad(ELBW_LIMIT(1)));
        t5_init = T05(0,0,deg2rad(ELBW_LIMIT(1)));
        t6_init = T06(0,0,deg2rad(ELBW_LIMIT(1)));
        t7_init = T07(0,0,deg2rad(ELBW_LIMIT(1)));
        
        % Create transforms matrix properly
        transforms = [t0_init(1:3); t1_init(1:3,4)'; t2_init(1:3,4)'; t3_init(1:3,4)'; 
                      t4_init(1:3,4)'; t5_init(1:3,4)'; t6_init(1:3,4)'; t7_init(1:3,4)'];
        
        updateRobotDisplay(transforms);
        updateGoalRobot(transforms);
        updateGoalSettingRobot(transforms);
    end

    function resetModelPositions()
        % Reset model positions to origin
        for plotIdx = 1:4
            clearpoints(models{plotIdx});
            if plotIdx == 4  % 3D plot
                addpoints(models{plotIdx}, 0, 0, 0);
            else  % 2D plots
                addpoints(models{plotIdx}, 0, 0);
            end
        end
    end

%% ========================================================================
%  HELPER FUNCTIONS - DATA PROCESSING
%  ========================================================================

    function success = waitForSerialData()
        % Non-blocking wait with timeout for serial data
        maxWaitTime = 0.05; % 50ms timeout
        waitStart = tic;
        
        while botSerial.BytesAvailable < botSerial.rxPacketLen
            if toc(waitStart) > maxWaitTime
                fprintf('Data timeout - skipping frame (available: %d, needed: %d)\n', ...
                    botSerial.BytesAvailable, botSerial.rxPacketLen);
                success = false;
                return;
            end
            pause(0.001);
        end
        
        ReadFrame(botSerial);
        success = true;
    end

    function [robotData, goalData, modelData, statusData] = extractFrameData()
        % Extract and organize frame data
        robotData.q1 = botSerial.frameData(2);
        robotData.q2 = botSerial.frameData(3);
        robotData.q4 = botSerial.frameData(4);
        robotData.x = botSerial.frameData(8);
        robotData.y = botSerial.frameData(9);
        robotData.z = botSerial.frameData(10);
        
        goalData.q1 = botSerial.frameData(14);
        goalData.q2 = botSerial.frameData(15);
        goalData.q4 = botSerial.frameData(16);
        
        if fullRobot
            modelData.forceXcts = botSerial.frameData(20);
            modelData.forceYcts = botSerial.frameData(21);
            modelData.forceZcts = botSerial.frameData(22);
            modelData.forceX = botSerial.frameData(26);
            modelData.forceY = botSerial.frameData(27);
            modelData.forceZ = botSerial.frameData(28);
            modelData.x = botSerial.frameData(32);
            modelData.y = botSerial.frameData(33);
            modelData.z = botSerial.frameData(34);
            statusData.torqueVal = botSerial.frameData(38);
            statusData.lTime = botSerial.frameData(39);
        else
            modelData.q1Cts = botSerial.frameData(20);
            modelData.q2Cts = botSerial.frameData(21);
            modelData.q4Cts = botSerial.frameData(22);
            modelData.gQ1cts = botSerial.frameData(26);
            modelData.gQ2cts = botSerial.frameData(27);
            modelData.gQ4cts = botSerial.frameData(28);
            statusData.torqueVal = botSerial.frameData(32);
            statusData.lTime = botSerial.frameData(33);
        end
        
        statusData.elapsedTime = botSerial.frameData(1);
    end

    function [prevX, prevY, prevZ, prevQ1, prevQ2, prevQ4] = updatePreviousValues(robotData)
        % Update previous values for movement detection
        prevX = robotData.x;
        prevY = robotData.y;
        prevZ = robotData.z;
        prevQ1 = robotData.q1;
        prevQ2 = robotData.q2;
        prevQ4 = robotData.q4;
    end

    function transforms = calculateRobotTransforms(robotData)
        % Calculate transformation matrices for current robot state
        t0_r = [0,0,0];
        t1_r = T01(robotData.q1);
        t2_r = T02(robotData.q1, robotData.q2);
        t3_r = T03(robotData.q1, robotData.q2);
        t4_r = T04(robotData.q1, robotData.q2, robotData.q4);
        t5_r = T05(robotData.q1, robotData.q2, robotData.q4);
        t6_r = T06(robotData.q1, robotData.q2, robotData.q4);
        t7_r = T07(robotData.q1, robotData.q2, robotData.q4);
        
        transforms = [t0_r(1:3); t1_r(1:3,4)'; t2_r(1:3,4)'; t3_r(1:3,4)'; 
                      t4_r(1:3,4)'; t5_r(1:3,4)'; t6_r(1:3,4)'; t7_r(1:3,4)'];
    end

    function updateGoalDisplay(goalData)
        % Update goal robot display
        t0_g = [0,0,0];
        t1_g = T01(goalData.q1);
        t2_g = T02(goalData.q1, goalData.q2);
        t3_g = T03(goalData.q1, goalData.q2);
        t4_g = T04(goalData.q1, goalData.q2, goalData.q4);
        t5_g = T05(goalData.q1, goalData.q2, goalData.q4);
        t6_g = T06(goalData.q1, goalData.q2, goalData.q4);
        t7_g = T07(goalData.q1, goalData.q2, goalData.q4);
        
        goalTransforms = [t0_g(1:3); t1_g(1:3,4)'; t2_g(1:3,4)'; t3_g(1:3,4)'; 
                          t4_g(1:3,4)'; t5_g(1:3,4)'; t6_g(1:3,4)'; t7_g(1:3,4)'];
        updateGoalRobot(goalTransforms);
    end

    function updateGoalSettingDisplay(goalQ)
        % Update goal setting robot display
        t0_g = [0,0,0];
        t1_g = T01(goalQ(1));
        t2_g = T02(goalQ(1), goalQ(2));
        t3_g = T03(goalQ(1), goalQ(2));
        t4_g = T04(goalQ(1), goalQ(2), goalQ(3));
        t5_g = T05(goalQ(1), goalQ(2), goalQ(3));
        t6_g = T06(goalQ(1), goalQ(2), goalQ(3));
        t7_g = T07(goalQ(1), goalQ(2), goalQ(3));
        
        goalTransforms = [t0_g(1:3); t1_g(1:3,4)'; t2_g(1:3,4)'; t3_g(1:3,4)'; 
                          t4_g(1:3,4)'; t5_g(1:3,4)'; t6_g(1:3,4)'; t7_g(1:3,4)'];
        updateGoalSettingRobot(goalTransforms);
    end

%% ========================================================================
%  HELPER FUNCTIONS - DISPLAY UPDATES
%  ========================================================================

    function updateAllTextDisplays(robotData, goalData, modelData, statusData)
        % Update all text displays with current data
        
        % Robot joint positions
        set(Hq1Value,'String',num2str(robotData.q1, '%.3f'));
        set(Hq2Value,'String',num2str(robotData.q2, '%.3f'));
        set(Hq4Value,'String',num2str(robotData.q4, '%.3f'));
        
        % Robot cartesian positions
        set(HRxValue,'String',num2str(robotData.x, '%.3f'));
        set(HRyValue,'String',num2str(robotData.y, '%.3f'));
        set(HRzValue,'String',num2str(robotData.z, '%.3f'));
        
        % Goal joint positions
        set(HgoalQ1Value,'String',num2str(goalData.q1, '%.3f'));
        set(HgoalQ2Value,'String',num2str(goalData.q2, '%.3f'));
        set(HgoalQ4Value,'String',num2str(goalData.q4, '%.3f'));
        
        % Calculate goal cartesian position
        t7_g = T07(goalData.q1, goalData.q2, goalData.q4);
        set(HGxValue,'String',num2str(t7_g(1,4), '%.3f'));
        set(HGyValue,'String',num2str(t7_g(2,4), '%.3f'));
        set(HGzValue,'String',num2str(t7_g(3,4), '%.3f'));
        
        % Status information
        set(HtimeValue,'String',num2str(statusData.elapsedTime));
        set(HloopValue,'String',num2str(statusData.lTime));
        set(HdriveModeValue,'String',num2str(statusData.torqueVal));
        
        % Mode-specific displays
        if fullRobot
            set(HMxValue,'String',num2str(modelData.x, '%.3f'));
            set(HMyValue,'String',num2str(modelData.y, '%.3f'));
            set(HMzValue,'String',num2str(modelData.z, '%.3f'));
            set(HFxValue,'String',num2str(modelData.forceX, '%.3f'));
            set(HFyValue,'String',num2str(modelData.forceY, '%.3f'));
            set(HFzValue,'String',num2str(modelData.forceZ, '%.3f'));
            set(HFxCtsValue,'String',num2str(modelData.forceXcts));
            set(HFyCtsValue,'String',num2str(modelData.forceYcts));
            set(HFzCtsValue,'String',num2str(modelData.forceZcts));
        else
            set(HgoalQ1ctsValue,'String',num2str(modelData.gQ1cts));
            set(HgoalQ2ctsValue,'String',num2str(modelData.gQ2cts));
            set(HgoalQ4ctsValue,'String',num2str(modelData.gQ4cts));
            set(Hq1CtsValue,'String',num2str(modelData.q1Cts));
            set(Hq2CtsValue,'String',num2str(modelData.q2Cts));
            set(Hq4CtsValue,'String',num2str(modelData.q4Cts));
        end
    end

%% ========================================================================
%  HELPER FUNCTIONS - ANIMATEDLINE UPDATES
%  ========================================================================

    function updateRobotDisplay(transforms)
        % Update current robot position (blue/orange links)
        coordMaps = {[1,2], [1,3], [2,3], [1,2,3]}; % XY, XZ, YZ, XYZ
        
        for plotIdx = 1:4
            coords = coordMaps{plotIdx};
            for linkIdx = 1:7
                clearpoints(robotLinks{plotIdx,linkIdx});
                if plotIdx == 4  % 3D plot (last)
                    addpoints(robotLinks{plotIdx,linkIdx}, ...
                        [transforms(linkIdx,coords(1)), transforms(linkIdx+1,coords(1))], ...
                        [transforms(linkIdx,coords(2)), transforms(linkIdx+1,coords(2))], ...
                        [transforms(linkIdx,coords(3)), transforms(linkIdx+1,coords(3))]);
                else % 2D plots (first three)
                    addpoints(robotLinks{plotIdx,linkIdx}, ...
                        [transforms(linkIdx,coords(1)), transforms(linkIdx+1,coords(1))], ...
                        [transforms(linkIdx,coords(2)), transforms(linkIdx+1,coords(2))]);
                end
            end
        end
    end

    function updateGoalRobot(transforms)
        % Update goal robot position (transparent black/magenta links)
        coordMaps = {[1,2], [1,3], [2,3], [1,2,3]};
        
        for plotIdx = 1:4
            coords = coordMaps{plotIdx};
            for linkIdx = 1:7
                clearpoints(goalLinks{plotIdx,linkIdx});
                if plotIdx == 4  % 3D plot (last)
                    addpoints(goalLinks{plotIdx,linkIdx}, ...
                        [transforms(linkIdx,coords(1)), transforms(linkIdx+1,coords(1))], ...
                        [transforms(linkIdx,coords(2)), transforms(linkIdx+1,coords(2))], ...
                        [transforms(linkIdx,coords(3)), transforms(linkIdx+1,coords(3))]);
                else % 2D plots (first three)
                    addpoints(goalLinks{plotIdx,linkIdx}, ...
                        [transforms(linkIdx,coords(1)), transforms(linkIdx+1,coords(1))], ...
                        [transforms(linkIdx,coords(2)), transforms(linkIdx+1,coords(2))]);
                end
            end
        end
    end

    function updateGoalSettingRobot(transforms)
        % Update goal setting robot position (transparent red/green links)
        coordMaps = {[1,2], [1,3], [2,3], [1,2,3]};
        
        for plotIdx = 1:4
            coords = coordMaps{plotIdx};
            for linkIdx = 1:7
                clearpoints(goalSetLinks{plotIdx,linkIdx});
                if plotIdx == 4  % 3D plot (last)
                    addpoints(goalSetLinks{plotIdx,linkIdx}, ...
                        [transforms(linkIdx,coords(1)), transforms(linkIdx+1,coords(1))], ...
                        [transforms(linkIdx,coords(2)), transforms(linkIdx+1,coords(2))], ...
                        [transforms(linkIdx,coords(3)), transforms(linkIdx+1,coords(3))]);
                else % 2D plots (first three)
                    addpoints(goalSetLinks{plotIdx,linkIdx}, ...
                        [transforms(linkIdx,coords(1)), transforms(linkIdx+1,coords(1))], ...
                        [transforms(linkIdx,coords(2)), transforms(linkIdx+1,coords(2))]);
                end
            end
        end
    end

    function updateEndEffector(x, y, z)
        % Update end effector position across all plots
        % Plot order: XY, XZ, YZ, XYZ
        endCoords = [x, y, 0; x, z, 0; y, z, 0; x, y, z];
        
        for plotIdx = 1:4
            clearpoints(endEffectors{plotIdx});
            if plotIdx == 4  % 3D plot (last)
                addpoints(endEffectors{plotIdx}, endCoords(plotIdx,1), endCoords(plotIdx,2), endCoords(plotIdx,3));
            else  % 2D plots (first three)
                addpoints(endEffectors{plotIdx}, endCoords(plotIdx,1), endCoords(plotIdx,2));
            end
        end
    end

    function updateGoalEndEffector(x, y, z)
        % Update goal end effector position across all plots
        % Plot order: XY, XZ, YZ, XYZ
        goalEndCoords = [x, y, 0; x, z, 0; y, z, 0; x, y, z];
        
        for plotIdx = 1:4
            clearpoints(goalEndEffectors{plotIdx});
            if plotIdx == 4  % 3D plot (last)
                addpoints(goalEndEffectors{plotIdx}, goalEndCoords(plotIdx,1), goalEndCoords(plotIdx,2), goalEndCoords(plotIdx,3));
            else  % 2D plots (first three)
                addpoints(goalEndEffectors{plotIdx}, goalEndCoords(plotIdx,1), goalEndCoords(plotIdx,2));
            end
        end
    end

    function updateModelPosition(modelX, modelY, modelZ)
        % Update model position markers across all plots (for Teensy mode)
        % Plot order: XY, XZ, YZ, XYZ
        modelCoords = [modelX, modelY 0; modelX, modelZ 0; modelY, modelZ, 0; modelX, modelY, modelZ];
        
        for plotIdx = 1:4
            clearpoints(models{plotIdx});
            if plotIdx == 4  % 3D plot (last)
                addpoints(models{plotIdx}, modelCoords(plotIdx,1), modelCoords(plotIdx,2), modelCoords(plotIdx,3));
            else  % 2D plots (first three)
                addpoints(models{plotIdx}, modelCoords(plotIdx,1), modelCoords(plotIdx,2));
            end
        end
    end

%% ========================================================================
%  INVERSE KINEMATICS FUNCTION
%  ========================================================================

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