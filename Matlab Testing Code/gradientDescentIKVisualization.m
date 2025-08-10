function gradientDescentIKVisualization()
%% Single-Step Gradient Descent IK Visualization
% Simulates real-time gradient descent with one iteration per timestep (1ms loop)
% Matches actual robot controller implementation
% Code by Erick Nunez (modified for single-step gradient descent)

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

SHDR_LIMIT = [0, 270];      % degrees
ELVN_LIMIT = [-45, 45];     % degrees
ELBW_LIMIT = [atand(offset/L2), 180];  % degrees

%% Single-Step Gradient Descent Parameters
gdParams = struct();
gdParams.learningRate = 0.02;  % Tuned for single-step performance
gdParams.alpha = 0.9;          % Higher momentum for single-step
gdParams.adaptiveLR = true;
gdParams.minLR = 0.001;
gdParams.maxLR = 0.1;
gdParams.tolerance = 1e-3;     % Position error tolerance (m)
gdParams.loopTime = 0.001;     % 1ms loop time to match robot

%% Simulation Parameters
simParams = struct();
simParams.maxTime = 5.0;       % Maximum simulation time (seconds)
simParams.displayRate = 30;    % Display update rate (Hz)
simParams.recordHistory = true;

%% Test Configuration
testMode = 'step';  % Options: 'step', 'trajectory', 'disturbance', 'multi-target'
targetUpdateRate = 0.5;  % How often to change targets (Hz)

%% Transformation Matrix Functions
rotateX = @(a) [1, 0, 0;
                0, cos(a), -sin(a);
                0, sin(a), cos(a)];
rotateZ = @(c) [cos(c), -sin(c), 0;
                sin(c), cos(c), 0;
                0, 0, 1];
matrixT = @(R,P) [R, P; 0, 0, 0, 1];

%% Forward Kinematics Functions
T01 = @(q1) matrixT(rotateZ(q1), [0, 0, 0]');
T12 = @(q2) matrixT(rotateX(pi/2)*rotateZ(q2), [A1, 0, 0]');
T23 = @(q2) matrixT(rotateZ(-q2), [L1, 0, 0]');
T34 = @(q4) matrixT(rotateX(-pi/2)*rotateZ(q4), [A2, 0, 0]');
T45 = matrixT(rotateZ(0), [0, 0, A3]');
T56 = matrixT(rotateZ(0), [0, -offset, 0]');
T67 = matrixT(rotateZ(0), [L2, 0, 0]');

T07 = @(q1,q2,q4) T01(q1)*T12(q2)*T23(q2)*T34(q4)*T45*T56*T67;

%% Forward Kinematics Function
function [x, y, z] = forwardKinematics(q1, q2, q4)
    T = T07(q1, q2, q4);
    x = T(1, 4);
    y = T(2, 4);
    z = T(3, 4);
end

%% ========================================================================
%  SINGLE-STEP GRADIENT DESCENT IK
%  ========================================================================

%% State structure for persistent variables (mimics robot controller)
ikState = struct();
ikState.q = [pi; 0; pi/2];           % Current joint angles
ikState.velocity = [0; 0; 0];        % Momentum velocity
ikState.prevError = inf;             % Previous error for adaptive LR
ikState.learningRate = gdParams.learningRate;
ikState.iterationCount = 0;          % Total iterations for current target
ikState.converged = false;

function ikState = singleStepGradientDescent(targetX, targetY, targetZ, ikState, params)
    % Single iteration of gradient descent (1ms loop time)
    
    % Current end effector position
    [currX, currY, currZ] = forwardKinematics(ikState.q(1), ikState.q(2), ikState.q(3));
    
    % Position error
    error = [targetX - currX; targetY - currY; targetZ - currZ];
    errorNorm = norm(error);
    
    % Check convergence
    if errorNorm < params.tolerance
        ikState.converged = true;
        return;
    else
        ikState.converged = false;
    end
    
    % Adaptive learning rate (adjust based on error change)
    if params.adaptiveLR
        if errorNorm > ikState.prevError
            % Error increased, reduce learning rate
            ikState.learningRate = max(params.minLR, ikState.learningRate * 0.7);
        else
            % Error decreased, slightly increase learning rate
            ikState.learningRate = min(params.maxLR, ikState.learningRate * 1.02);
        end
    end
    
    % Compute Jacobian numerically
    J = computeJacobian(ikState.q(1), ikState.q(2), ikState.q(3));
    
    % Single gradient descent step with momentum
    gradient = J' * error;
    ikState.velocity = params.alpha * ikState.velocity + ikState.learningRate * gradient;
    
    % Update joint angles
    qNew = ikState.q + ikState.velocity * params.loopTime;
    
    % Apply joint limits
    qNew(1) = constrainAngle(qNew(1), deg2rad(SHDR_LIMIT));
    qNew(2) = constrainAngle(qNew(2), deg2rad(ELVN_LIMIT));
    qNew(3) = constrainAngle(qNew(3), deg2rad(ELBW_LIMIT));
    
    % Update state
    ikState.q = qNew;
    ikState.prevError = errorNorm;
    ikState.iterationCount = ikState.iterationCount + 1;
end

function J = computeJacobian(q1, q2, q4)
    % Numerical Jacobian computation
    delta = 1e-6;
    [x0, y0, z0] = forwardKinematics(q1, q2, q4);
    
    % Partial derivatives
    [x1, y1, z1] = forwardKinematics(q1 + delta, q2, q4);
    dx_dq1 = (x1 - x0) / delta;
    dy_dq1 = (y1 - y0) / delta;
    dz_dq1 = (z1 - z0) / delta;
    
    [x2, y2, z2] = forwardKinematics(q1, q2 + delta, q4);
    dx_dq2 = (x2 - x0) / delta;
    dy_dq2 = (y2 - y0) / delta;
    dz_dq2 = (z2 - z0) / delta;
    
    [x3, y3, z3] = forwardKinematics(q1, q2, q4 + delta);
    dx_dq4 = (x3 - x0) / delta;
    dy_dq4 = (y3 - y0) / delta;
    dz_dq4 = (z3 - z0) / delta;
    
    J = [dx_dq1, dx_dq2, dx_dq4;
         dy_dq1, dy_dq2, dy_dq4;
         dz_dq1, dz_dq2, dz_dq4];
end

function angle = constrainAngle(angle, limits)
    % Constrain angle to joint limits
    angle = max(limits(1), min(limits(2), angle));
end

%% ========================================================================
%  GUI SETUP AND INITIALIZATION
%  ========================================================================

%% Figure Setup
Hfigure = figure('Name', 'Single-Step Gradient Descent IK (1ms Loop)', 'NumberTitle', 'off');
set(Hfigure, 'Units', 'normalized', 'Position', [0.02 0.05 0.96 0.85]);
set(Hfigure, 'UserData', struct('running', false, 'mode', testMode, 'ikState', ikState));
set(Hfigure, 'Renderer', 'opengl');
set(Hfigure, 'DoubleBuffer', 'on');

%% Create Main Visualization Subplots
% 1. 3D Robot Visualization
ax3D = subplot(2, 4, [1, 2, 5, 6]);
title('3D Robot - Single Step Gradient Descent (1ms loop)');
xlabel('X'); ylabel('Y'); zlabel('Z');
grid on; hold on;
axis equal;
axis([-1.2 1.2 -1.2 1.2 -0.6 0.6]);
view(45, 30);

% 2. Real-time Error Plot
axError = subplot(2, 4, 3);
title('Position Error vs Time');
xlabel('Time (s)');
ylabel('Position Error (m)');
grid on; hold on;
set(axError, 'YScale', 'log');
ylim([1e-4, 1]);

% 3. Learning Rate Plot
axLR = subplot(2, 4, 4);
title('Adaptive Learning Rate');
xlabel('Time (s)');
ylabel('Learning Rate');
grid on; hold on;

% 4. Joint Angles Plot
axJoints = subplot(2, 4, 7);
title('Joint Angles');
xlabel('Time (s)');
ylabel('Angle (deg)');
grid on; hold on;
legend('Q1', 'Q2', 'Q4', 'Location', 'best');

% 5. Joint Velocities Plot
axVelocity = subplot(2, 4, 8);
title('Joint Velocities (Gradient)');
xlabel('Time (s)');
ylabel('Velocity (rad/s)');
grid on; hold on;
legend('dQ1/dt', 'dQ2/dt', 'dQ4/dt', 'Location', 'best');

%% Create Visualization Elements
% Robot links (3D)
robotLinks3D = cell(7, 1);
for i = 1:7
    if i == 3 || i == 7
        robotLinks3D{i} = plot3(ax3D, [0 0], [0 0], [0 0], 'r-', 'LineWidth', 3);
    else
        robotLinks3D{i} = plot3(ax3D, [0 0], [0 0], [0 0], 'b-', 'LineWidth', 3);
    end
end

% End effector and target markers
endEffector3D = plot3(ax3D, 0, 0, 0, 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
target3D = plot3(ax3D, 0, 0, 0, 'r*', 'MarkerSize', 15, 'LineWidth', 2);

% Trajectory trace
trajectoryTrace = plot3(ax3D, [], [], [], 'g:', 'LineWidth', 1);

% Error plot lines
errorLine = plot(axError, [], [], 'b-', 'LineWidth', 2);
convergenceLine = plot(axError, [0, 10], [gdParams.tolerance, gdParams.tolerance], 'r--', 'LineWidth', 1);

% Learning rate line
lrLine = plot(axLR, [], [], 'm-', 'LineWidth', 2);

% Joint angle lines
q1Line = plot(axJoints, [], [], 'b-', 'LineWidth', 1.5);
q2Line = plot(axJoints, [], [], 'r-', 'LineWidth', 1.5);
q4Line = plot(axJoints, [], [], 'g-', 'LineWidth', 1.5);

% Joint velocity lines
dq1Line = plot(axVelocity, [], [], 'b-', 'LineWidth', 1.5);
dq2Line = plot(axVelocity, [], [], 'r-', 'LineWidth', 1.5);
dq4Line = plot(axVelocity, [], [], 'g-', 'LineWidth', 1.5);

%% Control Panel
controlPanel = uipanel('Position', [0.01 0.01 0.18 0.35]);

% Title
uicontrol(controlPanel, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0 0.92 1 0.08], 'String', 'Real-Time Control (1ms Loop)', ...
    'FontSize', 11, 'FontWeight', 'bold');

% Mode selection
uicontrol(controlPanel, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0 0.84 0.4 0.07], 'String', 'Mode:', 'FontSize', 10);
modeDropdown = uicontrol(controlPanel, 'Style', 'popupmenu', 'Units', 'normalized', ...
    'Position', [0.4 0.84 0.6 0.07], ...
    'String', {'Step Response', 'Trajectory', 'Disturbance', 'Multi-Target'}, ...
    'Callback', @changeMode, 'FontSize', 9);

% Start/Stop buttons
startBtn = uicontrol(controlPanel, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.05 0.74 0.4 0.08], 'String', 'START', ...
    'Callback', @startSimulation, 'BackgroundColor', [0.2 0.8 0.2], ...
    'FontSize', 10, 'FontWeight', 'bold');
stopBtn = uicontrol(controlPanel, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.55 0.74 0.4 0.08], 'String', 'STOP', ...
    'Callback', @stopSimulation, 'BackgroundColor', [0.8 0.2 0.2], ...
    'FontSize', 10, 'FontWeight', 'bold');

% Reset button
resetBtn = uicontrol(controlPanel, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.05 0.64 0.9 0.08], 'String', 'RESET', ...
    'Callback', @resetSimulation, 'BackgroundColor', [0.5 0.5 0.8], ...
    'FontSize', 10);

% Parameter controls
uicontrol(controlPanel, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0 0.54 1 0.06], 'String', '── Parameters ──', 'FontSize', 9);

uicontrol(controlPanel, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.02 0.46 0.45 0.06], 'String', 'Learn Rate:', 'FontSize', 9);
lrEdit = uicontrol(controlPanel, 'Style', 'edit', 'Units', 'normalized', ...
    'Position', [0.5 0.46 0.48 0.06], 'String', num2str(gdParams.learningRate), ...
    'Callback', @updateParams, 'FontSize', 9);

uicontrol(controlPanel, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.02 0.38 0.45 0.06], 'String', 'Momentum:', 'FontSize', 9);
momentumEdit = uicontrol(controlPanel, 'Style', 'edit', 'Units', 'normalized', ...
    'Position', [0.5 0.38 0.48 0.06], 'String', num2str(gdParams.alpha), ...
    'Callback', @updateParams, 'FontSize', 9);

uicontrol(controlPanel, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.02 0.30 0.45 0.06], 'String', 'Tolerance:', 'FontSize', 9);
tolEdit = uicontrol(controlPanel, 'Style', 'edit', 'Units', 'normalized', ...
    'Position', [0.5 0.30 0.48 0.06], 'String', num2str(gdParams.tolerance), ...
    'Callback', @updateParams, 'FontSize', 9);

adaptiveCheck = uicontrol(controlPanel, 'Style', 'checkbox', 'Units', 'normalized', ...
    'Position', [0.02 0.22 0.96 0.06], 'String', 'Adaptive Learning Rate', ...
    'Value', gdParams.adaptiveLR, 'Callback', @updateParams, 'FontSize', 9);

% Target position inputs
uicontrol(controlPanel, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0 0.14 1 0.06], 'String', '── Target Position ──', 'FontSize', 9);

uicontrol(controlPanel, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.02 0.08 0.15 0.05], 'String', 'X:', 'FontSize', 9);
targetXEdit = uicontrol(controlPanel, 'Style', 'edit', 'Units', 'normalized', ...
    'Position', [0.18 0.08 0.25 0.05], 'String', '0.6', 'FontSize', 9);

uicontrol(controlPanel, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.35 0.08 0.15 0.05], 'String', 'Y:', 'FontSize', 9);
targetYEdit = uicontrol(controlPanel, 'Style', 'edit', 'Units', 'normalized', ...
    'Position', [0.51 0.08 0.25 0.05], 'String', '0.2', 'FontSize', 9);

uicontrol(controlPanel, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.68 0.08 0.15 0.05], 'String', 'Z:', 'FontSize', 9);
targetZEdit = uicontrol(controlPanel, 'Style', 'edit', 'Units', 'normalized', ...
    'Position', [0.73 0.08 0.25 0.05], 'String', '0.1', 'FontSize', 9);

setTargetBtn = uicontrol(controlPanel, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.05 0.01 0.9 0.06], 'String', 'Set Target', ...
    'Callback', @setNewTarget, 'FontSize', 9);

%% Statistics Panel
statsPanel = uipanel('Position', [0.82 0.01 0.17 0.35]);

uicontrol(statsPanel, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0 0.92 1 0.08], 'String', 'Real-Time Statistics', ...
    'FontSize', 11, 'FontWeight', 'bold');

statsText = uicontrol(statsPanel, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.02 0.02 0.96 0.88], 'String', '', 'FontSize', 9, ...
    'HorizontalAlignment', 'left', 'FontName', 'FixedWidth');

%% ========================================================================
%  CALLBACK FUNCTIONS
%  ========================================================================

function changeMode(src, ~)
    modes = {'step', 'trajectory', 'disturbance', 'multi-target'};
    userData = get(Hfigure, 'UserData');
    userData.mode = modes{get(src, 'Value')};
    set(Hfigure, 'UserData', userData);
end

function updateParams(~, ~)
    gdParams.learningRate = str2double(get(lrEdit, 'String'));
    gdParams.alpha = str2double(get(momentumEdit, 'String'));
    gdParams.tolerance = str2double(get(tolEdit, 'String'));
    gdParams.adaptiveLR = get(adaptiveCheck, 'Value');
    
    % Update convergence line
    set(convergenceLine, 'YData', [gdParams.tolerance, gdParams.tolerance]);
end

function setNewTarget(~, ~)
    targetX = str2double(get(targetXEdit, 'String'));
    targetY = str2double(get(targetYEdit, 'String'));
    targetZ = str2double(get(targetZEdit, 'String'));
    
    % Update target marker
    set(target3D, 'XData', targetX, 'YData', targetY, 'ZData', targetZ);
    
    % Reset iteration count for new target
    userData = get(Hfigure, 'UserData');
    userData.ikState.iterationCount = 0;
    userData.ikState.converged = false;
    set(Hfigure, 'UserData', userData);
end

function startSimulation(~, ~)
    userData = get(Hfigure, 'UserData');
    userData.running = true;
    set(Hfigure, 'UserData', userData);
    
    switch userData.mode
        case 'step'
            runStepResponse();
        case 'trajectory'
            runTrajectoryTracking();
        case 'disturbance'
            runDisturbanceRejection();
        case 'multi-target'
            runMultiTargetSequence();
    end
end

function stopSimulation(~, ~)
    userData = get(Hfigure, 'UserData');
    userData.running = false;
    set(Hfigure, 'UserData', userData);
end

function resetSimulation(~, ~)
    % Reset state
    userData = get(Hfigure, 'UserData');
    userData.ikState.q = [pi; 0; pi/2];
    userData.ikState.velocity = [0; 0; 0];
    userData.ikState.learningRate = gdParams.learningRate;
    userData.ikState.iterationCount = 0;
    userData.ikState.converged = false;
    set(Hfigure, 'UserData', userData);
    
    % Clear plots
    set(errorLine, 'XData', [], 'YData', []);
    set(lrLine, 'XData', [], 'YData', []);
    set(q1Line, 'XData', [], 'YData', []);
    set(q2Line, 'XData', [], 'YData', []);
    set(q4Line, 'XData', [], 'YData', []);
    set(dq1Line, 'XData', [], 'YData', []);
    set(dq2Line, 'XData', [], 'YData', []);
    set(dq4Line, 'XData', [], 'YData', []);
    set(trajectoryTrace, 'XData', [], 'YData', [], 'ZData', []);
    
    % Update robot to home position
    updateRobotVisualization([pi; 0; pi/2]);
    
    % Clear statistics
    set(statsText, 'String', 'Ready to start simulation');
end

%% ========================================================================
%  SIMULATION FUNCTIONS
%  ========================================================================

function runStepResponse()
    % Single target step response test
    targetX = str2double(get(targetXEdit, 'String'));
    targetY = str2double(get(targetYEdit, 'String'));
    targetZ = str2double(get(targetZEdit, 'String'));
    
    % Update target marker
    set(target3D, 'XData', targetX, 'YData', targetY, 'ZData', targetZ);
    
    % Initialize data storage
    timeHistory = [];
    errorHistory = [];
    lrHistory = [];
    jointHistory = [];
    velocityHistory = [];
    trajectoryHistory = [];
    
    % Get initial state
    userData = get(Hfigure, 'UserData');
    ikState = userData.ikState;
    
    % Simulation loop (1ms timesteps)
    simTime = 0;
    lastDisplayTime = 0;
    convergenceTime = [];
    
    while simTime < simParams.maxTime
        userData = get(Hfigure, 'UserData');
        if ~userData.running
            break;
        end
        
        % Single gradient descent step (1ms loop)
        ikState = singleStepGradientDescent(targetX, targetY, targetZ, ikState, gdParams);
        
        % Get current position
        [currX, currY, currZ] = forwardKinematics(ikState.q(1), ikState.q(2), ikState.q(3));
        posError = norm([targetX - currX; targetY - currY; targetZ - currZ]);
        
        % Store data
        timeHistory(end+1) = simTime;
        errorHistory(end+1) = posError;
        lrHistory(end+1) = ikState.learningRate;
        jointHistory(:, end+1) = ikState.q;
        velocityHistory(:, end+1) = ikState.velocity;
        trajectoryHistory(:, end+1) = [currX; currY; currZ];
        
        % Check for first convergence
        if ikState.converged && isempty(convergenceTime)
            convergenceTime = simTime;
        end
        
        % Update display at display rate
        if simTime - lastDisplayTime >= 1/simParams.displayRate
            % Update robot visualization
            updateRobotVisualization(ikState.q);
            
            % Update plots
            set(errorLine, 'XData', timeHistory, 'YData', errorHistory);
            set(lrLine, 'XData', timeHistory, 'YData', lrHistory);
            set(q1Line, 'XData', timeHistory, 'YData', rad2deg(jointHistory(1,:)));
            set(q2Line, 'XData', timeHistory, 'YData', rad2deg(jointHistory(2,:)));
            set(q4Line, 'XData', timeHistory, 'YData', rad2deg(jointHistory(3,:)));
            set(dq1Line, 'XData', timeHistory, 'YData', velocityHistory(1,:));
            set(dq2Line, 'XData', timeHistory, 'YData', velocityHistory(2,:));
            set(dq4Line, 'XData', timeHistory, 'YData', velocityHistory(3,:));
            set(trajectoryTrace, 'XData', trajectoryHistory(1,:), ...
                                'YData', trajectoryHistory(2,:), ...
                                'ZData', trajectoryHistory(3,:));
            
            % Update statistics
            updateStatistics(simTime, posError, ikState, convergenceTime);
            
            % Update axes
            xlim(axError, [0, max(simTime, 0.1)]);
            xlim(axLR, [0, max(simTime, 0.1)]);
            xlim(axJoints, [0, max(simTime, 0.1)]);
            xlim(axVelocity, [0, max(simTime, 0.1)]);
            
            drawnow;
            lastDisplayTime = simTime;
        end
        
        % Advance time (1ms)
        simTime = simTime + gdParams.loopTime;
        
        % Real-time pause (optional - remove for faster simulation)
        % pause(gdParams.loopTime);
    end
    
    % Store final state
    userData.ikState = ikState;
    set(Hfigure, 'UserData', userData);
end

function runTrajectoryTracking()
    % Continuous trajectory tracking
    
    % Initialize data storage
    timeHistory = [];
    errorHistory = [];
    targetHistory = [];
    
    % Get initial state
    userData = get(Hfigure, 'UserData');
    ikState = userData.ikState;
    
    % Simulation loop
    simTime = 0;
    lastDisplayTime = 0;
    
    while simTime < simParams.maxTime
        userData = get(Hfigure, 'UserData');
        if ~userData.running
            break;
        end
        
        % Generate smooth trajectory
        targetX = 0.6 + 0.2 * cos(2*pi*0.2*simTime);
        targetY = 0.2 * sin(2*pi*0.2*simTime);
        targetZ = 0.1 + 0.05 * sin(2*pi*0.4*simTime);
        
        % Update target marker
        set(target3D, 'XData', targetX, 'YData', targetY, 'ZData', targetZ);
        
        % Single gradient descent step
        ikState = singleStepGradientDescent(targetX, targetY, targetZ, ikState, gdParams);
        
        % Get current position
        [currX, currY, currZ] = forwardKinematics(ikState.q(1), ikState.q(2), ikState.q(3));
        posError = norm([targetX - currX; targetY - currY; targetZ - currZ]);
        
        % Store data
        timeHistory(end+1) = simTime;
        errorHistory(end+1) = posError;
        targetHistory(:, end+1) = [targetX; targetY; targetZ];
        
        % Update display at display rate
        if simTime - lastDisplayTime >= 1/simParams.displayRate
            % Update robot
            updateRobotVisualization(ikState.q);
            
            % Update error plot
            set(errorLine, 'XData', timeHistory, 'YData', errorHistory);
            
            % Show target trajectory
            if size(targetHistory, 2) > 1
                set(trajectoryTrace, 'XData', targetHistory(1,:), ...
                                    'YData', targetHistory(2,:), ...
                                    'ZData', targetHistory(3,:));
            end
            
            % Update statistics
            updateStatistics(simTime, posError, ikState, []);
            
            xlim(axError, [max(0, simTime-5), max(simTime, 0.1)]);
            xlim(axLR, [max(0, simTime-5), max(simTime, 0.1)]);
            
            drawnow;
            lastDisplayTime = simTime;
        end
        
        % Advance time (1ms)
        simTime = simTime + gdParams.loopTime;
    end
    
    % Store final state
    userData.ikState = ikState;
    set(Hfigure, 'UserData', userData);
end

function runDisturbanceRejection()
    % Test disturbance rejection with sudden changes
    
    % Fixed target
    targetX = 0.6;
    targetY = 0.0;
    targetZ = 0.1;
    
    set(target3D, 'XData', targetX, 'YData', targetY, 'ZData', targetZ);
    
    % Initialize data storage
    timeHistory = [];
    errorHistory = [];
    disturbanceHistory = [];
    
    % Get initial state
    userData = get(Hfigure, 'UserData');
    ikState = userData.ikState;
    
    % Simulation loop
    simTime = 0;
    lastDisplayTime = 0;
    lastDisturbanceTime = 0;
    
    while simTime < simParams.maxTime
        userData = get(Hfigure, 'UserData');
        if ~userData.running
            break;
        end
        
        % Apply random disturbance every 2 seconds
        if simTime - lastDisturbanceTime > 2.0
            disturbance = 0.2 * (rand(3,1) - 0.5);
            ikState.q = ikState.q + disturbance;
            lastDisturbanceTime = simTime;
            disturbanceHistory(end+1) = norm(disturbance);
        else
            disturbanceHistory(end+1) = 0;
        end
        
        % Single gradient descent step
        ikState = singleStepGradientDescent(targetX, targetY, targetZ, ikState, gdParams);
        
        % Get current position
        [currX, currY, currZ] = forwardKinematics(ikState.q(1), ikState.q(2), ikState.q(3));
        posError = norm([targetX - currX; targetY - currY; targetZ - currZ]);
        
        % Store data
        timeHistory(end+1) = simTime;
        errorHistory(end+1) = posError;
        
        % Update display
        if simTime - lastDisplayTime >= 1/simParams.displayRate
            updateRobotVisualization(ikState.q);
            
            % Show disturbances as vertical lines
            distTimes = find(disturbanceHistory > 0);
            for dt = distTimes
                if dt <= length(timeHistory)
                    line(axError, [timeHistory(dt), timeHistory(dt)], [1e-4, 1], ...
                         'Color', 'r', 'LineStyle', '--', 'LineWidth', 1);
                end
            end
            
            set(errorLine, 'XData', timeHistory, 'YData', errorHistory);
            
            updateStatistics(simTime, posError, ikState, []);
            
            xlim(axError, [0, max(simTime, 0.1)]);
            
            drawnow;
            lastDisplayTime = simTime;
        end
        
        % Advance time
        simTime = simTime + gdParams.loopTime;
    end
    
    userData.ikState = ikState;
    set(Hfigure, 'UserData', userData);
end

function runMultiTargetSequence()
    % Multiple targets in sequence
    targets = [0.6, 0.2, 0.1;
               0.7, -0.1, 0.2;
               0.5, -0.3, 0.0;
               0.8, 0.0, 0.15;
               0.6, 0.3, -0.1];
    
    targetIdx = 1;
    targetSwitchTime = 1.5; % seconds per target
    lastSwitchTime = 0;
    
    % Initialize data storage
    timeHistory = [];
    errorHistory = [];
    convergenceTimes = [];
    
    % Get initial state
    userData = get(Hfigure, 'UserData');
    ikState = userData.ikState;
    
    % Simulation loop
    simTime = 0;
    lastDisplayTime = 0;
    
    while simTime < simParams.maxTime && targetIdx <= size(targets, 1)
        userData = get(Hfigure, 'UserData');
        if ~userData.running
            break;
        end
        
        % Switch target periodically
        if simTime - lastSwitchTime >= targetSwitchTime
            targetIdx = min(targetIdx + 1, size(targets, 1));
            lastSwitchTime = simTime;
            ikState.iterationCount = 0;
            ikState.converged = false;
        end
        
        % Current target
        targetX = targets(targetIdx, 1);
        targetY = targets(targetIdx, 2);
        targetZ = targets(targetIdx, 3);
        
        set(target3D, 'XData', targetX, 'YData', targetY, 'ZData', targetZ);
        
        % Single gradient descent step
        ikState = singleStepGradientDescent(targetX, targetY, targetZ, ikState, gdParams);
        
        % Get current position
        [currX, currY, currZ] = forwardKinematics(ikState.q(1), ikState.q(2), ikState.q(3));
        posError = norm([targetX - currX; targetY - currY; targetZ - currZ]);
        
        % Check convergence
        if ikState.converged && (isempty(convergenceTimes) || ...
            length(convergenceTimes) < targetIdx)
            convergenceTimes(targetIdx) = simTime - lastSwitchTime;
        end
        
        % Store data
        timeHistory(end+1) = simTime;
        errorHistory(end+1) = posError;
        
        % Update display
        if simTime - lastDisplayTime >= 1/simParams.displayRate
            updateRobotVisualization(ikState.q);
            
            % Mark target switches
            for i = 1:targetIdx-1
                switchTime = (i-1) * targetSwitchTime;
                line(axError, [switchTime, switchTime], [1e-4, 1], ...
                     'Color', 'k', 'LineStyle', ':', 'LineWidth', 1);
            end
            
            set(errorLine, 'XData', timeHistory, 'YData', errorHistory);
            
            % Update statistics with target info
            statsStr = sprintf(['Time: %.2f s\n' ...
                               'Target %d/%d: [%.2f, %.2f, %.2f]\n' ...
                               'Position Error: %.4f m\n' ...
                               'Converged: %s\n' ...
                               'Iterations: %d\n' ...
                               'Learning Rate: %.4f\n' ...
                               'Convergence Times:\n'], ...
                              simTime, targetIdx, size(targets, 1), ...
                              targetX, targetY, targetZ, ...
                              posError, string(ikState.converged), ...
                              ikState.iterationCount, ikState.learningRate);
            
            for i = 1:length(convergenceTimes)
                statsStr = [statsStr, sprintf('  Target %d: %.3f s\n', i, convergenceTimes(i))];
            end
            
            set(statsText, 'String', statsStr);
            
            xlim(axError, [0, max(simTime, 0.1)]);
            
            drawnow;
            lastDisplayTime = simTime;
        end
        
        % Advance time
        simTime = simTime + gdParams.loopTime;
    end
    
    userData.ikState = ikState;
    set(Hfigure, 'UserData', userData);
end

%% ========================================================================
%  VISUALIZATION UPDATE FUNCTIONS
%  ========================================================================

function updateRobotVisualization(q)
    % Calculate all transformation matrices
    q1 = q(1);
    q2 = q(2);
    q4 = q(3);
    
    t0 = [0, 0, 0];
    t1 = T01(q1); t1 = t1(1:3, 4)';
    t2 = T01(q1)*T12(q2); t2 = t2(1:3, 4)';
    t3 = T01(q1)*T12(q2)*T23(q2); t3 = t3(1:3, 4)';
    t4 = T01(q1)*T12(q2)*T23(q2)*T34(q4); t4 = t4(1:3, 4)';
    t5 = T01(q1)*T12(q2)*T23(q2)*T34(q4)*T45; t5 = t5(1:3, 4)';
    t6 = T01(q1)*T12(q2)*T23(q2)*T34(q4)*T45*T56; t6 = t6(1:3, 4)';
    t7 = T07(q1, q2, q4); t7 = t7(1:3, 4)';
    
    transforms = [t0; t1; t2; t3; t4; t5; t6; t7];
    
    % Update 3D robot
    for i = 1:7
        set(robotLinks3D{i}, 'XData', [transforms(i, 1), transforms(i+1, 1)], ...
                            'YData', [transforms(i, 2), transforms(i+1, 2)], ...
                            'ZData', [transforms(i, 3), transforms(i+1, 3)]);
    end
    
    % Update end effector
    set(endEffector3D, 'XData', t7(1), 'YData', t7(2), 'ZData', t7(3));
end

function updateStatistics(simTime, posError, ikState, convergenceTime)
    % Update statistics display
    statsStr = sprintf(['Simulation Time: %.3f s\n' ...
                       'Loop Time: %.1f ms\n' ...
                       '──────────────────\n' ...
                       'Position Error: %.4f m\n' ...
                       'Converged: %s\n' ...
                       'Iterations: %d\n' ...
                       'Iteration Rate: %.1f Hz\n' ...
                       '──────────────────\n' ...
                       'Learning Rate: %.4f\n' ...
                       'Momentum: %.2f\n' ...
                       'Tolerance: %.4f m\n' ...
                       '──────────────────\n' ...
                       'Joint Angles (deg):\n' ...
                       '  Q1: %6.2f\n' ...
                       '  Q2: %6.2f\n' ...
                       '  Q4: %6.2f\n'], ...
                      simTime, gdParams.loopTime*1000, ...
                      posError, string(ikState.converged), ...
                      ikState.iterationCount, ...
                      ikState.iterationCount/max(simTime, 0.001), ...
                      ikState.learningRate, gdParams.alpha, ...
                      gdParams.tolerance, ...
                      rad2deg(ikState.q(1)), ...
                      rad2deg(ikState.q(2)), ...
                      rad2deg(ikState.q(3)));
    
    if ~isempty(convergenceTime)
        statsStr = [statsStr, sprintf('──────────────────\n')];
        statsStr = [statsStr, sprintf('Convergence Time: %.3f s\n', convergenceTime)];
        statsStr = [statsStr, sprintf('Convergence Rate: %.1f iter/s\n', ...
                                     ikState.iterationCount/convergenceTime)];
    end
    
    set(statsText, 'String', statsStr);
end

%% ========================================================================
%  PERFORMANCE ANALYSIS FUNCTIONS
%  ========================================================================

function analyzeConvergenceRate()
    % Analyze convergence rate across workspace
    figure('Name', 'Convergence Rate Analysis');
    
    % Test grid
    xRange = linspace(0.4, 0.9, 10);
    yRange = linspace(-0.3, 0.3, 10);
    zRange = linspace(-0.1, 0.2, 5);
    
    convergenceTimes = zeros(length(xRange), length(yRange), length(zRange));
    
    % Test each point
    for ix = 1:length(xRange)
        for iy = 1:length(yRange)
            for iz = 1:length(zRange)
                targetX = xRange(ix);
                targetY = yRange(iy);
                targetZ = zRange(iz);
                
                % Reset state
                testState = struct();
                testState.q = [pi; 0; pi/2];
                testState.velocity = [0; 0; 0];
                testState.learningRate = gdParams.learningRate;
                testState.iterationCount = 0;
                testState.converged = false;
                testState.prevError = inf;
                
                % Simulate until convergence or timeout
                simTime = 0;
                while simTime < 2.0 && ~testState.converged
                    testState = singleStepGradientDescent(targetX, targetY, targetZ, ...
                                                         testState, gdParams);
                    simTime = simTime + gdParams.loopTime;
                end
                
                convergenceTimes(ix, iy, iz) = simTime;
            end
        end
    end
    
    % Plot results
    meanTimes = mean(convergenceTimes, 3);
    imagesc(xRange, yRange, meanTimes');
    colorbar;
    xlabel('X (m)');
    ylabel('Y (m)');
    title('Mean Convergence Time (s)');
    axis equal tight;
end

%% ========================================================================
%  MAIN EXECUTION
%  ========================================================================

% Initialize display
updateRobotVisualization([pi; 0; pi/2]);

% Add analysis button
analysisBtn = uicontrol('Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.20 0.01 0.15 0.04], 'String', 'Analyze Convergence', ...
    'Callback', @(~,~) analyzeConvergenceRate(), 'FontSize', 10);

% Set initial target
setNewTarget();

% Initial statistics
set(statsText, 'String', sprintf(['Single-Step Gradient Descent\n' ...
                                  '1ms Loop Time Simulation\n' ...
                                  '──────────────────\n' ...
                                  'Ready to start\n' ...
                                  '\n' ...
                                  'This simulates the actual\n' ...
                                  'robot controller with one\n' ...
                                  'gradient step per 1ms loop\n' ...
                                  '\n' ...
                                  'Select mode and press START']));

fprintf('===================================\n');
fprintf('Single-Step Gradient Descent IK\n');
fprintf('1ms Loop Time Simulation\n');
fprintf('===================================\n');
fprintf('This simulates your actual robot\n');
fprintf('controller with one gradient step\n');
fprintf('per millisecond loop.\n');
fprintf('\n');
fprintf('Select test mode and click START\n');
fprintf('===================================\n');

end