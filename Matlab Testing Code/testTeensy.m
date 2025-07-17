%% OpenCM code tester
% Erick Nunez

%% Clean workspace
delete(instrfindall);
clear; clc; close all;

%% Ports
robotSerialPort = 'COM7';
robotBaud = 1000000;
botSerial = armSupportRobot(robotSerialPort, robotBaud);
packetLen = 150;

%% Log CSV file
% logFileID = '_OpenCMframe_';
% fileTime = datestr(now,'mmddyyHHMM');
% csvFile = ['./Logs/Log',logFileID,fileTime,'.csv'];
% csvFileHeader = {'mTime','rTime','Q1','Q2','Q4','dQ1','dQ2','dQ4','I1','I2','I4',...
%                 'X','Y','Z','dX','dY','dZ','lTime'};
% writecell(csvFileHeader,csvFile);

%% start main collection loop
botSerial.Start();
testRunning = true;
mTime = 0;
frames = 0;
data = nan(4000,9);

while testRunning
    loopTime = tic;
    frames = frames + 1;
    while botSerial.BytesAvailable < packetLen
    end
    if botSerial.BytesAvailable >= packetLen
        ReadFrame(botSerial);
    end
    lTime = toc(loopTime);
    mTime = mTime + lTime;
    fprintf('frame:%d,X=%f\tY=%f\tZ=%f\ttime:%f\t%f\t%f\n',...
        frames,botSerial.frameData(32),botSerial.frameData(33),botSerial.frameData(34),...
        botSerial.frameData(1),...
        mTime, lTime);
    data(frames,:) = [frames,mTime,lTime,...
                      botSerial.frameData(1),botSerial.frameData(39),botSerial.frameData(38),...
                      botSerial.frameData(32),botSerial.frameData(33),botSerial.frameData(34)];
    if botSerial.frameData(1) > 30
        testRunning = false;
    end
    %writematrix([mTime,botSerial.frameData],csvFile,'WriteMode','append');
end

%% post cleanup
botSerial.Stop();
disp('.........Closing Port to Robot...............')
data = data(~isnan(data(:,1)),:);
mean(diff(data(:,2)))
mean(data(:,3))
