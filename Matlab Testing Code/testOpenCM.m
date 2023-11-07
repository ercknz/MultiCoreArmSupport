%% OpenCM code tester
% Erick Nunez

%% Clean workspace
delete(instrfindall);
clear; clc; close all;

%% Ports
robotSerialPort = 'COM7';
robotBaud = 115200;
botSerial = CommOpenCM(robotSerialPort, robotBaud);
packetLen = 80;

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
loopTime = tic;
frames = 0;

while testRunning
    frames = frames + 1;
    botSerial.SendRequest();
    while botSerial.BytesAvailable < packetLen
    end
    if botSerial.BytesAvailable >= packetLen
        ReadFrame(botSerial);
    end
    mTime = mTime + toc(loopTime);
    loopTime = tic;
    fprintf('frame%f,q1=%f\tq2=%f\tq4=%f\ttime:%f\t%f\t%f\t%f\t%f\n',...
        frames,botSerial.frameData(2),botSerial.frameData(3),botSerial.frameData(4),...
        botSerial.frameData(1),...
        botSerial.frameData(18),botSerial.frameData(19),botSerial.frameData(20),botSerial.frameData(21));
    if frames > 10
        testRunning = false;
    end
    %writematrix([mTime,botSerial.frameData],csvFile,'WriteMode','append');
end

%% post cleanup
instrreset;
botSerial.Stop();
disp('.........Closing Port to Robot...............')

% test = readmatrix(csvFile);
% subplot(3,1,1)
% plot(test(:,1));
% grid on; title('Robot Read Times Per Loop');
