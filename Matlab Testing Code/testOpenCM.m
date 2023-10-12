%% OpenCM code tester
% Erick Nunez

%% Clean workspace
delete(instrfindall);
clear; clc; close all;

%% Ports
robotSerialPort = 'COM5';
robotBaud = 115200;
robotPktLen = 60;
rawPacket = nan(1,robotPktLen);
botSerial = serialport(robotSerialPort, robotBaud);

%% Log CSV file
logFileID = '_OpenCMframe_';
rawDataFileID = 'OpenCMraw_';
fileTime = datestr(now,'mmddyyHHMM');
csvFile = ['./Logs/Log',logFileID,fileTime,'.csv'];
rawFile = ['./Logs/Log',rawDataFileID,fileTime,'.csv'];
csvFileHeader = {'mTime','rTime','Q1','Q2','Q4','dQ1','dQ2','dQ4','I1','I2','I4','lTime'...
                'gQ1','gQ2','gQ4','gdQ1','gdQ2','gdQ4','gI1','gI2','gI4'};
writecell(csvFileHeader,csvFile);

%% start main collection loop
testRunning = true;
mTime = 0;
loopTime = tic;
while testRunning
    
    while botSerial.NumBytesAvailable < robotPktLen
    end
    if botSerial.NumBytesAvailable >= robotPktLen
        robotData = ReadFrame(botSerial);
        fprintf('q1=%f\tq2=%f\tq4=%f\n',robotData.frame(2),robotData.frame(3),robotData.frame(4));
    end
    mTime = mTime + toc(loopTime);
    loopTime = tic;
    if mTime > 10
        testRunning = false;
    end
    writematrix([mTime,robotData.frame],csvFile,'WriteMode','append');
end

%% post cleanup
instrreset;
botSerial = nan;
disp('.........Closing Port to Robot...............')

test = readmatrix(csvFile);
subplot(3,1,1)
plot(test(:,1));
grid on; title('Robot Read Times Per Loop');
