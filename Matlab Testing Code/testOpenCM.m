%% OpenCM code tester
% Erick Nunez
function testOpenCM()

%% Clean workspace
delete(instrfindall);

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

end

%% ReadFrame function
function out = ReadFrame(obj)
    rxHeader = uint8([170, 6, 9, 69]);
    packetLen = 60;
    frameData = nan(1,23);

    rawBytes = read(obj,packetLen,'uint8');

    tempHeader = rawBytes(1:4);
    inCS = swapbytes(typecast(uint8(rawBytes(end-1:end)),'uint16'));
    cCS = sum(rawBytes(1:end-2));
    if (sum(tempHeader == rxHeader)==4) && (inCS == cCS)
        % Total Time
        frameData(1) = typecast(uint8(rawBytes(5:8)),'uint32');
        % Pres Positions [Q]
        frameData(2) = double(typecast(uint8(rawBytes(9:12)),'int32'))/10000;
        frameData(3) = double(typecast(uint8(rawBytes(13:16)),'int32'))/10000;
        frameData(4) = double(typecast(uint8(rawBytes(17:20)),'int32'))/10000;
        % Pres Velocities [dQ]
        frameData(5) = double(typecast(uint8(rawBytes(21:24)),'int32'))/10000;
        frameData(6) = double(typecast(uint8(rawBytes(25:28)),'int32'))/10000;
        frameData(7) = double(typecast(uint8(rawBytes(29:32)),'int32'))/10000;
        % Pres Currents [I]
        frameData(8) = double(typecast(uint8(rawBytes(33:36)),'int32'))/10000;
        frameData(9) = double(typecast(uint8(rawBytes(37:40)),'int32'))/10000;
        frameData(10) = double(typecast(uint8(rawBytes(41:44)),'int32'))/10000;
        % Loop Time
        frameData(11) = typecast(uint8(rawBytes(end-5:end-2)),'uint32');
    end
    out.frame = frameData;
    out.rawBytes = rawBytes;
end

%% SendPVI Function
function SendGoal(obj, driveMode, q1, q2, q4, dq1, dq2, dq4, i1, i2, i4)
    txHeader = uint8([150, 10, 1, 101]);
    packetLen = 60;
    
    writePacket = uint8(zeros(1,packetLen));
    writePacket(1:4) = txHeader;
    writePacket(6) = uint8(driveMode);
    % Goal Positions [Q]
    writePacket(9:12) = typecast(int32(q1*10000),'uint8');
    writePacket(13:16) = typecast(int32(q2*10000),'uint8');
    writePacket(17:20) = typecast(int32(q4*10000),'uint8');
    % Goal Velocities [dQ]
    writePacket(21:24) = typecast(int32(dq1*10000),'uint8');
    writePacket(25:28) = typecast(int32(dq2*10000),'uint8');
    writePacket(29:32) = typecast(int32(dq4*10000),'uint8');
    % Goal Currents [I]
    writePacket(33:36) = typecast(int32(i1*10000),'uint8');
    writePacket(37:40) = typecast(int32(i2*10000),'uint8');
    writePacket(41:44) = typecast(int32(i4*10000),'uint8');
    
    checkSum = sum(writePacket);
    writePacket(end-1) = uint8(floor(checkSum/256));
    writePacket(end) = uint8(mod(checkSum,256));
    
    write(obj,writePacket,'uint8');
end