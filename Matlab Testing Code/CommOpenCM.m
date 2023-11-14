classdef CommOpenCM < handle
    %% OpenCM microcontroller communication class
    % This class is used to test the robotics arm support portion that is
    % controlled by the OpenCM microcontroller.
    %
    % Script by erick nunez
    
    %% ------------------------------------------------------------------------
    % Properties
    %--------------------------------------------------------------------------
    properties (Access = public)
        serialObj
        baud
        dt = 0.008
        port
        rawBytes
        frameData = nan(1,24)
        
        txHeader = uint8([150, 10, 1, 101])
        txPacketLen = 60
        
        rxHeader = uint8([170, 6, 9, 69])
        rxPacketLen = 100
    end
    
    properties (Access = private)
        CommOpen = false;
    end
    
    %% ------------------------------------------------------------------------
    % Methods
    %--------------------------------------------------------------------------
    methods
        function obj = CommOpenCM(port, baud)
            obj.rawBytes = nan(1,obj.rxPacketLen);
            obj.port = port;
            obj.baud = baud;
        end
        
        function Start(obj)
            obj.serialObj = serialport(obj.port, obj.baud);
            obj.CommOpen = true;
        end
        
        function Stop(obj)
            instrreset;
            obj.serialObj = nan;
            obj.CommOpen = false;
        end
        
        function out = IsOpen(obj)
            out = obj.CommOpen;
        end
        
        function numOfBytes = BytesAvailable(obj)
            numOfBytes = obj.serialObj.NumBytesAvailable;
        end
        
        function ReadFrame(obj)
            while obj.serialObj.NumBytesAvailable < obj.rxPacketLen
                if ~obj.CommOpen
                    break
                end
            end
            if obj.CommOpen
                obj.rawBytes = read(obj.serialObj,obj.rxPacketLen,'uint8');
            end
            tempHeader = obj.rawBytes(1:4);
            inCS = swapbytes(typecast(uint8(obj.rawBytes(end-1:end)),'uint16'));
            cCS = sum(obj.rawBytes(1:end-2));
            if (sum(tempHeader == obj.rxHeader)==4) && (inCS == cCS)
                % Total Time
                obj.frameData(1) = typecast(uint8(obj.rawBytes(5:8)),'uint32');
                % presQ
                obj.frameData(2) = double(typecast(uint8(obj.rawBytes(9:12)),'int32'));
                obj.frameData(3) = double(typecast(uint8(obj.rawBytes(13:16)),'int32'));
                obj.frameData(4) = double(typecast(uint8(obj.rawBytes(17:20)),'int32'));
                % presQdot
                obj.frameData(5) = double(typecast(uint8(obj.rawBytes(21:24)),'int32'));
                obj.frameData(6) = double(typecast(uint8(obj.rawBytes(25:28)),'int32'));
                obj.frameData(7) = double(typecast(uint8(obj.rawBytes(29:32)),'int32'));
                % presQ Cts
                obj.frameData(8) = double(typecast(uint8(obj.rawBytes(33:36)),'int32'));
                obj.frameData(9) = double(typecast(uint8(obj.rawBytes(37:40)),'int32'));
                obj.frameData(10) = double(typecast(uint8(obj.rawBytes(41:44)),'int32'));
                % presXYZ
                obj.frameData(11) = double(typecast(uint8(obj.rawBytes(45:48)),'int32'));
                obj.frameData(12) = double(typecast(uint8(obj.rawBytes(49:52)),'int32'));
                obj.frameData(13) = double(typecast(uint8(obj.rawBytes(53:56)),'int32'));
                % presXYZdot
                obj.frameData(14) = double(typecast(uint8(obj.rawBytes(57:60)),'int32'));
                obj.frameData(15) = double(typecast(uint8(obj.rawBytes(61:64)),'int32'));
                obj.frameData(16) = double(typecast(uint8(obj.rawBytes(65:68)),'int32'));
                % goalQ
                obj.frameData(17) = double(typecast(uint8(obj.rawBytes(69:72)),'int32'));
                obj.frameData(18) = double(typecast(uint8(obj.rawBytes(73:76)),'int32'));
                obj.frameData(19) = double(typecast(uint8(obj.rawBytes(77:80)),'int32'));
                % goalQ cts
                obj.frameData(20) = double(typecast(uint8(obj.rawBytes(81:84)),'int32'));
                obj.frameData(21) = double(typecast(uint8(obj.rawBytes(85:88)),'int32'));
                obj.frameData(22) = double(typecast(uint8(obj.rawBytes(89:92)),'int32'));
                % Torque Mode
                obj.frameData(23) = double(obj.rawBytes(94));
                % Loop Time
                obj.frameData(24) = typecast(uint8(obj.rawBytes(end-5:end-2)),'uint32');
                % Corrections
                obj.frameData(1) = obj.frameData(1)*0.001;
                obj.frameData(2:7) = obj.frameData(2:7)./10000;
                obj.frameData(11:19) = obj.frameData(11:19)./10000;
                obj.frameData(24) = obj.frameData(24)*0.001;
            end
        end
        
        function SendGoal(obj, X, Y, Z, Xdot, Ydot, Zdot)
            writePacket = uint8(zeros(1,obj.txPacketLen));
            writePacket(1:4) = obj.txHeader;
            writePacket(6) = uint8(2);
            writePacket(9:12) = typecast(int32(X*10000),'uint8');
            writePacket(13:16) = typecast(int32(Y*10000),'uint8');
            writePacket(17:20) = typecast(int32(Z*10000),'uint8');
            writePacket(21:24) = typecast(int32(Xdot*10000),'uint8');
            writePacket(25:28) = typecast(int32(Ydot*10000),'uint8');
            writePacket(29:32) = typecast(int32(Zdot*10000),'uint8');
            checkSum = sum(writePacket);
            writePacket(end-1) = uint8(floor(checkSum/256));
            writePacket(end) = uint8(mod(checkSum,256));
            if obj.CommOpen
                write(obj.serialObj,writePacket,'uint8');
            end
        end
        
        function SendRequest(obj)
            writePacket = uint8(zeros(1,obj.txPacketLen));
            writePacket(1:4) = obj.txHeader;
            writePacket(6) = uint8(4);
            checkSum = sum(writePacket);
            writePacket(end-1) = uint8(floor(checkSum/256));
            writePacket(end) = uint8(mod(checkSum,256));
            if obj.CommOpen
                write(obj.serialObj,writePacket,'uint8');
            end
        end
        
        function SendTorqueChange(obj,torqueMode)
            %  5: Full Admittance Control
            % 10: Planer Admittance Control
            % 15: Vertical Admittance Control
            % 20: Fully Passive
            writePacket = uint8(zeros(1,obj.txPacketLen));
            writePacket(1:4) = obj.txHeader;
            writePacket(6) = uint8(1);
            writePacket(8) = uint8(torqueMode);
            checkSum = sum(writePacket);
            writePacket(end-1) = uint8(floor(checkSum/256));
            writePacket(end) = uint8(mod(checkSum,256));
            if obj.CommOpen
                write(obj.serialObj,writePacket,'uint8');
            end
        end
    end
    
end