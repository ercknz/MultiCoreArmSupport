classdef armSupportRobot < handle
    %% Arm Support Communication class
    % This class is setup to recieve data from the robotic arm support in order
    % to log data and interact with virtual objects such as springs and walls.
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
        frameData = nan(1,39)
        
        configHeader = uint8([150, 0, 69, 8])
        modHeader = uint8([150, 10, 10, 96])
        ctrlHeader = uint8([150, 50, 50, 175])
        txPacketLen = 60
        
        rxHeader = uint8([170, 8, 69, 0])
        rxPacketLen = 170
    end
    
    properties (Access = private)
        CommOpen = false;
    end
    
    %% ------------------------------------------------------------------------
    % Methods
    %--------------------------------------------------------------------------
    methods
        function obj = armSupportRobot(port, baud)
            obj.rawBytes = nan(1,obj.rxPacketLen);
            obj.port = port;
            obj.baud = baud;
        end
        
        function Start(obj)
            obj.serialObj = serialport(obj.port, obj.baud);
            obj.CommOpen = true;
        end
        
        function Stop(obj)
            % instrreset;
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
                % presXYZ
                obj.frameData(8) = double(typecast(uint8(obj.rawBytes(33:36)),'int32'));
                obj.frameData(9) = double(typecast(uint8(obj.rawBytes(37:40)),'int32'));
                obj.frameData(10) = double(typecast(uint8(obj.rawBytes(41:44)),'int32'));
                % presXYZdot
                obj.frameData(11) = double(typecast(uint8(obj.rawBytes(45:48)),'int32'));
                obj.frameData(12) = double(typecast(uint8(obj.rawBytes(49:52)),'int32'));
                obj.frameData(13) = double(typecast(uint8(obj.rawBytes(53:56)),'int32'));
                % goalQ
                obj.frameData(14) = double(typecast(uint8(obj.rawBytes(57:60)),'int32'));
                obj.frameData(15) = double(typecast(uint8(obj.rawBytes(61:64)),'int32'));
                obj.frameData(16) = double(typecast(uint8(obj.rawBytes(65:68)),'int32'));
                % goalQdot
                obj.frameData(17) = double(typecast(uint8(obj.rawBytes(69:72)),'int32'));
                obj.frameData(18) = double(typecast(uint8(obj.rawBytes(73:76)),'int32'));
                obj.frameData(19) = double(typecast(uint8(obj.rawBytes(77:80)),'int32'));
                % Raw Force and Torque Values (Cts)
                % obj.frameData(20) = double(typecast(uint8(obj.rawBytes(83:84)),'uint16'));
                % obj.frameData(21) = double(typecast(uint8(obj.rawBytes(85:86)),'uint16'));
                % obj.frameData(22) = double(typecast(uint8(obj.rawBytes(87:88)),'uint16'));
                % obj.frameData(23) = double(typecast(uint8(obj.rawBytes(89:90)),'uint16'));
                % obj.frameData(24) = double(typecast(uint8(obj.rawBytes(91:92)),'uint16'));
                % obj.frameData(25) = double(typecast(uint8(obj.rawBytes(93:94)),'uint16'));
                % XYZ Global Forces
                % obj.frameData(26) = double(typecast(uint8(obj.rawBytes(95:98)),'int32'));
                % obj.frameData(27) = double(typecast(uint8(obj.rawBytes(99:102)),'int32'));
                % obj.frameData(28) = double(typecast(uint8(obj.rawBytes(103:106)),'int32'));
                % % XYZ Global Torques
                % obj.frameData(29) = double(typecast(uint8(obj.rawBytes(107:110)),'int32'));
                % obj.frameData(30) = double(typecast(uint8(obj.rawBytes(111:114)),'int32'));
                % obj.frameData(31) = double(typecast(uint8(obj.rawBytes(115:118)),'int32'));
                % % Model XYZ Pos 
                % obj.frameData(32) = double(typecast(uint8(obj.rawBytes(119:122)),'int32'));
                % obj.frameData(33) = double(typecast(uint8(obj.rawBytes(123:126)),'int32'));
                % obj.frameData(34) = double(typecast(uint8(obj.rawBytes(127:130)),'int32'));
                % % Model XYZ Vel
                % obj.frameData(35) = double(typecast(uint8(obj.rawBytes(131:134)),'int32'));
                % obj.frameData(36) = double(typecast(uint8(obj.rawBytes(135:138)),'int32'));
                % obj.frameData(37) = double(typecast(uint8(obj.rawBytes(139:142)),'int32'));
                
                % Torque Mode
                obj.frameData(38) = double(obj.rawBytes(164));
                % Loop Time
                obj.frameData(39) = typecast(uint8(obj.rawBytes(end-5:end-2)),'uint32');
                % Corrections
                obj.frameData(1) = obj.frameData(1)*0.001;
                obj.frameData(2:19) = obj.frameData(2:19)./1000;
                % obj.frameData(26:37) = obj.frameData(26:37)./1000;
                obj.frameData(39) = obj.frameData(39)*0.001;
            end
        end
        
        function SendMassXY(obj, MassXY)
            writePacket = uint8(zeros(1,obj.txPacketLen));
            writePacket(1:4) = obj.modHeader;
            writePacket(5) = uint8(1);
            writePacket(6:9) = typecast(int32(MassXY*1000),'uint8');
            checkSum = sum(writePacket);
            writePacket(end-1) = uint8(floor(checkSum/256));
            writePacket(end) = uint8(mod(checkSum,256));
            if obj.CommOpen
                write(obj.serialObj,writePacket,'uint8');
            end
        end
        
        function SendMassZ(obj, MassZ)
            writePacket = uint8(zeros(1,obj.txPacketLen));
            writePacket(1:4) = obj.modHeader;
            writePacket(5) = uint8(2);
            writePacket(10:13) = typecast(int32(MassZ*1000),'uint8');
            checkSum = sum(writePacket);
            writePacket(end-1) = uint8(floor(checkSum/256));
            writePacket(end) = uint8(mod(checkSum,256));
            if obj.CommOpen
                write(obj.serialObj,writePacket,'uint8');
            end
        end
        
        function SendDampingXY(obj, DampingXY)
            writePacket = uint8(zeros(1,obj.txPacketLen));
            writePacket(1:4) = obj.modHeader;
            writePacket(5) = uint8(4);
            writePacket(14:17) = typecast(int32(DampingXY*1000),'uint8');
            checkSum = sum(writePacket);
            writePacket(end-1) = uint8(floor(checkSum/256));
            writePacket(end) = uint8(mod(checkSum,256));
            if obj.CommOpen
                write(obj.serialObj,writePacket,'uint8');
            end
        end
        
        function SendDampingZ(obj, DampingZ)
            writePacket = uint8(zeros(1,obj.txPacketLen));
            writePacket(1:4) = obj.modHeader;
            writePacket(5) = uint8(8);
            writePacket(18:21) = typecast(int32(DampingZ*1000),'uint8');
            checkSum = sum(writePacket);
            writePacket(end-1) = uint8(floor(checkSum/256));
            writePacket(end) = uint8(mod(checkSum,256));
            if obj.CommOpen
                write(obj.serialObj,writePacket,'uint8');
            end
        end
        
        function SendSpring(obj, SpringRatio)
            writePacket = uint8(zeros(1,obj.txPacketLen));
            writePacket(1:4) = obj.modHeader;
            writePacket(6) = uint8(1);
            writePacket(36) = uint8(SpringRatio*100);
            checkSum = sum(writePacket);
            writePacket(end-1) = uint8(floor(checkSum/256));
            writePacket(end) = uint8(mod(checkSum,256));
            if obj.CommOpen
                write(obj.serialObj,writePacket,'uint8');
            end
        end

        function SendForceFilter(obj, forceFilterVal)
            writePacket = uint8(zeros(1,obj.txPacketLen));
            writePacket(1:4) = obj.modHeader;
            writePacket(6) = uint8(2);
            writePacket(37) = uint8(forceFilterVal*100);
            checkSum = sum(writePacket);
            writePacket(end-1) = uint8(floor(checkSum/256));
            writePacket(end) = uint8(mod(checkSum,256));
            if obj.CommOpen
                write(obj.serialObj,writePacket,'uint8');
            end
        end
        
        function SendExtForces(obj, eFx, eFy, eFz)
            writePacket = uint8(zeros(1,obj.txPacketLen));
            writePacket(1:4) = obj.modHeader;
            writePacket(5) = uint8(16);
            writePacket(24:27) = typecast(int32(eFx*1000),'uint8');
            writePacket(28:31) = typecast(int32(eFy*1000),'uint8');
            writePacket(32:35) = typecast(int32(eFz*1000),'uint8');
            checkSum = sum(writePacket);
            writePacket(end-1) = uint8(floor(checkSum/256));
            writePacket(end) = uint8(mod(checkSum,256));
            if obj.CommOpen
                write(obj.serialObj,writePacket,'uint8');
            end
        end
        
        function SendNewBotGoals(obj, q1, q2, q4)
            writePacket = uint8(zeros(1,obj.txPacketLen));
            writePacket(1:4) = obj.ctrlHeader;
            writePacket(5) = uint8(1);
            writePacket(8:11) = typecast(int32(q1*1000),'uint8');
            writePacket(12:15) = typecast(int32(q2*1000),'uint8');
            writePacket(16:19) = typecast(int32(q4*1000),'uint8');
            checkSum = sum(writePacket);
            writePacket(end-1) = uint8(floor(checkSum/256));
            writePacket(end) = uint8(mod(checkSum,256));
            if obj.CommOpen
                write(obj.serialObj,writePacket,'uint8');
            end
        end

        function SendNewModelGoals(obj, X, Y, Z)
            writePacket = uint8(zeros(1,obj.txPacketLen));
            writePacket(1:4) = obj.ctrlHeader;
            writePacket(6) = uint8(1);
            writePacket(32:35) = typecast(int32(X*1000),'uint8');
            writePacket(36:39) = typecast(int32(Y*1000),'uint8');
            writePacket(40:43) = typecast(int32(Z*1000),'uint8');
            checkSum = sum(writePacket);
            writePacket(end-1) = uint8(floor(checkSum/256));
            writePacket(end) = uint8(mod(checkSum,256));
            if obj.CommOpen
                write(obj.serialObj,writePacket,'uint8');
            end
        end

        function SendTorqueChange(obj, torqueMode)
            %  5: Full Admittance Control
            % 10: Planer Admittance Control
            % 15: Vertical Admittance Control
            % 20: Fully Passive
            writePacket = uint8(zeros(1,obj.txPacketLen));
            writePacket(1:4) = obj.ctrlHeader;
            writePacket(7) = uint8(1);
            writePacket(57) = uint8(torqueMode);
            checkSum = sum(writePacket);
            writePacket(end-1) = uint8(floor(checkSum/256));
            writePacket(end) = uint8(mod(checkSum,256));
            if obj.CommOpen
                write(obj.serialObj,writePacket,'uint8');
            end
        end
        
        function CommSendConfig(obj)
            writePacket = uint8(zeros(1,obj.txPacketLen));
            writePacket(1:4) = obj.configHeader;
            writePacket(5) = uint8(0);
            checkSum = sum(writePacket);
            writePacket(end-1) = uint8(floor(checkSum/256));
            writePacket(end) = uint8(mod(checkSum,256));
            if obj.CommOpen
                write(obj.serialObj,writePacket,'uint8');
            end
        end
    end
    
end