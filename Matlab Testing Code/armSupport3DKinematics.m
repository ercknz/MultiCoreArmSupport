function armSupport3DKinematics()
%% Arm support 3D kinematics
% code by Erick Nunez

%% Constants and limits
L1 = 0.419;
L2 = 0.520;
A1 = 0.073;
A2 = 0.082;
A3 = 0.072;
A4 = 0.035;

SHDR_LIMIT = [0,270];
ELVN_LIMIT = [-45,45];
ELBW_LIMIT = [atand(A4/L2),180];

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
T56 = matrixT(rotateZ(0),[0,-A4,0]');
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
HofL2 = sqrt(A4^2 + L2^2);
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

%% GUI
H = figure(100);
set(H,'Units','normalized','Position',[0 0 1 1])
axes; Hax = gca;
set(Hax,'Position',[0.1 0.15 0.5 0.8])
set(Hax,'View',[45,45])
axis([-1.2 1.2 -1.2 1.2 -1.2 1.2]);
xlabel('X');ylabel('Y');zlabel('Z');
animatedata(0,0,deg2rad(ELBW_LIMIT(1)))

Halpha = uicontrol('Style','text');
set(Halpha,'Units','normalized','Position',[0.7 0.85 0.2 0.05],...
    'String','0','Fontsize',20)
Hbeta = uicontrol('Style','text');
set(Hbeta,'Units','normalized','Position',[0.7 0.65 0.2 0.05],...
    'String','0','Fontsize',20)
Htheta = uicontrol('Style','text');
set(Htheta,'Units','normalized','Position',[0.7 0.45 0.2 0.05],...
    'String','0','Fontsize',20)

HXYZ = uicontrol('Style','text');
set(HXYZ,'Units','normalized','Position',[0.6 0.25 0.35 0.05],...
    'String',['fKine = X: ',num2str(t7(1,4)),' Y: ',num2str(t7(2,4)),' Z: ',num2str(t7(3,4))],'Fontsize',20)
HQ124 = uicontrol('Style','text');
set(HQ124,'Units','normalized','Position',[0.6 0.2 0.35 0.05],...
    'String',['iKine = Q1: ',num2str(0),' Q2: ',num2str(0),' Q4: ',num2str(0)],'Fontsize',20)

HsldAlpha = uicontrol('Style','slider');
set(HsldAlpha,'Units','normalized','Position',[0.65 0.75 0.3 0.05],...
    'Callback',@slide,'Max',SHDR_LIMIT(2),'Min',SHDR_LIMIT(1),...
    'Value',SHDR_LIMIT(1),'SliderStep',[0.01 0.1])
HsldBeta = uicontrol('Style','slider');
set(HsldBeta,'Units','normalized','Position',[0.65 0.55 0.3 0.05],...
    'Callback',@slide,'Max',ELVN_LIMIT(2),'Min',ELVN_LIMIT(1),...
    'Value',0,'SliderStep',[0.01 0.1])
HsldTheta = uicontrol('Style','slider');
set(HsldTheta,'Units','normalized','Position',[0.65 0.35 0.3 0.05],...
    'Callback',@slide,'Max',ELBW_LIMIT(2),'Min',ELBW_LIMIT(1),...
    'Value',ELBW_LIMIT(1),'SliderStep',[0.01 0.1])

    function slide(source,eventdata)
        % Callback function for slider
        alpha=round(get(HsldAlpha,'Value'));
        beta=round(get(HsldBeta,'Value'));
        theta=round(get(HsldTheta,'Value'));
        animatedata(deg2rad(alpha),deg2rad(beta),deg2rad(theta))
        set(Halpha,'String',[num2str(alpha),'(',num2str(alpha*pi/180),')'])
        set(Hbeta,'String',[num2str(beta),'(',num2str(beta*pi/180),')'])
        set(Htheta,'String',[num2str(theta),'(',num2str(theta*pi/180),')'])
        set(HXYZ,'String',['fKine = X: ',num2str(t7(1,4)),' Y: ',num2str(t7(2,4)),' Z: ',num2str(t7(3,4))])
        set(HQ124,'String',['iKine = Q1: ',num2str(Q(1)),' Q2: ',num2str(Q(2)),' Q4: ',num2str(Q(3))],'Fontsize',20)
    end

%% Animation of data
    function animatedata(q1,q2,q4)
        cla; hold on; grid on;
        t0 = [0,0,0];
        t1 = T01(q1);
        t2 = T02(q1,q2);
        t3 = T03(q1,q2);
        t4 = T04(q1,q2,q4);
        t5 = T05(q1,q2,q4);
        t6 = T06(q1,q2,q4);
        t7 = T07(q1,q2,q4);
        line([t0(1) t1(1,4)], [t0(2) t1(2,4)], [t0(3) t1(3,4)])
        line([t1(1,4) t2(1,4)], [t1(2,4) t2(2,4)], [t1(3,4) t2(3,4)])
        line([t2(1,4) t3(1,4)], [t2(2,4) t3(2,4)], [t2(3,4) t3(3,4)])
        line([t3(1,4) t4(1,4)], [t3(2,4) t4(2,4)], [t3(3,4) t4(3,4)])
        line([t4(1,4) t5(1,4)], [t4(2,4) t5(2,4)], [t4(3,4) t5(3,4)])
        line([t5(1,4) t6(1,4)], [t5(2,4) t6(2,4)], [t5(3,4) t6(3,4)])
        line([t6(1,4) t7(1,4)], [t6(2,4) t7(2,4)], [t6(3,4) t7(3,4)])
        drawnow
        iKine(t7(1,4),t7(2,4),t7(3,4))
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