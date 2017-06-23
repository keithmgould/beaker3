clear; clc;
%% Two-Wheeled Inverted Pendulum Controller Design
%
% For Sumission:    Matlab & Simulink Low Cost Hardware Challenge
% Author:           James Megariotis
%
% This File creates a continuous time state-space model of the 2 wheeled
% inverted pendulum robot system where the input is the applied voltage.
% This model is then converted to a discrete time state-space model to
% match the timestep specified in the simulink model.
%
% The K Gain matrix will be used as an Linear Quardatic Regulator in the
% simulink model.
%
% Use the value K in the K_LQR gain block in simulink file.

%% Continuous Time State-Space Model

% Declare Constant Values

mp=1.8;            % Mass of the pendulum
mw=0.2;            % Mass of the wheel
L=0.209;             % Length to Center of Gravity
r=0.042;            % Radius of the wheel
ip=0.049;           % inertia of the pendulum around the wheel axis
iw=0.000172125;     % inertia of the wheel around the wheel axis
R=0.0024;           % The resistance of the DC Motor
ke=0.0342857;       % Voltage constant for the DC Motor
km=0.017;           % Motor Torque Constant of the DC Motor
g=9.81;             % Gravity

% Declare additional constants to aviod repeated calculation

beta = 2*mw+((2*iw)/(r*r))+mp;
alpha = (ip*beta)+(2*mp*L*L*(mw+(iw/(r*r))));

% State-Space A Matrix Indices

a22=(2*km*ke*(mp*L*r-ip-mp*L*L))/(R*r*r*alpha);
a23=(mp*mp*g*L*L)/alpha;
a42=(2*km*ke*(r*beta-mp*L))/(R*r*r*alpha);
a43=(mp*g*L*beta)/alpha;
b21=(2*km*(ip+mp*L*L-mp*L*r))/(R*r*alpha);
b24=(2*km*(mp*L-r*beta))/(R*r*alpha);

% Construct Continuous Time State-Space Matrices (x'=Ax+Bu , y=Cx+Du)

a=[0 1 0 0; 0 a22 a23 0;0 0 0 1;0 a42 a43 0];
b=[0 ;b21 ; 0 ;b24];
c=[1 0 0 0; 0 0 1 0;]; % only observe x and theta
d=[0;0];

% Construct State-Space System

sys_ss = ss(a,b,c,d);


%% Discrete Time State-Space with LQR Modeling

% Set timestep value in seconds

Ts= 50 / 1000;

% Convert Continous Time Model to Discrete Time Model

sys_d = c2d(sys_ss,Ts,'zoh');

% Construct Discrete Time State-Space Matrices

A = sys_d.a;
B = sys_d.b;
C = sys_d.c;
D = sys_d.d;

% State Weights for Q weight matrix (Assigned arbitrarily/trial and error)

w = 1;          % x state variable weight
x = 0;          % xDot state variable weight
y = 1;          % phi state variable weight
z = 0;          % phiDot state variable weight

% Construct Q matrix (symmetric diagonal)

Q = [w 0 0 0;0 x 0 0;0 0 y 0;0 0 0 z];

% Assign R value for LQR input weight

R = 1;

% Find LQR gain Matrix K and new poles e

[K,S,e] = dlqr(A,B,Q,R);


dlmwrite('Ad_matrix.csv',A, 'precision', 5);
dlmwrite('Bd_matrix.csv',B, 'precision', 5);
dlmwrite('Kd_matrix.csv',K, 'precision', 5);


%% Closed Loop Discrete Time State-Space Model

% Construct state feedback discrete time state-space model matrices

Ac = [(A-B*K)];
Bc = [B];
Cc = [C];
Dc = [D];

% Construct discrete time state-space model for state feedback simulation

sys_cl = ss(Ac,Bc,Cc,Dc,Ts);


% Initialize time array in increments of model timestep

t = 0:Ts:10;

% Set initial conditions for simulation

x0=[0 0 0.2 0];     % Inintial angle: 0.2 radians

% Run simulation of system response based on initial angle of 0.2 radians.

% All states should converge on a value of zero to ensure robot maintains
% constant position, speed, tilt angle, and tilt rate of 0. Robot stays
% vertical and at initial position.

[y,t,x]=initial(sys_cl,x0,t);

% Plot all state output

figure;
plot(t,y(:,1),t,y(:,2));
legend('x','phi')
title('Response with Digital LQR Control')

%Plot control input due to LQR state feedback gain

figure;
plot(t,(K(1).*y(:,1)+K(2).*y(:,2)))
legend('Voltage Applied')
title('Control Input from Digital LQR Control')