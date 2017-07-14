clear; clc;

% properties of robot and earth
M  = 0.2;       % mass of both wheels kg
m  = 1.66;      % mass of body kg
g  = 9.8;       % gravity yo.
l  = 0.181;     % length from wheels to robot's COM (meters)
bf = 0.0001;    % friction between wheels and floor

% calculations
Lfull = l * 2;              % full length or robot
I  = (1/3)* m * Lfull^2;    % rotational intertia of robot ( was .071)
denom = M*l^2*m+I*M+I*m;

% matrix prep
a22 = bf*(m*l^2+I) / denom;
a23 = m^2*g*l^2 / denom;
a42 = -bf*m*l / denom;
a43 = m*l*g*(M + m) / denom;

a = [ 0 1   0   0;
      0 a22 a23 0;
      0 0   0   1;
      0 a42 a43 0;
    ];

b2 = (I + m * l^2) / denom;
b4 = m * l / denom;

b = [0; b2; 0; b4];

c = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];

d = [0; 0; 0; 0];

% Name States, Inputs, and Outputs

states = {'xDot' 'xDDot' 'ThDot' 'ThDDot'};
inputs = ('Va');
outputs= {'x' 'xDot' 'Theta' 'ThDot'};

% Construct State-Space System

sys_ss = ss(a,b,c,d,'statename',states,'inputname',inputs,'outputname',outputs);

poles_continuous = eig(a);

%% Discrete Time State-Space with LQR Modeling

% Set timestep value in seconds
Ts=0.02; % 50Hz

% Convert Continous Time Model to Discrete Time Model

sys_d = c2d(sys_ss,Ts,'zoh');

% Construct Discrete Time State-Space Matrices

A = sys_d.a;
B = sys_d.b;
C = sys_d.c;
D = sys_d.d;

poles_discrete = eig(A);

% State Weights for Q weight matrix (Assigned arbitrarily/trial and error)

w = 1;            % x state variable weight
x = 1;          % xDot state variable weight
y = 1;           % theta state variable weight
z = 1;            % thetaDot state variable weight

% Construct Q matrix (symmetric diagonal)

Q = [w 0 0 0;
     0 x 0 0;
     0 0 y 0;
     0 0 0 z];

% Assign R value for LQR input weight

R = 1;

% Find LQR gain Matrix K and new poles e

[K,S,e] = dlqr(A,B,Q,R);

%% Save the matrices
dlmwrite('Ad_matrix.csv',A);
dlmwrite('Bd_matrix.csv',B);
dlmwrite('Kd_matrix.csv',K);

%% Closed Loop Discrete Time State-Space Model

% Construct state feedback discrete time state-space model matrices

Ac = (A-B*K);
Bc = B;
Cc = C;
Dc = D;

% Closed loop poles from state feedback

poles_LQR = eig(Ac);

% Construct discrete time state-space model for state feedback simulation

sys_cl = ss(Ac,Bc,Cc,Dc,Ts,'statename',states,'inputname',inputs,'outputname',outputs);

% Verify poles match last 2 pole values

poles = eig(sys_cl.a);

% Initialize time array in increments of model timestep

t = 0:Ts:10;

% Set initial conditions for simulation

x0=[0 0 .02 0];     % Inintial angle: 0.2 radians

% Run simulation of system response based on initial angle of 0.2 radians.

% All states should converge on a value of zero to ensure robot maintains
% constant position, speed, tilt angle, and tilt rate of 0. Robot stays
% vertical and at initial position.

[y,t,x]=initial(sys_cl,x0,t);

% Plot all state output

figure;
plot(t,y(:,1),t,y(:,2),t,y(:,3),t,y(:,4));
legend('x','xDot','theta','thetaDot')
title('Response with Digital LQR Control')

%Plot control input due to LQR state feedback gain

figure;
plot(t,(K(1).*y(:,1)+K(2).*y(:,2)+K(3).*y(:,3)+K(4).*y(:,4)))
legend('Voltage Applied')
title('Control Input from Digital LQR Control')
