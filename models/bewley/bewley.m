clear; clc;


%% Build a and b matrix for Mobile Inverted Pendulum

% For Equations of Motion,
% see http://fccr.ucsd.edu/pubs/NR.pdf
% page 504

% For solution to above equations, solving
% for phi double dot and theta double dot, see 
% accompanying bewley_maple.mw file,
% which requires Maple software.

% properties of robot and earth
m__w  = 0.2;        % mass of both wheels (kg)
m__b  = 1.66;       % mass of body (kg)
I__b  = .069;       % inertia of body
I__w  = .0001764;   % inertia of both wheels
g     = 9.81;       % gravity yo. (m/s/s)
l     = 0.181;      % length from wheels to robot's COM (meters)
r     = 0.042;      % radius of wheel (meters)

% massive denominator
denom = -l^2*m__b^2*r^2+(l^2*m__b+I__b)*(m__b*r^2+m__w*r^2+I__w);

% 'a' matrix prep
a23 = -m__b^2*g*l^2*r / denom;
a43 = (m__b*r^2+m__w*r^2+I__w)*g*l*m__b / denom;

a = [0 1  0  0;
     0 0 a23 0;
     0 0  0  1;
     0 0 a43 0];

% 'b' matrix prep
b2 = (l^2*m__b+l*m__b*r+I__b) / denom;
b4 = (-l*m__b*r-m__b*r^2-m__w*r^2-I__w) / denom;

b = [0; b2; 0; b4];

% Check out the open continuous loop poles.
poles_continuous = eig(a);

%% Construct State-Space System

states = {'phiDot' 'phiDDot' 'ThDot' 'ThDDot'};
inputs = ('Va');
outputs= {'phi' 'phiDot' 'Theta' 'ThDot'};

c = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
d = [0; 0; 0; 0];
sys_ss = ss(a,b,c,d,'statename',states,'inputname',inputs,'outputname',outputs);

%% Convert Continuous to Discrete Time State-Space

% Set timestep value in seconds
Ts=0.02; % 50Hz

% Convert Continous Time Model to Discrete Time Model
sys_d = c2d(sys_ss,Ts,'zoh');

% Construct Discrete Time State-Space Matrices
A = sys_d.a;
B = sys_d.b;
C = sys_d.c;
D = sys_d.d;

% Note that log(poles_discrete)/Ts == poles_continuous
poles_discrete = eig(A);
        
%% Prepare for LQR

% State Weights for Q weight matrix
w = 1;          % phi state variable weight
x = 1;          % phiDot state variable weight
y = 1;          % theta state variable weight
z = 1;          % thetaDot state variable weight

% Construct Q matrix (symmetric diagonal)
Q = [w 0 0 0;
     0 x 0 0;
     0 0 y 0;
     0 0 0 z];

% Assign R value for LQR input weight
R = 1;

% Find LQR gain Matrix K and new poles e
[K,S,e] = dlqr(A,B,Q,R);

% Not sure what is going on here:
something_from_dlqr = log(e)/Ts;

%% Manual Pole Placement

my_poles =  [0.9786 0.9553 0.8964 0.8499];
my_K = place(A,B,my_poles);

my_poles2 =  [0.9786 0.9553 0.9264 0.9199];
my_K2 = place(A,B,my_poles2);

%% Calculate continuous K for non-linear simulation below

cont_K = lqr(a, b, Q, R);


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

% Run simulation of system response based on initial angle of 0.2 radians.

% All states should converge on a value of zero to ensure robot maintains
% constant position, speed, tilt angle, and tilt rate of 0. Robot stays
% vertical and at initial position.

% Initialize time array in increments of model timestep
t = 0:Ts:10;

% Set initial conditions for simulation
x0=[0 2 .02 0];     % Inintial angle: 0.2 radians

% [y,t,x]=initial(sys_cl,x0,t);

% Plot all state output

% figure;
% plot(t,y(:,1),t,y(:,2));
% legend('phi','phiDot')
% title('Response with Digital LQR Control')
% 
% 
% % Plot all state output
% 
% figure;
% plot(t,y(:,3),t,y(:,4));
% legend('theta','thetaDot')
% title('Response with Digital LQR Control')
% 
% %Plot control input due to LQR state feedback gain
% 
% figure;
% plot(t,(K(1).*y(:,1)+K(2).*y(:,2)+K(3).*y(:,3)+K(4).*y(:,4)))
% legend('Voltage Applied')
% title('Control Input from Digital LQR Control')

%% Now lets simulate.

%Initial conditions
y0 = [0; 2; 0.10; 0];
tspan = 0:.001:5;

% closed loop:
% [t,y] = ode45(@(t,y)odes(y,I__b, I__w, m__b,m__w,l,g,r,-cont_K*y),tspan,y0);

% open loop:
% [t,y] = ode45(@(t,y)odes(y,I__b, I__w, m__b,m__w,l,g,r,0),tspan,y0);
%
% figure;
% subplot(2,3,1);
% plot(t, y(:,1));
% title('non-linear phi (wheels)');
% 
% subplot(2,3,2);
% plot(t, y(:,2));
% title('non-linear phi dot (wheels)');
% 
% subplot(2,3,3);
% plot(t, y(:,3));
% title('non-linear theta (body)');
% 
% subplot(2,3,4);
% plot(t, y(:,4));
% title('non-linear theta dot (body)');
% 
% subplot(2,3,5);
% plot(t,(K(1).*y(:,1)+K(2).*y(:,2)+K(3).*y(:,3)+K(4).*y(:,4)))
% title('gain u');
%
% for k=1:100:length(t)
%     drawpend(y(k,:),r,l);
% end

%% These guys are used to power the Kalman filter in Simulink
kalman_C = [1 0 0 0; 0 0 1 0; 0 0 0 1];
kalman_D = [0; 0; 0];
kalman_R = [0.0001, 0, 0;
            0, 0.001, 0;
            0, 0, 0.001];
