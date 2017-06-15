clc; clear;

MassWheels = 0.2;     % kg
MassRobot = 1.8;      % kg
Friction = 0.1;       % ?
Length = 0.209;       % m
Inertia = 0.06;       % ?
Gravity = 9.81;       % m/s/s

% helper variables
Denom = Inertia * (MassWheels + MassRobot) + MassWheels * MassRobot * Length * Length;
A22 = -(Inertia + MassRobot * Length * Length) * Friction / Denom;
A23 = (MassRobot * MassRobot * Gravity * Length * Length) / Denom;
A42 = -(MassRobot * Length * Friction) / Denom;
A43 = MassRobot * Gravity * Length * (MassWheels + MassRobot) / Denom;
B21 = (Inertia + MassRobot * Length * Length) / Denom;
B41 = (MassRobot * Length) / Denom;

% System Matrix
Ac = [0, 1, 0, 0;
     0, A22, A23, 0;
     0, 0, 0, 1;
     0, A42, A43, 0];

% Input Matrix
Bc = [0;
     B21;
     0;
     B41];

% Define the output matrix
% We only observe x (meters) and theta (radians).
Cc = [1, 0, 0, 0;
     0, 0, 1, 0];

Dc = [0; 0];

sys_c = ss(Ac,Bc,Cc,Dc);

td = 50 / 1000; % seconds
sys_d = c2d(sys_c, td);

% Extract Discrete Time State-Space Matrices
Ad = sys_d.a;
Bd = sys_d.b;
Cd = sys_d.c;
Dd = sys_d.d;

% weights for states
Q = [1, 0, 0,  0;
     0, 1, 0,  0;
     0, 0, 10, 0;
     0, 0, 0,  100];

% weight for U
R = 1;

Kd = dlqr(Ad, Bd, Q, R);

dlmwrite('Ad_matrix.csv',Ad);
dlmwrite('Bd_matrix.csv',Bd);
dlmwrite('Kd_matrix.csv',Kd);