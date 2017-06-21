clear; clc;

% MassWheels = 0.2;     % kg
% MassRobot = 1.8;      % kg
% Friction = 0.1;       % ?
% Length = 0.209;       % m
% Inertia = 0.06;       % ?
% Gravity = 9.81;       % m/s/s

mp = 1.8;   % kg
mw = 0.2;   % kg
l = 0.209;  % m
r = 0.042;  % m
g = 9.804;  % m/s/s


A=[0 1 0 0;
   0 0 (-1*mp*g)/(r*(mp+4*mw)) 0;
   0 0 0 1;
   0 0 (g*(mp+2*mw))/(l*(mp+4*mw)) 0];

B=[0;
2/(r^2*(mp+4*mw));
0;
-1/(l*r*(mp+4*mw))];

Q=[5 0 0 0;
   0 5 0 0;
  0 0 1 0;
  0 0 0 1];

R=1500;

[K,S,e]=lqr(A,B,Q,R);

%------------------------
C = [1 0 0 0;
    0 0 1 0];

D = [0; 0];
ss_cont = ss(A,B,C,D);

ts = 20 / 1000; % 50Hz

ss_discrete = c2d(ss_cont, ts);

Ad = ss_discrete.a;
Bd = ss_discrete.b;

[Kd, Sd, ed] = dlqr(Ad, Bd, Q, R);


dlmwrite('Ad_matrix.csv',Ad);
dlmwrite('Bd_matrix.csv',Bd);
dlmwrite('Kd_matrix.csv',Kd);

