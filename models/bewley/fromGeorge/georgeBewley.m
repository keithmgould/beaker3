clear
close all
clc

% Plant model (cont. time !!!)
% linearized bewley model comes here
mw  = 0.2;        % mass of both wheels (kg)
mb  = 1.66;       % mass of body (kg)
Ib  = .069;       % inertia of body
Iw  = .0001764;   % inertia of both wheels
g   = 9.81;       % gravity yo. (m/s/s)
l   = 0.181;      % length from wheels to robot's COM (meters)
r   = 0.042;      % radius of wheel (meters)

denom = -l^2*mb^2*r^2+(l^2*mb+Ib)*(mb*r^2+mw*r^2+Iw);

% matrix prep
a23 = -mb^2*g*l^2*r / denom;
a43 = (mb*r^2+mw*r^2+Iw)*g*l*mb / denom;

A = [0 1  0  0;
     0 0 a23 0;
     0 0  0  1;
     0 0 a43 0];

b2 = (l^2*mb+l*mb*r+Ib) / denom;
b4 = (-l*mb*r-mb*r^2-mw*r^2-Iw) / denom;

B = [0; b2; 0; b4];
C = [1 0 0 0 
     0 0 1 0 
     0 0 0 1];
D = [0; 0; 0];

% Control objectives (poles for transients):
% Here we set the desired closed loop poles. We would like to use a complex
% conjugate with a little overshoot as dominant poles and the remaining
% poles are set to be three times faster than the dominant one.
xi = 0.9;    % damping (related to overshoot)     
w0 = 5;      % nat. eig. freq. (related to speed) 
s1 = -w0*xi + 1j*w0*sqrt(1 - xi^2); % dominant poles
s2 = -w0*xi - 1j*w0*sqrt(1 - xi^2);
scinf = -3*w0*xi; % 3x times faster poles than dominant (controller)

% Observer desired poles:
% The estimation error should be way more faster vanish than the control
% transients, therefore we assume identical observer poles that are 5x
% times faster than the dominant poles.
soinf = -5*w0*xi;

% 1) Pole placement:
s_new = [s1 s2 scinf scinf];
K = acker(A,B,s_new);

% % 2) Set-point correction: this is coming from mathematical derivation
% NxNu = [A B;C 0]\[0;0;0;0;1];
% Nx = NxNu(1:4);
% Nu = NxNu(5);

% 3) Load estimator for the extended system:
Ah = [A B; 0 0 0 0 0];
Bh = [B; 0];
Ch = [1 0 0 0 0
      0 0 1 0 0
      0 0 0 1 0];

s_obs = [soinf soinf-1 soinf-2 soinf-3 soinf-4];  
Ght = place(Ah',Ch',s_obs);

Gh = Ght';
Fh = Ah - Gh*Ch;
Hh = Bh;

