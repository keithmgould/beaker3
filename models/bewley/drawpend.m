function drawpend(y,r,l)

% y(1) = phi
% y(2) = phi_dot
% y(3) = theta
% y(4) = theta_dot

% calculate x position
wx = y(1) * r;
wy = 0;

% l is COM so full l is 2l
fl = 2*l;

th = y(3);

% pendulum position
px = wx - fl*sin(th);
py = fl*cos(th);

% draw the ground
plot([-10 10],[0 0],'k','LineWidth',2)
hold on

% draw the wheels
rectangle('Position',[wx - r/2,wy,r,r],'Curvature',1,'FaceColor',[1 0.1 0.1])

% draw the pendulum body
plot([wx px],[r/2 py],'k','LineWidth',2)

xlim([-1 1]);
ylim([-0.1 0.5]);
set(gcf,'Position',[100 550 1000 400])
drawnow
hold off