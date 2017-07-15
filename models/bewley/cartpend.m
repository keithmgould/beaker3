function dy = cartpend(y,I__b, I__w, m__b,m__w,l,g,r,u)
% System of ODEs for pendulum on wheels.
% See associated bewley_maple.mw for maple file

% y(1) = phi
% y(2) = phi_dot
% y(3) = theta
% y(4) = theta_dot

Sy = sin(y(3));
Cy = cos(y(3));

denom = -Cy^2*l^2*m__b^2*r^2+[m__b*r^2+m__w*r^2+I__w]*l^2*m__b+I__b*[m__b*r^2+m__w*r^2+I__w];
phi_num = Sy*(y(4))^2*l^3*m__b^2*r-Cy*Sy*g*l^2*m__b^2*r+I__b*Sy*(y(4))^2*l*m__b*r+Cy*l*m__b*r*u+l^2*m__b*u+I__b*u;
theta_num = -Cy*Sy*(y(4))^2*l^2*m__b^2*r^2+[m__b*r^2+m__w*r^2+I__w]*Sy*g*l*m__b-Cy*l*m__b*r*u-[m__b*r^2+m__w*r^2+I__w]*u;

dy(1,1) = y(2);
dy(2,1) = phi_num / denom;
dy(3,1) = y(4);
dy(4,1) = theta_num / denom;