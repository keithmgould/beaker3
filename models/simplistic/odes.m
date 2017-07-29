function dy = odes(y,I__b, m__b,m__w,b,l,g,u)
% System of ODEs for pendulum on wheels.
% See associated simplistic_maple.mw for maple file
% y(1) = x
% y(2) = x_dot
% y(3) = theta
% y(4) = theta_dot

Sy = sin(y(3));
Cy = cos(y(3));

denom = (-m__b^2*l^2*Cy^2+l^2*m__b^2+l^2*m__b*m__w+I__b*m__b+I__b*m__w);
x_dd = -(1/denom)*(-(y(4))^2*Sy*l^3*m__b^2-m__b^2*l^2*g*Sy*Cy-I__b*(y(4))^2*Sy*l*m__b+(y(2))*b*l^2*m__b-l^2*m__b*u+I__b*(y(2))*b-I__b*u);
theta_dd = (1/denom)*((-(y(4))^2*Sy*Cy*l*m__b+(y(2))*Cy*b-Sy*g*m__b-Sy*g*m__w-Cy*u)*m__b*l);

dy(1,1) = y(2);
dy(2,1) = x_dd;
dy(3,1) = y(4);
dy(4,1) = theta_dd;

