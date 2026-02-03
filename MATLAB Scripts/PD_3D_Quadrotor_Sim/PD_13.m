% Dynamics - Kiran ma'am paper, considering angular velocity

clc; clear; close all;

% Parameters
p.m  = 0.5;
p.g  = 9.81;

p.jx = 0.0023;
p.jy = 0.0023;
p.jz = 0.004;

% Reference trajectory (hover → step → hover)
p.x_des = @(t) 2*(t>3);
p.y_des = @(t) 0;
p.z_des = @(t) 2;

p.x_dot_des = @(t) 0;
p.y_dot_des = @(t) 0;
p.z_dot_des = @(t) 0;

p.psi_des = @(t) 0;

% Initial condition
x0 = zeros(12,1);

% Simulation
T = 20;
tspan = linspace(0,T,400);

function dx = quad_dynamics(t,x,p)

% States
xpos = x(1); ypos = x(2); zpos = x(3);
phi  = x(4); theta = x(5); psi = x(6);
xdot = x(7); ydot  = x(8); zdot = x(9);
phidot = x(10); thetadot = x(11); psidot = x(12);

% Gains
Kp_x = 6;   Kd_x = 8;
Kp_y = 6;   Kd_y = 8;
Kp_z = 20;  Kd_z = 8;

Kp_phi = 50;   Kd_phi = 12;
Kp_theta = 50; Kd_theta = 12;
Kp_psi = 20;   Kd_psi = 8;

% References
xdes = p.x_des(t);   ydes = p.y_des(t);   zdes = p.z_des(t);
xdot_des = p.x_dot_des(t);
ydot_des = p.y_dot_des(t);
zdot_des = p.z_dot_des(t);
psi_des  = p.psi_des(t);

% Outer-loop PD (position)
ax_cmd = Kp_x*(xdes-xpos) + Kd_x*(xdot_des-xdot);
ay_cmd = Kp_y*(ydes-ypos) + Kd_y*(ydot_des-ydot);
az_cmd = Kp_z*(zdes-zpos) + Kd_z*(zdot_des-zdot);

% Desired roll & pitch (small-angle)
phi_des   = -ay_cmd/p.g;
theta_des =  ax_cmd/p.g;

phi_des   = min(max(phi_des,-deg2rad(30)),deg2rad(30));
theta_des = min(max(theta_des,-deg2rad(30)),deg2rad(30));

% Thrust (gravity compensated)
u1 = p.m*(p.g + az_cmd);

% Inner-loop PD (attitude)
u2 = p.jx*( Kp_phi*(phi_des-phi)     - Kd_phi*phidot );
u3 = p.jy*( Kp_theta*(theta_des-theta) - Kd_theta*thetadot );
u4 = p.jz*( Kp_psi*(psi_des-psi)     - Kd_psi*psidot );

% Saturation
u1 = max(0,min(u1,2*p.m*p.g));
u2 = max(-0.5,min(u2,0.5));
u3 = max(-0.5,min(u3,0.5));
u4 = max(-0.3,min(u4,0.3));

% Dynamics (Eq. 5)
xddot = (u1/p.m)*(cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi));
yddot = (u1/p.m)*(cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi));
zddot = (u1/p.m)*cos(phi)*cos(theta) - p.g;

phiddot   = ((p.jy-p.jz)/p.jx)*thetadot*psidot + u2/p.jx;
thetaddot = ((p.jz-p.jx)/p.jy)*phidot*psidot   + u3/p.jy;
psiddot   = ((p.jx-p.jy)/p.jz)*phidot*thetadot + u4/p.jz;

% State derivative
dx = [ xdot;
       ydot;
       zdot;
       phidot;
       thetadot;
       psidot;
       xddot;
       yddot;
       zddot;
       phiddot;
       thetaddot;
       psiddot ];
end

[t,x] = ode45(@(t,x) quad_dynamics(t,x,p), tspan, x0);

% Plots
figure;
subplot(3,1,1); plot(t,x(:,1),t,x(:,2),t,x(:,3)); grid on;
legend('x','y','z'); ylabel('Position (m)');

subplot(3,1,2); plot(t,x(:,7),t,x(:,8),t,x(:,9)); grid on;
legend('xdot','ydot','zdot'); ylabel('Velocity (m/s)');

subplot(3,1,3); plot(t,x(:,4:6)*180/pi); grid on;
legend('\phi','\theta','\psi'); ylabel('Angles (deg)');
xlabel('Time (s)');