clc;
clear;
close all;

% parameters
params.m  = 0.5;
params.g  = 9.81;
params.I  = 0.0023;

% reference trajectory (y–z plane)
params.y_des      = @(t) 2*sin(0.3*t);
params.z_des      = @(t) 0.5 + 0.1*t;
params.y_dot_des  = @(t) 0.6*cos(0.3*t);
params.z_dot_des  = @(t) 0.1;
params.y_ddot_des = @(t) -0.18*sin(0.3*t);
params.z_ddot_des = @(t) 0;

% initial condition
x0 = zeros(9,1);

% simulation
T = 30;
tspan = linspace(0,T,1000);

[t,x] = ode45(@(t,x) quadrotor2D_dynamics(t,x,params),tspan,x0);

quadrotor2D_animate(t,x);
quadrotor2D_plots(t,x,params);

function dx = quadrotor2D_dynamics(t,x,p)

% states
y     = x(1);
z     = x(2);
theta = x(3);
y_dot = x(4);
z_dot = x(5);
theta_dot = x(6);
eiy   = x(7);
eiz   = x(8);
eit   = x(9);

% reference
y_des = p.y_des(t);
z_des = p.z_des(t);
y_dot_des = p.y_dot_des(t);
z_dot_des = p.z_dot_des(t);

% errors
ey = y_des - y;
ez = z_des - z;
ey_dot = y_dot_des - y_dot;
ez_dot = z_dot_des - z_dot;

% gains (kept consistent with 3D magnitudes)
Kp_y = 10; Ki_y = 2; Kd_y = 14;
Kp_z = 20; Ki_z = 5; Kd_z = 8;
Kp_t = 50; Ki_t = 10; Kd_t = 15;

% outer loop
y_ddot_cmd = Kp_y*ey + Kd_y*ey_dot + Ki_y*eiy;
z_ddot_cmd = Kp_z*ez + Kd_z*ez_dot + Ki_z*eiz;

% desired pitch
theta_des = -(1/p.g)*y_ddot_cmd;
theta_des = min(max(theta_des,deg2rad(-45)),deg2rad(45));

% control
u1 = p.m*(p.g + z_ddot_cmd);
u2 = Kp_t*(theta_des-theta) + Kd_t*(0-theta_dot) + Ki_t*eit;

% saturation
u1 = max(0,min(u1,2*p.m*p.g));
u2 = max(-0.5,min(u2,0.5));

% dynamics
y_ddot = -(u1/p.m)*sin(theta);
z_ddot =  (u1/p.m)*cos(theta) - p.g;
theta_ddot = u2/p.I;

dx = [
    y_dot
    z_dot
    theta_dot
    y_ddot
    z_ddot
    theta_ddot
    ey
    ez
    (theta_des-theta)
];
end

function quadrotor2D_animate(t,x)

figure('Color','w');
axis equal;
axis([-3 3 0 5]);
grid on;
xlabel('Y (m)');
ylabel('Z (m)');
title('Quadrotor 2D (Y–Z Plane)');
hold on;

plot(x(:,1),x(:,2),'b--','LineWidth',1.2);

h_body = plot(0,0,'ro','MarkerFaceColor','r');
h_arm  = plot([0 0],[0 0],'k','LineWidth',2);
h_time = text(-2,4.5,'');

L = 0.3;

for i=1:5:length(t)
    y = x(i,1);
    z = x(i,2);
    th = x(i,3);

    arm = [ -L*cos(th), L*cos(th)
             L*sin(th),-L*sin(th)];

    set(h_arm,'XData',y+arm(1,:),'YData',z+arm(2,:));
    set(h_body,'XData',y,'YData',z);
    set(h_time,'String',sprintf('t = %.2f s',t(i)));
    drawnow;
end
end

function quadrotor2D_plots(t,x,p)

y_ref = arrayfun(p.y_des,t);
z_ref = arrayfun(p.z_des,t);

figure('Color','w');
subplot(2,1,1);
plot(t,x(:,1),'b',t,y_ref,'r--','LineWidth',1.5);
ylabel('y (m)');
title('Position Y');
grid on;

subplot(2,1,2);
plot(t,x(:,2),'b',t,z_ref,'r--','LineWidth',1.5);
ylabel('z (m)');
xlabel('Time (s)');
title('Position Z');
grid on;

figure('Color','w');
subplot(2,1,1);
plot(t,x(:,3)*180/pi,'LineWidth',1.5);
ylabel('\theta (deg)');
title('Pitch');
grid on;

subplot(2,1,2);
plot(t,x(:,6),'LineWidth',1.5);
ylabel('\dot{\theta} (rad/s)');
xlabel('Time (s)');
grid on;
end