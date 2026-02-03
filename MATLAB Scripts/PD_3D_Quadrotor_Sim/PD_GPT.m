%% Quadrotor 6-DoF Newton–Euler Model with PD Control
clc; clear; close all;

%% Parameters
params.m   = 0.5;
params.g   = 9.81;
params.Ixx = 0.0023;
params.Iyy = 0.0023;
params.Izz = 0.004;

%% Reference trajectories
params.x_des = @(t) 2*cos(0.3*t);
params.y_des = @(t) 2*sin(0.3*t);
params.z_des = @(t) 0.5 + 0.1*t;

params.x_dot_des = @(t) 0;
params.y_dot_des = @(t) 0;
params.z_dot_des = @(t) 0;

params.x_ddot_des = @(t) 0;
params.y_ddot_des = @(t) 0;
params.z_ddot_des = @(t) 0;

params.psi_des = @(t) 0;
params.psi_ref = @(t) 0;

%% Initial state
% x = [x y z phi theta psi xdot ydot zdot p q r]
x0 = zeros(12,1);

%% Simulation
T = 30;
tspan = linspace(0,T,10000);
[t,x] = ode45(@(t,x) quad3D_dynamics(t,x,params), tspan, x0);

%% ===================== POST-PROCESSING =====================

% States
x_pos = x(:,1);  y_pos = x(:,2);  z_pos = x(:,3);
phi   = x(:,4);  theta = x(:,5);  psi = x(:,6);
xdot  = x(:,7);  ydot  = x(:,8);  zdot = x(:,9);
p_b   = x(:,10); q_b   = x(:,11); r_b = x(:,12);

% References
x_ref   = arrayfun(params.x_des, t);
y_ref   = arrayfun(params.y_des, t);
z_ref   = arrayfun(params.z_des, t);
psi_ref = arrayfun(params.psi_des, t);

% Errors
ex = x_ref - x_pos;
ey = y_ref - y_pos;
ez = z_ref - z_pos;

% Gains
Kp_x = 6;   Kd_x = 10;
Kp_y = 6;   Kd_y = 10;
Kp_z = 20;  Kd_z = 8;

Kp_phi = 50;  Kd_phi = 15;
Kp_th  = 50;  Kd_th  = 15;
Kp_psi = 50;  Kd_psi = 15;

% Commanded accelerations
x_ddot_c = Kp_x*ex - Kd_x*xdot;
y_ddot_c = Kp_y*ey - Kd_y*ydot;
z_ddot_c = Kp_z*ez - Kd_z*zdot;

% Desired angles
phi_des   = -(1/params.g)*y_ddot_c;
theta_des =  (1/params.g)*x_ddot_c;

% Control inputs
u1 = params.m*(params.g + z_ddot_c);
u2 = Kp_phi*(phi_des - phi)    - Kd_phi*p_b;
u3 = Kp_th *(theta_des-theta)  - Kd_th *q_b;
u4 = Kp_psi*(psi_ref - psi)    - Kd_psi*r_b;

%% ===================== PLOTS =====================

% Positions
figure('Color','w');
subplot(3,1,1)
plot(t,x_pos,'b',t,x_ref,'r--','LineWidth',1.5)
ylabel('x (m)'); grid on; legend('x','x_{ref}')

subplot(3,1,2)
plot(t,y_pos,'b',t,y_ref,'r--','LineWidth',1.5)
ylabel('y (m)'); grid on; legend('y','y_{ref}')

subplot(3,1,3)
plot(t,z_pos,'b',t,z_ref,'r--','LineWidth',1.5)
ylabel('z (m)'); xlabel('Time (s)')
grid on; legend('z','z_{ref}')

% Position errors
figure('Color','w');
subplot(3,1,1)
plot(t,ex,'LineWidth',1.5)
ylabel('e_x (m)'); grid on

subplot(3,1,2)
plot(t,ey,'LineWidth',1.5)
ylabel('e_y (m)'); grid on

subplot(3,1,3)
plot(t,ez,'LineWidth',1.5)
ylabel('e_z (m)'); xlabel('Time (s)')
grid on

% Attitude
figure('Color','w');
subplot(3,1,1)
plot(t,phi*180/pi,'LineWidth',1.5)
ylabel('\phi (deg)'); grid on

subplot(3,1,2)
plot(t,theta*180/pi,'LineWidth',1.5)
ylabel('\theta (deg)'); grid on

subplot(3,1,3)
plot(t,psi*180/pi,'LineWidth',1.5)
ylabel('\psi (deg)'); xlabel('Time (s)')
grid on

% Body angular rates
figure('Color','w');
subplot(3,1,1)
plot(t,p_b,'LineWidth',1.5)
ylabel('p (rad/s)'); grid on

subplot(3,1,2)
plot(t,q_b,'LineWidth',1.5)
ylabel('q (rad/s)'); grid on

subplot(3,1,3)
plot(t,r_b,'LineWidth',1.5)
ylabel('r (rad/s)'); xlabel('Time (s)')
grid on

% Control inputs
figure('Color','w');
subplot(4,1,1)
plot(t,u1,'LineWidth',1.5)
ylabel('u_1 (N)'); grid on

subplot(4,1,2)
plot(t,u2,'LineWidth',1.5)
ylabel('u_2 (Nm)'); grid on

subplot(4,1,3)
plot(t,u3,'LineWidth',1.5)
ylabel('u_3 (Nm)'); grid on

subplot(4,1,4)
plot(t,u4,'LineWidth',1.5)
ylabel('u_4 (Nm)'); xlabel('Time (s)')
grid on

%% ===================== DYNAMICS FUNCTION =====================
function dx = quad3D_dynamics(t,x,p)

    % States
    x_pos = x(1); y_pos = x(2); z_pos = x(3);
    phi   = x(4); theta = x(5); psi = x(6);
    xdot  = x(7); ydot  = x(8); zdot = x(9);
    p_b   = x(10); q_b  = x(11); r_b = x(12);

    % References
    x_des = p.x_des(t);
    y_des = p.y_des(t);
    z_des = p.z_des(t);
    psi_des = p.psi_des(t);

    % Errors
    ex = x_des - x_pos;
    ey = y_des - y_pos;
    ez = z_des - z_pos;

    % Gains
    Kp_x = 6; Kd_x = 10;
    Kp_y = 6; Kd_y = 10;
    Kp_z = 20; Kd_z = 8;

    Kp_phi = 50; Kd_phi = 15;
    Kp_th  = 50; Kd_th  = 15;
    Kp_psi = 50; Kd_psi = 15;

    % Commanded accelerations
    x_ddot_c = Kp_x*ex - Kd_x*xdot;
    y_ddot_c = Kp_y*ey - Kd_y*ydot;
    z_ddot_c = Kp_z*ez - Kd_z*zdot;

    % Desired attitude
    phi_des   = -(1/p.g)*y_ddot_c;
    theta_des =  (1/p.g)*x_ddot_c;

    % Thrust
    u1 = p.m*(p.g + z_ddot_c);

    % Torques
    u2 = Kp_phi*(phi_des - phi)   - Kd_phi*p_b;
    u3 = Kp_th *(theta_des-theta) - Kd_th *q_b;
    u4 = Kp_psi*(psi_des - psi)   - Kd_psi*r_b;

    % Translational dynamics
    x_ddot = (u1/p.m)*(cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi));
    y_ddot = (u1/p.m)*(cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi));
    z_ddot = (u1/p.m)*(cos(phi)*cos(theta)) - p.g;

    % Euler kinematics
    phi_dot   = p_b + q_b*sin(phi)*tan(theta) + r_b*cos(phi)*tan(theta);
    theta_dot = q_b*cos(phi) - r_b*sin(phi);
    psi_dot   = q_b*sin(phi)/cos(theta) + r_b*cos(phi)/cos(theta);

    % Body rate dynamics
    p_dot = ((p.Iyy - p.Izz)/p.Ixx)*q_b*r_b + u2/p.Ixx;
    q_dot = ((p.Izz - p.Ixx)/p.Iyy)*p_b*r_b + u3/p.Iyy;
    r_dot = ((p.Ixx - p.Iyy)/p.Izz)*p_b*q_b + u4/p.Izz;

    % State derivative
    dx = [
        xdot; ydot; zdot;
        phi_dot; theta_dot; psi_dot;
        x_ddot; y_ddot; z_ddot;
        p_dot; q_dot; r_dot
    ];
end