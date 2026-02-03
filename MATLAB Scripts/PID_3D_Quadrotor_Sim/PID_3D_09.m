% Dynamics - ma'am paper
% Trajectory - Hold Tilt, Change Altitude
% Control - PID

clc;
clear;
close all;

% parameters
params.m = 0.5;
params.g = 9.81;
params.Ixx = 0.0023;
params.Iyy = 0.0023;
params.Izz = 0.004;

% reference trajectory 
params.x_des = @(t) 3*sin(0.2*t);
params.y_des = @(t) 0;
params.z_des = @(t) 2 + 1*(t>8);

params.x_dot_des = @(t) 0.6*cos(0.2*t);
params.y_dot_des = @(t) 0;
params.z_dot_des = @(t) 0;

params.x_ddot_des = @(t) -0.12*sin(0.2*t);
params.y_ddot_des = @(t) 0;
params.z_ddot_des = @(t) 0;

params.psi_des = @(t) 0;
params.psi_ref = @(t) 0;

% initial condition
% x = [ x y z phi theta psi x_dot y_dot z_dot phi_dot theta_dot psi_dot]
x0 = zeros(18,1);

% simulation parameters
T = 30;
tspan = linspace(0, T, 1000);

function dx = quadrotor3D_dynamics(t, x, p)
    
    % states
    x_pos = x(1);
    y_pos = x(2);
    z_pos = x(3);

    phi = x(4);
    theta = x(5);
    psi = x(6);

    x_pos_dot = x(7);
    y_pos_dot = x(8);
    z_pos_dot = x(9);

    phi_dot = x(10);
    theta_dot = x(11);
    psi_dot = x(12);

    e_int_x = x(13);
    e_int_y = x(14);
    e_int_z = x(15);

    e_int_phi   = x(16);
    e_int_theta = x(17);
    e_int_psi   = x(18);


    % reference trajectory
    x_des = p.x_des(t);
    y_des = p.y_des(t);
    z_des = p.z_des(t);

    x_dot_des = p.x_dot_des(t);
    y_dot_des = p.y_dot_des(t);
    z_dot_des = p.z_dot_des(t);

    % errors 
    e_x = x_des - x_pos;
    e_x_dot = x_dot_des - x_pos_dot;
    e_y = y_des - y_pos;
    e_y_dot = y_dot_des - y_pos_dot;
    e_z = z_des - z_pos;
    e_z_dot = z_dot_des - z_pos_dot;

    e_int_x_dot = e_x;
    e_int_y_dot = e_y;
    e_int_z_dot = e_z;
    


    % gains
    Kp_x = 20;
    Ki_x = 2;
    Kd_x = 14;
    Kp_y = 10;
    Ki_y = 2;
    Kd_y = 14;
    Kp_z = 20;
    Ki_z = 5;
    Kd_z = 8;

    Kp_phi = 50;
    Ki_phi = 10;
    Kd_phi = 15;
    Kp_theta = 50;
    Ki_theta = 10;
    Kd_theta = 15;
    Kp_psi = 50;
    Ki_psi = 8;
    Kd_psi = 15;    

    % desired (commanded) acceleration/ outer loop PD (pos)
    x_ddot_cmd = Kp_x * e_x + Kd_x * e_x_dot + Ki_x * e_int_x;
    y_ddot_cmd = Kp_y * e_y + Kd_y * e_y_dot + Ki_y * e_int_y;
    z_ddot_cmd = Kp_z * e_z + Kd_z * e_z_dot + Ki_z * e_int_z;

    % desired roll & pitch from outer loop (small angle)
    phi_des = -(1/p.g) * (y_ddot_cmd);
    theta_des = (1/p.g) * (x_ddot_cmd);

        % bounds
    phi_des = min(max(phi_des, deg2rad(-90)), deg2rad(90));
    theta_des = min(max(theta_des, deg2rad(-90)), deg2rad(90));

    % yaw 
    psi_des_raw = p.psi_des(t);
    psi_des = min(max(psi_des_raw, deg2rad(-180)), deg2rad(180));
    
    e_int_phi_dot   = (phi_des - phi);
    e_int_theta_dot = (theta_des - theta);
    e_int_psi_dot   = (psi_des - psi);

    % thrust (T = u1)
    u1 = p.m * (p.g + z_ddot_cmd);

    % inner loop PD (attitude) / torques (tau_phi = u2, tau_theta = u3,
    % tau_psi = u4)
    u2 = Kp_phi * (phi_des - phi) + Kd_phi * (0 - phi_dot) + Ki_phi * e_int_phi;
    u3 = Kp_theta * (theta_des - theta) + Kd_theta * (0 - theta_dot) + Ki_theta * e_int_theta;
    u4 = Kp_psi * (psi_des - psi) + Kd_psi * (0 - psi_dot) + Ki_psi * e_int_psi;

    % control input bounds/ saturation
    u1 = max(0, min(u1, 2 * p.m * p.g));
    u2 = max(-0.5, min(u2, 0.5));
    u3 = max(-0.5, min(u3, 0.5));
    u4 = max(-0.3, min(u4, 0.3));

    % u1 = min(max(u1, 0), 2 * p.m * p.g);
    % u2 = min(max(u2, -0.5), 0.5);
    % u3 = min(max(u3, -0.5), 0.5);
    % u4 = min(max(u4, -0.3), 0.3);


    % dynamics (accelerations)
    x_ddot = (u1/p.m) * (cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi));
    y_ddot = (u1/p.m) * (cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi));
    z_ddot = (u1/p.m) * (cos(phi) * cos(theta)) - p.g;

    phi_ddot = ((p.Iyy - p.Izz)/p.Ixx) * theta_dot * psi_dot + u2 / p.Ixx;
    theta_ddot = ((p.Izz - p.Ixx)/p.Iyy) * phi_dot * psi_dot + u3 / p.Iyy;
    psi_ddot = ((p.Ixx - p.Iyy)/p.Izz) * phi_dot * theta_dot + u4 / p.Izz;
    
        % bounds
    % phi = min(max(phi, deg2rad(-45)), deg2rad(45));
    % theta = min(max(theta, deg2rad(-45)), deg2rad(45));
    % psi = min(max(psi, deg2rad(-180)), deg2rad(180));

    dx = [x_pos_dot; y_pos_dot; z_pos_dot; phi_dot; theta_dot; psi_dot;
        x_ddot; y_ddot; z_ddot; phi_ddot; theta_ddot; psi_ddot; 
        e_int_x_dot; e_int_y_dot; e_int_z_dot;
        e_int_phi_dot; e_int_theta_dot; e_int_psi_dot];
end

% simulate
[t, x] = ode45(@(t, x) quadrotor3D_dynamics(t, x, params), tspan, x0);

function quadrotor3D_animate(t, x)
    figure('Color', 'W');
    axis equal;
    axis([-4, 4 0 3 -4 4]);
    view(45, 25);
    grid on;
    xlabel('X (m)');
    ylabel('Y(m)');
    zlabel('Z(m)');
    title('Quadrotor 3D');
    hold on;

    plot3(x(:,1), x(:,2), x(:, 3), 'b--', 'LineWidth', 1);

    h_arm1 = plot3([0 0], [0 0], [0 0], 'k-', 'LineWidth', 2);
    h_arm2 = plot3([0 0], [0 0], [0 0], 'k-', 'LineWidth', 2);
    h_center = plot3(0, 0, 0, 'ro', 'MarkerSize', 6, "MarkerFaceColor", 'r');
    h_time = text(1, 1.5, 3.5, '', 'FontSize', 10);

    L = 0.3;

    for i = 1:5:length(t)
        pos = x(i,1:3);
        phi = x(i, 4);
        theta = x(i, 5);
        psi = x(i, 6);

        R = eul2rotm([psi, theta, phi], 'ZYX');

        arm_x = R*[L; 0; 0];
        arm_y = R*[0; L; 0];

        p1a = pos' + arm_x; 
        p1b = pos' - arm_x;
        p2a = pos' + arm_y;
        p2b = pos' - arm_y;

        set(h_arm1, 'XData', [p1a(1), p1b(1)], 'YData', [p1a(2), p1b(2)], 'ZData', [p1a(3), p1b(3)]);
        set(h_arm2, 'XData', [p2a(1), p2b(1)], 'YData', [p2a(2), p2b(2)], 'ZData', [p2a(3), p2b(3)]);
        set(h_center, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3));
        set(h_time, 'String', sprintf('t = %.2f s', t(i)));

        drawnow;
    end
end

% animate 
quadrotor3D_animate(t, x);

% plots
% references for plots
x_ref   = arrayfun(params.x_des, t);
y_ref   = arrayfun(params.y_des, t);
z_ref   = arrayfun(params.z_des, t);
psi_ref = arrayfun(params.psi_des, t);
phi_ref   = zeros(length(t),1);
theta_ref = zeros(length(t),1);
% psi_ref   = zeros(length(t),1);


% position tracking errors
ex = zeros(length(t),1);
ey = zeros(length(t),1);
ez = zeros(length(t),1);

ex_dot = zeros(length(t),1);
ey_dot = zeros(length(t),1);
ez_dot = zeros(length(t),1);

for i = 1:length(t)
    ex(i) = params.x_des(t(i)) - x(i,1);
    ey(i) = params.y_des(t(i)) - x(i,2);
    ez(i) = params.z_des(t(i)) - x(i,3);

    ex_dot(i) = params.x_dot_des(t(i)) - x(i,7);
    ey_dot(i) = params.y_dot_des(t(i)) - x(i,8);
    ez_dot(i) = params.z_dot_des(t(i)) - x(i,9);

    ephi   = phi_ref   - x(:,4);   % roll error
    etheta = theta_ref - x(:,5);   % pitch error
    epsi   = psi_ref   - x(:,6);   % yaw error
end

% position
figure('Color', 'w');
subplot(3,1,1);
plot(t, x(:,1), 'b', t, x_ref, 'r--', 'LineWidth', 1.5);
ylabel('x (m)');
title('Position X');
grid on;
legend('x','x_{ref}')

subplot(3,1,2);
plot(t, x(:,2), 'b', t, y_ref, 'r--', 'LineWidth', 1.5);
ylabel('y (m)');
title('Position Y');
grid on;
legend('y','y_{ref}')

subplot(3,1,3);
plot(t, x(:,3), 'b', t, z_ref, 'r--','LineWidth', 1.5);
ylabel('z (m)');
title('Position Z');
grid on;
legend('z','z_{ref}')

% position errors
figure('Color','w');
subplot(3,1,1);
plot(t, ex, 'LineWidth', 1.5);
ylabel('e_x (m)');
title('Position Tracking Error');
grid on;

subplot(3,1,2);
plot(t, ey, 'LineWidth', 1.5);
ylabel('e_y (m)');
grid on;

subplot(3,1,3);
plot(t, ez, 'LineWidth', 1.5);
ylabel('e_z (m)');
xlabel('Time (s)');
grid on;

% attitude
figure ('Color', 'W');
subplot(3,1,1);
plot(t, x(:,4)*180/pi, 'LineWidth', 1.5);
ylabel('\phi (deg)');
title('Roll');
grid on;

subplot(3,1,2);
plot(t, x(:,5)*180/pi, 'LineWidth', 1.5);
ylabel('\theta (deg)');
title('Pitch');
grid on;

subplot(3,1,3);
plot(t, x(:,6)*180/pi, 'LineWidth', 1.5);
ylabel('\psi (deg)');
title('Yaw');
grid on;

% % velocity
% figure('Color', 'W');
% 
% subplot(3,1,1);
% plot(t, x(:,7), 'LineWidth', 1.5);
% ylabel('ẋ (m/s)'); 
% xlabel('Time (s)');
% title('Velocity X');
% grid on;
% 
% subplot(3,1,2);
% plot(t, x(:,8), 'LineWidth', 1.5);
% ylabel('ẏ (m/s)');
% xlabel('Time (s)');
% title('Velocity Y');
% grid on;
% 
% subplot(3,1,3);
% plot(t, x(:,9), 'LineWidth', 1.5);
% ylabel('ż (m/s)');
% xlabel('Time (s)');
% title('Velocity Z');
% grid on;
% 
% % velocity errors
% figure('Color','w');
% 
% subplot(3,1,1);
% plot(t, ex_dot, 'LineWidth', 1.5);
% ylabel('e_dot_x (m/s)');
% title('Velocity Tracking Error');
% grid on;
% 
% subplot(3,1,2);
% plot(t, ey_dot, 'LineWidth', 1.5);
% ylabel('e_dot_y (m/s)');
% grid on;
% 
% subplot(3,1,3);
% plot(t, ez_dot, 'LineWidth', 1.5);
% ylabel('e_dot_z (m/s)');
% xlabel('Time (s)');
% grid on;

% Preallocate control inputs
u1 = zeros(length(t),1);
u2 = zeros(length(t),1);
u3 = zeros(length(t),1);
u4 = zeros(length(t),1);

for i = 1:length(t)
    % Extract states
    x_pos     = x(i,1);  y_pos     = x(i,2);  z_pos     = x(i,3);
    phi       = x(i,4);  theta     = x(i,5);  psi       = x(i,6);
    x_pos_dot = x(i,7);  y_pos_dot = x(i,8);  z_pos_dot = x(i,9);
    phi_dot   = x(i,10); theta_dot = x(i,11); psi_dot   = x(i,12);

    % Desired trajectory
    x_des     = params.x_des(t(i));
    y_des     = params.y_des(t(i));
    z_des     = params.z_des(t(i));
    x_dot_des = params.x_dot_des(t(i));
    y_dot_des = params.y_dot_des(t(i));
    z_dot_des = params.z_dot_des(t(i));

    % Errors
    e_x = x_des - x_pos;   e_x_dot = x_dot_des - x_pos_dot;
    e_y = y_des - y_pos;   e_y_dot = y_dot_des - y_pos_dot;
    e_z = z_des - z_pos;   e_z_dot = z_dot_des - z_pos_dot;

    % Gains (same as inside dynamics)
    Kp_x=10; Kd_x=14; Kp_y=10; Kd_y=14; Kp_z=20; Kd_z=8;
    Kp_phi=50; Kd_phi=15; Kp_theta=50; Kd_theta=15; Kp_psi=50; Kd_psi=15;

    % Outer loop commanded accelerations
    x_ddot_cmd = Kp_x*e_x + Kd_x*e_x_dot;
    y_ddot_cmd = Kp_y*e_y + Kd_y*e_y_dot;
    z_ddot_cmd = Kp_z*e_z + Kd_z*e_z_dot;

    % Desired roll & pitch
    phi_des   = -(1/params.g) * (y_ddot_cmd);
    theta_des =  (1/params.g) * (x_ddot_cmd);
    psi_des   = params.psi_des(t(i));

    % Control inputs
    u1(i) = params.m * (params.g + z_ddot_cmd);
    u2(i) = Kp_phi*(phi_des - phi)   + Kd_phi*(0 - phi_dot);
    u3(i) = Kp_theta*(theta_des - theta) + Kd_theta*(0 - theta_dot);
    u4(i) = Kp_psi*(psi_des - psi)   + Kd_psi*(0 - psi_dot);
end
% control inputs
figure('Color','w');
subplot(4,1,1);
plot(t,u1,'LineWidth',1.5);
ylabel('u1 (Thrust)');
title('Control Inputs');
grid on;

subplot(4,1,2);
plot(t,u2,'LineWidth',1.5);
ylabel('u2 (Roll Torque)');
grid on;

subplot(4,1,3);
plot(t,u3,'LineWidth',1.5);
ylabel('u3 (Pitch Torque)');
grid on;

subplot(4,1,4);
plot(t,u4,'LineWidth',1.5);
ylabel('u4 (Yaw Torque)');
xlabel('Time (s)');
grid on;


function performance_metrics(t, ex, ey, ez, ephi, etheta, epsi)
    RMS_ex = sqrt(mean(ex.^2));
    RMS_ey = sqrt(mean(ey.^2));
    RMS_ez = sqrt(mean(ez.^2));

    RMS_phi   = sqrt(mean(ephi.^2));
    RMS_theta = sqrt(mean(etheta.^2));
    RMS_psi   = sqrt(mean(epsi.^2));

    % IAE_x = trapz(t, abs(ex));
    % IAE_y = trapz(t, abs(ey));
    % IAE_z = trapz(t, abs(ez));
    % 
    % ISE_x = trapz(t, ex.^2);
    % ISE_y = trapz(t, ey.^2);
    % ISE_z = trapz(t, ez.^2);
    % 
    % ITAE_x = trapz(t, t .* abs(ex));
    % ITAE_y = trapz(t, t .* abs(ey));
    % ITAE_z = trapz(t, t .* abs(ez));

    ess_x = ex(end);
    ess_y = ey(end);
    ess_z = ez(end);

    ess_phi = ephi(end);
    ess_theta = etheta(end);
    ess_psi = epsi(end);


    fprintf('\nTracking Performance Metrics\n');

    fprintf('RMS Position Error:    [x y z] = [%.4f %.4f %.4f]\n', RMS_ex, RMS_ey, RMS_ez);
    fprintf('RMS Attitude Error: [phi theta psi] = [%.4f %.4f %.4f]\n', RMS_phi, RMS_theta, RMS_psi);
    % fprintf('IAE:          [x y z] = [%.4f %.4f %.4f]\n', IAE_x, IAE_y, IAE_z);
    % fprintf('ISE:          [x y z] = [%.4f %.4f %.4f]\n', ISE_x, ISE_y, ISE_z);
    % fprintf('ITAE:         [x y z] = [%.4f %.4f %.4f]\n', ITAE_x, ITAE_y, ITAE_z);
    fprintf('Position Steady-State: [x y z] = [%.4f %.4f %.4f]\n', ess_x, ess_y, ess_z);
    fprintf('Attitude Steady-State: [phi theta psi] = [%.4f %.4f %.4f]\n', ess_phi, ess_theta, ess_psi);
end

performance_metrics(t, ex, ey, ez, ephi, etheta, epsi);