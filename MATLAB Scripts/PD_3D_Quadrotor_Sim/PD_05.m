% Trajectory - constant velocity flight

clc;
clear;
close all;

% Parameters
params.m = 0.5;
params.g = 9.81;
params.Ixx = 0.0023;
params.Iyy = 0.0023;
params.Izz = 0.004;

% Reference 
v = 0.5;
params.x_des = @(t) v*t;
params.y_des = @(t) 0;
params.z_des = @(t) 2;

params.x_dot_des = @(t) v;
params.y_dot_des = @(t) 0;
params.z_dot_des = @(t) 0;

params.x_ddot_des = @(t) 0;
params.y_ddot_des = @(t) 0;
params.z_ddot_des = @(t) 0;

params.psi_des = @(t) 0;
params.psi_ref = @(t) 0;

% Initial condition
x0 = zeros(12, 1);

T = 20;
tspan = linspace(0, T, 10000);

% Dynamics
function dx = quad3D_dynamics(t, x, p)
    % states
    x_pos = x(1);
    y_pos = x(2);
    z_pos = x(3);

    phi = x(4);
    theta = x(5);
    psi = x(6);

    x_dot = x(7);
    y_dot = x(8);
    z_dot = x(9);

    phi_dot = x(10);
    theta_dot = x(11);
    psi_dot = x(12);

    % reference
    x_des = p.x_des(t);
    y_des = p.y_des(t);
    z_des = p.z_des(t);

    x_dot_des = p.x_dot_des(t);
    y_dot_des = p.y_dot_des(t);
    z_dot_des = p.z_dot_des(t);

    % errors
    e_x = x_des - x_pos;
    e_x_dot = x_dot_des - x_dot;
    e_y = y_des - y_pos;
    e_y_dot = y_dot_des - y_dot;
    e_z = z_des - z_pos;
    e_z_dot = z_dot_des - z_dot;

    % gains
    Kp_x = 6;
    Kd_x = 10;
    Kp_y = 6;
    Kd_y = 10;
    Kp_z = 20;
    Kd_z = 8;
    Kp_phi = 50;
    Kd_phi = 15;
    Kp_theta = 50;
    Kd_theta = 15;
    Kp_psi = 50;
    Kd_psi = 15;

    % desired acc
    x_ddot_c = Kp_x*e_x + Kd_x*e_x_dot;
    y_ddot_c = Kp_y*e_y + Kd_y*e_y_dot;
    z_ddot_c = Kp_z*e_z + Kd_z*e_z_dot;

    % desired roll and pitch from outer loop
    phi_des = -(1/p.g)*(y_ddot_c);
    theta_des = (1/p.g)*(x_ddot_c);

    % bounds
    phi_des = min(max(phi_des, deg2rad(-30)), deg2rad(30));
    theta_des = min(max(theta_des, deg2rad(-30)), deg2rad(30));
    psi_des_raw = p.psi_des(t);
    psi_des = min(max(psi_des_raw, deg2rad(-180)), deg2rad(180));

    % thrust
    T = p.m * (p.g + z_ddot_c); %change
    T = min(max(T, 0), 2*p.m*p.g);

    % torque
    tau_phi = Kp_phi*(phi_des - phi) + Kd_phi*(0-phi_dot);
    tau_theta = Kp_theta*(theta_des - theta) + Kd_theta*(0-theta_dot);
    tau_psi = Kp_psi*(psi_des - psi) + Kd_psi*(0-psi_dot);

    % bounds
    tau_phi = min(max(tau_phi, -0.5), 0.5);
    tau_theta = min(max(tau_theta, -0.5), 0.5);
    tau_psi = min(max(tau_psi, -0.3), 0.3);

    % accelerations
    x_ddot = (T/p.m)*(cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi));
    y_ddot = (T/p.m)*(cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi));
    z_ddot = (T/p.m)*cos(phi)*cos(theta) - p.g;

    phi_ddot = tau_phi / p.Ixx;
    theta_ddot = tau_theta / p.Iyy;
    psi_ddot = tau_psi / p.Izz;

    % bounds
    x_dot = min(max(x_dot, -5), 5);
    y_dot = min(max(y_dot, -5), 5);
    z_dot = min(max(z_dot, -5), 5);

    phi = min(max(phi, deg2rad(-45)), deg2rad(45));
    theta = min(max(theta, deg2rad(-45)), deg2rad(45));
    psi = min(max(psi, deg2rad(-180)), deg2rad(180));
    
    dx = [x_dot; y_dot; z_dot; phi_dot; theta_dot; psi_dot;
        x_ddot; y_ddot; z_ddot; phi_ddot; theta_ddot; psi_ddot];
end 

% Simulation 
[t, x] = ode45(@(t, x) quad3D_dynamics(t, x, params), tspan, x0);

function quad3D_plot_animate(t, x)
    figure('Color', 'w');
    axis  equal;
    axis([-1 12 -1 6 0 5]);
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

quad3D_plot_animate(t, x);

% Plots
figure('Color', 'w');

subplot(3,3,1);
plot(t, x(:,1), 'LineWidth', 1.5);
ylabel('x (m)');
title('Position X');
grid on;

subplot(3,3,2);
plot(t, x(:,2), 'LineWidth', 1.5);
ylabel('y (m)');
title('Position Y');
grid on;

subplot(3,3,3);
plot(t, x(:,3), 'LineWidth', 1.5);
ylabel('z (m)');
title('Position Z');
grid on;

subplot(3,3,4);
plot(t, x(:,4)*180/pi, 'LineWidth', 1.5);
ylabel('\phi (deg)');
title('Roll');
grid on;

subplot(3,3,5);
plot(t, x(:,5)*180/pi, 'LineWidth', 1.5);
ylabel('\theta (deg)');
title('Pitch');
grid on;

subplot(3,3,6);
plot(t, x(:,6)*180/pi, 'LineWidth', 1.5);
ylabel('\psi (deg)');
title('Yaw');
grid on;

subplot(3,3,7);
plot(t, x(:,7), 'LineWidth', 1.5);
ylabel('ẋ (m/s)'); 
xlabel('Time (s)');
title('Velocity X');
grid on;

subplot(3,3,8);
plot(t, x(:,8), 'LineWidth', 1.5);
ylabel('ẏ (m/s)');
xlabel('Time (s)');
title('Velocity Y');
grid on;

subplot(3,3,9);
plot(t, x(:,9), 'LineWidth', 1.5);
ylabel('ż (m/s)');
xlabel('Time (s)');
title('Velocity Z');
grid on;
