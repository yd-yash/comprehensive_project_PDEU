% dynamics with angular velocity
% Trajectory - helix
% updated u1 = T = thrust

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


% Initial condition
x0 = zeros(12, 1);

T = 30;
tspan = linspace(0, T, 10000);

% Dynamics
function dx = quad3D_dynamics(t,x,p)
    % States
    x_pos = x(1); y_pos = x(2); z_pos = x(3);
    phi   = x(4); theta = x(5); psi = x(6);
    xdot  = x(7); ydot  = x(8); zdot = x(9);
    p_b   = x(10); q_b  = x(11); r_b = x(12);

    % Reference
    x_des = p.x_des(t);
    y_des = p.y_des(t);
    z_des = p.z_des(t);

    xdot_des = p.x_dot_des(t);
    ydot_des = p.y_dot_des(t);
    zdot_des = p.z_dot_des(t);

    psi_des = p.psi_des(t);

    % Position errors
    ex = x_des - x_pos;
    ey = y_des - y_pos;
    ez = z_des - z_pos;

    ex_dot = xdot_des - xdot;
    ey_dot = ydot_des - ydot;
    ez_dot = zdot_des - zdot;

    % Gains
    Kp_x = 6;   Kd_x = 10;
    Kp_y = 6;   Kd_y = 10;
    Kp_z = 20;  Kd_z = 8;

    Kp_phi = 50;  Kd_phi = 15;
    Kp_th  = 50;  Kd_th  = 15;
    Kp_psi = 50;  Kd_psi = 15;

    % Desired accelerations (outer loop)
    x_ddot_c = Kp_x*ex + Kd_x*ex_dot;
    y_ddot_c = Kp_y*ey + Kd_y*ey_dot;
    z_ddot_c = Kp_z*ez + Kd_z*ez_dot;

    % Desired roll & pitch (small-angle inversion)
    phi_des   = -(1/p.g)*y_ddot_c;
    theta_des =  (1/p.g)*x_ddot_c;

    phi_des   = max(min(phi_des,deg2rad(30)),deg2rad(-30));
    theta_des = max(min(theta_des,deg2rad(30)),deg2rad(-30));

    % Thrust
    u1 = p.m * (p.g + z_ddot_c) / (cos(phi)*cos(theta));
    % u1 = p.m*(p.g + z_ddot_c);
    u1 = max(min(u1,2*p.m*p.g),0);

    % Torques (still PD, now acting on body rates)
    u2 = Kp_phi*(phi_des - phi)   - Kd_phi*p_b;
    u3 = Kp_th *(theta_des-theta)- Kd_th *q_b;
    u4 = Kp_psi*(psi_des - psi)  - Kd_psi*r_b;

    % Translational dynamics (Eq. 1)
    x_ddot = (u1/p.m)*( cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi) );
    y_ddot = (u1/p.m)*( cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi) );
    z_ddot = (u1/p.m)*( cos(phi)*cos(theta) ) - p.g;

    % Euler angle kinematics (Eq. 2)
    phi_dot   = p_b + q_b*sin(phi)*tan(theta) + r_b*cos(phi)*tan(theta);
    theta_dot = q_b*cos(phi) - r_b*sin(phi);
    psi_dot   = q_b*sin(phi)/cos(theta) + r_b*cos(phi)/cos(theta);

    % Body angular velocity dynamics (Eq. 3)
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

% Simulation 
[t, x] = ode45(@(t, x) quad3D_dynamics(t, x, params), tspan, x0);

function quad3D_plot_animate(t, x)
    figure('Color', 'w');
    axis  equal;
    axis([-3 12 -3 6 0 5]);
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