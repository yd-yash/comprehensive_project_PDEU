% PID Controller for Quadcopter System
% 3D motion control with Lissajous Curve Reference Trajectory
% Following dynamics from Ban Wang
% claude

% clear all;
% close all;
% clc;

parameters = set_parameters();
tspan = 0:0.02:20;

% Initial state: [x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r]
initial_state = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];

state_history = zeros(length(initial_state), length(tspan));
control_history = zeros(4, length(tspan));
reference_history = zeros(6, length(tspan));

state_history(:, 1) = initial_state;

% Main simulation loop
for i = 1:length(tspan)-1
    current_state = state_history(:, i);
    current_time = tspan(i);
    
    % Lissajous curve reference trajectory
    reference = time_varying_reference(current_time);
    reference_history(:, i) = reference;
    
    % PID controller
    u = pid_controller(current_state, reference, current_time, parameters);
    
    control_history(:, i) = u;
    
    % Integrate dynamics
    next_state = integrate_dynamics(tspan(i), tspan(i+1), current_state, u, parameters);
    
    state_history(:, i+1) = next_state;
end

% Final reference and control
reference_history(:, end) = time_varying_reference(tspan(end));
control_history(:, end) = pid_controller(state_history(:, end), reference_history(:, end), tspan(end), parameters);

% Plot all results
plot_results(tspan, state_history, control_history, reference_history, parameters);

%% Reference Trajectory Generation - Lissajous Curve
function reference = time_varying_reference(t)
    % Lissajous curve reference trajectory
    % x(t) = Ax * sin(ax*t + δx)
    % y(t) = Ay * sin(ay*t + δy)
    % z(t) = Az * sin(az*t + δz)
    
    % Lissajous parameters
    A_x = 2.0;          % Amplitude for x (m)
    A_y = 2.0;          % Amplitude for y (m)
    A_z = 0.8;          % Amplitude for z (m)
    
    a_x = 1.0;          % Frequency ratio for x
    a_y = 2.0;          % Frequency ratio for y (creates figure-8 in x-y)
    a_z = 1.5;          % Frequency ratio for z
    
    omega = 0.3;        % Base frequency (rad/s)
    
    delta_x = 0;        % Phase shift for x
    delta_y = pi/2;     % Phase shift for y
    delta_z = 0;        % Phase shift for z
    
    z_center = 2.0;     % Center height (m)
    
    if t < 3
        % Initial hover - smooth transition to start position
        progress = t/3;
        % Start position of Lissajous curve at t=0
        x_start = A_x * sin(delta_x);
        y_start = A_y * sin(delta_y);
        z_start = A_z * sin(delta_z) + z_center;
        
        x_ref = x_start * progress;
        y_ref = y_start * progress;
        z_ref = 0.5 + (z_start - 0.5) * progress;
        phi_ref = 0;
        theta_ref = 0;
        psi_ref = 0;
        
    elseif t < 17
        % Lissajous curve trajectory
        t_adj = t - 3;
        x_ref = A_x * sin(a_x * omega * t_adj + delta_x);
        y_ref = A_y * sin(a_y * omega * t_adj + delta_y);
        z_ref = A_z * sin(a_z * omega * t_adj + delta_z) + z_center;
        
        % Keep attitude level
        phi_ref = 0;
        theta_ref = 0;
        psi_ref = 0;
        
    else
        % Final hover - transition to origin
        t_transition = (t - 17) / 3;
        t_transition = min(t_transition, 1);
        
        % Get final Lissajous values
        t_final = 14;
        x_final = A_x * sin(a_x * omega * t_final + delta_x);
        y_final = A_y * sin(a_y * omega * t_final + delta_y);
        z_final = A_z * sin(a_z * omega * t_final + delta_z) + z_center;
        
        % Smooth transition to hover at origin
        x_ref = x_final * (1 - t_transition);
        y_ref = y_final * (1 - t_transition);
        z_ref = z_final * (1 - t_transition) + 2.0 * t_transition;
        phi_ref = 0;
        theta_ref = 0;
        psi_ref = 0;
    end
    
    reference = [x_ref; y_ref; z_ref; phi_ref; theta_ref; psi_ref];
end

function deriv = time_varying_reference_deriv(t)
    % Compute derivatives of Lissajous reference trajectory
    A_x = 2.0;
    A_y = 2.0;
    A_z = 0.8;
    
    a_x = 1.0;
    a_y = 2.0;
    a_z = 1.5;
    
    omega = 0.3;
    
    delta_x = 0;
    delta_y = pi/2;
    delta_z = 0;
    
    if t < 3
        % Initial hover phase
        x_start = A_x * sin(delta_x);
        y_start = A_y * sin(delta_y);
        z_start = A_z * sin(delta_z) + 2.0;
        
        x_ref_dot = x_start / 3;
        y_ref_dot = y_start / 3;
        z_ref_dot = (z_start - 0.5) / 3;
        
    elseif t < 17
        % Lissajous curve trajectory
        t_adj = t - 3;
        x_ref_dot = A_x * a_x * omega * cos(a_x * omega * t_adj + delta_x);
        y_ref_dot = A_y * a_y * omega * cos(a_y * omega * t_adj + delta_y);
        z_ref_dot = A_z * a_z * omega * cos(a_z * omega * t_adj + delta_z);
        
    else
        % Final hover - approximate transition rate
        x_ref_dot = 0;
        y_ref_dot = 0;
        z_ref_dot = 0;
    end
    
    phi_ref_dot = 0;
    theta_ref_dot = 0;
    psi_ref_dot = 0;
    
    deriv = [x_ref_dot; y_ref_dot; z_ref_dot; phi_ref_dot; theta_ref_dot; psi_ref_dot];
end

%% System Parameters
function parameters = set_parameters()
    % Physical parameters
    parameters.m = 1.0;             % Mass (kg)
    parameters.g = 9.81;            % Gravity (m/s^2)
    parameters.Ixx = 0.0075;        % Moment of inertia x-axis (kg·m^2)
    parameters.Iyy = 0.0075;        % Moment of inertia y-axis (kg·m^2)
    parameters.Izz = 0.013;         % Moment of inertia z-axis (kg·m^2)
    parameters.l = 0.23;            % Arm length (m)
    parameters.k = 1.0e-6;          % Thrust coefficient
    parameters.b = 1.0e-7;          % Drag coefficient
    
    % PID Controller gains for position (x, y, z) - REDUCED for stability
    parameters.Kp_x = 3.0;
    parameters.Ki_x = 0.1;
    parameters.Kd_x = 4.0;
    
    parameters.Kp_y = 3.0;
    parameters.Ki_y = 0.1;
    parameters.Kd_y = 4.0;
    
    parameters.Kp_z = 8.0;
    parameters.Ki_z = 0.8;
    parameters.Kd_z = 6.0;
    
    % PID Controller gains for attitude (phi, theta, psi) - REDUCED for stability
    parameters.Kp_phi = 4.0;
    parameters.Ki_phi = 0.05;
    parameters.Kd_phi = 2.5;
    
    parameters.Kp_theta = 4.0;
    parameters.Ki_theta = 0.05;
    parameters.Kd_theta = 2.5;
    
    parameters.Kp_psi = 2.5;
    parameters.Ki_psi = 0.05;
    parameters.Kd_psi = 1.5;
    
    % Integration parameters
    parameters.dt = 0.01;
    
    % Control limits - INCREASED saturation limits
    parameters.u_min = 0.0;
    parameters.u_max = 25.0;
    parameters.torque_max = 3.0;  % Increased from 2.0
    
    % Integral error storage (persistent across calls)
    parameters.int_error_x = 0;
    parameters.int_error_y = 0;
    parameters.int_error_z = 0;
    parameters.int_error_phi = 0;
    parameters.int_error_theta = 0;
    parameters.int_error_psi = 0;
    parameters.prev_time = 0;
end

%% System Dynamics
function dxdt = system_dynamics(t, state, u, parameters)
    % State vector: [x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r]
    x = state(1);
    y = state(2);
    z = state(3);
    phi = state(4);
    theta = state(5);
    psi = state(6);
    x_dot = state(7);
    y_dot = state(8);
    z_dot = state(9);
    p = state(10);
    q = state(11);
    r = state(12);
    
    % Parameters
    m = parameters.m;
    g = parameters.g;
    Ixx = parameters.Ixx;
    Iyy = parameters.Iyy;
    Izz = parameters.Izz;
    
    % Control inputs: [u1, u2, u3, u4]
    % u1 = total thrust, u2 = roll torque, u3 = pitch torque, u4 = yaw torque
    u1 = u(1);
    u2 = u(2);
    u3 = u(3);
    u4 = u(4);
    
    % Translational dynamics (from dynamics equations)
    x_ddot = (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi)) * u1/m;
    y_ddot = (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi)) * u1/m;
    z_ddot = (cos(phi)*cos(theta)) * u1/m - g;
    
    % Rotational kinematics
    phi_dot = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r;
    theta_dot = cos(phi)*q - sin(phi)*r;
    psi_dot = (sin(phi)/cos(theta))*q + (cos(phi)/cos(theta))*r;
    
    % Rotational dynamics
    p_dot = ((Iyy - Izz)/Ixx)*q*r + u2/Ixx;
    q_dot = ((Izz - Ixx)/Iyy)*p*r + u3/Iyy;
    r_dot = ((Ixx - Iyy)/Izz)*p*q + u4/Izz;
    
    % Return derivative vector
    dxdt = [x_dot; y_dot; z_dot; phi_dot; theta_dot; psi_dot; ...
            x_ddot; y_ddot; z_ddot; p_dot; q_dot; r_dot];
end

%% PID Controller
function u = pid_controller(state, reference, current_time, parameters)
    persistent int_error_x int_error_y int_error_z
    persistent int_error_phi int_error_theta int_error_psi
    persistent prev_time
    
    % Initialize persistent variables
    if isempty(prev_time)
        prev_time = 0;
        int_error_x = 0;
        int_error_y = 0;
        int_error_z = 0;
        int_error_phi = 0;
        int_error_theta = 0;
        int_error_psi = 0;
    end
    
    % Calculate dt
    dt = current_time - prev_time;
    if dt <= 0
        dt = parameters.dt;
    end
    prev_time = current_time;
    
    % Extract states
    x = state(1);
    y = state(2);
    z = state(3);
    phi = state(4);
    theta = state(5);
    psi = state(6);
    x_dot = state(7);
    y_dot = state(8);
    z_dot = state(9);
    p = state(10);
    q = state(11);
    r = state(12);
    
    % Extract references
    x_ref = reference(1);
    y_ref = reference(2);
    z_ref = reference(3);
    phi_ref = reference(4);
    theta_ref = reference(5);
    psi_ref = reference(6);
    
    % Get reference derivatives
    ref_deriv = time_varying_reference_deriv(current_time);
    x_ref_dot = ref_deriv(1);
    y_ref_dot = ref_deriv(2);
    z_ref_dot = ref_deriv(3);
    phi_ref_dot = ref_deriv(4);
    theta_ref_dot = ref_deriv(5);
    psi_ref_dot = ref_deriv(6);
    
    % Position errors
    e_x = x - x_ref;
    e_y = y - y_ref;
    e_z = z - z_ref;
    
    % Velocity errors
    e_x_dot = x_dot - x_ref_dot;
    e_y_dot = y_dot - y_ref_dot;
    e_z_dot = z_dot - z_ref_dot;
    
    % Update integral errors with anti-windup
    int_error_x = int_error_x + e_x * dt;
    int_error_y = int_error_y + e_y * dt;
    int_error_z = int_error_z + e_z * dt;
    
    % Anti-windup with smaller limits
    int_error_x = max(-0.5, min(0.5, int_error_x));
    int_error_y = max(-0.5, min(0.5, int_error_y));
    int_error_z = max(-0.5, min(0.5, int_error_z));
    
    % Position PID control
    u_x = -parameters.Kp_x * e_x - parameters.Ki_x * int_error_x - parameters.Kd_x * e_x_dot;
    u_y = -parameters.Kp_y * e_y - parameters.Ki_y * int_error_y - parameters.Kd_y * e_y_dot;
    u_z = parameters.m * parameters.g - parameters.Kp_z * e_z - parameters.Ki_z * int_error_z - parameters.Kd_z * e_z_dot;
    
    % Desired angles from position control (with limits)
    phi_des = (1/parameters.g) * (u_x * sin(psi_ref) - u_y * cos(psi_ref));
    theta_des = (1/parameters.g) * (u_x * cos(psi_ref) + u_y * sin(psi_ref));
    
    % Limit desired angles to prevent aggressive maneuvers
    phi_des = max(-0.25, min(0.25, phi_des));      % ±14 degrees
    theta_des = max(-0.25, min(0.25, theta_des));  % ±14 degrees
    
    % Attitude errors
    e_phi = phi - phi_des;
    e_theta = theta - theta_des;
    e_psi = psi - psi_ref;
    
    % Normalize psi error to [-pi, pi]
    e_psi = atan2(sin(e_psi), cos(e_psi));
    
    % Angular velocity errors
    e_phi_dot = p - phi_ref_dot;
    e_theta_dot = q - theta_ref_dot;
    e_psi_dot = r - psi_ref_dot;
    
    % Update integral errors for attitude with anti-windup
    int_error_phi = int_error_phi + e_phi * dt;
    int_error_theta = int_error_theta + e_theta * dt;
    int_error_psi = int_error_psi + e_psi * dt;
    
    % Anti-windup for attitude with smaller limits
    int_error_phi = max(-0.3, min(0.3, int_error_phi));
    int_error_theta = max(-0.3, min(0.3, int_error_theta));
    int_error_psi = max(-0.3, min(0.3, int_error_psi));
    
    % Attitude PID control
    u2 = -parameters.Kp_phi * e_phi - parameters.Ki_phi * int_error_phi - parameters.Kd_phi * e_phi_dot;
    u3 = -parameters.Kp_theta * e_theta - parameters.Ki_theta * int_error_theta - parameters.Kd_theta * e_theta_dot;
    u4 = -parameters.Kp_psi * e_psi - parameters.Ki_psi * int_error_psi - parameters.Kd_psi * e_psi_dot;
    
    % Total thrust
    u1 = u_z / (cos(phi) * cos(theta));
    
    % Apply control limits
    u1 = max(parameters.u_min, min(parameters.u_max, u1));
    u2 = max(-parameters.torque_max, min(parameters.torque_max, u2));
    u3 = max(-parameters.torque_max, min(parameters.torque_max, u3));
    u4 = max(-parameters.torque_max, min(parameters.torque_max, u4));
    
    u = [u1; u2; u3; u4];
end

%% Numerical Integration
function next_state = integrate_dynamics(t, t_next, state, u, parameters)
    options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8, 'MaxStep', parameters.dt/2);
    
    [t_out, states] = ode45(@(t, s) system_dynamics(t, s, u, parameters), [t t_next], state, options);
    
    next_state = states(end, :)';
end

%% Plotting Results
function plot_results(tspan, state_history, control_history, reference_history, parameters)
    % Extract states
    x = state_history(1, :);
    y = state_history(2, :);
    z = state_history(3, :);
    phi = state_history(4, :);
    theta = state_history(5, :);
    psi = state_history(6, :);
    x_dot = state_history(7, :);
    y_dot = state_history(8, :);
    z_dot = state_history(9, :);
    p = state_history(10, :);
    q = state_history(11, :);
    r = state_history(12, :);
    
    % Extract controls
    u1 = control_history(1, :);
    u2 = control_history(2, :);
    u3 = control_history(3, :);
    u4 = control_history(4, :);
    
    % Extract references
    x_ref = reference_history(1, :);
    y_ref = reference_history(2, :);
    z_ref = reference_history(3, :);
    phi_ref = reference_history(4, :);
    theta_ref = reference_history(5, :);
    psi_ref = reference_history(6, :);
    
    % Plot 1: 3D Trajectory
    figure('Name', '3D Trajectory - PID Controller (Lissajous)');
    plot3(x_ref, y_ref, z_ref, 'r--', 'LineWidth', 2.5);
    hold on;
    plot3(x, y, z, 'b-', 'LineWidth', 2);
    plot3(x(1), y(1), z(1), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
    plot3(x(end), y(end), z(end), 'rs', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('3D Lissajous Trajectory: Actual vs Reference');
    legend('Reference', 'Actual', 'Start', 'End', 'Location', 'best');
    grid on;
    view(45, 30);
    
    % Plot 2: Position States vs Time
    figure('Name', 'Position States vs Time - PID Controller');
    subplot(3,1,1);
    plot(tspan, x_ref, 'r--', 'LineWidth', 2.5);
    hold on;
    plot(tspan, x, 'b-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('x (m)');
    title('X Position');
    legend('x_{ref}', 'x', 'Location', 'best');
    grid on;
    
    subplot(3,1,2);
    plot(tspan, y_ref, 'r--', 'LineWidth', 2.5);
    hold on;
    plot(tspan, y, 'b-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('y (m)');
    title('Y Position');
    legend('y_{ref}', 'y', 'Location', 'best');
    grid on;
    
    subplot(3,1,3);
    plot(tspan, z_ref, 'r--', 'LineWidth', 2.5);
    hold on;
    plot(tspan, z, 'b-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('z (m)');
    title('Z Position');
    legend('z_{ref}', 'z', 'Location', 'best');
    grid on;
    
    % Plot 3: Attitude States vs Time
    figure('Name', 'Attitude States vs Time - PID Controller');
    subplot(3,1,1);
    plot(tspan, rad2deg(phi_ref), 'r--', 'LineWidth', 2.5);
    hold on;
    plot(tspan, rad2deg(phi), 'b-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('\phi (deg)');
    title('Roll Angle');
    legend('\phi_{ref}', '\phi', 'Location', 'best');
    grid on;
    
    subplot(3,1,2);
    plot(tspan, rad2deg(theta_ref), 'r--', 'LineWidth', 2.5);
    hold on;
    plot(tspan, rad2deg(theta), 'b-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('\theta (deg)');
    title('Pitch Angle');
    legend('\theta_{ref}', '\theta', 'Location', 'best');
    grid on;
    
    subplot(3,1,3);
    plot(tspan, rad2deg(psi_ref), 'r--', 'LineWidth', 2.5);
    hold on;
    plot(tspan, rad2deg(psi), 'b-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('\psi (deg)');
    title('Yaw Angle');
    legend('\psi_{ref}', '\psi', 'Location', 'best');
    grid on;
    
    % Plot 4: Velocity States
    figure('Name', 'Velocity States - PID Controller');
    subplot(3,1,1);
    plot(tspan, x_dot, 'b-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('x_{dot} (m/s)');
    title('X Velocity');
    grid on;
    
    subplot(3,1,2);
    plot(tspan, y_dot, 'b-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('y_{dot} (m/s)');
    title('Y Velocity');
    grid on;
    
    subplot(3,1,3);
    plot(tspan, z_dot, 'b-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('z_{dot} (m/s)');
    title('Z Velocity');
    grid on;
    
    % Plot 5: Control Efforts
    figure('Name', 'Control Effort - PID Controller');
    subplot(4,1,1);
    plot(tspan, u1, 'b-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('u_1 (N)');
    title('Total Thrust');
    grid on;
    
    subplot(4,1,2);
    plot(tspan, u2, 'r-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('u_2 (N·m)');
    title('Roll Torque');
    grid on;
    
    subplot(4,1,3);
    plot(tspan, u3, 'g-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('u_3 (N·m)');
    title('Pitch Torque');
    grid on;
    
    subplot(4,1,4);
    plot(tspan, u4, 'm-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('u_4 (N·m)');
    title('Yaw Torque');
    grid on;
    
    % Plot 6: Tracking Errors
    figure('Name', 'Tracking Errors - PID Controller');
    subplot(3,2,1);
    plot(tspan, x - x_ref, 'b-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('e_x (m)');
    title('X Position Error');
    grid on;
    
    subplot(3,2,2);
    plot(tspan, rad2deg(phi - phi_ref), 'b-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('e_\phi (deg)');
    title('Roll Angle Error');
    grid on;
    
    subplot(3,2,3);
    plot(tspan, y - y_ref, 'r-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('e_y (m)');
    title('Y Position Error');
    grid on;
    
    subplot(3,2,4);
    plot(tspan, rad2deg(theta - theta_ref), 'r-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('e_\theta (deg)');
    title('Pitch Angle Error');
    grid on;
    
    subplot(3,2,5);
    plot(tspan, z - z_ref, 'g-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('e_z (m)');
    title('Z Position Error');
    grid on;
    
    subplot(3,2,6);
    plot(tspan, rad2deg(psi - psi_ref), 'g-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('e_\psi (deg)');
    title('Yaw Angle Error');
    grid on;
    
    % Plot 7: X-Y Trajectory
    figure('Name', 'X-Y Trajectory - PID Controller (Lissajous)');
    plot(x_ref, y_ref, 'r--', 'LineWidth', 2.5);
    hold on;
    plot(x, y, 'b-', 'LineWidth', 2);
    plot(x(1), y(1), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
    plot(x(end), y(end), 'rs', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
    xlabel('X (m)');
    ylabel('Y (m)');
    title('X-Y Plane Lissajous Trajectory');
    legend('Reference', 'Actual', 'Start', 'End', 'Location', 'best');
    grid on;
    axis equal;
    
    % Plot 8: Reference Signals
    figure('Name', 'Reference Trajectory Signals - Lissajous Curve');
    subplot(2,2,1);
    plot(tspan, x_ref, 'r-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('x_{ref} (m)');
    title('X Reference Signal');
    grid on;
    
    subplot(2,2,2);
    plot(tspan, y_ref, 'b-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('y_{ref} (m)');
    title('Y Reference Signal');
    grid on;
    
    subplot(2,2,3);
    plot(tspan, z_ref, 'g-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('z_{ref} (m)');
    title('Z Reference Signal');
    grid on;
    
    subplot(2,2,4);
    plot3(x_ref, y_ref, z_ref, 'k-', 'LineWidth', 2);
    xlabel('x_{ref} (m)');
    ylabel('y_{ref} (m)');
    zlabel('z_{ref} (m)');
    title('3D Reference Lissajous Curve');
    grid on;
    view(45, 30);
    
    % Calculate and display performance metrics
    fprintf('\n========== PID Controller Performance Metrics (Lissajous) ==========\n');
    fprintf('\nLissajous Trajectory Parameters:\n');
    fprintf('  X amplitude: %.2f m, frequency ratio: %.1f\n', 2.0, 1.0);
    fprintf('  Y amplitude: %.2f m, frequency ratio: %.1f\n', 2.0, 2.0);
    fprintf('  Z amplitude: %.2f m, frequency ratio: %.1f\n', 0.8, 1.5);
    fprintf('  Base frequency: %.2f rad/s\n', 0.3);
    fprintf('  Z center height: %.2f m\n', 2.0);
    
    fprintf('\n--- Position Tracking Performance ---\n');
    fprintf('X Position:\n');
    fprintf('  Final error: %.4f m\n', abs(x(end) - x_ref(end)));
    fprintf('  RMS error: %.4f m\n', sqrt(mean((x - x_ref).^2)));
    fprintf('  Max error: %.4f m\n', max(abs(x - x_ref)));
    fprintf('  Steady-state error (last 2s): %.4f m\n', mean(abs(x(end-100:end) - x_ref(end-100:end))));
    
    fprintf('\nY Position:\n');
    fprintf('  Final error: %.4f m\n', abs(y(end) - y_ref(end)));
    fprintf('  RMS error: %.4f m\n', sqrt(mean((y - y_ref).^2)));
    fprintf('  Max error: %.4f m\n', max(abs(y - y_ref)));
    fprintf('  Steady-state error (last 2s): %.4f m\n', mean(abs(y(end-100:end) - y_ref(end-100:end))));
    
    fprintf('\nZ Position:\n');
    fprintf('  Final error: %.4f m\n', abs(z(end) - z_ref(end)));
    fprintf('  RMS error: %.4f m\n', sqrt(mean((z - z_ref).^2)));
    fprintf('  Max error: %.4f m\n', max(abs(z - z_ref)));
    fprintf('  Steady-state error (last 2s): %.4f m\n', mean(abs(z(end-100:end) - z_ref(end-100:end))));
    
    fprintf('\n--- Attitude Tracking Performance ---\n');
    fprintf('Roll Angle (phi):\n');
    fprintf('  Final error: %.4f deg\n', rad2deg(abs(phi(end) - phi_ref(end))));
    fprintf('  RMS error: %.4f deg\n', rad2deg(sqrt(mean((phi - phi_ref).^2))));
    fprintf('  Max error: %.4f deg\n', rad2deg(max(abs(phi - phi_ref))));
    fprintf('  Steady-state error (last 2s): %.4f deg\n', rad2deg(mean(abs(phi(end-100:end) - phi_ref(end-100:end)))));
    
    fprintf('\nPitch Angle (theta):\n');
    fprintf('  Final error: %.4f deg\n', rad2deg(abs(theta(end) - theta_ref(end))));
    fprintf('  RMS error: %.4f deg\n', rad2deg(sqrt(mean((theta - theta_ref).^2))));
    fprintf('  Max error: %.4f deg\n', rad2deg(max(abs(theta - theta_ref))));
    fprintf('  Steady-state error (last 2s): %.4f deg\n', rad2deg(mean(abs(theta(end-100:end) - theta_ref(end-100:end)))));
    
    fprintf('\nYaw Angle (psi):\n');
    fprintf('  Final error: %.4f deg\n', rad2deg(abs(psi(end) - psi_ref(end))));
    fprintf('  RMS error: %.4f deg\n', rad2deg(sqrt(mean((psi - psi_ref).^2))));
    fprintf('  Max error: %.4f deg\n', rad2deg(max(abs(psi - psi_ref))));
    fprintf('  Steady-state error (last 2s): %.4f deg\n', rad2deg(mean(abs(psi(end-100:end) - psi_ref(end-100:end)))));
    
    fprintf('\n--- Final State Values ---\n');
    fprintf('Position: [%.4f, %.4f, %.4f] m\n', x(end), y(end), z(end));
    fprintf('Velocity: [%.4f, %.4f, %.4f] m/s\n', x_dot(end), y_dot(end), z_dot(end));
    fprintf('Attitude: [%.2f, %.2f, %.2f] deg\n', rad2deg(phi(end)), rad2deg(theta(end)), rad2deg(psi(end)));
    fprintf('Angular rates: [%.4f, %.4f, %.4f] deg/s\n', rad2deg(p(end)), rad2deg(q(end)), rad2deg(r(end)));
    
    fprintf('\n--- Control Effort Metrics ---\n');
    fprintf('Thrust (u1):\n');
    fprintf('  Mean: %.4f N\n', mean(u1));
    fprintf('  Max: %.4f N\n', max(u1));
    fprintf('  Min: %.4f N\n', min(u1));
    fprintf('  Std Dev: %.4f N\n', std(u1));
    
    fprintf('\nRoll Torque (u2):\n');
    fprintf('  Mean magnitude: %.4f N·m\n', mean(abs(u2)));
    fprintf('  Max magnitude: %.4f N·m\n', max(abs(u2)));
    fprintf('  Std Dev: %.4f N·m\n', std(u2));
    
    fprintf('\nPitch Torque (u3):\n');
    fprintf('  Mean magnitude: %.4f N·m\n', mean(abs(u3)));
    fprintf('  Max magnitude: %.4f N·m\n', max(abs(u3)));
    fprintf('  Std Dev: %.4f N·m\n', std(u3));
    
    fprintf('\nYaw Torque (u4):\n');
    fprintf('  Mean magnitude: %.4f N·m\n', mean(abs(u4)));
    fprintf('  Max magnitude: %.4f N·m\n', max(abs(u4)));
    fprintf('  Std Dev: %.4f N·m\n', std(u4));
    
    fprintf('\n--- Overall Performance Summary ---\n');
    fprintf('Total 3D position RMS error: %.4f m\n', sqrt(mean((x - x_ref).^2 + (y - y_ref).^2 + (z - z_ref).^2)));
    fprintf('Total attitude RMS error: %.4f deg\n', rad2deg(sqrt(mean((phi - phi_ref).^2 + (theta - theta_ref).^2 + (psi - psi_ref).^2))));
    fprintf('Simulation time: %.2f s\n', tspan(end));
    fprintf('Number of time steps: %d\n', length(tspan));
    
    fprintf('\n===================================================================\n');
end