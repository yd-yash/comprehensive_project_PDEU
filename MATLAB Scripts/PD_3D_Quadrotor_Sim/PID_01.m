% PID Controller for Quadcopter System
% 3D motion control with Time-Varying Reference Trajectory
% Following dynamics from Ban Wang
% claude
% Trajectory - x, y : circular and z : sinosuidal

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
    
    % Time-varying reference trajectory
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

%% Reference Trajectory Generation
function reference = time_varying_reference(t)
    % Time-varying reference signals for 3D trajectory
    % Reference: [x_ref, y_ref, z_ref, phi_ref, theta_ref, psi_ref]
    
    % Trajectory parameters
    A_x = 1.0;          % Amplitude for x (m)
    A_y = 1.0;          % Amplitude for y (m)
    A_z = 0.5;          % Amplitude for z (m)
    freq_xy = 0.15;     % Frequency for x-y circle (Hz)
    freq_z = 0.2;       % Frequency for z (Hz)
    z_center = 1.5;     % Center height (m)
    
    if t < 3
        % Initial hover - transition to start position
        x_ref = 0;
        y_ref = 0;
        z_ref = 0.5 + (z_center - 0.5) * (t/3);
        phi_ref = 0;
        theta_ref = 0;
        psi_ref = 0;
    elseif t < 15
        % Circular trajectory in x-y with sinusoidal z
        t_adj = t - 3;
        x_ref = A_x * cos(2*pi*freq_xy*t_adj);
        y_ref = A_y * sin(2*pi*freq_xy*t_adj);
        z_ref = A_z * sin(2*pi*freq_z*t_adj) + z_center;
        phi_ref = 0;
        theta_ref = 0;
        psi_ref = 2*pi*freq_xy*t_adj;  % Yaw follows circular path
    else
        % Final hover - transition to final position
        t_transition = (t - 15) / 3;
        t_transition = min(t_transition, 1);
        
        % Get final trajectory values
        t_final = 12;
        x_final = A_x * cos(2*pi*freq_xy*t_final);
        y_final = A_y * sin(2*pi*freq_xy*t_final);
        z_final = A_z * sin(2*pi*freq_z*t_final) + z_center;
        psi_final = 2*pi*freq_xy*t_final;
        
        % Smooth transition to hover
        x_ref = x_final * (1 - t_transition);
        y_ref = y_final * (1 - t_transition);
        z_ref = z_final * (1 - t_transition) + 1.0 * t_transition;
        phi_ref = 0;
        theta_ref = 0;
        psi_ref = psi_final * (1 - t_transition);
    end
    
    reference = [x_ref; y_ref; z_ref; phi_ref; theta_ref; psi_ref];
end

function deriv = time_varying_reference_deriv(t)
    % Compute derivatives of reference trajectory
    A_x = 1.0;
    A_y = 1.0;
    A_z = 0.5;
    freq_xy = 0.15;
    freq_z = 0.2;
    z_center = 1.5;
    
    if t < 3
        x_ref_dot = 0;
        y_ref_dot = 0;
        z_ref_dot = (z_center - 0.5) / 3;
        phi_ref_dot = 0;
        theta_ref_dot = 0;
        psi_ref_dot = 0;
    elseif t < 15
        t_adj = t - 3;
        x_ref_dot = -A_x * 2*pi*freq_xy * sin(2*pi*freq_xy*t_adj);
        y_ref_dot = A_y * 2*pi*freq_xy * cos(2*pi*freq_xy*t_adj);
        z_ref_dot = A_z * 2*pi*freq_z * cos(2*pi*freq_z*t_adj);
        phi_ref_dot = 0;
        theta_ref_dot = 0;
        psi_ref_dot = 2*pi*freq_xy;
    else
        x_ref_dot = 0;
        y_ref_dot = 0;
        z_ref_dot = 0;
        phi_ref_dot = 0;
        theta_ref_dot = 0;
        psi_ref_dot = 0;
    end
    
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
    parameters.k = 1.0e-6;          % Thrust coefficient
    parameters.b = 1.0e-7;          % Drag coefficient
    
    % PID Controller gains for position (x, y, z)
    parameters.Kp_x = 8.0;
    parameters.Ki_x = 0.5;
    parameters.Kd_x = 4.0;
    
    parameters.Kp_y = 8.0;
    parameters.Ki_y = 0.5;
    parameters.Kd_y = 4.0;
    
    parameters.Kp_z = 10.0;
    parameters.Ki_z = 2.0;
    parameters.Kd_z = 5.0;
    
    % PID Controller gains for attitude (phi, theta, psi)
    parameters.Kp_phi = 6.0;
    parameters.Ki_phi = 0.1;
    parameters.Kd_phi = 3.0;
    
    parameters.Kp_theta = 6.0;
    parameters.Ki_theta = 0.1;
    parameters.Kd_theta = 3.0;
    
    parameters.Kp_psi = 4.0;
    parameters.Ki_psi = 0.1;
    parameters.Kd_psi = 2.0;
    
    % Integration parameters
    parameters.dt = 0.01;
    
    % Control limits
    parameters.u_min = 0.0;
    parameters.u_max = 20.0;
    
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
    
    % Update integral errors
    int_error_x = int_error_x + e_x * dt;
    int_error_y = int_error_y + e_y * dt;
    int_error_z = int_error_z + e_z * dt;
    
    % Anti-windup
    int_error_x = max(-1.0, min(1.0, int_error_x));
    int_error_y = max(-1.0, min(1.0, int_error_y));
    int_error_z = max(-1.0, min(1.0, int_error_z));
    
    % Position PID control
    u_x = -parameters.Kp_x * e_x - parameters.Ki_x * int_error_x - parameters.Kd_x * e_x_dot;
    u_y = -parameters.Kp_y * e_y - parameters.Ki_y * int_error_y - parameters.Kd_y * e_y_dot;
    u_z = parameters.m * parameters.g - parameters.Kp_z * e_z - parameters.Ki_z * int_error_z - parameters.Kd_z * e_z_dot;
    
    % Desired angles from position control
    phi_des = (1/parameters.g) * (u_x * sin(psi_ref) - u_y * cos(psi_ref));
    theta_des = (1/parameters.g) * (u_x * cos(psi_ref) + u_y * sin(psi_ref));
    
    % Limit desired angles
    phi_des = max(-0.3, min(0.3, phi_des));
    theta_des = max(-0.3, min(0.3, theta_des));
    
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
    
    % Update integral errors for attitude
    int_error_phi = int_error_phi + e_phi * dt;
    int_error_theta = int_error_theta + e_theta * dt;
    int_error_psi = int_error_psi + e_psi * dt;
    
    % Anti-windup for attitude
    int_error_phi = max(-0.5, min(0.5, int_error_phi));
    int_error_theta = max(-0.5, min(0.5, int_error_theta));
    int_error_psi = max(-0.5, min(0.5, int_error_psi));
    
    % Attitude PID control
    u2 = -parameters.Kp_phi * e_phi - parameters.Ki_phi * int_error_phi - parameters.Kd_phi * e_phi_dot;
    u3 = -parameters.Kp_theta * e_theta - parameters.Ki_theta * int_error_theta - parameters.Kd_theta * e_theta_dot;
    u4 = -parameters.Kp_psi * e_psi - parameters.Ki_psi * int_error_psi - parameters.Kd_psi * e_psi_dot;
    
    % Total thrust
    u1 = u_z / (cos(phi) * cos(theta));
    
    % Apply control limits
    u1 = max(parameters.u_min, min(parameters.u_max, u1));
    u2 = max(-5.0, min(5.0, u2));
    u3 = max(-5.0, min(5.0, u3));
    u4 = max(-2.0, min(2.0, u4));
    
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
    figure('Name', '3D Trajectory - PID Controller');
    plot3(x, y, z, 'b-', 'LineWidth', 2);
    hold on;
    plot3(x_ref, y_ref, z_ref, 'r--', 'LineWidth', 1.5);
    plot3(x(1), y(1), z(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot3(x(end), y(end), z(end), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('3D Trajectory: Actual vs Reference');
    legend('Actual', 'Reference', 'Start', 'End', 'Location', 'best');
    grid on;
    view(45, 30);
    
    % Plot 2: Position States vs Time
    figure('Name', 'Position States vs Time - PID Controller');
    subplot(3,1,1);
    plot(tspan, x, 'b-', 'LineWidth', 2);
    hold on;
    plot(tspan, x_ref, 'r--', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('x (m)');
    title('X Position');
    legend('x', 'x_{ref}', 'Location', 'best');
    grid on;
    
    subplot(3,1,2);
    plot(tspan, y, 'b-', 'LineWidth', 2);
    hold on;
    plot(tspan, y_ref, 'r--', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('y (m)');
    title('Y Position');
    legend('y', 'y_{ref}', 'Location', 'best');
    grid on;
    
    subplot(3,1,3);
    plot(tspan, z, 'b-', 'LineWidth', 2);
    hold on;
    plot(tspan, z_ref, 'r--', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('z (m)');
    title('Z Position');
    legend('z', 'z_{ref}', 'Location', 'best');
    grid on;
    
    % Plot 3: Attitude States vs Time
    figure('Name', 'Attitude States vs Time - PID Controller');
    subplot(3,1,1);
    plot(tspan, rad2deg(phi), 'b-', 'LineWidth', 2);
    hold on;
    plot(tspan, rad2deg(phi_ref), 'r--', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('\phi (deg)');
    title('Roll Angle');
    legend('\phi', '\phi_{ref}', 'Location', 'best');
    grid on;
    
    subplot(3,1,2);
    plot(tspan, rad2deg(theta), 'b-', 'LineWidth', 2);
    hold on;
    plot(tspan, rad2deg(theta_ref), 'r--', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('\theta (deg)');
    title('Pitch Angle');
    legend('\theta', '\theta_{ref}', 'Location', 'best');
    grid on;
    
    subplot(3,1,3);
    plot(tspan, rad2deg(psi), 'b-', 'LineWidth', 2);
    hold on;
    plot(tspan, rad2deg(psi_ref), 'r--', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('\psi (deg)');
    title('Yaw Angle');
    legend('\psi', '\psi_{ref}', 'Location', 'best');
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
    figure('Name', 'X-Y Trajectory - PID Controller');
    plot(x, y, 'b-', 'LineWidth', 2);
    hold on;
    plot(x_ref, y_ref, 'r--', 'LineWidth', 1.5);
    plot(x(1), y(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot(x(end), y(end), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    xlabel('X (m)');
    ylabel('Y (m)');
    title('X-Y Plane Trajectory');
    legend('Actual', 'Reference', 'Start', 'End', 'Location', 'best');
    grid on;
    axis equal;
    
    % Calculate and display performance metrics
    fprintf('\n========== PID Controller Performance Metrics ==========\n');
    fprintf('\nTrajectory Parameters:\n');
    fprintf('  Circular trajectory radius: %.2f m\n', 1.0);
    fprintf('  X-Y frequency: %.2f Hz\n', 0.15);
    fprintf('  Z amplitude: %.2f m\n', 0.5);
    fprintf('  Z frequency: %.2f Hz\n', 0.2);
    fprintf('  Z center height: %.2f m\n', 1.5);
    
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
    
    fprintf('\n=======================================================\n')
end 