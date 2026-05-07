% All simulation plots — called once from main.m
% INPUTS:
%   t    - time vector
%   x    - state matrix (Nx18)
%   logs - reconstructed signals struct (from reconstruct_signals.m)
%   p    - params struct

function plot_results(t, x, logs, p)

    fault_line = p.fault_time;

    % 1. Actuator Effectiveness
    figure('Color', 'w');
    plot(t, logs.alpha_log, 'LineWidth', 2);
    xline(fault_line, 'r--', 'Fault');
    ylabel('\alpha_{roll}');
    xlabel('Time (s)');
    title('Actuator effectiveness');
    grid on;

    % 2. Position Tracking
    figure('Color', 'w');
    subplot(3, 1, 1)
    plot(t, x(:, 1), 'b', t, logs.x_ref, 'r--', 'LineWidth', 1.5)
    xline(fault_line, 'k--')
    ylabel('x (m)')
    legend('x', 'x_{ref}')
    grid on

    subplot(3, 1, 2)
    plot(t, x(:, 2), 'b', t, logs.y_ref, 'r--', 'LineWidth', 1.5)
    xline(fault_line, 'k--')
    ylabel('y (m)')
    legend('y', 'y_{ref}')
    grid on

    subplot(3, 1, 3)
    plot(t, x(:, 3), 'b', t, logs.z_ref, 'r--', 'LineWidth', 1.5)
    xline(fault_line, 'k--')
    ylabel('z (m)')
    xlabel('Time (s)')
    legend('z', 'z_{ref}')
    grid on
    sgtitle('Position states');

    % 3. Position Errors
    % figure('Color', 'w');
    % subplot(3, 1, 1)
    % plot(t, logs.ex, 'LineWidth', 1.5)
    % xline(fault_line, 'k--')
    % ylabel('e_x (m)')
    % grid on
    % 
    % subplot(3, 1, 2)
    % plot(t, logs.ey, 'LineWidth', 1.5)
    % xline(fault_line, 'k--')
    % ylabel('e_y (m)')
    % grid on
    % 
    % subplot(3, 1, 3)
    % plot(t, logs.ez, 'LineWidth', 1.5)
    % xline(fault_line, 'k--')
    % ylabel('e_z (m)')
    % xlabel('Time (s)')
    % grid on
    % sgtitle('Position errors');

    % 4. Attitude States
    figure('Color', 'w');
    subplot(3, 1, 1)
    plot(t, x(:, 4)*180/pi, 'LineWidth', 1.5)
    xline(fault_line, 'k--')
    ylabel('\phi (deg)')
    grid on

    subplot(3, 1, 2)
    plot(t, x(:, 5)*180/pi, 'LineWidth', 1.5)
    xline(fault_line, 'k--')
    ylabel('\theta (deg)')
    grid on

    subplot(3, 1, 3)
    plot(t, x(:, 6)*180/pi, 'LineWidth', 1.5)
    xline(fault_line, 'k--')
    ylabel('\psi (deg)')
    xlabel('Time (s)')
    grid on
    sgtitle('Attitude states');

    % 5. Attitude Errors
    % figure('Color', 'w');
    % subplot(3, 1, 1)
    % plot(t, logs.ephi*180/pi, 'LineWidth', 1.5)
    % xline(fault_line, 'k--')
    % ylabel('e_\phi (deg)')
    % grid on
    % 
    % subplot(3, 1, 2)
    % plot(t, logs.etheta*180/pi, 'LineWidth', 1.5)
    % xline(fault_line, 'k--')
    % ylabel('e_\theta (deg)')
    % grid on
    % 
    % subplot(3, 1, 3)
    % plot(t, logs.epsi*180/pi, 'LineWidth', 1.5)
    % xline(fault_line, 'k--')
    % ylabel('e_\psi (deg)')
    % xlabel('Time (s)')
    % grid on
    % sgtitle('Attitude errors');

    % 6. Total Thrust
    figure('Color', 'w');
    plot(t, logs.total_thrust_log, 'LineWidth', 1.5)
    xline(fault_line, 'k--')
    ylabel('Total Thrust (N)')
    xlabel('Time (s)')
    title('Total Thrust')
    grid on

    % 7. Torques
    figure('Color', 'w');
    subplot(3, 1, 1)
    plot(t, logs.tau_phi_log, 'LineWidth', 1.5)
    xline(fault_line, 'k--')
    ylabel('\tau_\phi (Nm)')
    title('Roll Torque')
    grid on

    subplot(3, 1, 2)
    plot(t, logs.tau_theta_log, 'LineWidth', 1.5)
    xline(fault_line, 'k--')
    ylabel('\tau_\theta (Nm)')
    title('Pitch Torque')
    grid on

    subplot(3, 1, 3)
    plot(t, logs.tau_psi_log, 'LineWidth', 1.5)
    xline(fault_line, 'k--')
    ylabel('\tau_\psi (Nm)')
    xlabel('Time (s)')
    title('Yaw Torque')
    grid on

    % 8. Individual Rotor Thrusts
    figure('Color', 'w');
    subplot(4, 1, 1)
    plot(t, logs.T1_log, 'LineWidth', 1.5)
    xline(fault_line, 'k--')
    ylabel('T1 (N)')
    grid on

    subplot(4, 1, 2)
    plot(t, logs.T2_log, 'LineWidth', 1.5)
    xline(fault_line, 'k--')
    ylabel('T2 (N)')
    grid on

    subplot(4, 1, 3)
    plot(t, logs.T3_log, 'LineWidth', 1.5)
    xline(fault_line, 'k--')
    ylabel('T3 (N)')
    grid on

    subplot(4, 1, 4)
    plot(t, logs.T4_log, 'LineWidth', 1.5)
    xline(fault_line, 'k--')
    ylabel('T4 (N)')
    xlabel('Time (s)')
    grid on
    sgtitle('Individual Rotor Thrusts');

end