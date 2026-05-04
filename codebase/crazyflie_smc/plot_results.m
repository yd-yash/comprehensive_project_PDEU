function plot_results(t, X, logs, p)

    lw = 1.5;
    t  = t(:);

    % position states
    figure('Color', 'w', 'Name', 'Position Tracking');

    subplot(3,1,1)
    plot(t, X(:,1), 'b', t, logs.x_ref, 'r--', 'LineWidth', lw)
    mark_fault(p); ylabel('x (m)'); legend('Actual','Reference'); grid on

    subplot(3,1,2)
    plot(t, X(:,2), 'b', t, logs.y_ref, 'r--', 'LineWidth', lw)
    mark_fault(p); ylabel('y (m)'); legend('Actual','Reference'); grid on

    subplot(3,1,3)
    plot(t, X(:,3), 'b', t, logs.z_ref, 'r--', 'LineWidth', lw)
    mark_fault(p); ylabel('z (m)'); xlabel('Time (s)'); legend('Actual','Reference'); grid on

    sgtitle('Position Tracking');

    % attitude states
    figure('Color', 'w', 'Name', 'Attitude States');

    subplot(3,1,1)
    plot(t, rad2deg(X(:,4)), 'b', t, rad2deg(logs.phi_cmd), 'r--', 'LineWidth', lw)
    mark_fault(p); ylabel('\phi (deg)'); legend('Actual','\phi_{cmd}'); grid on

    subplot(3,1,2)
    plot(t, rad2deg(X(:,5)), 'b', t, rad2deg(logs.theta_cmd), 'r--', 'LineWidth', lw)
    mark_fault(p); ylabel('\theta (deg)'); legend('Actual','\theta_{cmd}'); grid on

    subplot(3,1,3)
    plot(t, rad2deg(X(:,6)), 'b', t, rad2deg(logs.psi_ref), 'r--', 'LineWidth', lw)
    mark_fault(p); ylabel('\psi (deg)'); xlabel('Time (s)'); legend('Actual','Reference'); grid on

    sgtitle('Attitude States');

    % position errors
    figure('Color', 'w', 'Name', 'Position Errors');

    subplot(3,1,1)
    plot(t, logs.ex, 'b', 'LineWidth', lw); yline(0,'k--')
    mark_fault(p); ylabel('e_x (m)'); grid on

    subplot(3,1,2)
    plot(t, logs.ey, 'b', 'LineWidth', lw); yline(0,'k--')
    mark_fault(p); ylabel('e_y (m)'); grid on

    subplot(3,1,3)
    plot(t, logs.ez, 'b', 'LineWidth', lw); yline(0,'k--')
    mark_fault(p); ylabel('e_z (m)'); xlabel('Time (s)'); grid on

    sgtitle('Position Errors');

    % actuator effectiveness
    figure('Color', 'w', 'Name', 'Effectiveness Factors');
    channel_labels = {'\alpha_1 (Thrust)', '\alpha_2 (Roll)', ...
                      '\alpha_3 (Pitch)',   '\alpha_4 (Yaw)'};
    for k = 1:4
        subplot(4,1,k)
        plot(t, logs.alpha_log(:,k), 'k', 'LineWidth', lw)
        mark_fault(p); ylabel(channel_labels{k}); ylim([-0.1, 1.2]); grid on
        if k == 4; xlabel('Time (s)'); end
    end
    sgtitle('Actuator Effectiveness Factors \alpha_i');

    % control inputs
    % figure('Color', 'w', 'Name', 'Control Inputs');
    % 
    % U_cmd_logs    = {logs.U1_cmd_log,    logs.U2_cmd_log,    logs.U3_cmd_log,    logs.U4_cmd_log};
    % U_faulty_logs = {logs.U1_faulty_log, logs.U2_faulty_log, logs.U3_faulty_log, logs.U4_faulty_log};
    % U_ylabels     = {'U_1 (N)', 'U_2 (N\cdotm)', 'U_3 (N\cdotm)', 'U_4 (N\cdotm)'};
    % U_titles      = {'Total Thrust', 'Roll Torque', 'Pitch Torque', 'Yaw Torque'};
    % 
    % for k = 1:4
    %     subplot(4,1,k)
    %     plot(t, U_cmd_logs{k},    'b',   'LineWidth', lw); hold on
    %     plot(t, U_faulty_logs{k}, 'r--', 'LineWidth', lw)
    %     mark_fault(p); ylabel(U_ylabels{k}); title(U_titles{k})
    %     legend('Commanded U_{cmd}', 'Delivered U_{faulty}'); grid on
    %     if k == 4; xlabel('Time (s)'); end
    % end
    % sgtitle('Control Inputs: Commanded vs Faulty');

    % rotor speeds
    % figure('Color', 'w', 'Name', 'Rotor Speeds');
    % 
    % omega_labels = {'\Omega_1 (rad/s)', '\Omega_2 (rad/s)', ...
    %                 '\Omega_3 (rad/s)', '\Omega_4 (rad/s)'};
    % for k = 1:4
    %     subplot(4,1,k)
    %     plot(t, logs.Omega_log(:,k), 'LineWidth', lw)
    %     mark_fault(p); ylabel(omega_labels{k}); grid on
    %     if k == 4; xlabel('Time (s)'); end
    % end
    % sgtitle('Individual Rotor Speeds');

    % rotor thrusts
    figure('Color', 'w', 'Name', 'Rotor Thrusts');

    colors   = {'b','r','m','g'};
    T_labels = {'T_1 (N)','T_2 (N)','T_3 (N)','T_4 (N)'};
    for k = 1:4
        subplot(5,1,k)
        plot(t, logs.T_log(:,k), colors{k}, 'LineWidth', lw)
        mark_fault(p); ylabel(T_labels{k}); grid on
    end
    subplot(5,1,5)
    plot(t, logs.T_total_log, 'k', 'LineWidth', lw)
    mark_fault(p); ylabel('T_{total} (N)'); xlabel('Time (s)'); grid on
    sgtitle('Individual Rotor Thrusts and Total Thrust');

    % 3D trajectory
    figure('Color', 'w', 'Name', '3D Trajectory');
    plot3(X(:,1), X(:,2), X(:,3), 'b', 'LineWidth', lw); hold on
    plot3(logs.x_ref, logs.y_ref, logs.z_ref, 'r--', 'LineWidth', lw)
    plot3(X(1,1),   X(1,2),   X(1,3),   'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g')
    plot3(X(end,1), X(end,2), X(end,3), 'rs', 'MarkerSize', 8, 'MarkerFaceColor', 'r')
    grid on; xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)')
    legend('Actual','Reference','Start','End'); title('3D Trajectory'); view(45,30)

    % sliding variable evolution
    figure('Color', 'w', 'Name', 'Sliding Variable Evolution');

    subplot(3,2,1)
    plot(t, logs.s_z, 'b', 'LineWidth', lw); yline(0,'k--')
    mark_fault(p); ylabel('s_z'); title('Altitude'); grid on

    subplot(3,2,2)
    plot(t, logs.s_psi, 'b', 'LineWidth', lw); yline(0,'k--')
    mark_fault(p); ylabel('s_\psi'); title('Yaw'); grid on

    subplot(3,2,3)
    plot(t, logs.s_x, 'b', 'LineWidth', lw); yline(0,'k--')
    mark_fault(p); ylabel('s_x'); title('X Position'); grid on

    subplot(3,2,4)
    plot(t, logs.s_y, 'b', 'LineWidth', lw); yline(0,'k--')
    mark_fault(p); ylabel('s_y'); title('Y Position'); grid on

    subplot(3,2,5)
    plot(t, logs.s_phi, 'b', 'LineWidth', lw); yline(0,'k--')
    mark_fault(p); ylabel('s_\phi'); xlabel('Time (s)'); title('Roll'); grid on

    subplot(3,2,6)
    plot(t, logs.s_theta, 'b', 'LineWidth', lw); yline(0,'k--')
    mark_fault(p); ylabel('s_\theta'); xlabel('Time (s)'); title('Pitch'); grid on

    sgtitle('Sliding Variable Evolution');

end


function mark_fault(p)

    if p.fault_enable ~= 1
        return;
    end

    ax = gca;
    hold(ax, 'on');

    if p.fault_mode == 1
        xline(0, 'k--', 'LineWidth', 1.2);

    elseif p.fault_mode == 2
        yl = ylim(ax);

        patch(ax, ...
              [p.fault_start, p.fault_end, p.fault_end, p.fault_start], ...
              [yl(1), yl(1), yl(2), yl(2)], ...
              [1, 0.7, 0.7], ...
              'FaceAlpha', 0.25, ...
              'EdgeColor', 'none', ...
              'HandleVisibility', 'off');

        xline(p.fault_start, 'k--', 'LineWidth', 1.2);
        xline(p.fault_end,   'k:',  'LineWidth', 1.2);

        ylim(ax, yl);
    end

    hold(ax, 'off');

end