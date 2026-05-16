% =========================================================================
% plots_thesis_stsmc.m — Thesis-Quality Simulation Plots (STSMC)
% =========================================================================
% Generates and saves all 7 required figures for the STSMC controller:
%   Fig 1 : 3D Trajectory
%   Fig 2 : Position Tracking   (x, y, z vs time)
%   Fig 3 : Attitude Tracking   (φ, θ, ψ vs time)
%   Fig 4 : Position Errors     (ex, ey, ez vs time)
%   Fig 5 : Attitude Errors     (eφ, eθ, eψ vs time)
%   Fig 6 : Control Inputs      (U1, U2, U3, U4 vs time)
%   Fig 7 : Sliding Variables   (sφ, sθ, sψ vs time)  ← active for STSMC
%
% INPUTS:
%   t    - time vector (Nx1)
%   x    - state matrix (Nx18)
%   logs - extended logs struct from reconstruct_signals_stsmc.m
%   p    - params struct
% =========================================================================

function plots_thesis_stsmc(t, x, logs, p)

    ctrl_name = 'STSMC';
    save_dir  = pwd;
    line_w    = 1.8;
    font_size = 12;
    fig_w     = 800;
    fig_h     = 600;

    colors.blue   = [0.122  0.467  0.706];
    colors.red    = [0.839  0.153  0.157];
    colors.green  = [0.172  0.627  0.172];
    colors.orange = [1.000  0.498  0.055];
    colors.purple = [0.580  0.404  0.741];
    colors.refclr = [0.839  0.153  0.157];

    save_fig = @(fig, fname) exportgraphics(fig, ...
        fullfile(save_dir, fname), 'Resolution', 300);

    % ======================================================================
    % FIGURE 1 — 3D Trajectory
    % ======================================================================
    fig1 = figure('Color', 'w', 'Position', [100 100 fig_w fig_h]);

    plot3(logs.x_ref, logs.y_ref, logs.z_ref, '--', ...
          'Color', colors.refclr, 'LineWidth', line_w, 'DisplayName', 'Reference');
    hold on;
    plot3(x(:,1), x(:,2), x(:,3), '-', ...
          'Color', colors.blue, 'LineWidth', line_w, 'DisplayName', [ctrl_name ' Response']);
    plot3(x(1,1),   x(1,2),   x(1,3),   'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'DisplayName', 'Start');
    plot3(x(end,1), x(end,2), x(end,3), 'ks', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', 'End');

    xlabel('x (m)', 'FontSize', font_size);
    ylabel('y (m)', 'FontSize', font_size);
    zlabel('z (m)', 'FontSize', font_size);
    title([ctrl_name ' — 3D Trajectory'], 'FontSize', font_size+1);
    legend('Location', 'best', 'FontSize', font_size-1);
    grid on; grid minor; view(45, 25);
    set(gca, 'FontSize', font_size);
    hold off;

    save_fig(fig1, [ctrl_name '_Fig1_3D_Trajectory.png']);
    fprintf('[Saved] %s_Fig1_3D_Trajectory.png\n', ctrl_name);

    % ======================================================================
    % FIGURE 2 — Position Tracking
    % ======================================================================
    fig2 = figure('Color', 'w', 'Position', [100 100 fig_w fig_h]);

    subplot(3,1,1)
    plot(t, logs.x_ref, '--', 'Color', colors.refclr, 'LineWidth', line_w, 'DisplayName', 'x_{ref}');
    hold on;
    plot(t, x(:,1), '-', 'Color', colors.blue, 'LineWidth', line_w, 'DisplayName', 'x');
    ylabel('x (m)', 'FontSize', font_size); legend('FontSize', font_size-1); grid on;
    set(gca, 'FontSize', font_size); hold off;

    subplot(3,1,2)
    plot(t, logs.y_ref, '--', 'Color', colors.refclr, 'LineWidth', line_w, 'DisplayName', 'y_{ref}');
    hold on;
    plot(t, x(:,2), '-', 'Color', colors.blue, 'LineWidth', line_w, 'DisplayName', 'y');
    ylabel('y (m)', 'FontSize', font_size); legend('FontSize', font_size-1); grid on;
    set(gca, 'FontSize', font_size); hold off;

    subplot(3,1,3)
    plot(t, logs.z_ref, '--', 'Color', colors.refclr, 'LineWidth', line_w, 'DisplayName', 'z_{ref}');
    hold on;
    plot(t, x(:,3), '-', 'Color', colors.blue, 'LineWidth', line_w, 'DisplayName', 'z');
    ylabel('z (m)', 'FontSize', font_size);
    xlabel('Time (s)', 'FontSize', font_size);
    legend('FontSize', font_size-1); grid on;
    set(gca, 'FontSize', font_size); hold off;

    sgtitle([ctrl_name ' — Position Tracking'], 'FontSize', font_size+1);
    save_fig(fig2, [ctrl_name '_Fig2_Position_Tracking.png']);
    fprintf('[Saved] %s_Fig2_Position_Tracking.png\n', ctrl_name);

    % ======================================================================
    % FIGURE 3 — Attitude Tracking
    % ======================================================================
    fig3 = figure('Color', 'w', 'Position', [100 100 fig_w fig_h]);

    subplot(3,1,1)
    plot(t, rad2deg(logs.phi_ref),  '--', 'Color', colors.refclr, 'LineWidth', line_w, 'DisplayName', '\phi_{ref}');
    hold on;
    plot(t, rad2deg(x(:,4)), '-', 'Color', colors.blue, 'LineWidth', line_w, 'DisplayName', '\phi');
    ylabel('\phi (deg)', 'FontSize', font_size); legend('FontSize', font_size-1); grid on;
    set(gca, 'FontSize', font_size); hold off;

    subplot(3,1,2)
    plot(t, rad2deg(logs.theta_ref), '--', 'Color', colors.refclr, 'LineWidth', line_w, 'DisplayName', '\theta_{ref}');
    hold on;
    plot(t, rad2deg(x(:,5)), '-', 'Color', colors.blue, 'LineWidth', line_w, 'DisplayName', '\theta');
    ylabel('\theta (deg)', 'FontSize', font_size); legend('FontSize', font_size-1); grid on;
    set(gca, 'FontSize', font_size); hold off;

    subplot(3,1,3)
    plot(t, rad2deg(logs.psi_ref), '--', 'Color', colors.refclr, 'LineWidth', line_w, 'DisplayName', '\psi_{ref}');
    hold on;
    plot(t, rad2deg(x(:,6)), '-', 'Color', colors.blue, 'LineWidth', line_w, 'DisplayName', '\psi');
    ylabel('\psi (deg)', 'FontSize', font_size);
    xlabel('Time (s)', 'FontSize', font_size);
    legend('FontSize', font_size-1); grid on;
    set(gca, 'FontSize', font_size); hold off;

    sgtitle([ctrl_name ' — Attitude Tracking'], 'FontSize', font_size+1);
    save_fig(fig3, [ctrl_name '_Fig3_Attitude_Tracking.png']);
    fprintf('[Saved] %s_Fig3_Attitude_Tracking.png\n', ctrl_name);

    % ======================================================================
    % FIGURE 4 — Position Errors
    % ======================================================================
    fig4 = figure('Color', 'w', 'Position', [100 100 fig_w fig_h]);

    subplot(3,1,1)
    plot(t, logs.ex, '-', 'Color', colors.blue, 'LineWidth', line_w);
    yline(0, 'k--', 'LineWidth', 0.8);
    ylabel('e_x (m)', 'FontSize', font_size); grid on; set(gca, 'FontSize', font_size);

    subplot(3,1,2)
    plot(t, logs.ey, '-', 'Color', colors.green, 'LineWidth', line_w);
    yline(0, 'k--', 'LineWidth', 0.8);
    ylabel('e_y (m)', 'FontSize', font_size); grid on; set(gca, 'FontSize', font_size);

    subplot(3,1,3)
    plot(t, logs.ez, '-', 'Color', colors.orange, 'LineWidth', line_w);
    yline(0, 'k--', 'LineWidth', 0.8);
    ylabel('e_z (m)', 'FontSize', font_size);
    xlabel('Time (s)', 'FontSize', font_size);
    grid on; set(gca, 'FontSize', font_size);

    sgtitle([ctrl_name ' — Position Errors'], 'FontSize', font_size+1);
    save_fig(fig4, [ctrl_name '_Fig4_Position_Errors.png']);
    fprintf('[Saved] %s_Fig4_Position_Errors.png\n', ctrl_name);

    % ======================================================================
    % FIGURE 5 — Attitude Errors
    % ======================================================================
    fig5 = figure('Color', 'w', 'Position', [100 100 fig_w fig_h]);

    subplot(3,1,1)
    plot(t, rad2deg(logs.ephi), '-', 'Color', colors.blue, 'LineWidth', line_w);
    yline(0, 'k--', 'LineWidth', 0.8);
    ylabel('e_\phi (deg)', 'FontSize', font_size); grid on; set(gca, 'FontSize', font_size);

    subplot(3,1,2)
    plot(t, rad2deg(logs.etheta), '-', 'Color', colors.green, 'LineWidth', line_w);
    yline(0, 'k--', 'LineWidth', 0.8);
    ylabel('e_\theta (deg)', 'FontSize', font_size); grid on; set(gca, 'FontSize', font_size);

    subplot(3,1,3)
    plot(t, rad2deg(logs.epsi), '-', 'Color', colors.orange, 'LineWidth', line_w);
    yline(0, 'k--', 'LineWidth', 0.8);
    ylabel('e_\psi (deg)', 'FontSize', font_size);
    xlabel('Time (s)', 'FontSize', font_size);
    grid on; set(gca, 'FontSize', font_size);

    sgtitle([ctrl_name ' — Attitude Errors'], 'FontSize', font_size+1);
    save_fig(fig5, [ctrl_name '_Fig5_Attitude_Errors.png']);
    fprintf('[Saved] %s_Fig5_Attitude_Errors.png\n', ctrl_name);

    % ======================================================================
    % FIGURE 6 — Control Inputs
    % ======================================================================
    fig6 = figure('Color', 'w', 'Position', [100 100 fig_w fig_h]);

    subplot(4,1,1)
    plot(t, logs.u1_log, '-', 'Color', colors.blue, 'LineWidth', line_w);
    ylabel('U_1 (N)', 'FontSize', font_size); title('Total Thrust', 'FontSize', font_size);
    grid on; set(gca, 'FontSize', font_size);

    subplot(4,1,2)
    plot(t, logs.u2_log, '-', 'Color', colors.green, 'LineWidth', line_w);
    ylabel('U_2 (Nm)', 'FontSize', font_size); title('Roll Torque', 'FontSize', font_size);
    grid on; set(gca, 'FontSize', font_size);

    subplot(4,1,3)
    plot(t, logs.u3_log, '-', 'Color', colors.orange, 'LineWidth', line_w);
    ylabel('U_3 (Nm)', 'FontSize', font_size); title('Pitch Torque', 'FontSize', font_size);
    grid on; set(gca, 'FontSize', font_size);

    subplot(4,1,4)
    plot(t, logs.u4_log, '-', 'Color', colors.purple, 'LineWidth', line_w);
    ylabel('U_4 (Nm)', 'FontSize', font_size);
    xlabel('Time (s)', 'FontSize', font_size);
    title('Yaw Torque', 'FontSize', font_size);
    grid on; set(gca, 'FontSize', font_size);

    sgtitle([ctrl_name ' — Control Inputs'], 'FontSize', font_size+1);
    save_fig(fig6, [ctrl_name '_Fig6_Control_Inputs.png']);
    fprintf('[Saved] %s_Fig6_Control_Inputs.png\n', ctrl_name);

    % ======================================================================
    % FIGURE 7 — Sliding Variables (sφ, sθ, sψ vs time)
    % ======================================================================
    fig7 = figure('Color', 'w', 'Position', [100 100 fig_w fig_h]);

    subplot(3,1,1)
    plot(t, logs.s_phi_log, '-', 'Color', colors.blue, 'LineWidth', line_w);
    yline(0, 'k--', 'LineWidth', 0.8);
    ylabel('s_\phi', 'FontSize', font_size); grid on; set(gca, 'FontSize', font_size);

    subplot(3,1,2)
    plot(t, logs.s_theta_log, '-', 'Color', colors.green, 'LineWidth', line_w);
    yline(0, 'k--', 'LineWidth', 0.8);
    ylabel('s_\theta', 'FontSize', font_size); grid on; set(gca, 'FontSize', font_size);

    subplot(3,1,3)
    plot(t, logs.s_psi_log, '-', 'Color', colors.orange, 'LineWidth', line_w);
    yline(0, 'k--', 'LineWidth', 0.8);
    ylabel('s_\psi', 'FontSize', font_size);
    xlabel('Time (s)', 'FontSize', font_size);
    grid on; set(gca, 'FontSize', font_size);

    sgtitle([ctrl_name ' — Sliding Variables (Attitude Channels)'], 'FontSize', font_size+1);
    save_fig(fig7, [ctrl_name '_Fig7_Sliding_Variables.png']);
    fprintf('[Saved] %s_Fig7_Sliding_Variables.png\n', ctrl_name);

    fprintf('\n[plots_thesis_stsmc] All 7 figures saved to: %s\n', save_dir);

end