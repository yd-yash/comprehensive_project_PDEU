% =========================================================================
% performance_metrics.m — Thesis Performance Metrics (PID Controller)
% =========================================================================
% Computes and prints all 13 required metrics per controller:
%
%   POSITION RMSE           : x, y, z                        (3 metrics)
%   STEADY-STATE ERROR      : x, y, z                        (3 metrics)
%   SETTLING TIME (position): x, y, z                        (3 metrics)
%   SETTLING TIME (attitude): phi, theta, psi                 (3 metrics)
%   SLIDING SURFACE REACH   : N/A for PID                    (1 metric)
%
% INPUTS:
%   t      - time vector (Nx1)
%   ex     - position error x   (Nx1)
%   ey     - position error y   (Nx1)
%   ez     - position error z   (Nx1)
%   ephi   - attitude error phi   (Nx1) [rad]
%   etheta - attitude error theta (Nx1) [rad]
%   epsi   - attitude error psi   (Nx1) [rad]
%
% NOTE: Settling time is defined as the last time the error exceeds
%       the 2% threshold of the initial reference amplitude.
%       Threshold is set per axis — adjust below if needed.
% =========================================================================

function performance_metrics(t, ex, ey, ez, ephi, etheta, epsi)

    ctrl_name = 'PID';

    % ---- Settling time thresholds (2% band) ------------------------------
    % Position: 2% of reference amplitude (circle r=0.5 m)
    thr_pos = 0.02 * 0.5;   % = 0.01 m  — adjust if trajectory changes

    % Attitude: 2% of typical max excursion (degrees → radians)
    % Max phi/theta excursion for circle ~0.26 deg; use 1 deg as practical band
    thr_att = deg2rad(1) * 0.02;   % = 0.000349 rad (~0.02% of 1 deg)
    % NOTE: If attitude errors are very small you may want thr_att = 1e-4 rad

    % ---- Helper: settling time -------------------------------------------
    % Returns the last time |error| > threshold, then +1 sample step.
    % Returns NaN if error never exceeds threshold (already settled).
    function ts = settling_time(t_vec, err_vec, threshold)
        idx = find(abs(err_vec) > threshold, 1, 'last');
        if isempty(idx)
            ts = 0;   % settled from the start
        elseif idx == length(t_vec)
            ts = NaN; % never settled within simulation window
        else
            ts = t_vec(idx + 1);
        end
    end

    % ======================================================================
    % 1. POSITION RMSE
    % ======================================================================
    RMSE_x = sqrt(mean(ex.^2));
    RMSE_y = sqrt(mean(ey.^2));
    RMSE_z = sqrt(mean(ez.^2));

    % ======================================================================
    % 2. STEADY-STATE ERROR (last 10% of simulation)
    % ======================================================================
    ss_idx  = round(0.90 * length(t)) : length(t);   % last 10% window

    ess_x     = mean(ex(ss_idx));
    ess_y     = mean(ey(ss_idx));
    ess_z     = mean(ez(ss_idx));
    ess_phi   = mean(ephi(ss_idx));
    ess_theta = mean(etheta(ss_idx));
    ess_psi   = mean(epsi(ss_idx));

    % ======================================================================
    % 3. SETTLING TIME — Position
    % ======================================================================
    ts_x = settling_time(t, ex,     thr_pos);
    ts_y = settling_time(t, ey,     thr_pos);
    ts_z = settling_time(t, ez,     thr_pos);

    % ======================================================================
    % 4. SETTLING TIME — Attitude
    % ======================================================================
    ts_phi   = settling_time(t, ephi,   thr_att);
    ts_theta = settling_time(t, etheta, thr_att);
    ts_psi   = settling_time(t, epsi,   thr_att);

    % ======================================================================
    % 5. SLIDING SURFACE REACH TIME — N/A for PID
    % ======================================================================
    ts_slide = NaN;   % not applicable

    % ======================================================================
    % Helper: format settling time for display
    % ======================================================================
    function s = fmt_ts(ts)
        if isnan(ts)
            s = '  N/A (not settled)';
        else
            s = sprintf('  %.4f s', ts);
        end
    end

    % ======================================================================
    % PRINT TO COMMAND WINDOW
    % ======================================================================
    sep = repmat('=', 1, 60);
    lin = repmat('-', 1, 60);

    fprintf('\n%s\n', sep);
    fprintf('   TRACKING PERFORMANCE METRICS — %s Controller\n', ctrl_name);
    fprintf('%s\n', sep);

    fprintf('\n  [1] POSITION RMSE\n');
    fprintf('%s\n', lin);
    fprintf('    RMSE x   :  %.6f  m\n', RMSE_x);
    fprintf('    RMSE y   :  %.6f  m\n', RMSE_y);
    fprintf('    RMSE z   :  %.6f  m\n', RMSE_z);

    fprintf('\n  [2] STEADY-STATE ERROR  (mean over last 10%% of simulation)\n');
    fprintf('%s\n', lin);
    fprintf('    e_ss x     :  %.6f  m\n',   ess_x);
    fprintf('    e_ss y     :  %.6f  m\n',   ess_y);
    fprintf('    e_ss z     :  %.6f  m\n',   ess_z);
    fprintf('    e_ss phi   :  %.6f  rad   (%.4f  deg)\n', ess_phi,   rad2deg(ess_phi));
    fprintf('    e_ss theta :  %.6f  rad   (%.4f  deg)\n', ess_theta, rad2deg(ess_theta));
    fprintf('    e_ss psi   :  %.6f  rad   (%.4f  deg)\n', ess_psi,   rad2deg(ess_psi));

    fprintf('\n  [3] SETTLING TIME — Position  (2%% band, thr = %.4f m)\n', thr_pos);
    fprintf('%s\n', lin);
    fprintf('    t_s x   :%s\n', fmt_ts(ts_x));
    fprintf('    t_s y   :%s\n', fmt_ts(ts_y));
    fprintf('    t_s z   :%s\n', fmt_ts(ts_z));

    fprintf('\n  [4] SETTLING TIME — Attitude  (thr = %.6f rad)\n', thr_att);
    fprintf('%s\n', lin);
    fprintf('    t_s phi   :%s\n', fmt_ts(ts_phi));
    fprintf('    t_s theta :%s\n', fmt_ts(ts_theta));
    fprintf('    t_s psi   :%s\n', fmt_ts(ts_psi));

    fprintf('\n  [5] SLIDING SURFACE REACH TIME\n');
    fprintf('%s\n', lin);
    fprintf('    t_reach   :  N/A  (not applicable for %s)\n', ctrl_name);

    fprintf('\n%s\n\n', sep);

    % ======================================================================
    % SUMMARY TABLE (copy-paste friendly for thesis)
    % All 6 states as separate columns: x | y | z | phi | theta | psi
    % ======================================================================
    sep2 = repmat('=', 1, 90);
    lin2 = repmat('-', 1, 90);

    fprintf('  SUMMARY TABLE (%s)\n', ctrl_name);
    fprintf('%s\n', sep2);
    fprintf('  %-30s  %-10s %-10s %-10s %-10s %-10s %-10s\n', ...
            'Metric', 'x', 'y', 'z', 'phi', 'theta', 'psi');
    fprintf('%s\n', lin2);

    % RMSE — position only (attitude RMSE rows added separately)
    RMSE_phi   = sqrt(mean(ephi.^2));
    RMSE_theta = sqrt(mean(etheta.^2));
    RMSE_psi   = sqrt(mean(epsi.^2));
    fprintf('  %-30s  %-10.6f %-10.6f %-10.6f %-10.6f %-10.6f %-10.6f\n', ...
            'RMSE (m / rad)', ...
            RMSE_x, RMSE_y, RMSE_z, RMSE_phi, RMSE_theta, RMSE_psi);

    fprintf('%s\n', lin2);

    % Steady-state error
    fprintf('  %-30s  %-10.6f %-10.6f %-10.6f %-10.6f %-10.6f %-10.6f\n', ...
            'Steady-State Err (m / rad)', ...
            ess_x, ess_y, ess_z, ess_phi, ess_theta, ess_psi);

    fprintf('%s\n', lin2);

    % Settling time — position
    fprintf('  %-30s  %-10s %-10s %-10s %-10s %-10s %-10s\n', ...
            'Settling Time (s)', ...
            strtrim(fmt_ts(ts_x)),     strtrim(fmt_ts(ts_y)),     strtrim(fmt_ts(ts_z)), ...
            strtrim(fmt_ts(ts_phi)),   strtrim(fmt_ts(ts_theta)), strtrim(fmt_ts(ts_psi)));

    fprintf('%s\n', lin2);

    % Sliding surface reach time
    fprintf('  %-30s  %-s\n', 'Sliding Reach Time (s)', 'N/A (not applicable for PID)');

    fprintf('%s\n\n', sep2);

end