% =========================================================================
% pm_thesis_stsmc.m — Thesis Performance Metrics (STSMC Controller)
% =========================================================================
% Same 13 metrics as pm_thesis.m but with sliding surface reach time active.
%
% INPUTS:
%   t        - time vector (Nx1)
%   ex/ey/ez - position errors (Nx1)
%   ephi/etheta/epsi - attitude errors (Nx1) [rad]
%   logs     - extended logs struct (must contain s_phi_log, s_theta_log,
%              s_psi_log from reconstruct_signals_stsmc.m)
% =========================================================================

function pm_thesis_stsmc(t, ex, ey, ez, ephi, etheta, epsi, logs)

    ctrl_name = 'STSMC';

    % ---- Settling time thresholds (2% band) ------------------------------
    thr_pos = 0.02 * 0.5;          % 2% of A=0.5 m  =>  0.01 m
    thr_att = deg2rad(1) * 0.02;   % 2% of 1 deg band

    % ---- Helper: settling time -------------------------------------------
    function ts = settling_time(t_vec, err_vec, threshold)
        idx = find(abs(err_vec) > threshold, 1, 'last');
        if isempty(idx)
            ts = 0;
        elseif idx == length(t_vec)
            ts = NaN;
        else
            ts = t_vec(idx + 1);
        end
    end

    % ---- Helper: sliding surface reach time ------------------------------
    % Defined as the first time |s| < s_threshold and stays there
    % (uses 5% of initial |s| as the "reached" band)
    function tr = reach_time(t_vec, s_vec, frac)
        if nargin < 3; frac = 0.05; end
        s0  = abs(s_vec(1));
        if s0 < 1e-10; tr = 0; return; end
        thr = frac * s0;
        idx = find(abs(s_vec) < thr, 1, 'first');
        if isempty(idx)
            tr = NaN;
        else
            tr = t_vec(idx);
        end
    end

    % ======================================================================
    % 1. POSITION RMSE
    % ======================================================================
    RMSE_x     = sqrt(mean(ex.^2));
    RMSE_y     = sqrt(mean(ey.^2));
    RMSE_z     = sqrt(mean(ez.^2));
    RMSE_phi   = sqrt(mean(ephi.^2));
    RMSE_theta = sqrt(mean(etheta.^2));
    RMSE_psi   = sqrt(mean(epsi.^2));

    % ======================================================================
    % 2. STEADY-STATE ERROR (last 10%)
    % ======================================================================
    ss_idx = round(0.90 * length(t)) : length(t);

    ess_x     = mean(ex(ss_idx));
    ess_y     = mean(ey(ss_idx));
    ess_z     = mean(ez(ss_idx));
    ess_phi   = mean(ephi(ss_idx));
    ess_theta = mean(etheta(ss_idx));
    ess_psi   = mean(epsi(ss_idx));

    % ======================================================================
    % 3. SETTLING TIME — Position
    % ======================================================================
    ts_x = settling_time(t, ex, thr_pos);
    ts_y = settling_time(t, ey, thr_pos);
    ts_z = settling_time(t, ez, thr_pos);

    % ======================================================================
    % 4. SETTLING TIME — Attitude
    % ======================================================================
    ts_phi   = settling_time(t, ephi,   thr_att);
    ts_theta = settling_time(t, etheta, thr_att);
    ts_psi   = settling_time(t, epsi,   thr_att);

    % ======================================================================
    % 5. SLIDING SURFACE REACH TIME
    %    Report the worst-case (latest) reach time across φ, θ, ψ channels
    % ======================================================================
    tr_phi   = reach_time(t, logs.s_phi_log);
    tr_theta = reach_time(t, logs.s_theta_log);
    tr_psi   = reach_time(t, logs.s_psi_log);

    % Use the attitude channels (φ,θ,ψ) — these are the SMC-controlled axes
    tr_vals = [tr_phi, tr_theta, tr_psi];
    tr_vals = tr_vals(~isnan(tr_vals));
    if isempty(tr_vals)
        ts_slide = NaN;
    else
        ts_slide = max(tr_vals);   % conservative: latest of the three
    end

    % ======================================================================
    % Helper: format settling/reach time
    % ======================================================================
    function s = fmt_ts(ts)
        if isnan(ts)
            s = '  N/A (not settled)';
        else
            s = sprintf('  %.4f s', ts);
        end
    end

    % ======================================================================
    % PRINT
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
    fprintf('    t_reach phi   :%s\n', fmt_ts(tr_phi));
    fprintf('    t_reach theta :%s\n', fmt_ts(tr_theta));
    fprintf('    t_reach psi   :%s\n', fmt_ts(tr_psi));
    fprintf('    t_reach (max) :%s\n', fmt_ts(ts_slide));

    fprintf('\n%s\n\n', sep);

    % ======================================================================
    % SUMMARY TABLE
    % ======================================================================
    sep2 = repmat('=', 1, 90);
    lin2 = repmat('-', 1, 90);

    fprintf('  SUMMARY TABLE (%s)\n', ctrl_name);
    fprintf('%s\n', sep2);
    fprintf('  %-30s  %-10s %-10s %-10s %-10s %-10s %-10s\n', ...
            'Metric', 'x', 'y', 'z', 'phi', 'theta', 'psi');
    fprintf('%s\n', lin2);

    fprintf('  %-30s  %-10.6f %-10.6f %-10.6f %-10.6f %-10.6f %-10.6f\n', ...
            'RMSE (m / rad)', ...
            RMSE_x, RMSE_y, RMSE_z, RMSE_phi, RMSE_theta, RMSE_psi);
    fprintf('%s\n', lin2);

    fprintf('  %-30s  %-10.6f %-10.6f %-10.6f %-10.6f %-10.6f %-10.6f\n', ...
            'Steady-State Err (m / rad)', ...
            ess_x, ess_y, ess_z, ess_phi, ess_theta, ess_psi);
    fprintf('%s\n', lin2);

    fprintf('  %-30s  %-10s %-10s %-10s %-10s %-10s %-10s\n', ...
            'Settling Time (s)', ...
            strtrim(fmt_ts(ts_x)),     strtrim(fmt_ts(ts_y)),     strtrim(fmt_ts(ts_z)), ...
            strtrim(fmt_ts(ts_phi)),   strtrim(fmt_ts(ts_theta)), strtrim(fmt_ts(ts_psi)));
    fprintf('%s\n', lin2);

    fprintf('  %-30s  phi: %s   theta: %s   psi: %s\n', ...
            'Sliding Reach Time (s)', ...
            strtrim(fmt_ts(tr_phi)), strtrim(fmt_ts(tr_theta)), strtrim(fmt_ts(tr_psi)));
    fprintf('%s\n\n', sep2);

end