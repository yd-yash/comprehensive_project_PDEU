% =========================================================================
% pm_thesis_stsmc_v3.m — Thesis Performance Metrics (STSMC, v3)
% =========================================================================
% 13 metrics matching PID/SMC thesis tables:
%   [1] RMSE             : x, y, z               (3)
%   [2] Steady-state err : x, y, z               (3)
%   [3] Settling time pos: x, y, z               (3)
%   [4] Settling time att: φ, θ, ψ               (3)
%   [5] Sliding reach    : reported (not N/A)    (1)
%
% INPUTS:
%   t_vec              - time vector (Nx1)
%   ex,ey,ez           - position errors (Nx1)  [m]
%   ephi,etheta,epsi   - attitude errors (Nx1)  [rad]
%   s_phi,s_theta,s_psi - sliding surfaces (Nx1)
% =========================================================================

function pm_thesis_stsmc_v3(t_vec, ex, ey, ez, ephi, etheta, epsi, ...
                              s_phi, s_theta, s_psi)

    ctrl_name = 'STSMC';

    % ---- Thresholds (2% band) -------------------------------------------
    thr_pos = 0.02 * 0.5;           % 2% of A=0.5 m  = 0.01 m
    thr_att = deg2rad(1) * 0.02;    % 2% of 1 deg band

    % ---- Nested helpers -------------------------------------------------
    function ts = settling_time(t_v, err_v, thr)
        idx = find(abs(err_v) > thr, 1, 'last');
        if isempty(idx);          ts = 0;
        elseif idx == length(t_v); ts = NaN;
        else;                      ts = t_v(idx+1);
        end
    end

    function tr = reach_time(t_v, s_v)
        % First time |s| drops below 5% of its initial value
        s0 = abs(s_v(1));
        if s0 < 1e-10; tr = 0; return; end
        idx = find(abs(s_v) < 0.05*s0, 1, 'first');
        if isempty(idx); tr = NaN; else; tr = t_v(idx); end
    end

    function s = fmt(v)
        if isnan(v); s = 'N/A'; else; s = sprintf('%.4f s', v); end
    end

    % =====================================================================
    % COMPUTE
    % =====================================================================
    RMSE_x     = sqrt(mean(ex.^2));
    RMSE_y     = sqrt(mean(ey.^2));
    RMSE_z     = sqrt(mean(ez.^2));
    RMSE_phi   = sqrt(mean(ephi.^2));
    RMSE_theta = sqrt(mean(etheta.^2));
    RMSE_psi   = sqrt(mean(epsi.^2));

    ss = round(0.90*length(t_vec)) : length(t_vec);
    ess_x     = mean(ex(ss));     ess_y   = mean(ey(ss));
    ess_z     = mean(ez(ss));     ess_phi = mean(ephi(ss));
    ess_theta = mean(etheta(ss)); ess_psi = mean(epsi(ss));

    ts_x     = settling_time(t_vec, ex,     thr_pos);
    ts_y     = settling_time(t_vec, ey,     thr_pos);
    ts_z     = settling_time(t_vec, ez,     thr_pos);
    ts_phi   = settling_time(t_vec, ephi,   thr_att);
    ts_theta = settling_time(t_vec, etheta, thr_att);
    ts_psi   = settling_time(t_vec, epsi,   thr_att);

    tr_phi   = reach_time(t_vec, s_phi);
    tr_theta = reach_time(t_vec, s_theta);
    tr_psi   = reach_time(t_vec, s_psi);
    tr_vals  = [tr_phi, tr_theta, tr_psi];
    tr_max   = max(tr_vals(~isnan(tr_vals)));
    if isempty(tr_max); tr_max = NaN; end

    % =====================================================================
    % PRINT
    % =====================================================================
    sep = repmat('=',1,62);
    lin = repmat('-',1,62);

    fprintf('\n%s\n', sep);
    fprintf('  TRACKING PERFORMANCE METRICS — %s\n', ctrl_name);
    fprintf('%s\n', sep);

    fprintf('\n  [1] POSITION RMSE\n%s\n', lin);
    fprintf('    RMSE x : %.6f m\n', RMSE_x);
    fprintf('    RMSE y : %.6f m\n', RMSE_y);
    fprintf('    RMSE z : %.6f m\n', RMSE_z);

    fprintf('\n  [2] STEADY-STATE ERROR (last 10%%)\n%s\n', lin);
    fprintf('    e_ss x     : %.6f m\n',   ess_x);
    fprintf('    e_ss y     : %.6f m\n',   ess_y);
    fprintf('    e_ss z     : %.6f m\n',   ess_z);
    fprintf('    e_ss phi   : %.6f rad  (%.4f deg)\n', ess_phi,   rad2deg(ess_phi));
    fprintf('    e_ss theta : %.6f rad  (%.4f deg)\n', ess_theta, rad2deg(ess_theta));
    fprintf('    e_ss psi   : %.6f rad  (%.4f deg)\n', ess_psi,   rad2deg(ess_psi));

    fprintf('\n  [3] SETTLING TIME — Position (thr = %.4f m)\n%s\n', thr_pos, lin);
    fprintf('    t_s x : %s\n', fmt(ts_x));
    fprintf('    t_s y : %s\n', fmt(ts_y));
    fprintf('    t_s z : %s\n', fmt(ts_z));

    fprintf('\n  [4] SETTLING TIME — Attitude (thr = %.6f rad)\n%s\n', thr_att, lin);
    fprintf('    t_s phi   : %s\n', fmt(ts_phi));
    fprintf('    t_s theta : %s\n', fmt(ts_theta));
    fprintf('    t_s psi   : %s\n', fmt(ts_psi));

    fprintf('\n  [5] SLIDING SURFACE REACH TIME\n%s\n', lin);
    fprintf('    t_reach phi   : %s\n', fmt(tr_phi));
    fprintf('    t_reach theta : %s\n', fmt(tr_theta));
    fprintf('    t_reach psi   : %s\n', fmt(tr_psi));
    fprintf('    t_reach (max) : %s\n', fmt(tr_max));

    fprintf('\n%s\n\n', sep);

    % =====================================================================
    % SUMMARY TABLE
    % =====================================================================
    w = repmat('=',1,92);
    d = repmat('-',1,92);
    fprintf('  SUMMARY TABLE (%s)\n%s\n', ctrl_name, w);
    fprintf('  %-32s %-10s %-10s %-10s %-10s %-10s %-10s\n', ...
            'Metric','x','y','z','phi','theta','psi');
    fprintf('%s\n', d);
    fprintf('  %-32s %-10.6f %-10.6f %-10.6f %-10.6f %-10.6f %-10.6f\n', ...
            'RMSE (m / rad)', RMSE_x,RMSE_y,RMSE_z,RMSE_phi,RMSE_theta,RMSE_psi);
    fprintf('%s\n', d);
    fprintf('  %-32s %-10.6f %-10.6f %-10.6f %-10.6f %-10.6f %-10.6f\n', ...
            'Steady-State Err (m / rad)', ess_x,ess_y,ess_z,ess_phi,ess_theta,ess_psi);
    fprintf('%s\n', d);
    fprintf('  %-32s %-10s %-10s %-10s %-10s %-10s %-10s\n', ...
            'Settling Time (s)', ...
            fmt(ts_x),fmt(ts_y),fmt(ts_z),fmt(ts_phi),fmt(ts_theta),fmt(ts_psi));
    fprintf('%s\n', d);
    fprintf('  %-32s phi: %-10s  theta: %-10s  psi: %-10s\n', ...
            'Sliding Reach Time (s)', fmt(tr_phi),fmt(tr_theta),fmt(tr_psi));
    fprintf('%s\n\n', w);

end