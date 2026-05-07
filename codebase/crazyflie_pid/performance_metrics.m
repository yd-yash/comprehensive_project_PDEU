% Computes and prints tracking performance metrics
% INPUTS:
%   t      - time vector
%   ex, ey, ez           - position errors
%   ephi, etheta, epsi   - attitude errors (rad)

function performance_metrics(t, ex, ey, ez, ephi, etheta, epsi)

    % RMS errors
    RMS_ex    = sqrt(mean(ex.^2));
    RMS_ey    = sqrt(mean(ey.^2));
    RMS_ez    = sqrt(mean(ez.^2));
    RMS_phi   = sqrt(mean(ephi.^2));
    RMS_theta = sqrt(mean(etheta.^2));
    RMS_psi   = sqrt(mean(epsi.^2));

    % IAE (Integral Absolute Error)
    IAE_x = trapz(t, abs(ex));
    IAE_y = trapz(t, abs(ey));
    IAE_z = trapz(t, abs(ez));

    % ISE (Integral Squared Error)
    ISE_x = trapz(t, ex.^2);
    ISE_y = trapz(t, ey.^2);
    ISE_z = trapz(t, ez.^2);

    % ITAE (Integral Time-weighted Absolute Error)
    ITAE_x = trapz(t, t .* abs(ex));
    ITAE_y = trapz(t, t .* abs(ey));
    ITAE_z = trapz(t, t .* abs(ez));

    % Steady-state errors
    ess_x     = ex(end);
    ess_y     = ey(end);
    ess_z     = ez(end);
    ess_phi   = ephi(end);
    ess_theta = etheta(end);
    ess_psi   = epsi(end);

    % Print
    fprintf('\n========== Tracking Performance Metrics ==========\n');
    fprintf('RMS Position Error:    [x y z]          = [%.4f  %.4f  %.4f]\n', RMS_ex,    RMS_ey,    RMS_ez);
    fprintf('RMS Attitude Error:    [phi theta psi]   = [%.4f  %.4f  %.4f]\n', RMS_phi,   RMS_theta, RMS_psi);
    fprintf('IAE:                   [x y z]          = [%.4f  %.4f  %.4f]\n', IAE_x,     IAE_y,     IAE_z);
    fprintf('ISE:                   [x y z]          = [%.4f  %.4f  %.4f]\n', ISE_x,     ISE_y,     ISE_z);
    fprintf('ITAE:                  [x y z]          = [%.4f  %.4f  %.4f]\n', ITAE_x,    ITAE_y,    ITAE_z);
    fprintf('Position Steady-State: [x y z]          = [%.4f  %.4f  %.4f]\n', ess_x,     ess_y,     ess_z);
    fprintf('Attitude Steady-State: [phi theta psi]  = [%.4f  %.4f  %.4f]\n', ess_phi,   ess_theta, ess_psi);
    fprintf('==================================================\n\n');

end