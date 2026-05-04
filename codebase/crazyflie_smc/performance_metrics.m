function performance_metrics(t, ex, ey, ez, ephi, etheta, epsi)
    RMS_x     = sqrt(mean(ex.^2));
    RMS_y     = sqrt(mean(ey.^2));
    RMS_z     = sqrt(mean(ez.^2));
    RMS_phi   = sqrt(mean(ephi.^2));
    RMS_theta = sqrt(mean(etheta.^2));
    RMS_psi   = sqrt(mean(epsi.^2));

    IAE_x = trapz(t, abs(ex));
    IAE_y = trapz(t, abs(ey));
    IAE_z = trapz(t, abs(ez));

    ISE_x = trapz(t, ex.^2);
    ISE_y = trapz(t, ey.^2);
    ISE_z = trapz(t, ez.^2);

    ITAE_x = trapz(t, t .* abs(ex));
    ITAE_y = trapz(t, t .* abs(ey));
    ITAE_z = trapz(t, t .* abs(ez));

    ess_x     = ex(end);
    ess_y     = ey(end);
    ess_z     = ez(end);
    ess_phi   = ephi(end);
    ess_theta = etheta(end);
    ess_psi   = epsi(end);

    fprintf('\n========== Tracking Performance Metrics ==========\n');
    fprintf('RMS Position Error:    [x y z]         = [%.4f  %.4f  %.4f]\n', RMS_x,     RMS_y,     RMS_z);
    fprintf('RMS Attitude Error:    [phi theta psi]  = [%.4f  %.4f  %.4f]\n', RMS_phi,   RMS_theta, RMS_psi);
    % fprintf('IAE:                   [x y z]         = [%.4f  %.4f  %.4f]\n', IAE_x,     IAE_y,     IAE_z);
    % fprintf('ISE:                   [x y z]         = [%.4f  %.4f  %.4f]\n', ISE_x,     ISE_y,     ISE_z);
    % fprintf('ITAE:                  [x y z]         = [%.4f  %.4f  %.4f]\n', ITAE_x,    ITAE_y,    ITAE_z);
    fprintf('Position Steady-State: [x y z]         = [%.4f  %.4f  %.4f]\n', ess_x,     ess_y,     ess_z);
    fprintf('Attitude Steady-State: [phi theta psi] = [%.4f  %.4f  %.4f]\n', ess_phi,   ess_theta, ess_psi);

end
