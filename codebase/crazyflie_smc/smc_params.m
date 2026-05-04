function p = smc_params(p)
% SMC gains for Bitcraze Crazyflie 2.1
%
% GAIN STRUCTURE NOTES
% ────────────────────
% Sliding surface:  s = ė + c·e
% Surface dynamics: ṡ + ks·sat(s/bl) + kl·s = 0
%
% For ATTITUDE channels the controller produces:
%   U2 = Ixx * (gyro_cross + phi_cmd_ddot + c_phi*e_phi_dot + ks_phi*sat + kl_phi*s)
% The gains ks, kl, c multiply Ixx INSIDE the controller, so they do NOT
% need rescaling when switching platforms — Ixx handles the torque scaling.
%
%   CF2.1: Ixx*ks_phi = 16.57e-6 * 0.3 = 4.97e-6 Nm  [tau_phi_max = 1.04e-2 Nm]
%   => switching term uses < 0.05% of available torque authority — safe.
%
% For POSITION/ALTITUDE channels the controller produces U1, Ux, Uy which
% are dimensionless or in Newtons scaled by m. These are more sensitive to
% platform scale. Gains are kept conservatively small for the indoor CF envelope.
%
% XY outer-loop: kpx generates a velocity reference, kdx generates an
% acceleration reference. For CF on a 0.5m circle:
%   vel_ref_max = kpx * 0.5 = 0.35 m/s   (kpx = 0.7)
%   accel_max   = kdx * 0.35 = 0.18 m/s^2 (kdx = 0.5)
%   phi_des ~ accel/g = 1 deg  — well within small-angle regime.
%
% c_z / c_x / c_y: surface slope = ratio of velocity weight to position
%   weight. Kept at 0.5 rad/s (same as original) — appropriate for slow
%   indoor trajectories. Increase to ~1.0 if faster convergence is needed.
%
% ks_z: switching gain for altitude. CF has 9 m/s^2 of accel headroom above
%   hover. ks_z = 0.4 keeps the switching impulse well within that margin.
%
% kl_z: linear gain on the sliding surface. Provides smooth closure when
%   s is small (inside the boundary layer). kl_z = 0.2 is conservative.


    % XY outer-loop cascade gains
    % vel_ref = kp * (xd - x),  accel_ref = kd * (vel_ref - x_dot)
    p.kpx = 0.7;    % x position -> velocity ref  [same as original, safe for CF]
    p.kpy = 0.7;    % y position -> velocity ref
    p.kdx = 0.5;    % x velocity -> accel ref
    p.kdy = 0.5;    % y velocity -> accel ref

    % ── Altitude (z) ─────────────────────────────────────────────────────────
    % U1 = (m/cos*cos) * (g + Kz*z_dot/m + Zd_ddot + c_z*e_z_dot + ks_z*sat + kl_z*s)
    % T_total_max = 0.64 N, m*g = 0.334 N => headroom = 9 m/s^2 in accel units
    p.c_z   = 0.5;  % sliding surface slope (rad/s)
    p.ks_z  = 0.4;  % switching gain  [reduced from 0.5 — CF has tighter T_max margin]
    p.kl_z  = 0.2;  % linear gain on surface

    % ── X position ───────────────────────────────────────────────────────────
    % Ux = (m/U1)*(accel_terms)  clamped to [-1,1] before attitude inversion
    % CF: (m/U1)~(1/g) at hover, so Ux is purely accel/g — no platform scaling needed
    p.c_x   = 0.5;
    p.ks_x  = 0.4;  % reduced slightly from 0.5 for smoother Ux
    p.kl_x  = 0.1;

    % ── Y position ───────────────────────────────────────────────────────────
    p.c_y   = 0.5;
    p.ks_y  = 0.4;
    p.kl_y  = 0.1;

    % ── Roll (phi) ───────────────────────────────────────────────────────────
    % U2 = Ixx*(... + c_phi*e_phi_dot + ks_phi*sat + kl_phi*s)
    % Ixx_CF = 16.57e-6; Ixx*ks_phi = 4.97e-6 Nm << tau_phi_max = 0.0104 Nm
    % Gains are safe unchanged; c_phi=1 gives 1 rad/s surface bandwidth.
    p.c_phi   = 1.0;
    p.ks_phi  = 0.3;
    p.kl_phi  = 0.0;

    % ── Pitch (theta) ────────────────────────────────────────────────────────
    % Iyy_CF = 16.66e-6 ≈ Ixx_CF; symmetric treatment
    p.c_theta  = 1.0;
    p.ks_theta = 0.3;
    p.kl_theta = 0.0;

    % ── Yaw (psi) ────────────────────────────────────────────────────────────
    % Izz_CF = 29.26e-6; Izz*ks_psi = 2.93e-6 Nm << tau_psi_max = 8.04e-3 Nm
    % Yaw authority is tighter than roll/pitch (smaller l, smaller d/b ratio).
    % c_psi = 0.5 gives slower yaw convergence — appropriate for CF.
    p.c_psi   = 0.5;
    p.ks_psi  = 0.1;
    p.kl_psi  = 0.3;  % reduced from 0.5 — yaw authority is tighter on CF

    % Boundary layer width (used in sat() function)
    % bl > 0 gives smooth saturation; bl = 0 collapses to pure sign() (chattering)
    p.sat_bl = 0.15;  % slightly tighter than original 0.2 for faster surface approach

end