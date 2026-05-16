function p = rt_thesis(p)
% =========================================================================
% rt_thesis.m — Thesis Reference Trajectory: 3D Lemniscate (Lissajous Fig-8)
% =========================================================================
% Matches the PID rt_thesis.m exactly:
%   A  = 0.5 m,  w = 0.5 rad/s,  z0 = 1.0 m,  h = 0.2 m
%
% TRAJECTORY:
%   x_d(t) = A * sin(w*t)
%   y_d(t) = A * sin(2*w*t)
%   z_d(t) = z0 + h * sin(w*t)
%   psi_d  = 0  (fixed heading)
%
%   Period        : T = 2*pi/w = 12.57 s
%   Max XY speed  : A*w = 0.25 m/s (x),   2*A*w = 0.50 m/s (y)
%   Max Z speed   : h*w = 0.10 m/s
%   XY extent     : ±0.5 m  (x),  ±0.5 m  (y)
%   Z extent      : 0.8 – 1.2 m
%
% OUTPUT FIELDS (consumed by smc_controller.m and reconstruct_signals.m):
%   p.x_des,  p.y_des,  p.z_des
%   p.zd_dot_des,   p.zd_ddot_des          (used by smc_controller U1 law)
%   p.psid_dot_des, p.psid_ddot_des        (used by smc_controller U4 law)
%   p.psi_des
%   p.xd_dot_des,   p.xd_ddot_des          (SMC feed-forward, optional)
%   p.yd_dot_des,   p.yd_ddot_des          (SMC feed-forward, optional)
%
% NOTE: Field names for z and psi use the SMC convention
%   (zd_dot_des / psid_dot_des) which is what smc_controller.m reads.
% =========================================================================

    % ── Trajectory parameters (MUST match PID rt_thesis.m) ───────────────
    A  = 0.5;   % XY amplitude            (m)
    w  = 0.5;   % base angular frequency  (rad/s)
    z0 = 1.0;   % nominal hover altitude  (m)   <- matches PID
    h  = 0.2;   % altitude oscillation    (m)   <- matches PID

    % ── Position references ──────────────────────────────────────────────
    p.x_des = @(t)  A * sin(w * t);
    p.y_des = @(t)  A * sin(2 * w * t);
    p.z_des = @(t)  z0 + h * sin(w * t);

    % ── Altitude derivatives (analytical) — SMC field names ─────────────
    p.zd_dot_des  = @(t)   h * w       * cos(w * t);
    p.zd_ddot_des = @(t)  -h * w^2     * sin(w * t);

    % ── Yaw references (fixed heading) — SMC field names ─────────────────
    p.psi_des       = @(t)  zeros(size(t));
    p.psid_dot_des  = @(t)  zeros(size(t));
    p.psid_ddot_des = @(t)  zeros(size(t));

    % ── XY analytical derivatives (SMC / STSMC / NSTSMC feed-forward) ───
    p.xd_dot_des  = @(t)   A *     w        * cos(w * t);
    p.xd_ddot_des = @(t)  -A *     w^2      * sin(w * t);

    p.yd_dot_des  = @(t)   A * 2 * w        * cos(2 * w * t);
    p.yd_ddot_des = @(t)  -A * 4 * w^2      * sin(2 * w * t);

    % ── Print summary ─────────────────────────────────────────────────────
    fprintf('\n[rt_thesis] Trajectory: 3D Lemniscate (Lissajous Figure-8)\n');
    fprintf('  x_d(t) = %.2f * sin(%.2f * t)              m\n', A, w);
    fprintf('  y_d(t) = %.2f * sin(%.2f * t)              m\n', A, 2*w);
    fprintf('  z_d(t) = %.2f + %.2f * sin(%.2f * t)      m\n', z0, h, w);
    fprintf('  psi_d  = 0 (fixed heading)\n');
    fprintf('  Period : %.4f s    Max XY speed: %.3f / %.3f m/s (x/y)\n\n', ...
            2*pi/w, A*w, 2*A*w);

end