% =========================================================================
% stsmc_main.m — Super-Twisting SMC for Crazyflie 2.1
%                3D Lissajous Figure-8 Trajectory
% =========================================================================
%
% CHANGES FROM ORIGINAL (v0_quad_discrete_xy_ux_uy_fault.m)
% ----------------------------------------------------------
%   1. Physical parameters updated to Bitcraze Crazyflie 2.1.
%   2. Trajectory: 3D Lissajous figure-8 (matches rt_thesis.m):
%        x_d = 0.5*sin(0.5*t),  y_d = 0.5*sin(t),
%        z_d = 1.0 + 0.2*sin(0.5*t),  psi_d = 0
%      Analytical zcmd_dot, zcmd_ddot, psicmd_dot, psicmd_ddot.
%   3. T_sim = 50 s (was 100 s);  tau = 0.01 s unchanged.
%   4. Gains re-tuned for Crazyflie (see inline notes).
%   5. k1_z / k2_z separated from k1 / k2:
%      CF Jx (~17 µN·m²) is ~1757x smaller than original, so altitude
%      (force) and attitude (torque) channels need different ST magnitudes.
%   6. ux / uy saturation corrected:
%      Original: f_sat(uxdash,-1,1) / u1_cmd → on CF (u1≈0.334 N) this
%      gives |ux| up to 3.0, causing thetacmd → 90°.
%      Fixed:    f_sat(uxdash, -m*g*sin_max, m*g*sin_max) / u1_cmd
%      → |ux|, |uy| ≤ sin_max ≈ 0.44  (26° max tilt), as physically intended.
%   7. Output saturations added on u1, u2, u3, u4 to enforce actuator
%      limits and prevent super-twisting integral (I_u2/I_u3/I_u4) windup:
%        |u2|, |u3| ≤ tau_att_max ≈ 0.012 Nm   (roll/pitch torque budget)
%        |u4|       ≤ tau_psi_max ≈ 0.006 Nm   (yaw torque budget)
%        u1         ∈ [0.001, 0.64] N
%      Analogous to the f_sat already applied to ux / uy.
%
% UNCHANGED
% ---------
%   Control law structure (STSMC surfaces, super-twisting integral
%   dynamics, global variable interface), ode4 integration, fault model,
%   helper functions: asin_norm, normalise, f_sat, ode4,
%   quad_discrete_function.
% =========================================================================

clc; clear all; close all;

global Jy Jx Jz m g ez sz u1 ephi sphi u2 etheta stheta u3 ...
       epsi spsi u4 ex sx ey sy

% =========================================================================
% Physical Parameters — Bitcraze Crazyflie 2.1
% =========================================================================
Jx = 16.571710e-6;   % MOI about X axis (kg m^2)
Jy = 16.655602e-6;   % MOI about Y axis (kg m^2)
Jz = 29.261652e-6;   % MOI about Z axis (kg m^2)
m  = 0.034;          % Mass (kg)
g  = 9.81;           % Gravity (m/s^2)
l  = 0.046;          % Arm length, centre to motor (m)
b  = 2.88e-8;        % Thrust coefficient    (N s^2/rad^2)
d  = 7.24e-10;       % Drag/torque coefficient (N m s^2/rad^2)
T_min = 0;
T_max = 0.16;        % Max thrust per rotor (N)

l_eff = l / sqrt(2);
A_mix = [ b,          b,          b,          b;
         -b*l_eff,   -b*l_eff,    b*l_eff,    b*l_eff;
          b*l_eff,   -b*l_eff,   -b*l_eff,    b*l_eff;
         -d,          d,          -d,           d];

% Physical actuator limits
u1_max      = 4 * T_max;           % 0.64 N total thrust
tau_att_max = 2 * l_eff * T_max;   % ~0.0104 Nm roll/pitch max
tau_psi_max = 2 * (d/b) * T_max * (b/d);  % approximate yaw max
tau_att_max = 0.012;   % Nm (generous margin above theoretical 0.0104)
tau_psi_max = 0.006;   % Nm

% Max normalised tilt (prevents asin blowing up): sin(26 deg) ~ 0.44
sin_max = sin(deg2rad(26));

% =========================================================================
% Fault Parameters  (healthy for comparison)
% =========================================================================
fault_enable = 0;
fault_mode   = 1;
fault_start  = 25;
fault_end    = 100;
alpha1 = 1.0;   alpha2 = 1.0;   alpha3 = 1.0;   alpha4 = 1.0;

% =========================================================================
% Trajectory — 3D Lissajous figure-8 (matches rt_thesis.m)
% =========================================================================
A_traj = 0.5;   % XY amplitude (m)
w_traj = 0.5;   % Base angular frequency (rad/s)
z0     = 1.0;   % Nominal hover altitude (m)
h_traj = 0.2;   % Altitude oscillation amplitude (m)

% =========================================================================
% Simulation Setup
% =========================================================================
tau   = 0.01;
T_sim = 50;
t     = 0 : tau : T_sim;
N     = length(t);

x_0   = zeros(1, 24);
x_k   = x_0;
x_arr = x_0;

phicmd_old       = 0;
phicmd_dot_old   = 0;
thetacmd_old     = 0;
thetacmd_dot_old = 0;

% =========================================================================
% Controller Gains — Crazyflie 2.1 on Lissajous
%
%  ATTITUDE error dynamics (linearised, ignoring ST):
%    e_ddot + (kp/kd)*e_dot + (ki/kd)*e = 0
%    omega_n = sqrt(ki/kd) = sqrt(1/0.05) = 4.47 rad/s
%    zeta    = kp / (2*kd*omega_n) = 0.5/(2*0.05*4.47) = 1.12  (overdamped)
%
%  ST magnitude constraint:
%    (k1/kd)*sqrt(|s|_typical) <= tau_att_max
%    at |s|~0.3:  k1 <= tau_att_max * kd / sqrt(0.3) = 0.012*0.05/0.548 = 0.0011
%    → k1 = 0.0008 (safe), output saturation ensures robustness
% =========================================================================

% XY outer-loop cascade
kpx = 2.0;   kpy = 3.0;
kdx = 1.5;   kdy = 2.0;

% Altitude: s_z = kpz*ez + kdz*ez_dot + kiz*integral_ez
kpz = 0.8;   kdz = 1.0;   kiz = 0.1;
k1_z = 0.4;  k2_z = 0.05;

% Attitude: s = kp*e + kd*e_dot + ki*integral_e
kp = 0.5;   kd = 0.05;   ki = 1.0;
k1 = 0.0008;   k2 = 0.000008;

% XY position surface + super-twisting
kp_x = 1.0;   kd_x = 1.0;   ki_x = 0.1;
kp_y = 1.5;   kd_y = 1.0;   ki_y = 0.1;
k1_x = 0.2;   k2_x = 0.008;
k1_y = 0.3;   k2_y = 0.010;

% =========================================================================
% Pre-allocate
% =========================================================================
u_1 = [];  u_2 = [];  u_3 = [];  u_4 = [];  u_x = [];  u_y = [];
e1  = [];  e2  = [];  e3  = [];  e4  = [];  e_x = [];  e_y = [];
s_z     = [];  s_phi   = [];
s_theta = [];  s_psi   = [];
s_x_arr = [];  s_y_arr = [];

U_cmd_log   = zeros(N-1, 4);
U_act_log   = zeros(N-1, 4);
alpha_log   = ones(N-1, 4);
T_log       = zeros(N-1, 4);
T_total_log = zeros(N-1, 1);

fprintf('Running STSMC simulation...\n');

% =========================================================================
% Main Loop
% =========================================================================
for i = 1 : N-1

    % Reference: Lissajous
    xd(i)         =  A_traj * sin(w_traj * t(i));
    yd(i)         =  A_traj * sin(2 * w_traj * t(i));
    zcmd(i)       =  z0     + h_traj * sin(w_traj * t(i));
    zcmd_dot(i)   =  h_traj * w_traj * cos(w_traj * t(i));
    zcmd_ddot(i)  = -h_traj * w_traj^2 * sin(w_traj * t(i));
    psicmd(i)     = 0;
    psicmd_dot(i) = 0;
    psicmd_ddot(i)= 0;

    % ── U1 ───────────────────────────────────────────────────────────────
    ez    = zcmd(i) - x_k(9);
    e1(i) = ez;
    ezdot = zcmd_dot(i) - x_k(12);
    I_u1  = x_k(13);
    sz       = kpz*ez + kdz*ezdot + kiz*x_k(14);
    s_z(i)   = sz;
    u1_cmd = (m / (kdz*cos(x_k(1))*cos(x_k(2)))) * ...
             (kdz*g + kd*zcmd_ddot(i) + kpz*ezdot + kiz*ez ...
              + k1_z*sqrt(abs(sz))*sign(sz) + k2_z*I_u1);
    u1_cmd = max(0.001, min(u1_max, u1_cmd));   % physical thrust limit

    % ── Outer loop ───────────────────────────────────────────────────────
    xd_dot  = kpx * (xd(i) - x_k(7));
    yd_dot  = kpy * (yd(i) - x_k(8));
    xd_ddot = kdx * (xd_dot - x_k(10));
    yd_ddot = kdy * (yd_dot - x_k(11));

    % ── Ux ───────────────────────────────────────────────────────────────
    ex     = xd(i) - x_k(7);
    e_x(i) = ex;
    exdot  = xd_dot - x_k(10);
    I_ux   = x_k(21);
    sx       = kp_x*ex + kd_x*exdot + ki_x*x_k(22);
    s_x_arr(i) = sx;
    uxdash = (m/kd_x) * (kd_x*xd_ddot + kp_x*exdot + ki_x*ex ...
              + k1_x*sqrt(abs(sx))*sign(sx) + k2_x*I_ux);
    % Physical force saturation before normalising (FIX for CF scaling)
    ux     = f_sat(uxdash, -m*g*sin_max, m*g*sin_max) / u1_cmd;
    u_x(i) = ux;

    % ── Uy ───────────────────────────────────────────────────────────────
    ey     = yd(i) - x_k(8);
    e_y(i) = ey;
    eydot  = yd_dot - x_k(11);
    I_uy   = x_k(23);
    sy       = kp_y*ey + kd_y*eydot + ki_y*x_k(24);
    s_y_arr(i) = sy;
    uydash = (m/kd_y) * (kd_y*yd_ddot + kp_y*eydot + ki_y*ey ...
              + k1_y*sqrt(abs(sy))*sign(sy) + k2_y*I_uy);
    uy     = f_sat(uydash, -m*g*sin_max, m*g*sin_max) / u1_cmd;
    u_y(i) = uy;

    % ── Attitude commands ────────────────────────────────────────────────
    phicmd(i)       = asin_norm(ux*sin(psicmd(i)) - uy*cos(psicmd(i)));
    phicmd_dot(i)   = normalise(phicmd(i) - phicmd_old) / tau;
    phicmd_old      = phicmd(i);
    phicmd_ddot(i)  = (phicmd_dot(i) - phicmd_dot_old) / tau;
    phicmd_dot_old  = phicmd_dot(i);

    thetacmd(i)      = asin_norm((ux*cos(psicmd(i)) + uy*sin(psicmd(i))) / cos(phicmd(i)));
    thetacmd_dot(i)  = normalise(thetacmd(i) - thetacmd_old) / tau;
    thetacmd_old     = thetacmd(i);
    thetacmd_ddot(i) = (thetacmd_dot(i) - thetacmd_dot_old) / tau;
    thetacmd_dot_old = thetacmd_dot(i);

    % ── U2: Roll ─────────────────────────────────────────────────────────
    ephi    = normalise(phicmd(i) - x_k(1));
    e2(i)   = ephi;
    ephidot = normalise(phicmd_dot(i) - x_k(4));
    I_u2    = x_k(17);
    sphi    = kp*ephi + kd*ephidot + ki*x_k(18);
    s_phi(i) = sphi;
    u2_cmd  = -(Jy-Jz)*x_k(5)*x_k(6) ...
               + Jx*(kp/kd)*ephidot + Jx*phicmd_ddot(i) + Jx*(ki/kd)*ephi ...
               + (k1/kd)*sqrt(abs(sphi))*sign(sphi) + (k2/kd)*I_u2;
    u2_cmd  = max(-tau_att_max, min(tau_att_max, u2_cmd));

    % ── U3: Pitch ────────────────────────────────────────────────────────
    etheta    = normalise(thetacmd(i) - x_k(2));
    e3(i)     = etheta;
    ethetadot = normalise(thetacmd_dot(i) - x_k(5));
    I_u3      = x_k(15);
    stheta    = kp*etheta + kd*ethetadot + ki*x_k(16);
    s_theta(i) = stheta;
    u3_cmd  = -(Jz-Jx)*x_k(4)*x_k(6) ...
               + Jy*(kp/kd)*ethetadot + Jy*thetacmd_ddot(i) + Jy*(ki/kd)*etheta ...
               + (k1/kd)*sqrt(abs(stheta))*sign(stheta) + (k2/kd)*I_u3;
    u3_cmd  = max(-tau_att_max, min(tau_att_max, u3_cmd));

    % ── U4: Yaw ──────────────────────────────────────────────────────────
    epsi    = normalise(psicmd(i) - x_k(3));
    e4(i)   = epsi;
    epsidot = normalise(psicmd_dot(i) - x_k(6));
    I_u4    = x_k(19);
    spsi    = kp*epsi + kd*epsidot + ki*x_k(20);
    s_psi(i) = spsi;
    u4_cmd  = -(Jx-Jy)*x_k(4)*x_k(5) ...
               + Jz*(kp/kd)*epsidot + Jz*psicmd_ddot(i) + Jz*(ki/kd)*epsi ...
               + (k1/kd)*sqrt(abs(spsi))*sign(spsi) + (k2/kd)*I_u4;
    u4_cmd  = max(-tau_psi_max, min(tau_psi_max, u4_cmd));

    % ── Fault model (unchanged) ───────────────────────────────────────────
    U_cmd     = [u1_cmd; u2_cmd; u3_cmd; u4_cmd];
    alpha_vec = ones(4, 1);
    if fault_enable == 1
        fault_active = false;
        if fault_mode == 1;       fault_active = true;
        elseif fault_mode == 2
            if fault_end > fault_start
                fault_active = (t(i) >= fault_start && t(i) <= fault_end);
            end
        end
        if fault_active;  alpha_vec = [alpha1; alpha2; alpha3; alpha4];  end
    end

    alpha_eff = zeros(4, 1);
    for ch = 1:4
        row_h = A_mix(ch, :);
        row_f = A_mix(ch, :) .* alpha_vec';
        hs    = sum(abs(row_h));
        alpha_eff(ch) = (hs > 0) * sum(abs(row_f)) / max(hs, 1e-12) + (hs == 0);
    end

    U_actual = alpha_eff .* U_cmd;

    Omega2_nom = max(0, A_mix \ U_cmd);
    T_nom      = b * Omega2_nom;
    T_motors   = alpha_vec .* T_nom;
    T_motors   = max(T_min, min(T_motors, T_max));
    T_total    = sum(T_motors);

    U_cmd_log(i,:)  = U_cmd';
    U_act_log(i,:)  = U_actual';
    alpha_log(i,:)  = alpha_vec';
    T_log(i,:)      = T_motors';
    T_total_log(i)  = T_total;

    u1 = U_actual(1);  u_1(i) = u1;
    u2 = U_actual(2);  u_2(i) = u2;
    u3 = U_actual(3);  u_3(i) = u3;
    u4 = U_actual(4);  u_4(i) = u4;

    sx = s_x_arr(i);
    sy = s_y_arr(i);

    x_step        = ode4(@quad_discrete_function, [t(i) t(i+1)], x_k);
    x_k           = x_step(end, :);
    x_arr(i+1,:)  = x_k;

end

fprintf('Simulation complete.\n');

% =========================================================================
% Assemble thesis-compatible logs struct
% =========================================================================
t_plot = t(1:end-1)';

% Reorder to standard [x y z phi theta psi xd yd zd pd td sd]
% STSMC: phi=1,theta=2,psi=3, phi_dot=4,theta_dot=5,psi_dot=6, x=7,y=8,z=9, xd=10,yd=11,zd=12
x_std = x_arr(1:end-1, [7 8 9 1 2 3 10 11 12 4 5 6]);

x_ref   = A_traj * sin(w_traj * t_plot);
y_ref   = A_traj * sin(2 * w_traj * t_plot);
z_ref   = z0 + h_traj * sin(w_traj * t_plot);
psi_ref = zeros(size(t_plot));

ex_log     = x_ref - x_std(:,1);
ey_log     = y_ref - x_std(:,2);
ez_log     = z_ref - x_std(:,3);
ephi_log   = -x_std(:,4);        % ref = 0 (consistent with SMC)
etheta_log = -x_std(:,5);
epsi_log   = psi_ref - x_std(:,6);

logs.x_ref     = x_ref;
logs.y_ref     = y_ref;
logs.z_ref     = z_ref;
logs.psi_ref   = psi_ref;
logs.phi_ref   = zeros(size(t_plot));
logs.theta_ref = zeros(size(t_plot));
logs.phi_cmd   = phicmd';
logs.theta_cmd = thetacmd';
logs.ex        = ex_log;
logs.ey        = ey_log;
logs.ez        = ez_log;
logs.ephi      = ephi_log;
logs.etheta    = etheta_log;
logs.epsi      = epsi_log;
logs.U1_cmd_log = U_cmd_log(:,1);
logs.U2_cmd_log = U_cmd_log(:,2);
logs.U3_cmd_log = U_cmd_log(:,3);
logs.U4_cmd_log = U_cmd_log(:,4);
logs.s_phi   = s_phi';
logs.s_theta = s_theta';
logs.s_psi   = s_psi';
logs.s_x     = s_x_arr';
logs.s_y     = s_y_arr';
logs.s_z     = s_z';

p.sat_bl = 0.05;

% =========================================================================
% Performance Metrics + Thesis Plots
% =========================================================================
pm_thesis_stsmc(t_plot, ex_log, ey_log, ez_log, ...
                ephi_log, etheta_log, epsi_log, logs, p);

plots_thesis_stsmc(t_plot, x_std, logs, p);