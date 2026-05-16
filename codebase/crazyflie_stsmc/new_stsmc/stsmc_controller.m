% =========================================================================
% stsmc_controller.m — Super-Twisting Sliding Mode Controller (STSMC)
% =========================================================================
% Drop-in replacement for pid_controller.m.
% Same input/output signature — swap @pid_controller → @stsmc_controller
% in main.m and nothing else changes.
%
% Architecture: cascaded, identical to the STSMC discrete formulation
%   Outer loop : ST-SMC on z, x, y  → u1, ux, uy
%   Inner loop : ST-SMC on φ, θ, ψ  → u2, u3, u4
%   φ_des / θ_des derived from ux/uy via inverse kinematics (small-angle)
%
% State vector assumed (18×1, same as pid_controller.m):
%   x(1:3)   [x  y  z]          position
%   x(4:6)   [φ  θ  ψ]          attitude (rad)
%   x(7:9)   [ẋ  ẏ  ż]          linear velocities
%   x(10:12) [φ̇  θ̇  ψ̇]         angular velocities
%   x(13:15) [∫ex ∫ey ∫ez]      position integral states
%   x(16:18) [∫eφ ∫eθ ∫eψ]     attitude integral states
%
% NOTE on integral states:
%   The quadrotor3D_dynamics.m integrates:
%     x(13)=∫(x_des-x),  x(14)=∫(y_des-y),  x(15)=∫(z_des-z)
%     x(16)=∫(φ_des-φ),  x(17)=∫(θ_des-θ),  x(18)=∫(ψ_des-ψ)
%   These are reused here as the ST integral term (I_u) in the STC law.
%
% STSMC law (per channel, generic):
%   s    = kp·e + kd·ė + ki·∫e
%   u_ST = [...nominal...] + k1·√|s|·sign(s) + k2·I_int
%   where I_int = ∫sign(s)  (carried in state vector)
%
% OUTPUTS  (struct u — identical fields to pid_controller.m):
%   u.u1        total thrust command  (N)
%   u.u2        roll torque           (Nm)
%   u.u3        pitch torque          (Nm)
%   u.u4        yaw torque            (Nm)
%   u.phi_des   desired roll angle    (rad)
%   u.theta_des desired pitch angle   (rad)
%   u.psi_des   desired yaw angle     (rad)
%
% EXTRA FIELDS (for sliding-variable logging in reconstruct_signals.m):
%   u.s_phi    sliding surface — roll
%   u.s_theta  sliding surface — pitch
%   u.s_psi    sliding surface — yaw
%   u.s_z      sliding surface — altitude
%   u.s_x      sliding surface — x position
%   u.s_y      sliding surface — y position
% =========================================================================

function u = stsmc_controller(t, x, p)

    % ---- Unpack states --------------------------------------------------
    x_pos     = x(1);   y_pos     = x(2);   z_pos     = x(3);
    phi       = x(4);   theta     = x(5);   psi       = x(6);
    x_dot     = x(7);   y_dot     = x(8);   z_dot     = x(9);
    phi_dot   = x(10);  theta_dot = x(11);  psi_dot   = x(12);
    % Integral states from dynamics integrators
    I_z       = x(13);  % ∫(x_des-x) — used as I_ux in x-channel below
    I_x       = x(14);  % ∫(y_des-y) — NOTE: dynamics order: 13=ex,14=ey,15=ez
    I_y       = x(15);  % ∫(z_des-z)
    % Attitude integral states
    I_phi     = x(16);
    I_theta   = x(17);
    I_psi     = x(18);

    % CORRECTION — dynamics.m integrates in order: ex,ey,ez,ephi,etheta,epsi
    % So:  x(13)=∫ex,  x(14)=∫ey,  x(15)=∫ez
    %      x(16)=∫eφ,  x(17)=∫eθ,  x(18)=∫eψ
    I_x   = x(13);
    I_y   = x(14);
    I_z   = x(15);
    I_phi = x(16);   I_theta = x(17);   I_psi = x(18);

    % ---- Reference trajectory -------------------------------------------
    x_des      = p.x_des(t);       y_des      = p.y_des(t);
    z_des      = p.z_des(t);
    x_dot_des  = p.x_dot_des(t);   y_dot_des  = p.y_dot_des(t);
    z_dot_des  = p.z_dot_des(t);
    x_ddot_des = p.x_ddot_des(t);  y_ddot_des = p.y_ddot_des(t);
    z_ddot_des = p.z_ddot_des(t);
    psi_des    = p.psi_des(t);

    % ---- Physical parameters --------------------------------------------
    m   = p.m;    g   = p.g;
    Jx  = p.Ixx;  Jy  = p.Iyy;   Jz  = p.Izz;

    % =====================================================================
    % SLIDING SURFACE PARAMETERS
    % =====================================================================
    % Position / altitude channels
    kpz = 8.0;    kdz = 4.0;    kiz = 0.5;     % z (altitude)
    kp_x = 6.0;   kd_x = 3.0;  ki_x = 0.4;    % x position
    kp_y = 6.0;   kd_y = 3.0;  ki_y = 0.4;    % y position

    % Attitude channels
    kp_att = 12.0;   kd_att = 2.5;   ki_att = 0.8;  % φ, θ
    kp_psi = 8.0;    kd_psi = 2.0;   ki_psi = 0.5;  % ψ

    % =====================================================================
    % SUPER-TWISTING GAINS
    % =====================================================================
    % Position channels
    k1z  = 0.35;   k2z  = 0.18;
    k1x  = 0.30;   k2x  = 0.15;
    k1y  = 0.30;   k2y  = 0.15;

    % Attitude channels (scaled to inertia)
    k1_att = 0.25;   k2_att = 0.12;
    k1_psi = 0.20;   k2_psi = 0.10;

    % =====================================================================
    % ALTITUDE LOOP — u1
    % =====================================================================
    ez      = z_des    - z_pos;
    ezdot   = z_dot_des - z_dot;

    s_z = kdz*ezdot + kpz*ez + kiz*I_z;

    % ST-SMC thrust: nominal + ST correction
    % u1_nom = m*(g + z_ddot_des) comes from feedback linearisation
    u1_fb  = kdz*z_ddot_des + kpz*ezdot + kiz*ez;
    u1_st  = k1z*sqrt(abs(s_z))*sign(s_z) + k2z*I_z;
    u1_raw = (m / (kdz * cos(phi)*cos(theta))) * (kdz*g + u1_fb + u1_st);

    u1 = max(0, min(u1_raw, 4 * p.T_max));

    % =====================================================================
    % HORIZONTAL POSITION LOOPS — ux, uy  (virtual accelerations)
    % =====================================================================
    ex    = x_des - x_pos;     exdot = x_dot_des - x_dot;
    ey    = y_des - y_pos;     eydot = y_dot_des - y_dot;

    s_x = kd_x*exdot + kp_x*ex + ki_x*I_x;
    s_y = kd_y*eydot + kp_y*ey + ki_y*I_y;

    ux_dash = kd_x*x_ddot_des + kp_x*exdot + ki_x*ex ...
              + k1x*sqrt(abs(s_x))*sign(s_x) + k2x*I_x;
    uy_dash = kd_y*y_ddot_des + kp_y*eydot + ki_y*ey ...
              + k1y*sqrt(abs(s_y))*sign(s_y) + k2y*I_y;

    % Normalise to get tilt commands (avoid divide-by-tiny-u1)
    u1_safe = max(u1, 0.5*m*g);
    ux = (m / kd_x) * ux_dash / u1_safe;
    uy = (m / kd_y) * uy_dash / u1_safe;

    % Saturate tilt (keep within ±30° for small-angle validity)
    ux = max(-0.5, min(ux, 0.5));
    uy = max(-0.5, min(uy, 0.5));

    % =====================================================================
    % INVERSE KINEMATICS — desired φ, θ from ux, uy
    % =====================================================================
    phi_des   = asin(max(-1, min(1,  ux*sin(psi_des) - uy*cos(psi_des))));
    cos_phi   = cos(phi_des);
    if abs(cos_phi) < 1e-4; cos_phi = sign(cos_phi)*1e-4; end
    theta_des = asin(max(-1, min(1, (ux*cos(psi_des) + uy*sin(psi_des)) / cos_phi)));

    % Numerical attitude rate references (same finite-difference approach
    % as original STSMC code; here approximated from trajectory jerk)
    phi_dot_des   = -(1/g) * p.y_dddot_des(t);
    theta_dot_des =  (1/g) * p.x_dddot_des(t);
    psi_dot_des   = 0;

    % =====================================================================
    % ATTITUDE INNER LOOPS — u2, u3, u4
    % =====================================================================
    % Roll (φ)
    ephi   = phi_des   - phi;
    ephidot = phi_dot_des - phi_dot;
    s_phi  = kd_att*ephidot + kp_att*ephi + ki_att*I_phi;
    u2 = -(Jy-Jz)*theta_dot*psi_dot ...
         + Jx*(kp_att/kd_att)*ephidot ...
         + Jx*(ki_att/kd_att)*ephi ...
         + (k1_att/kd_att)*sqrt(abs(s_phi))*sign(s_phi) ...
         + (k2_att/kd_att)*I_phi;

    % Pitch (θ)
    etheta   = theta_des - theta;
    ethetadot = theta_dot_des - theta_dot;
    s_theta  = kd_att*ethetadot + kp_att*etheta + ki_att*I_theta;
    u3 = -(Jz-Jx)*phi_dot*psi_dot ...
         + Jy*(kp_att/kd_att)*ethetadot ...
         + Jy*(ki_att/kd_att)*etheta ...
         + (k1_att/kd_att)*sqrt(abs(s_theta))*sign(s_theta) ...
         + (k2_att/kd_att)*I_theta;

    % Yaw (ψ)
    epsi   = psi_des   - psi;
    epsidot = psi_dot_des - psi_dot;
    s_psi  = kd_psi*epsidot + kp_psi*epsi + ki_psi*I_psi;
    u4 = -(Jx-Jy)*phi_dot*theta_dot ...
         + Jz*(kp_psi/kd_psi)*epsidot ...
         + Jz*(ki_psi/kd_psi)*epsi ...
         + (k1_psi/kd_psi)*sqrt(abs(s_psi))*sign(s_psi) ...
         + (k2_psi/kd_psi)*I_psi;

    % =====================================================================
    % PACK OUTPUTS
    % =====================================================================
    u.u1        = u1;
    u.u2        = u2;
    u.u3        = u3;
    u.u4        = u4;
    u.phi_des   = phi_des;
    u.theta_des = theta_des;
    u.psi_des   = psi_des;

    % Sliding surfaces — logged by reconstruct_signals_stsmc.m
    u.s_phi   = s_phi;
    u.s_theta = s_theta;
    u.s_psi   = s_psi;
    u.s_z     = s_z;
    u.s_x     = s_x;
    u.s_y     = s_y;

end