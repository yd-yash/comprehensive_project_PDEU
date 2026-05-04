% NS-STSMC: Non-Singular Super-Twisting Sliding Mode Control
% Quadrotor with actuator fault (loss-of-effectiveness)
% Sliding surface: s = e + (1/beta^(p/q)) * |edot|^(p/q) * sign(edot)
%                  with p=5, q=3  =>  p/q = 5/3
% STA injection  : v = -k1*|s|^(1/2)*sign(s) - k2*integ(sign(s))
% Control law    : u = (1/b_q)*( qdd_d - f_q - edotdot_req )
%                  edotdot_req = (v - edot) / mu,  mu clamped >= mu_min
% Fault model, mixing, plots — identical to v1 ST-SMC

clc
clear all
% close all

global Jy Jx Jz m g ez sz u1 ephi sphi u2 etheta stheta u3 ...
       epsi spsi u4 ex sx ey sy

u_1 = [];   u_2 = [];   u_3 = [];   u_4 = [];   u_x = [];   u_y = [];
e1  = [];   e2  = [];   e3  = [];   e4  = [];   e_x = [];   e_y = [];
s_z = [];   s_phi = [];  s_theta = [];  s_psi = [];

Jx = 0.029125;
Jy = 0.029125;
Jz = 0.055225;
m  = 1.5;
g  = 9.81;
l  = 0.23;

b = 3.13e-5;
d = 7.5e-5;

T_min = 0;
T_max = 2 * (m * g / 4);

l_eff = l / sqrt(2);
A_mix = [ b,           b,           b,           b;
         -b * l_eff,  -b * l_eff,   b * l_eff,   b * l_eff;
          b * l_eff,  -b * l_eff,  -b * l_eff,   b * l_eff;
         -d,           d,          -d,            d];

% fault parameters
fault_enable = 0;
fault_mode   = 1;
fault_start  = 25;
fault_end    = 100;

alpha1 = 0.5;
alpha2 = 1.0;
alpha3 = 1.0;
alpha4 = 1.0;

x_0 = zeros(1, 24);
tau = 0.01;
t   = 0 : tau : 100;
N   = length(t);
x_k = x_0;
x_arr = x_0;

phicmd_old     = 0;
phicmd_dot_old = 0;
thetacmd_old     = 0;
thetacmd_dot_old = 0;

% Cascaded outer-loop gains (position -> velocity reference) — unchanged
kpx = 0.7;
kpy = 0.7;
kdx = 0.5;
kdy = 0.5;

% =========================================================
% NS-STSMC PARAMETERS
% =========================================================
% Terminal exponent: p/q = 5/3  (p=5, q=3, both positive odd integers)
pq  = 5/3;          % p/q
qp  = 3/5;          % q/p  (used on manifold: edot = -beta*|e|^(q/p)*sign(e))
mu_min = 1e-3;      % lower clamp on mu to avoid division by near-zero edot

% beta: terminal surface shape gain (one per channel)
% larger beta -> faster convergence on manifold, smaller transient overshoot
beta_z     = 1.5;
beta_phi   = 1.5;
beta_theta = 1.5;
beta_psi   = 1.5;
beta_x     = 1.5;
beta_y     = 1.5;

% STA gains (k1 drives sqrt(|s|) term, k2 drives integrator term)
% Attitude channels
k1_att = 0.3;   k2_att = 0.15;
% Altitude channel
k1_z   = 0.3;   k2_z   = 0.15;
% Lateral virtual force channels
k1_x   = 0.2;   k2_x   = 0.4;
k1_y   = 0.2;   k2_y   = 0.4;
% =========================================================

alpha_log   = ones(N-1, 4);
T_log       = zeros(N-1, 4);
T_total_log = zeros(N-1, 1);
U_cmd_log   = zeros(N-1, 4);
U_act_log   = zeros(N-1, 4);

fprintf('Running NS-STSMC simulation...\n');

for i = 1 : N-1

    % ---- reference trajectories ----
    zcmd(i)      =  5 + 0*cos(1*t(i));
    zcmd_dot(i)  = -0*sin(1*t(i));
    zcmd_ddot(i) = -0*cos(1*t(i));

    psicmd(i)      =  0*pi/180*cos(0.2*t(i));
    psicmd_dot(i)  = -0*0.2*pi/180*sin(0.2*t(i));
    psicmd_ddot(i) = -0*0.04*pi/180*cos(0.2*t(i));

    xd(i) = 2*cos(0.1*t(i));
    yd(i) = 2*sin(0.2*t(i));

    % =========================================================
    % ALTITUDE — u1
    % dynamics: zddot = (u1/m)*cos(phi)*cos(theta) - g
    %   f_z = -g,  b_z = cos(phi)*cos(theta)/m
    % =========================================================
    ez     = zcmd(i) - x_k(9);
    e1(i)  = ez;
    ezdot  = zcmd_dot(i) - x_k(12);
    I_u1   = x_k(13);   % integ(sign(sz)) from dynamics

    % NTSM surface
    sz     = ez + (1/beta_z^pq) * abs(ezdot)^pq * sign(ezdot);
    s_z(i) = sz;

    % STA injection on sz
    v_z    = -k1_z * sqrt(abs(sz)) * sign(sz) - k2_z * I_u1;

    % mu_z = (p/q) * (1/beta_z^(p/q)) * |ezdot|^(p/q - 1)  — clamped
    mu_z   = max(mu_min, pq * (1/beta_z^pq) * abs(ezdot)^(pq - 1));

    % required eddot: (v_z - ezdot) / mu_z
    ezdotdot_req = (v_z - ezdot) / mu_z;

    % control law: u1 = (m / (cos(phi)*cos(theta))) * (zddot_d + g - ezdotdot_req)
    cphi  = cos(x_k(1));
    ctheta = cos(x_k(2));
    denom_z = cphi * ctheta;
    if abs(denom_z) < 1e-4; denom_z = sign(denom_z + 1e-10) * 1e-4; end
    u1_cmd = (m / denom_z) * (zcmd_ddot(i) + g - ezdotdot_req);

    % ---- cascaded lateral velocity/acceleration references ----
    xd_dot  = kpx * (xd(i) - x_k(7));
    yd_dot  = kpy * (yd(i) - x_k(8));
    xd_ddot = kdx * (xd_dot - x_k(10));
    yd_ddot = kdy * (yd_dot - x_k(11));

    % =========================================================
    % LATERAL X — virtual force ux
    % =========================================================
    ex      = xd(i) - x_k(7);
    e_x(i)  = ex;
    exdot   = xd_dot - x_k(10);
    I_ux    = x_k(21);

    sx      = ex + (1/beta_x^pq) * abs(exdot)^pq * sign(exdot);
    s_x(i)  = sx;

    v_x     = -k1_x * sqrt(abs(sx)) * sign(sx) - k2_x * I_ux;
    mu_x    = max(mu_min, pq * (1/beta_x^pq) * abs(exdot)^(pq - 1));
    exddot_req = (v_x - exdot) / mu_x;

    % ux* = m * (xddot_d - exddot_req),  then normalise and saturate
    uxdash  = m * (xd_ddot - exddot_req);
    ux      = f_sat(uxdash, -1, 1) / u1_cmd;
    u_x(i)  = ux;

    % =========================================================
    % LATERAL Y — virtual force uy
    % =========================================================
    ey      = yd(i) - x_k(8);
    e_y(i)  = ey;
    eydot   = yd_dot - x_k(11);
    I_uy    = x_k(23);

    sy      = ey + (1/beta_y^pq) * abs(eydot)^pq * sign(eydot);
    s_y(i)  = sy;

    v_y     = -k1_y * sqrt(abs(sy)) * sign(sy) - k2_y * I_uy;
    mu_y    = max(mu_min, pq * (1/beta_y^pq) * abs(eydot)^(pq - 1));
    eyddot_req = (v_y - eydot) / mu_y;

    uydash  = m * (yd_ddot - eyddot_req);
    uy      = f_sat(uydash, -1, 1) / u1_cmd;
    u_y(i)  = uy;

    % ---- attitude command inversion (unchanged) ----
    phicmd(i)      = asin_norm(ux*sin(psicmd(i)) - uy*cos(psicmd(i)));
    phicmd_dot(i)  = normalise(phicmd(i) - phicmd_old) / tau;
    phicmd_old     = phicmd(i);
    phicmd_ddot(i) = (phicmd_dot(i) - phicmd_dot_old) / tau;
    phicmd_dot_old = phicmd_dot(i);

    thetacmd(i)      = asin_norm((ux*cos(psicmd(i)) + uy*sin(psicmd(i))) / cos(phicmd(i)));
    thetacmd_dot(i)  = normalise(thetacmd(i) - thetacmd_old) / tau;
    thetacmd_old     = thetacmd(i);
    thetacmd_ddot(i) = (thetacmd_dot(i) - thetacmd_dot_old) / tau;
    thetacmd_dot_old = thetacmd_dot(i);

    % =========================================================
    % ROLL — u2
    % dynamics: phiddot = f_phi + u2/Jx
    %   f_phi = (Jy-Jz)/Jx * thetadot * psidot
    % =========================================================
    ephi      = normalise(phicmd(i) - x_k(1));
    e2(i)     = ephi;
    ephidot   = normalise(phicmd_dot(i) - x_k(4));
    I_u2      = x_k(17);
    f_phi     = (Jy - Jz) / Jx * x_k(5) * x_k(6);

    sphi      = ephi + (1/beta_phi^pq) * abs(ephidot)^pq * sign(ephidot);
    s_phi(i)  = sphi;

    v_phi     = -k1_att * sqrt(abs(sphi)) * sign(sphi) - k2_att * I_u2;
    mu_phi    = max(mu_min, pq * (1/beta_phi^pq) * abs(ephidot)^(pq - 1));
    ephiddot_req = (v_phi - ephidot) / mu_phi;

    % u2 = Jx * ( phiddot_d - f_phi - ephiddot_req )
    u2_cmd    = Jx * (phicmd_ddot(i) - f_phi - ephiddot_req);

    % =========================================================
    % PITCH — u3
    % dynamics: thetaddot = f_theta + u3/Jy
    %   f_theta = (Jz-Jx)/Jy * phidot * psidot
    % =========================================================
    etheta      = normalise(thetacmd(i) - x_k(2));
    e3(i)       = etheta;
    ethetadot   = normalise(thetacmd_dot(i) - x_k(5));
    I_u3        = x_k(15);
    f_theta     = (Jz - Jx) / Jy * x_k(4) * x_k(6);

    stheta      = etheta + (1/beta_theta^pq) * abs(ethetadot)^pq * sign(ethetadot);
    s_theta(i)  = stheta;

    v_theta     = -k1_att * sqrt(abs(stheta)) * sign(stheta) - k2_att * I_u3;
    mu_theta    = max(mu_min, pq * (1/beta_theta^pq) * abs(ethetadot)^(pq - 1));
    ethetaddot_req = (v_theta - ethetadot) / mu_theta;

    u3_cmd      = Jy * (thetacmd_ddot(i) - f_theta - ethetaddot_req);

    % =========================================================
    % YAW — u4
    % dynamics: psiddot = f_psi + u4/Jz
    %   f_psi = (Jx-Jy)/Jz * phidot * thetadot
    % =========================================================
    epsi      = normalise(psicmd(i) - x_k(3));
    e4(i)     = epsi;
    epsidot   = normalise(psicmd_dot(i) - x_k(6));
    I_u4      = x_k(19);
    f_psi     = (Jx - Jy) / Jz * x_k(4) * x_k(5);

    spsi      = epsi + (1/beta_psi^pq) * abs(epsidot)^pq * sign(epsidot);
    s_psi(i)  = spsi;

    v_psi     = -k1_att * sqrt(abs(spsi)) * sign(spsi) - k2_att * I_u4;
    mu_psi    = max(mu_min, pq * (1/beta_psi^pq) * abs(epsidot)^(pq - 1));
    epsiddot_req = (v_psi - epsidot) / mu_psi;

    u4_cmd    = Jz * (psicmd_ddot(i) - f_psi - epsiddot_req);

    % ---- NaN guard ----
    if isnan(u1_cmd) || isinf(u1_cmd); u1_cmd = 0; end
    if isnan(u2_cmd) || isinf(u2_cmd); u2_cmd = 0; end
    if isnan(u3_cmd) || isinf(u3_cmd); u3_cmd = 0; end
    if isnan(u4_cmd) || isinf(u4_cmd); u4_cmd = 0; end

    U_cmd = [u1_cmd; u2_cmd; u3_cmd; u4_cmd];

    % =========================================================
    % FAULT MODEL — identical to v1
    % =========================================================
    alpha_vec = ones(4, 1);

    if fault_enable == 1
        fault_active = false;
        if fault_mode == 1
            fault_active = true;
        elseif fault_mode == 2
            if fault_end > fault_start
                fault_active = (t(i) >= fault_start && t(i) <= fault_end);
            else
                warning('v2_quad_nsstsmc_fault: fault_end must be > fault_start. No fault applied.');
            end
        else
            warning('v2_quad_nsstsmc_fault: unknown fault_mode = %d. No fault applied.', fault_mode);
        end
        if fault_active
            alpha_vec = [alpha1; alpha2; alpha3; alpha4];
        end
    end

    alpha_eff = zeros(4, 1);
    for ch = 1:4
        row_healthy = A_mix(ch, :);
        row_faulty  = A_mix(ch, :) .* alpha_vec';
        healthy_sum = sum(abs(row_healthy));
        if healthy_sum > 0
            alpha_eff(ch) = sum(abs(row_faulty)) / healthy_sum;
        else
            alpha_eff(ch) = 1;
        end
    end

    U_actual = alpha_eff .* U_cmd;

    Omega2_nom = max(0, A_mix \ U_cmd);
    T_nom      = b * Omega2_nom;
    T_motors   = alpha_vec .* T_nom;
    T_motors   = max(T_min, min(T_motors, T_max));
    T_total    = sum(T_motors);

    U_cmd_log(i, :)  = U_cmd';
    U_act_log(i, :)  = U_actual';
    alpha_log(i, :)  = alpha_vec';
    T_log(i, :)      = T_motors';
    T_total_log(i)   = T_total;

    u1 = U_actual(1);
    u2 = U_actual(2);
    u3 = U_actual(3);
    u4 = U_actual(4);

    u_1(i) = u1;
    u_2(i) = u2;
    u_3(i) = u3;
    u_4(i) = u4;

    x      = ode4(@quad_discrete_function, [t(i) t(i+1)], x_k);
    x_k    = x(end, :);
    x_arr(i+1, :) = x_k;

end

fprintf('Simulation complete.\n');

% post-process psi double-dot
psi_arr     = x_arr(:, 3);
psidot_arr  = x_arr(:, 6);
psiddot_arr = zeros(N, 1);
for i = 2 : N-1
    psiddot_arr(i) = (psidot_arr(i+1) - psidot_arr(i-1)) / (2*tau);
end
psiddot_arr(1)   = psiddot_arr(2);
psiddot_arr(end) = psiddot_arr(end-1);

% =========================================================
% PLOTS — identical to v1
% =========================================================

figure('Color','w','Name','Position States')
subplot(3,1,1); hold on; grid on
plot(t(1:end-1), xd, 'r--')
plot(t, x_arr(:,7), 'b')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('x (m)'); legend('Reference','Actual'); title('Position Tracking')

subplot(3,1,2); hold on; grid on
plot(t(1:end-1), yd, 'r--')
plot(t, x_arr(:,8), 'b')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('y (m)'); legend('Reference','Actual')

subplot(3,1,3); hold on; grid on
plot(t(1:end-1), zcmd, 'r--')
plot(t, x_arr(:,9), 'b')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('z (m)'); xlabel('Time (s)'); legend('Reference','Actual')

figure('Color','w','Name','Attitude States')
subplot(3,1,1); hold on; grid on
plot(t(1:end-1), rad2deg(phicmd), 'r--')
plot(t, rad2deg(x_arr(:,1)),'b')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('\phi (deg)'); legend('\phi_{cmd}','Actual'); title('Attitude Tracking')

subplot(3,1,2); hold on; grid on
plot(t(1:end-1), rad2deg(thetacmd), 'r--')
plot(t, rad2deg(x_arr(:,2)),'b')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('\theta (deg)'); legend('\theta_{cmd}','Actual')

subplot(3,1,3); hold on; grid on
plot(t(1:end-1), rad2deg(psicmd), 'r--')
plot(t, rad2deg(x_arr(:,3)),'b')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('\psi (deg)'); xlabel('Time (s)'); legend('\psi_{cmd}','Actual')

eps_pos   = 0.05;
eps_att   = 0.5*pi/180;
eps_yaw   = 1.0*pi/180;

figure('Color','w','Name','Tracking Errors')
subplot(3,2,1); hold on; grid on
e_xp = xd - x_arr(1:end-1, 7)';
plot(t(1:end-1), e_xp, 'b'); yline(0,'k--')
draw_boundary(gca, t(1:end-1), eps_pos)
mark_convergence(gca, t(1:end-1), e_xp, eps_pos)
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('e_x (m)'); legend('e_x')

subplot(3,2,3); hold on; grid on
e_yp = yd - x_arr(1:end-1, 8)';
plot(t(1:end-1), e_yp, 'b'); yline(0,'k--')
draw_boundary(gca, t(1:end-1), eps_pos)
mark_convergence(gca, t(1:end-1), e_yp, eps_pos)
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('e_y (m)'); legend('e_y')

subplot(3,2,5); hold on; grid on
e_zp = zcmd - x_arr(1:end-1, 9)';
plot(t(1:end-1), e_zp, 'b'); yline(0,'k--')
draw_boundary(gca, t(1:end-1), eps_pos)
mark_convergence(gca, t(1:end-1), e_zp, eps_pos)
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('e_z (m)'); xlabel('Time (s)'); legend('e_z')

subplot(3,2,2); hold on; grid on
plot(t(1:end-1), e2, 'b'); yline(0,'k--')
draw_boundary(gca, t(1:end-1), eps_att)
mark_convergence(gca, t(1:end-1), e2, eps_att)
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('e_\phi (rad)'); legend('e_{\phi}')

subplot(3,2,4); hold on; grid on
plot(t(1:end-1), e3, 'b'); yline(0,'k--')
draw_boundary(gca, t(1:end-1), eps_att)
mark_convergence(gca, t(1:end-1), e3, eps_att)
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('e_\theta (rad)'); legend('e_{\theta}')

subplot(3,2,6); hold on; grid on
plot(t(1:end-1), e4, 'b'); yline(0,'k--')
draw_boundary(gca, t(1:end-1), eps_yaw)
mark_convergence(gca, t(1:end-1), e4, eps_yaw)
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('e_\psi (rad)'); xlabel('Time (s)'); legend('e_{\psi}')

figure('Color','w','Name','Control Inputs')
subplot(4,1,1); hold on; grid on
plot(t(1:end-1), U_cmd_log(:,1), 'r--')
plot(t(1:end-1), U_act_log(:,1), 'b')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('U_1 (N)'); legend('Commanded','Delivered'); title('Virtual Control Inputs')

subplot(4,1,2); hold on; grid on
plot(t(1:end-1), U_cmd_log(:,2), 'r--')
plot(t(1:end-1), U_act_log(:,2), 'b')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('U_2 (N·m)'); legend('Commanded','Delivered')

subplot(4,1,3); hold on; grid on
plot(t(1:end-1), U_cmd_log(:,3), 'r--')
plot(t(1:end-1), U_act_log(:,3), 'b')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('U_3 (N·m)'); legend('Commanded','Delivered')

subplot(4,1,4); hold on; grid on
plot(t(1:end-1), U_cmd_log(:,4), 'r--')
plot(t(1:end-1), U_act_log(:,4), 'b')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('U_4 (N·m)'); xlabel('Time (s)'); legend('Commanded','Delivered')

figure('Color','w','Name','Actuator Effectiveness')
alpha_labels = {'\alpha_1 (M1, front-right)', '\alpha_2 (M2, rear-left)', ...
                '\alpha_3 (M3, rear-right)',   '\alpha_4 (M4, front-left)'};
for k = 1:4
    subplot(4,1,k); hold on; grid on
    plot(t(1:end-1), alpha_log(:,k), 'k')
    mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
    ylabel(alpha_labels{k}); ylim([-0.1, 1.2])
    if k == 4; xlabel('Time (s)'); end
end
sgtitle('Actuator Effectiveness Factors \alpha_i')

figure('Color','w','Name','Rotor Thrusts')
colors   = {'b','r','m','g'};
T_labels = {'T_1 (N)','T_2 (N)','T_3 (N)','T_4 (N)'};
for k = 1:4
    subplot(5,1,k); hold on; grid on
    plot(t(1:end-1), T_log(:,k), colors{k})
    mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
    ylabel(T_labels{k})
end
subplot(5,1,5); hold on; grid on
plot(t(1:end-1), T_total_log, 'k')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('T_{total} (N)'); xlabel('Time (s)')
sgtitle('Individual Rotor Thrusts and Total Thrust')

figure('Color','w','Name','Sliding Variables')
subplot(3,2,1); hold on; grid on
plot(t(1:end-1), s_z, 'b'); yline(0,'k--')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('s_z'); title('Altitude')

subplot(3,2,2); hold on; grid on
plot(t(1:end-1), s_psi, 'b'); yline(0,'k--')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('s_\psi'); title('Yaw')

subplot(3,2,3); hold on; grid on
plot(t(1:end-1), s_x, 'b'); yline(0,'k--')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('s_x'); title('X Position')

subplot(3,2,4); hold on; grid on
plot(t(1:end-1), s_y, 'b'); yline(0,'k--')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('s_y'); title('Y Position')

subplot(3,2,5); hold on; grid on
plot(t(1:end-1), s_phi, 'b'); yline(0,'k--')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('s_\phi'); xlabel('Time (s)'); title('Roll')

subplot(3,2,6); hold on; grid on
plot(t(1:end-1), s_theta, 'b'); yline(0,'k--')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('s_\theta'); xlabel('Time (s)'); title('Pitch')

sgtitle('Sliding Variable Evolution')

figure('Color','w','Name','Yaw States')
subplot(3,1,1); hold on; grid on
plot(t(1:end-1), rad2deg(psicmd), 'r--')
plot(t, rad2deg(psi_arr), 'b')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('\psi (deg)'); legend('\psi_{cmd}','Actual'); title('Yaw State Evolution')

subplot(3,1,2); hold on; grid on
plot(t(1:end-1), rad2deg(psicmd_dot), 'r--')
plot(t, rad2deg(psidot_arr), 'b')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('\psi_{dot} (deg/s)'); legend('\psi_{dot,cmd}','Actual')

subplot(3,1,3); hold on; grid on
plot(t(1:end-1), rad2deg(psicmd_ddot), 'r--')
plot(t, rad2deg(psiddot_arr), 'b')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('\psi_{ddot} (deg/s²)'); xlabel('Time (s)'); legend('\psi_{ddot,cmd}','Actual')

figure('Color','w','Name','3D Trajectory')
plot3(x_arr(:,7), x_arr(:,8), x_arr(:,9), 'b'); hold on
plot3(xd, yd, zcmd, 'r--')
plot3(x_arr(1,7), x_arr(1,8), x_arr(1,9), 'go', 'MarkerSize',8,'MarkerFaceColor','g')
plot3(x_arr(end,7), x_arr(end,8), x_arr(end,9), 'rs', 'MarkerSize',8,'MarkerFaceColor','r')
grid on; xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)')
legend('Actual','Reference','Start','End'); title('3D Trajectory'); view(45,30)

% =========================================================
% LOCAL FUNCTIONS
% =========================================================

function mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
    if fault_enable ~= 1; return; end
    ax = gca; hold(ax, 'on');
    if fault_mode == 1
        xline(0, 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    elseif fault_mode == 2
        yl = ylim(ax);
        patch(ax, [fault_start, fault_end, fault_end, fault_start], ...
              [yl(1), yl(1), yl(2), yl(2)], [1, 0.7, 0.7], ...
              'FaceAlpha', 0.25, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        xline(fault_start, 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
        xline(fault_end,   'k:',  'LineWidth', 1.2, 'HandleVisibility', 'off');
        ylim(ax, yl);
    end
    hold(ax, 'off');
end

function draw_boundary(ax, t_vec, epsilon, color)
    if nargin < 4; color = [0.1 0.6 0.1]; end
    hold(ax, 'on');
    yline(ax,  epsilon, '--', 'Color', color, 'LineWidth', 1.0, 'HandleVisibility','off');
    yline(ax, -epsilon, '--', 'Color', color, 'LineWidth', 1.0, 'HandleVisibility','off');
    patch(ax, [t_vec(1) t_vec(end) t_vec(end) t_vec(1)], ...
              [epsilon, epsilon, -epsilon, -epsilon], ...
              color, 'FaceAlpha', 0.08, 'EdgeColor', 'none', 'HandleVisibility','off');
    hold(ax, 'off');
end

function mark_convergence(ax, t_vec, err_vec, epsilon)
    inside  = abs(err_vec) <= epsilon;
    entries = find(inside & [false, ~inside(1:end-1)]);
    if isempty(entries); return; end
    hold(ax, 'on');
    plot(ax, t_vec(entries), err_vec(entries), 'o', ...
        'MarkerSize', 7, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k', ...
        'HandleVisibility', 'off');
    hold(ax, 'off');
end