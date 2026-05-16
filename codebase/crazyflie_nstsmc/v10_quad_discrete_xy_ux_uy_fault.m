% extension of v6
% ma'am's derivation
% changed the way of writing (e_i_dot)^(2-gamma) to
% abs((e_i_dot)^(2-gamma)) * sign (e_i_dot)
clc
clear all
close all

global Jy Jx Jz m g ez sz u1 ephi sphi u2 etheta stheta u3 ...
       epsi spsi u4 ex sx ey sy

u_1 = [];   u_2 = [];   u_3 = [];   u_4 = [];   u_x = [];   u_y = [];
e1  = [];   e2  = [];   e3  = [];   e4  = [];   e_x = [];   e_y = [];
s_z = [];   s_phi = [];  s_theta = [];  s_psi = [];

Jx = 0.029125;      % MOI about X axis (kg·m²)
Jy = 0.029125;      % MOI about Y axis (kg·m²)
Jz = 0.055225;      % MOI about Z axis (kg·m²)
m  = 1.5;           % Mass (kg)
g  = 9.81;          % Gravity (m/s²)
l  = 0.23;          % Arm length, centre to motor (m)

% Thrust and drag coefficients
b = 3.13e-5;        % Thrust coefficient  (N·s²/rad²)
d = 7.5e-5;         % Drag/torque coefficient (N·m·s²/rad²)

% Rotor thrust saturation
T_min = 0;                      % Minimum rotor thrust (N)
T_max = 2 * (m * g / 4);        % Maximum rotor thrust (N)

% NTSM parameters
p = 5; q = 3;
gamma = p/q;

beta_z = 3; % previously all beta were one
beta_phi = 3;
beta_theta = 2;
beta_psi = 1;
beta_x = 2;
beta_y = 2;

mu_min = 0.05;   % singularity protection

% Mixing matrix  — X-configuration, arm projected length l/sqrt(2)
l_eff = l / sqrt(2);
A_mix = [ b,           b,           b,           b;
         -b * l_eff,  -b * l_eff,   b * l_eff,   b * l_eff;
          b * l_eff,  -b * l_eff,  -b * l_eff,   b * l_eff;
         -d,           d,          -d,            d];

% fault parameters
fault_enable = 0;   % 0 = healthy (all alphas = 1)
                    % 1 = fault active

fault_mode   = 2;   % 1 = pre-existing fault (entire simulation, from t = 0)
                    % 2 = inject during [fault_start, fault_end]

fault_start  = 10;   % fault onset time  (s)  — used only when fault_mode = 2
fault_end    = 50;  % fault recovery time (s) — used only when fault_mode = 2

alpha1 = 0.5;       % rotor 1 (M1, front-right) effectiveness  [0, 1]
alpha2 = 0.5;       % rotor 2 (M2, rear-left)   effectiveness
alpha3 = 0.5;       % rotor 3 (M3, rear-right)  effectiveness
alpha4 = 0.5;       % rotor 4 (M4, front-left)  effectiveness

% phi theta psi phi_dot theta_dot psi_dot x y z x_dot y_dot z_dot  +12 integrals
x_0 = zeros(1, 24);
tau = 0.01;
t   = 0 : tau : 200;
N   = length(t);
x_k = x_0;
x_arr = x_0;

% Initialise attitude command memory (derivative computation)
% phicmd_old     = 0;
% phicmd_dot_old = 0;
% thetacmd_old     = 0;
% thetacmd_dot_old = 0;
phicmd_old         = 0;
phicmd_dot_old     = 0;
thetacmd_old       = 0;
thetacmd_dot_old   = 0;
phicmd_dot_filt    = 0;
phicmd_ddot_filt   = 0;
thetacmd_dot_filt  = 0;
thetacmd_ddot_filt = 0;
alpha_f_phi   = 0.5;   % phi filter — already working well
alpha_f_theta = 0.5;   % theta needs stronger filtering due to cos(phicmd) division

% Control gain for velocity and acceleration
kpx = 0.90; %0.7;
kpy = 0.85; %0.7;
kdx = 0.58; %0.5;
kdy = 0.60; %0.5;


% Super-twisting gains
% % altitude
% k1_z = 1.0;   k2_z = 1.5;
% 
% % roll/pitch
% k1_att = 2.0; k2_att = 3.0;
% 
% % yaw
% k1_psi = 0.5; k2_psi = 0.8;
% 
% k1_x = 0.1;   k2_x = 0.35;
% k1_y = 0.1;   k2_y = 0.35;
k1_z   = 3.0;    k2_z   = 2.5;
k1_att = 2.6;    k2_att = 2.0;
k1_psi = 0.5;    k2_psi = 0.6;
k1_x   = 0.45;   k2_x   = 0.42;
k1_y   = 0.35;   k2_y   = 0.40;


% Logging arrays for fault / rotor signals
alpha_log   = ones(N-1, 4);     % per-rotor effectiveness at each step
T_log       = zeros(N-1, 4);    % per-rotor thrust (N)
T_total_log = zeros(N-1, 1);    % total thrust (N)
U_cmd_log   = zeros(N-1, 4);    % virtual inputs commanded by ST-SMC
U_act_log   = zeros(N-1, 4);    % virtual inputs actually delivered (post-fault)

fprintf('Running simulation...\n');

for i = 1 : N-1

    % reference trajectories
    zcmd(i)      =  10 + 0*cos(1*t(i));
    zcmd_dot(i)  = -0*sin(1*t(i));
    zcmd_ddot(i) = -0*cos(1*t(i));

    psicmd(i)      =  0*pi/180*cos(0.2*t(i));
    psicmd_dot(i)  = -0*0.2*pi/180*sin(0.2*t(i));
    psicmd_ddot(i) = -0*0.04*pi/180*cos(0.2*t(i));

    xd(i) = 2*cos(0.1*t(i));
    yd(i) = 2*sin(0.2*t(i));

    % u1
    ez    = zcmd(i) - x_k(9);
    e1(i) = ez;
    ezdot = zcmd_dot(i) - x_k(12);
    I_u1  = x_k(13);

  

    % NTSM surface
    % sz = ez + (1/beta_z^(gamma)) * ezdot^gamma;
    sz = ez + (1/beta_z^(gamma)) * (abs(ezdot)^gamma) * sign(ezdot);
    s_z(i) = sz;
    
    u1_cmd = (m / (cos(x_k(1)) * cos(x_k(2)))) * (zcmd_ddot(i) + g + beta_z^gamma*(1/gamma)*(abs(ezdot)^(2-gamma)) * sign(ezdot) + k1_z * sqrt(abs(sz)) * sign(sz) + k2_z * I_u1);
  
    % xy - vel, acc references
    xd_dot  = kpx * (xd(i) - x_k(7));
    yd_dot  = kpy * (yd(i) - x_k(8));
    xd_ddot = kdx * (xd_dot - x_k(10));
    yd_ddot = kdy * (yd_dot - x_k(11));

    % ux
    ex    = xd(i) - x_k(7);
    e_x(i) = ex;
    exdot = xd_dot - x_k(10);
    I_ux  = x_k(21);

    % sx = ex + (1/beta_x^(gamma)) *(exdot)^gamma;
    sx = ex + (1/beta_x^(gamma)) * (abs(exdot)^gamma) * sign(exdot);
    s_x(i) = sx;
    
    uxdash = m * (xd_ddot + beta_x^gamma * (1/gamma) * (abs(exdot)^(2-gamma)) * sign(exdot) + k1_x * sqrt(abs(sx)) * sign(sx) + k2_x * I_ux);

    % ux = f_sat(uxdash / max(u1_cmd, 1), -0.5, 0.5);
    ux = f_sat(uxdash / max(abs(u1_cmd), m*g*0.3), -0.5, 0.5);
    u_x(i) = ux;
 

    % uy
    ey    = yd(i) - x_k(8);
    e_y(i) = ey;
    eydot = yd_dot - x_k(11);
    I_uy  = x_k(23);

    % sy = ey + (1/beta_y^(gamma)) * (eydot)^gamma;
    sy = ey + (1/beta_y^(gamma)) * (abs(eydot)^gamma) * sign(eydot);
    s_y(i) = sy;
    
    uydash = m * (yd_ddot + beta_y^gamma * (1/gamma) * (abs(eydot)^(2-gamma)) * sign(eydot) + k1_y * sqrt(abs(sy)) * sign(sy) + k2_y * I_uy);

    % uy = f_sat(uydash, -1, 1) / u1_cmd;
    uy = f_sat(uydash, -1, 1) / max(abs(u1_cmd), m*g*0.3);
    u_y(i) = uy;

    % attitude commands
    phicmd(i)      = asin_norm(ux*sin(psicmd(i)) - uy*cos(psicmd(i)));
    % phicmd_dot(i)  = normalise(phicmd(i) - phicmd_old) / tau;
    % phicmd_old     = phicmd(i);
    % phicmd_ddot(i) = (phicmd_dot(i) - phicmd_dot_old) / tau;
    % phicmd_dot_old = phicmd_dot(i);
    raw_phicmd_dot    = normalise(phicmd(i) - phicmd_old) / tau;
    phicmd_dot_filt   = alpha_f_phi*raw_phicmd_dot   + (1-alpha_f_phi)*phicmd_dot_filt;
    phicmd_dot(i)     = phicmd_dot_filt;
    phicmd_old        = phicmd(i);
    
    raw_phicmd_ddot   = (phicmd_dot_filt - phicmd_dot_old) / tau;
    phicmd_ddot_filt  = alpha_f_phi*raw_phicmd_ddot  + (1-alpha_f_phi)*phicmd_ddot_filt;
    phicmd_ddot(i)    = phicmd_ddot_filt;
    phicmd_dot_old    = phicmd_dot_filt;


    thetacmd(i)      = asin_norm((ux*cos(psicmd(i)) + uy*sin(psicmd(i))) / cos(phicmd(i)));
    % thetacmd_dot(i)  = normalise(thetacmd(i) - thetacmd_old) / tau;
    % thetacmd_old     = thetacmd(i);
    % thetacmd_ddot(i) = (thetacmd_dot(i) - thetacmd_dot_old) / tau;
    % thetacmd_dot_old = thetacmd_dot(i);
    raw_thetacmd_dot   = normalise(thetacmd(i) - thetacmd_old) / tau;
    thetacmd_dot_filt  = alpha_f_theta*raw_thetacmd_dot  + (1-alpha_f_theta)*thetacmd_dot_filt;
    thetacmd_dot(i)    = thetacmd_dot_filt;
    thetacmd_old       = thetacmd(i);
    
    raw_thetacmd_ddot  = (thetacmd_dot_filt - thetacmd_dot_old) / tau;
    thetacmd_ddot_filt = alpha_f_theta*raw_thetacmd_ddot + (1-alpha_f_theta)*thetacmd_ddot_filt;
    thetacmd_ddot(i)   = thetacmd_ddot_filt;
    thetacmd_dot_old   = thetacmd_dot_filt;

    % u2
    ephi = normalise(phicmd(i) - x_k(1));
    e2(i) = ephi;
    ephidot = normalise(phicmd_dot(i) - x_k(4));
    I_u2  = x_k(17);

    % sphi = ephi + (1/beta_phi^gamma) * ephidot^gamma;
    sphi = ephi + (1/beta_phi^gamma) * (abs(ephidot)^gamma) * sign(ephidot);
    s_phi(i) = sphi;
    
    u2_cmd = Jx * (phicmd_ddot(i) - ((Jy-Jz)/Jx)*x_k(5)*x_k(6) + beta_phi^gamma * (1/gamma) * (abs(ephidot)^(2-gamma)) * sign(ephidot) + k1_att * sqrt(abs(sphi)) * sign(sphi) + k2_att * I_u2);

    % u3
    etheta = normalise(thetacmd(i) - x_k(2));
    e3(i)  = etheta;
    ethetadot = normalise(thetacmd_dot(i) - x_k(5));
    I_u3   = x_k(15);

    % stheta = etheta + (1/beta_theta^gamma)*(ethetadot)^gamma;
    stheta = etheta + (1/beta_theta^gamma) * (abs(ethetadot)^gamma) * sign(ethetadot);
    s_theta(i) = stheta;

    u3_cmd = Jy * (thetacmd_ddot(i) - ((Jz-Jx)/Jy)*x_k(4)*x_k(6) + beta_theta^gamma * (1/gamma) * (abs(ethetadot)^(2-gamma)) * sign(ethetadot) + k1_att * sqrt(abs(stheta)) * sign(stheta) + k2_att * I_u3);

    % u4
    epsi = normalise(psicmd(i) - x_k(3));
    e4(i) = epsi;
    epsidot = normalise(psicmd_dot(i) - x_k(6));
    I_u4 = x_k(19);
   
    % spsi = epsi + (1/beta_psi^gamma)*(epsidot)^gamma;
    spsi = epsi + (1/beta_psi^gamma) * (abs(epsidot)^gamma) * sign(epsidot);
    s_psi(i) = spsi;
    
    u4_cmd = Jz * (psicmd_ddot(i) - ((Jx-Jy)/Jz)*x_k(4)*x_k(5) + beta_psi^gamma * (1/gamma) * (abs(epsidot)^(2-gamma)) * sign(epsidot) + k1_psi * sqrt(abs(spsi)) * sign(spsi) + k2_psi * I_u4);
    
    % Collect commanded virtual inputs
    U_cmd = [u1_cmd; u2_cmd; u3_cmd; u4_cmd];

    % fault model
    alpha_vec = ones(4, 1);  

    if fault_enable == 1
        fault_active = false;

        if fault_mode == 1
            fault_active = true;

        elseif fault_mode == 2
            if fault_end > fault_start
                fault_active = (t(i) >= fault_start && t(i) <= fault_end);
            else
                warning('quad_discrete_xy_ux_uy_fault: fault_end must be > fault_start. No fault applied.');
            end
        else
            warning('quad_discrete_xy_ux_uy_fault: unknown fault_mode = %d. No fault applied.', fault_mode);
        end

        if fault_active
            alpha_vec = [alpha1; alpha2; alpha3; alpha4];
        end
    end

    % Fault effectiveness direct scaling on virtual inputs
    alpha_eff = zeros(4, 1);
    for ch = 1:4
        row_healthy = A_mix(ch, :);                    % healthy row weights
        row_faulty  = A_mix(ch, :) .* alpha_vec';      % faulted row weights
        healthy_sum = sum(abs(row_healthy));
        if healthy_sum > 0
            alpha_eff(ch) = sum(abs(row_faulty)) / healthy_sum;
        else
            alpha_eff(ch) = 1;
        end
    end

    % Scale virtual inputs by per-channel effectiveness
    U_actual = alpha_eff .* U_cmd;

    % Approximate per-rotor thrusts for logging (healthy nominal allocation)
    Omega2_nom = max(0, A_mix \ U_cmd);   % nominal Omega^2 per rotor
    T_nom      = b * Omega2_nom;          % nominal thrust per rotor
    T_motors   = alpha_vec .* T_nom;      % faulted thrust per rotor 
    T_motors   = max(T_min, min(T_motors, T_max));
    T_total    = sum(T_motors);

    % Log commanded vs actual virtual inputs, rotor signals
    U_cmd_log(i, :)  = U_cmd';
    U_act_log(i, :)  = U_actual';
    alpha_log(i, :)  = alpha_vec';
    T_log(i, :)      = T_motors';
    T_total_log(i)   = T_total;

    % Expose actual virtual inputs to dynamics via globals
    u1 = U_actual(1);
    u2 = U_actual(2);
    u3 = U_actual(3);
    u4 = U_actual(4);

    % Log for plots (using actual delivered inputs)
    u_1(i) = u1;
    u_2(i) = u2;
    u_3(i) = u3;
    u_4(i) = u4;

    x      = ode4(@quad_discrete_function, [t(i) t(i+1)], x_k);
    x_k    = x(end, :);
    x_arr(i+1, :) = x_k;

end

fprintf('Simulation complete.\n');

% Post-simulation: compute psi double-dot via finite difference 
psi_arr     = x_arr(:, 3);
psidot_arr  = x_arr(:, 6);
psiddot_arr = zeros(N, 1);
for i = 2 : N-1
    psiddot_arr(i) = (psidot_arr(i+1) - psidot_arr(i-1)) / (2*tau);
end
psiddot_arr(1)   = psiddot_arr(2);
psiddot_arr(end) = psiddot_arr(end-1);

% =========================================================================
% BUILD logs STRUCT AND CALL THESIS PLOTS + PERFORMANCE METRICS
% =========================================================================

% Pad reference signals (length N-1) to length N by repeating last sample
pad = @(v) [v(:); v(end)];

logs.x_ref    = pad(xd(:));
logs.y_ref    = pad(yd(:));
logs.z_ref    = pad(zcmd(:));
logs.phi_ref  = pad(phicmd(:));
logs.theta_ref = pad(thetacmd(:));
logs.psi_ref  = pad(psicmd(:));

logs.ex       = pad(xd(:)     - x_arr(1:end-1, 7));
logs.ey       = pad(yd(:)     - x_arr(1:end-1, 8));
logs.ez       = pad(zcmd(:)   - x_arr(1:end-1, 9));
logs.ephi     = pad(e2(:));
logs.etheta   = pad(e3(:));
logs.epsi     = pad(e4(:));

logs.u1_log   = pad(u_1(:));
logs.u2_log   = pad(u_2(:));
logs.u3_log   = pad(u_3(:));
logs.u4_log   = pad(u_4(:));

% sliding variables (for Fig 7)
logs.s_z      = pad(s_z(:));
logs.s_phi    = pad(s_phi(:));
logs.s_theta  = pad(s_theta(:));
logs.s_psi    = pad(s_psi(:));
logs.s_x      = pad(s_x(:));
logs.s_y      = pad(s_y(:));

% fault info (passed through for mark_fault shading)
logs.fault_enable = fault_enable;
logs.fault_mode   = fault_mode;
logs.fault_start  = fault_start;
logs.fault_end    = fault_end;
logs.alpha_log    = alpha_log;
logs.T_log        = T_log;
logs.T_total_log  = T_total_log;
logs.U_cmd_log    = U_cmd_log;
logs.U_act_log    = U_act_log;

% State matrix expected by plots_thesis: [x y z phi theta psi ...]
% x_arr columns: 1=phi,2=theta,3=psi,4=p,5=q,6=r,7=x,8=y,9=z,...
x_plot = [x_arr(:,7), x_arr(:,8), x_arr(:,9), ...   % x y z
          x_arr(:,1), x_arr(:,2), x_arr(:,3)];       % phi theta psi

% Call thesis-quality plots
plots_thesis(t(:), x_plot, logs);

% Call performance metrics
performance_metrics(t(:), logs.ex, logs.ey, logs.ez, ...
                    logs.ephi, logs.etheta, logs.epsi);

% =========================================================================
% LOCAL HELPER FUNCTIONS (unchanged from original)
% =========================================================================

function mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
    if fault_enable ~= 1
        return
    end
    ax = gca;
    hold(ax, 'on');

    if fault_mode == 1
        xline(0, 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');

    elseif fault_mode == 2
        yl = ylim(ax);
        patch(ax, ...
              [fault_start, fault_end, fault_end, fault_start], ...
              [yl(1), yl(1), yl(2), yl(2)], ...
              [1, 0.7, 0.7], ...
              'FaceAlpha', 0.25, 'EdgeColor', 'none', ...
              'HandleVisibility', 'off');
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
    inside = abs(err_vec) <= epsilon;
    entries = find(inside & [false, ~inside(1:end-1)]);
    if isempty(entries); return; end
    hold(ax, 'on');
    plot(ax, t_vec(entries), err_vec(entries), 'o', ...
        'MarkerSize', 7, ...
        'MarkerFaceColor', 'k', ...
        'MarkerEdgeColor', 'k', ...
        'HandleVisibility', 'off');
    hold(ax, 'off');
end

% =========================================================================
% THESIS PLOTS FUNCTION
% =========================================================================

function plots_thesis(t, x, logs)

    ctrl_name  = 'NTSM';
    save_dir   = pwd;
    line_w     = 1.8;
    font_size  = 12;
    fig_w      = 800;
    fig_h      = 600;

    colors.blue   = [0.122  0.467  0.706];
    colors.red    = [0.839  0.153  0.157];
    colors.green  = [0.172  0.627  0.172];
    colors.orange = [1.000  0.498  0.055];
    colors.purple = [0.580  0.404  0.741];
    colors.refclr = [0.839  0.153  0.157];

    save_fig = @(fig, fname) exportgraphics(fig, ...
        fullfile(save_dir, fname), 'Resolution', 300);

    % ------------------------------------------------------------------
    % FIGURE 1 — 3D Trajectory
    % ------------------------------------------------------------------
    fig1 = figure('Color', 'w', 'Position', [100 100 fig_w fig_h]);

    plot3(logs.x_ref, logs.y_ref, logs.z_ref, '--', ...
          'Color', colors.refclr, 'LineWidth', line_w, 'DisplayName', 'Reference');
    hold on;
    plot3(x(:,1), x(:,2), x(:,3), '-', ...
          'Color', colors.blue,   'LineWidth', line_w, 'DisplayName', [ctrl_name ' Response']);
    plot3(x(1,1),   x(1,2),   x(1,3),   'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'DisplayName', 'Start');
    plot3(x(end,1), x(end,2), x(end,3), 'ks', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', 'End');

    xlabel('x (m)', 'FontSize', font_size);
    ylabel('y (m)', 'FontSize', font_size);
    zlabel('z (m)', 'FontSize', font_size);
    title([ctrl_name ' — 3D Trajectory'], 'FontSize', font_size+1);
    legend('Location', 'best', 'FontSize', font_size-1);
    grid on; grid minor; view(45, 25);
    set(gca, 'FontSize', font_size);
    hold off;

    save_fig(fig1, [ctrl_name '_Fig1_3D_Trajectory.png']);
    fprintf('[Saved] %s_Fig1_3D_Trajectory.png\n', ctrl_name);

    % ------------------------------------------------------------------
    % FIGURE 2 — Position Tracking
    % ------------------------------------------------------------------
    fig2 = figure('Color', 'w', 'Position', [100 100 fig_w fig_h]);

    subplot(3,1,1)
    plot(t, logs.x_ref, '--', 'Color', colors.refclr, 'LineWidth', line_w, 'DisplayName', 'x_{ref}');
    hold on;
    plot(t, x(:,1), '-', 'Color', colors.blue, 'LineWidth', line_w, 'DisplayName', 'x');
    mark_fault(t, logs.fault_enable, logs.fault_mode, logs.fault_start, logs.fault_end);
    ylabel('x (m)', 'FontSize', font_size); legend('FontSize', font_size-1); grid on;
    set(gca, 'FontSize', font_size); hold off;

    subplot(3,1,2)
    plot(t, logs.y_ref, '--', 'Color', colors.refclr, 'LineWidth', line_w, 'DisplayName', 'y_{ref}');
    hold on;
    plot(t, x(:,2), '-', 'Color', colors.blue, 'LineWidth', line_w, 'DisplayName', 'y');
    mark_fault(t, logs.fault_enable, logs.fault_mode, logs.fault_start, logs.fault_end);
    ylabel('y (m)', 'FontSize', font_size); legend('FontSize', font_size-1); grid on;
    set(gca, 'FontSize', font_size); hold off;

    subplot(3,1,3)
    plot(t, logs.z_ref, '--', 'Color', colors.refclr, 'LineWidth', line_w, 'DisplayName', 'z_{ref}');
    hold on;
    plot(t, x(:,3), '-', 'Color', colors.blue, 'LineWidth', line_w, 'DisplayName', 'z');
    mark_fault(t, logs.fault_enable, logs.fault_mode, logs.fault_start, logs.fault_end);
    ylabel('z (m)', 'FontSize', font_size); xlabel('Time (s)', 'FontSize', font_size);
    legend('FontSize', font_size-1); grid on;
    set(gca, 'FontSize', font_size); hold off;

    sgtitle([ctrl_name ' — Position Tracking'], 'FontSize', font_size+1);
    save_fig(fig2, [ctrl_name '_Fig2_Position_Tracking.png']);
    fprintf('[Saved] %s_Fig2_Position_Tracking.png\n', ctrl_name);

    % ------------------------------------------------------------------
    % FIGURE 3 — Attitude Tracking
    % ------------------------------------------------------------------
    fig3 = figure('Color', 'w', 'Position', [100 100 fig_w fig_h]);

    subplot(3,1,1)
    plot(t, rad2deg(logs.phi_ref), '--', 'Color', colors.refclr, 'LineWidth', line_w, 'DisplayName', '\phi_{ref}');
    hold on;
    plot(t, rad2deg(x(:,4)), '-', 'Color', colors.blue, 'LineWidth', line_w, 'DisplayName', '\phi');
    mark_fault(t, logs.fault_enable, logs.fault_mode, logs.fault_start, logs.fault_end);
    ylabel('\phi (deg)', 'FontSize', font_size); legend('FontSize', font_size-1); grid on;
    set(gca, 'FontSize', font_size); hold off;

    subplot(3,1,2)
    plot(t, rad2deg(logs.theta_ref), '--', 'Color', colors.refclr, 'LineWidth', line_w, 'DisplayName', '\theta_{ref}');
    hold on;
    plot(t, rad2deg(x(:,5)), '-', 'Color', colors.blue, 'LineWidth', line_w, 'DisplayName', '\theta');
    mark_fault(t, logs.fault_enable, logs.fault_mode, logs.fault_start, logs.fault_end);
    ylabel('\theta (deg)', 'FontSize', font_size); legend('FontSize', font_size-1); grid on;
    set(gca, 'FontSize', font_size); hold off;

    subplot(3,1,3)
    plot(t, rad2deg(logs.psi_ref), '--', 'Color', colors.refclr, 'LineWidth', line_w, 'DisplayName', '\psi_{ref}');
    hold on;
    plot(t, rad2deg(x(:,6)), '-', 'Color', colors.blue, 'LineWidth', line_w, 'DisplayName', '\psi');
    mark_fault(t, logs.fault_enable, logs.fault_mode, logs.fault_start, logs.fault_end);
    ylabel('\psi (deg)', 'FontSize', font_size); xlabel('Time (s)', 'FontSize', font_size);
    legend('FontSize', font_size-1); grid on;
    set(gca, 'FontSize', font_size); hold off;

    sgtitle([ctrl_name ' — Attitude Tracking'], 'FontSize', font_size+1);
    save_fig(fig3, [ctrl_name '_Fig3_Attitude_Tracking.png']);
    fprintf('[Saved] %s_Fig3_Attitude_Tracking.png\n', ctrl_name);

    % ------------------------------------------------------------------
    % FIGURE 4 — Position Errors
    % ------------------------------------------------------------------
    eps_pos = 0.05;
    fig4 = figure('Color', 'w', 'Position', [100 100 fig_w fig_h]);

    subplot(3,1,1)
    plot(t, logs.ex, '-', 'Color', colors.blue, 'LineWidth', line_w);
    yline(0, 'k--', 'LineWidth', 0.8);
    draw_boundary(gca, t, eps_pos);
    mark_fault(t, logs.fault_enable, logs.fault_mode, logs.fault_start, logs.fault_end);
    ylabel('e_x (m)', 'FontSize', font_size); grid on; set(gca, 'FontSize', font_size);

    subplot(3,1,2)
    plot(t, logs.ey, '-', 'Color', colors.green, 'LineWidth', line_w);
    yline(0, 'k--', 'LineWidth', 0.8);
    draw_boundary(gca, t, eps_pos);
    mark_fault(t, logs.fault_enable, logs.fault_mode, logs.fault_start, logs.fault_end);
    ylabel('e_y (m)', 'FontSize', font_size); grid on; set(gca, 'FontSize', font_size);

    subplot(3,1,3)
    plot(t, logs.ez, '-', 'Color', colors.orange, 'LineWidth', line_w);
    yline(0, 'k--', 'LineWidth', 0.8);
    draw_boundary(gca, t, eps_pos);
    mark_fault(t, logs.fault_enable, logs.fault_mode, logs.fault_start, logs.fault_end);
    ylabel('e_z (m)', 'FontSize', font_size); xlabel('Time (s)', 'FontSize', font_size);
    grid on; set(gca, 'FontSize', font_size);

    sgtitle([ctrl_name ' — Position Errors'], 'FontSize', font_size+1);
    save_fig(fig4, [ctrl_name '_Fig4_Position_Errors.png']);
    fprintf('[Saved] %s_Fig4_Position_Errors.png\n', ctrl_name);

    % ------------------------------------------------------------------
    % FIGURE 5 — Attitude Errors
    % ------------------------------------------------------------------
    eps_att = 0.5*pi/180;
    eps_yaw = 1.0*pi/180;
    fig5 = figure('Color', 'w', 'Position', [100 100 fig_w fig_h]);

    subplot(3,1,1)
    plot(t, rad2deg(logs.ephi), '-', 'Color', colors.blue, 'LineWidth', line_w);
    yline(0, 'k--', 'LineWidth', 0.8);
    draw_boundary(gca, t, rad2deg(eps_att));
    mark_fault(t, logs.fault_enable, logs.fault_mode, logs.fault_start, logs.fault_end);
    ylabel('e_\phi (deg)', 'FontSize', font_size); grid on; set(gca, 'FontSize', font_size);

    subplot(3,1,2)
    plot(t, rad2deg(logs.etheta), '-', 'Color', colors.green, 'LineWidth', line_w);
    yline(0, 'k--', 'LineWidth', 0.8);
    draw_boundary(gca, t, rad2deg(eps_att));
    mark_fault(t, logs.fault_enable, logs.fault_mode, logs.fault_start, logs.fault_end);
    ylabel('e_\theta (deg)', 'FontSize', font_size); grid on; set(gca, 'FontSize', font_size);

    subplot(3,1,3)
    plot(t, rad2deg(logs.epsi), '-', 'Color', colors.orange, 'LineWidth', line_w);
    yline(0, 'k--', 'LineWidth', 0.8);
    draw_boundary(gca, t, rad2deg(eps_yaw));
    mark_fault(t, logs.fault_enable, logs.fault_mode, logs.fault_start, logs.fault_end);
    ylabel('e_\psi (deg)', 'FontSize', font_size); xlabel('Time (s)', 'FontSize', font_size);
    grid on; set(gca, 'FontSize', font_size);

    sgtitle([ctrl_name ' — Attitude Errors'], 'FontSize', font_size+1);
    save_fig(fig5, [ctrl_name '_Fig5_Attitude_Errors.png']);
    fprintf('[Saved] %s_Fig5_Attitude_Errors.png\n', ctrl_name);

    % ------------------------------------------------------------------
    % FIGURE 6 — Control Inputs (Commanded vs Delivered)
    % ------------------------------------------------------------------
    fig6 = figure('Color', 'w', 'Position', [100 100 fig_w fig_h]);

    t_ctrl = t(1:end-1);   % length N-1 to match U_cmd_log rows

    subplot(4,1,1)
    plot(t_ctrl, logs.U_cmd_log(:,1), '--', 'Color', colors.refclr, 'LineWidth', line_w, 'DisplayName', 'Commanded');
    hold on;
    plot(t_ctrl, logs.U_act_log(:,1), '-',  'Color', colors.blue,   'LineWidth', line_w, 'DisplayName', 'Delivered');
    mark_fault(t, logs.fault_enable, logs.fault_mode, logs.fault_start, logs.fault_end);
    ylabel('U_1 (N)',   'FontSize', font_size); legend('FontSize', font_size-1); grid on;
    set(gca, 'FontSize', font_size); hold off;

    subplot(4,1,2)
    plot(t_ctrl, logs.U_cmd_log(:,2), '--', 'Color', colors.refclr, 'LineWidth', line_w, 'DisplayName', 'Commanded');
    hold on;
    plot(t_ctrl, logs.U_act_log(:,2), '-',  'Color', colors.green,  'LineWidth', line_w, 'DisplayName', 'Delivered');
    mark_fault(t, logs.fault_enable, logs.fault_mode, logs.fault_start, logs.fault_end);
    ylabel('U_2 (N·m)', 'FontSize', font_size); legend('FontSize', font_size-1); grid on;
    set(gca, 'FontSize', font_size); hold off;

    subplot(4,1,3)
    plot(t_ctrl, logs.U_cmd_log(:,3), '--', 'Color', colors.refclr, 'LineWidth', line_w, 'DisplayName', 'Commanded');
    hold on;
    plot(t_ctrl, logs.U_act_log(:,3), '-',  'Color', colors.orange, 'LineWidth', line_w, 'DisplayName', 'Delivered');
    mark_fault(t, logs.fault_enable, logs.fault_mode, logs.fault_start, logs.fault_end);
    ylabel('U_3 (N·m)', 'FontSize', font_size); legend('FontSize', font_size-1); grid on;
    set(gca, 'FontSize', font_size); hold off;

    subplot(4,1,4)
    plot(t_ctrl, logs.U_cmd_log(:,4), '--', 'Color', colors.refclr, 'LineWidth', line_w, 'DisplayName', 'Commanded');
    hold on;
    plot(t_ctrl, logs.U_act_log(:,4), '-',  'Color', colors.purple, 'LineWidth', line_w, 'DisplayName', 'Delivered');
    mark_fault(t, logs.fault_enable, logs.fault_mode, logs.fault_start, logs.fault_end);
    ylabel('U_4 (N·m)', 'FontSize', font_size); xlabel('Time (s)', 'FontSize', font_size);
    legend('FontSize', font_size-1); grid on;
    set(gca, 'FontSize', font_size); hold off;

    sgtitle([ctrl_name ' — Control Inputs'], 'FontSize', font_size+1);
    save_fig(fig6, [ctrl_name '_Fig6_Control_Inputs.png']);
    fprintf('[Saved] %s_Fig6_Control_Inputs.png\n', ctrl_name);

    % ------------------------------------------------------------------
    % FIGURE 7 — Sliding Variables
    % ------------------------------------------------------------------
    fig7 = figure('Color', 'w', 'Position', [100 100 fig_w fig_h]);

    subplot(3,2,1)
    plot(t, logs.s_z, '-', 'Color', colors.blue, 'LineWidth', line_w);
    yline(0, 'k--', 'LineWidth', 0.8);
    mark_fault(t, logs.fault_enable, logs.fault_mode, logs.fault_start, logs.fault_end);
    ylabel('s_z', 'FontSize', font_size); title('Altitude', 'FontSize', font_size);
    grid on; set(gca, 'FontSize', font_size);

    subplot(3,2,2)
    plot(t, logs.s_psi, '-', 'Color', colors.purple, 'LineWidth', line_w);
    yline(0, 'k--', 'LineWidth', 0.8);
    mark_fault(t, logs.fault_enable, logs.fault_mode, logs.fault_start, logs.fault_end);
    ylabel('s_\psi', 'FontSize', font_size); title('Yaw', 'FontSize', font_size);
    grid on; set(gca, 'FontSize', font_size);

    subplot(3,2,3)
    plot(t, logs.s_x, '-', 'Color', colors.blue, 'LineWidth', line_w);
    yline(0, 'k--', 'LineWidth', 0.8);
    mark_fault(t, logs.fault_enable, logs.fault_mode, logs.fault_start, logs.fault_end);
    ylabel('s_x', 'FontSize', font_size); title('X Position', 'FontSize', font_size);
    grid on; set(gca, 'FontSize', font_size);

    subplot(3,2,4)
    plot(t, logs.s_y, '-', 'Color', colors.green, 'LineWidth', line_w);
    yline(0, 'k--', 'LineWidth', 0.8);
    mark_fault(t, logs.fault_enable, logs.fault_mode, logs.fault_start, logs.fault_end);
    ylabel('s_y', 'FontSize', font_size); title('Y Position', 'FontSize', font_size);
    grid on; set(gca, 'FontSize', font_size);

    subplot(3,2,5)
    plot(t, logs.s_phi, '-', 'Color', colors.orange, 'LineWidth', line_w);
    yline(0, 'k--', 'LineWidth', 0.8);
    mark_fault(t, logs.fault_enable, logs.fault_mode, logs.fault_start, logs.fault_end);
    ylabel('s_\phi', 'FontSize', font_size); xlabel('Time (s)', 'FontSize', font_size);
    title('Roll', 'FontSize', font_size); grid on; set(gca, 'FontSize', font_size);

    subplot(3,2,6)
    plot(t, logs.s_theta, '-', 'Color', colors.red, 'LineWidth', line_w);
    yline(0, 'k--', 'LineWidth', 0.8);
    mark_fault(t, logs.fault_enable, logs.fault_mode, logs.fault_start, logs.fault_end);
    ylabel('s_\theta', 'FontSize', font_size); xlabel('Time (s)', 'FontSize', font_size);
    title('Pitch', 'FontSize', font_size); grid on; set(gca, 'FontSize', font_size);

    sgtitle([ctrl_name ' — Sliding Variable Evolution'], 'FontSize', font_size+1);
    save_fig(fig7, [ctrl_name '_Fig7_Sliding_Variables.png']);
    fprintf('[Saved] %s_Fig7_Sliding_Variables.png\n', ctrl_name);

    fprintf('\n[plots_thesis] All figures saved to: %s\n', save_dir);

end

% =========================================================================
% PERFORMANCE METRICS FUNCTION
% =========================================================================

function performance_metrics(t, ex, ey, ez, ephi, etheta, epsi)

    ctrl_name = 'NTSM';

    thr_pos = 0.02 * 2.0;       % 2% of trajectory amplitude (r=2 m)
    thr_att = deg2rad(1) * 0.02; % 2% of 1 deg practical band

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

    RMSE_x = sqrt(mean(ex.^2));
    RMSE_y = sqrt(mean(ey.^2));
    RMSE_z = sqrt(mean(ez.^2));

    ss_idx    = round(0.90 * length(t)) : length(t);
    ess_x     = mean(ex(ss_idx));
    ess_y     = mean(ey(ss_idx));
    ess_z     = mean(ez(ss_idx));
    ess_phi   = mean(ephi(ss_idx));
    ess_theta = mean(etheta(ss_idx));
    ess_psi   = mean(epsi(ss_idx));

    ts_x     = settling_time(t, ex,     thr_pos);
    ts_y     = settling_time(t, ey,     thr_pos);
    ts_z     = settling_time(t, ez,     thr_pos);
    ts_phi   = settling_time(t, ephi,   thr_att);
    ts_theta = settling_time(t, etheta, thr_att);
    ts_psi   = settling_time(t, epsi,   thr_att);

    % Sliding surface reach time: first time ALL |s| < 1e-3 simultaneously
    % (uses s signals already in workspace — not passed in, so reported via logs)
    ts_slide = NaN;   % populated separately if needed

    function s = fmt_ts(ts)
        if isnan(ts)
            s = '  N/A (not settled)';
        else
            s = sprintf('  %.4f s', ts);
        end
    end

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
    fprintf('    t_reach   :  see s_z/s_phi/s_theta/s_psi plots (Fig 7)\n');

    fprintf('\n%s\n\n', sep);

    % Summary table
    sep2 = repmat('=', 1, 90);
    lin2 = repmat('-', 1, 90);

    RMSE_phi   = sqrt(mean(ephi.^2));
    RMSE_theta = sqrt(mean(etheta.^2));
    RMSE_psi   = sqrt(mean(epsi.^2));

    fprintf('  SUMMARY TABLE (%s)\n', ctrl_name);
    fprintf('%s\n', sep2);
    fprintf('  %-30s  %-10s %-10s %-10s %-10s %-10s %-10s\n', ...
            'Metric', 'x', 'y', 'z', 'phi', 'theta', 'psi');
    fprintf('%s\n', lin2);
    fprintf('  %-30s  %-10.6f %-10.6f %-10.6f %-10.6f %-10.6f %-10.6f\n', ...
            'RMSE (m / rad)', RMSE_x, RMSE_y, RMSE_z, RMSE_phi, RMSE_theta, RMSE_psi);
    fprintf('%s\n', lin2);
    fprintf('  %-30s  %-10.6f %-10.6f %-10.6f %-10.6f %-10.6f %-10.6f\n', ...
            'Steady-State Err (m / rad)', ess_x, ess_y, ess_z, ess_phi, ess_theta, ess_psi);
    fprintf('%s\n', lin2);
    fprintf('  %-30s  %-10s %-10s %-10s %-10s %-10s %-10s\n', ...
            'Settling Time (s)', ...
            strtrim(fmt_ts(ts_x)),     strtrim(fmt_ts(ts_y)),     strtrim(fmt_ts(ts_z)), ...
            strtrim(fmt_ts(ts_phi)),   strtrim(fmt_ts(ts_theta)), strtrim(fmt_ts(ts_psi)));
    fprintf('%s\n\n', sep2);

end