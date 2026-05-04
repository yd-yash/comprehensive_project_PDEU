% added \psi, \dot{\psi} and \ddot{\psi} plots
% added a boundary on error plots to observe convergence
% added a marking on error plots which graph enters the boundary (i.e.
% converges)

clc
clear all
% close all

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

% Mixing matrix  — X-configuration, arm projected length l/sqrt(2)
l_eff = l / sqrt(2);
A_mix = [ b,           b,           b,           b;
         -b * l_eff,  -b * l_eff,   b * l_eff,   b * l_eff;
          b * l_eff,  -b * l_eff,  -b * l_eff,   b * l_eff;
         -d,           d,          -d,            d];

% fault parameters
fault_enable = 1;   % 0 = healthy (all alphas = 1)
                    % 1 = fault active

fault_mode   = 1;   % 1 = pre-existing fault (entire simulation, from t = 0)
                    % 2 = inject during [fault_start, fault_end]

fault_start  = 25;   % fault onset time  (s)  — used only when fault_mode = 2
fault_end    = 100;  % fault recovery time (s) — used only when fault_mode = 2

alpha1 = 0.5;       % rotor 1 (M1, front-right) effectiveness  [0, 1]
alpha2 = 1.0;       % rotor 2 (M2, rear-left)   effectiveness
alpha3 = 1.0;       % rotor 3 (M3, rear-right)  effectiveness
alpha4 = 1.0;       % rotor 4 (M4, front-left)  effectiveness

% phi theta psi phi_dot theta_dot psi_dot x y z x_dot y_dot z_dot  +12 integrals
x_0 = zeros(1, 24);
tau = 0.01;
t   = 0 : tau : 100;
N   = length(t);
x_k = x_0;
x_arr = x_0;

% Initialise attitude command memory (derivative computation)
phicmd_old     = 0;
phicmd_dot_old = 0;
thetacmd_old     = 0;
thetacmd_dot_old = 0;

% Control gain for velocity and acceleration
kpx = 0.7;
kpy = 0.7;
kdx = 0.5;
kdy = 0.5;

% Sliding surface parameters
kp = 0.2;
kd = 0.05;
ki = 0.05;

% Super-twisting gains
k1   = 0.2;    k2   = 0.1;
k1_x = 0.1;   k2_x = 0.35;
k1_y = 0.1;   k2_y = 0.35;

% Logging arrays for fault / rotor signals
alpha_log   = ones(N-1, 4);     % per-rotor effectiveness at each step
T_log       = zeros(N-1, 4);    % per-rotor thrust (N)
T_total_log = zeros(N-1, 1);    % total thrust (N)
U_cmd_log   = zeros(N-1, 4);    % virtual inputs commanded by ST-SMC
U_act_log   = zeros(N-1, 4);    % virtual inputs actually delivered (post-fault)

fprintf('Running simulation...\n');

for i = 1 : N-1

    % reference trajectories
    zcmd(i)      =  5 + 0*cos(1*t(i));
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

    kpz = 0.02;
    kdz = 0.05;
    kiz = 0.00001;

    sz    = kpz*ez + kdz*ezdot + kiz*x_k(14);
    s_z(i) = sz;

    u1_cmd = (m / (kdz*cos(x_k(1))*cos(x_k(2)))) * ...
             (kdz*g + kd*zcmd_ddot(i) + kpz*ezdot + kiz*ez ...
              + k1*sqrt(abs(sz))*sign(sz) + k2*I_u1);

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

    kp_x = 0.7;
    kd_x = 0.5;
    ki_x = 0.5;

    sx    = kp_x*ex + kd_x*exdot + ki_x*x_k(22);
    s_x(i) = sx;
    
    uxdash = (m/kd_x) * (kd_x*xd_ddot + kp_x*exdot + ki_x*ex ...
              + k1_x*sqrt(abs(sx))*sign(sx) + k2_x*I_ux);
    ux = f_sat(uxdash, -1, 1) / u1_cmd;
    u_x(i) = ux;

    % uy
    ey    = yd(i) - x_k(8);
    e_y(i) = ey;
    eydot = yd_dot - x_k(11);
    I_uy  = x_k(23);

    kp_y = 0.7;
    kd_y = 0.5;
    ki_y = 0.5;

    sy    = kp_y*ey + kd_y*eydot + ki_y*x_k(24);
    s_y(i) = sy;
    uydash = (m/kd_y) * (kd_y*yd_ddot + kp_y*eydot + ki_y*ey ...
              + k1_y*sqrt(abs(sy))*sign(sy) + k2_y*I_uy);
    uy = f_sat(uydash, -1, 1) / u1_cmd;
    u_y(i) = uy;

    % attitude commands
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

    % u2
    ephi = normalise(phicmd(i) - x_k(1));
    e2(i) = ephi;
    ephidot = normalise(phicmd_dot(i) - x_k(4));
    I_u2  = x_k(17);
    sphi  = kp*ephi + kd*ephidot + ki*x_k(18);
    s_phi(i) = sphi;
    u2_cmd = -(Jy-Jz)*x_k(5)*x_k(6) + Jx*(kp/kd)*ephidot + Jx*phicmd_ddot(i) ...
              + Jx*(ki/kd)*ephi + (k1/kd)*sqrt(abs(sphi))*sign(sphi) + (k2/kd)*I_u2;

    % u3
    etheta = normalise(thetacmd(i) - x_k(2));
    e3(i)  = etheta;
    ethetadot = normalise(thetacmd_dot(i) - x_k(5));
    I_u3   = x_k(15);
    stheta = kp*etheta + kd*ethetadot + ki*x_k(16);
    s_theta(i) = stheta;
    u3_cmd = -(Jz-Jx)*x_k(4)*x_k(6) + Jy*(kp/kd)*ethetadot + Jy*thetacmd_ddot(i) ...
              + Jy*(ki/kd)*etheta + (k1/kd)*sqrt(abs(stheta))*sign(stheta) + (k2/kd)*I_u3;

    % u4
    epsi = normalise(psicmd(i) - x_k(3));
    e4(i) = epsi;
    epsidot = normalise(psicmd_dot(i) - x_k(6));
    I_u4 = x_k(19);
    spsi = kp*epsi + kd*epsidot + ki*x_k(20);
    s_psi(i) = spsi;
    u4_cmd = -(Jx-Jy)*x_k(4)*x_k(5) + Jz*(kp/kd)*epsidot + Jz*psicmd_ddot(i) ...
              + Jz*(ki/kd)*epsi + (k1/kd)*sqrt(abs(spsi))*sign(spsi) + (k2/kd)*I_u4;

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

% position states
figure('Color','w','Name','Position States')

subplot(3,1,1); 
hold on; 
grid on
plot(t(1:end-1), xd, 'r--')
plot(t, x_arr(:,7), 'b')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('x (m)'); 
legend('Reference','Actual');
title('Position Tracking')

subplot(3,1,2); 
hold on; 
grid on
plot(t(1:end-1), yd, 'r--')
plot(t, x_arr(:,8), 'b')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('y (m)');
legend('Reference','Actual')

subplot(3,1,3);
hold on; 
grid on
plot(t(1:end-1), zcmd, 'r--')
plot(t, x_arr(:,9), 'b')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('z (m)'); 
xlabel('Time (s)');
legend('Reference','Actual')

% attitude states
figure('Color','w','Name','Attitude States')

subplot(3,1,1); 
hold on; 
grid on
plot(t(1:end-1), rad2deg(phicmd), 'r--')
plot(t, rad2deg(x_arr(:,1)),'b')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('\phi (deg)'); 
legend('\phi_{cmd}','Actual'); 
title('Attitude Tracking')

subplot(3,1,2); 
hold on; 
grid on
plot(t(1:end-1), rad2deg(thetacmd), 'r--')
plot(t, rad2deg(x_arr(:,2)),'b')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('\theta (deg)');
legend('\theta_{cmd}','Actual')

subplot(3,1,3);
hold on; 
grid on
plot(t(1:end-1), rad2deg(psicmd), 'r--')
plot(t, rad2deg(x_arr(:,3)),'b')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('\psi (deg)');
xlabel('Time (s)');
legend('\psi_{cmd}','Actual')

% error with band for convergence
eps_pos   = 0.05;
eps_att   = 0.5*pi/180;
eps_yaw   = 1.0*pi/180;
min_dwell = 3.0;          % seconds — tune this per your simulation length

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

% control inputs
figure('Color','w','Name','Control Inputs')

subplot(4,1,1); 
hold on; 
grid on
plot(t(1:end-1), U_cmd_log(:,1), 'r--')
plot(t(1:end-1), U_act_log(:,1), 'b')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('U_1 (N)'); 
legend('Commanded','Delivered'); 
title('Virtual Control Inputs')

subplot(4,1,2); 
hold on; 
grid on
plot(t(1:end-1), U_cmd_log(:,2), 'r--')
plot(t(1:end-1), U_act_log(:,2), 'b')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('U_2 (N·m)'); 
legend('Commanded','Delivered')

subplot(4,1,3); 
hold on; 
grid on
plot(t(1:end-1), U_cmd_log(:,3), 'r--')
plot(t(1:end-1), U_act_log(:,3), 'b')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('U_3 (N·m)'); 
legend('Commanded','Delivered')

subplot(4,1,4); 
hold on; 
grid on
plot(t(1:end-1), U_cmd_log(:,4), 'r--')
plot(t(1:end-1), U_act_log(:,4), 'b')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('U_4 (N·m)'); xlabel('Time (s)'); 
legend('Commanded','Delivered')

% actuator effectiveness
figure('Color','w','Name','Actuator Effectiveness')
alpha_labels = {'\alpha_1 (M1, front-right)', '\alpha_2 (M2, rear-left)', ...
                '\alpha_3 (M3, rear-right)',   '\alpha_4 (M4, front-left)'};
for k = 1:4
    subplot(4,1,k); 
    hold on; 
    grid on
    plot(t(1:end-1), alpha_log(:,k), 'k')
    mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
    ylabel(alpha_labels{k}); ylim([-0.1, 1.2])
    if k == 4; xlabel('Time (s)'); end
end

sgtitle('Actuator Effectiveness Factors \alpha_i')

% rotor thrusts
figure('Color','w','Name','Rotor Thrusts')
colors   = {'b','r','m','g'};
T_labels = {'T_1 (N)','T_2 (N)','T_3 (N)','T_4 (N)'};
for k = 1:4
    subplot(5,1,k); 
    hold on; 
    grid on
    plot(t(1:end-1), T_log(:,k), colors{k})
    mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
    ylabel(T_labels{k})
end
subplot(5,1,5);
hold on; 
grid on
plot(t(1:end-1), T_total_log, 'k')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('T_{total} (N)'); xlabel('Time (s)')
sgtitle('Individual Rotor Thrusts and Total Thrust')

% sliding variable evolution
figure('Color','w','Name','Sliding Variables')

subplot(3,2,1); 
hold on; 
grid on
plot(t(1:end-1), s_z, 'b');
yline(0,'k--')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('s_z');
title('Altitude')

subplot(3,2,2); hold on; grid on
plot(t(1:end-1), s_psi, 'b'); yline(0,'k--')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('s_\psi'); title('Yaw')

subplot(3,2,3); 
hold on; 
grid on
plot(t(1:end-1), s_x, 'b'); 
yline(0,'k--')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('s_x'); 
title('X Position')

subplot(3,2,4); 
hold on; 
grid on
plot(t(1:end-1), s_y, 'b');
yline(0,'k--')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('s_y'); 
title('Y Position')

subplot(3,2,5); hold on; grid on
plot(t(1:end-1), s_phi, 'b');
yline(0,'k--')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('s_\phi'); 
xlabel('Time (s)');
title('Roll')

subplot(3,2,6); hold on; grid on
plot(t(1:end-1), s_theta, 'b');
yline(0,'k--')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('s_\theta'); 
xlabel('Time (s)');
title('Pitch')

sgtitle('Sliding Variable Evolution')

% yaw states: psi, psi_dot, psi_ddot
figure('Color','w','Name','Yaw States')

subplot(3,1,1);
hold on; grid on
plot(t(1:end-1), rad2deg(psicmd), 'r--')
plot(t, rad2deg(psi_arr), 'b')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('\psi (deg)')
legend('\psi_{cmd}','Actual')
title('Yaw State Evolution')

subplot(3,1,2);
hold on; grid on
plot(t(1:end-1), rad2deg(psicmd_dot), 'r--')
plot(t, rad2deg(psidot_arr), 'b')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('\psi_{dot} (deg/s)')
legend('\psi_{dot,cmd}','Actual')

subplot(3,1,3);
hold on; grid on
plot(t(1:end-1), rad2deg(psicmd_ddot), 'r--')
plot(t, rad2deg(psiddot_arr), 'b')
mark_fault(t, fault_enable, fault_mode, fault_start, fault_end)
ylabel('\psi_{ddot} (deg/s²)')
xlabel('Time (s)')
legend('\psi_{ddot,cmd}','Actual')

% 3D trajectory
figure('Color','w','Name','3D Trajectory')
plot3(x_arr(:,7), x_arr(:,8), x_arr(:,9), 'b'); 
hold on
plot3(xd, yd, zcmd, 'r--')
plot3(x_arr(1,7), x_arr(1,8), x_arr(1,9), 'go', 'MarkerSize',8,'MarkerFaceColor','g')
plot3(x_arr(end,7), x_arr(end,8), x_arr(end,9), 'rs', 'MarkerSize',8,'MarkerFaceColor','r')
grid on; xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)')
legend('Actual','Reference','Start','End'); title('3D Trajectory'); view(45,30)

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
    % if nargin < 5; color = [0.8 0 0]; end
    % idx = find(abs(err_vec) <= epsilon, 1, 'first');
    entries = find(inside & [false, ~inside(1:end-1)]);
    % if isempty(idx); return; end
    if isempty(entries); return; end
    hold(ax, 'on');
    % plot(ax, t_vec(idx), err_vec(idx), 'o', ...
    plot(ax, t_vec(entries), err_vec(entries), 'o', ...
        'MarkerSize', 7, ...
        'MarkerFaceColor', 'k', ...
        'MarkerEdgeColor', 'k', ...
        'HandleVisibility', 'off');
    hold(ax, 'off');
end

% function t_conv = find_convergence_time(t_vec, err_vec, epsilon, min_dwell)
% % Returns the time at which err_vec enters [-epsilon, +epsilon] and
% % stays there for at least min_dwell seconds continuously.
% % Returns NaN if convergence never occurs.
%     if nargin < 4; min_dwell = 2.0; end   % seconds inside band before declared converged
%     dt       = t_vec(2) - t_vec(1);
%     n_dwell  = round(min_dwell / dt);
%     inside   = abs(err_vec) <= epsilon;
%     t_conv   = NaN;
%     for k = 1 : length(inside) - n_dwell
%         if all(inside(k : k + n_dwell))
%             t_conv = t_vec(k);
%             return
%         end
%     end
% end
% 
% function mark_convergence(ax, t_vec, err_vec, epsilon, min_dwell, color)
%     if nargin < 5; min_dwell = 2.0; end
%     if nargin < 6; color = [0.8 0 0]; end
%     t_conv = find_convergence_time(t_vec, err_vec, epsilon, min_dwell);
%     if isnan(t_conv); return; end
%     idx    = find(t_vec >= t_conv, 1);
%     e_val  = err_vec(idx);
%     hold(ax, 'on');
%     plot(ax, t_conv, e_val, 'o', ...
%         'MarkerSize', 7, ...
%         'MarkerFaceColor', color, ...
%         'MarkerEdgeColor', color, ...
%         'HandleVisibility', 'off');
%     xline(ax, t_conv, ':', 'Color', color, 'LineWidth', 1.0, 'HandleVisibility','off');
%     % text(ax, t_conv, e_val*1.6, sprintf('t=%.1fs', t_conv), ...
%     %     'FontSize', 7, 'Color', color, 'HorizontalAlignment','left');
%     hold(ax, 'off');
% end