% =========================================================================
% reconstruct_signals_stsmc.m — Post-simulation signal reconstruction
%                                for STSMC (adds sliding surface logs)
% =========================================================================
% Identical to reconstruct_signals.m but also captures the extra fields
% that stsmc_controller.m returns:
%   u.s_phi, u.s_theta, u.s_psi, u.s_z, u.s_x, u.s_y
%
% INPUTS / OUTPUTS: same as reconstruct_signals.m, logs struct extended
%   with additional fields:
%     logs.s_phi_log, logs.s_theta_log, logs.s_psi_log
%     logs.s_z_log,   logs.s_x_log,     logs.s_y_log
% =========================================================================

function logs = reconstruct_signals_stsmc(t, x, p, ctrl_func)

    N = length(t);

    % preallocate base signals
    alpha_log        = ones(N, 1);
    u1_log           = zeros(N, 1);
    u2_log           = zeros(N, 1);
    u3_log           = zeros(N, 1);
    u4_log           = zeros(N, 1);
    T1_log           = zeros(N, 1);
    T2_log           = zeros(N, 1);
    T3_log           = zeros(N, 1);
    T4_log           = zeros(N, 1);
    total_thrust_log = zeros(N, 1);
    tau_phi_log      = zeros(N, 1);
    tau_theta_log    = zeros(N, 1);
    tau_psi_log      = zeros(N, 1);

    % preallocate sliding surfaces
    s_phi_log   = zeros(N, 1);
    s_theta_log = zeros(N, 1);
    s_psi_log   = zeros(N, 1);
    s_z_log     = zeros(N, 1);
    s_x_log     = zeros(N, 1);
    s_y_log     = zeros(N, 1);

    % reference trajectories
    x_ref   = arrayfun(p.x_des,   t);
    y_ref   = arrayfun(p.y_des,   t);
    z_ref   = arrayfun(p.z_des,   t);
    psi_ref = arrayfun(p.psi_des, t);

    phi_ref   = zeros(N, 1);
    theta_ref = zeros(N, 1);

    % position and attitude errors
    ex     = x_ref   - x(:, 1);
    ey     = y_ref   - x(:, 2);
    ez     = z_ref   - x(:, 3);
    ephi   = phi_ref   - x(:, 4);
    etheta = theta_ref - x(:, 5);
    epsi   = psi_ref   - x(:, 6);

    for i = 1:N

        alpha = inducing_fault(t(i), p);
        alpha_log(i) = alpha;

        u = ctrl_func(t(i), x(i, :)', p);

        if alpha == 0
            u.u4 = 0;
        end

        u1_log(i) = u.u1;
        u2_log(i) = u.u2;
        u3_log(i) = u.u3;
        u4_log(i) = u.u4;

        % sliding surfaces — only present for STSMC
        if isfield(u, 's_phi');   s_phi_log(i)   = u.s_phi;   end
        if isfield(u, 's_theta'); s_theta_log(i) = u.s_theta; end
        if isfield(u, 's_psi');   s_psi_log(i)   = u.s_psi;   end
        if isfield(u, 's_z');     s_z_log(i)     = u.s_z;     end
        if isfield(u, 's_x');     s_x_log(i)     = u.s_x;     end
        if isfield(u, 's_y');     s_y_log(i)     = u.s_y;     end

        alloc = control_allocation(u, alpha, p);

        T1_log(i)           = alloc.T1;
        T2_log(i)           = alloc.T2;
        T3_log(i)           = alloc.T3;
        T4_log(i)           = alloc.T4;
        total_thrust_log(i) = alloc.total_thrust;
        tau_phi_log(i)      = alloc.tau_phi;
        tau_theta_log(i)    = alloc.tau_theta;
        tau_psi_log(i)      = alloc.tau_psi;

    end

    % pack all outputs
    logs.alpha_log        = alpha_log;
    logs.u1_log           = u1_log;
    logs.u2_log           = u2_log;
    logs.u3_log           = u3_log;
    logs.u4_log           = u4_log;
    logs.T1_log           = T1_log;
    logs.T2_log           = T2_log;
    logs.T3_log           = T3_log;
    logs.T4_log           = T4_log;
    logs.total_thrust_log = total_thrust_log;
    logs.tau_phi_log      = tau_phi_log;
    logs.tau_theta_log    = tau_theta_log;
    logs.tau_psi_log      = tau_psi_log;
    logs.x_ref            = x_ref;
    logs.y_ref            = y_ref;
    logs.z_ref            = z_ref;
    logs.psi_ref          = psi_ref;
    logs.phi_ref          = phi_ref;
    logs.theta_ref        = theta_ref;
    logs.ex               = ex;
    logs.ey               = ey;
    logs.ez               = ez;
    logs.ephi             = ephi;
    logs.etheta           = etheta;
    logs.epsi             = epsi;

    % sliding surfaces
    logs.s_phi_log   = s_phi_log;
    logs.s_theta_log = s_theta_log;
    logs.s_psi_log   = s_psi_log;
    logs.s_z_log     = s_z_log;
    logs.s_x_log     = s_x_log;
    logs.s_y_log     = s_y_log;

end