% Post-simulation signal reconstruction
% Uses the SAME controller and allocation functions as the dynamics.
% No code duplication — any change to pid_controller.m or
% control_allocation.m is automatically reflected here.
% INPUTS:
%   t         - time vector (Nx1)
%   x         - state matrix (Nx18) from ode45
%   p         - params struct
%   ctrl_func - function handle (same one passed to dynamics)
% OUTPUTS:
%   logs - struct containing all logged signals for plotting

function logs = reconstruct_signals(t, x, p, ctrl_func)

    N = length(t);

    % preallocate
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

    % reconstruct control & allocation at each timestep
    for i = 1:N

        % fault effectiveness
        alpha = inducing_fault(t(i), p);
        alpha_log(i) = alpha;

        % controller call
        u = ctrl_func(t(i), x(i, :)', p);

        % override yaw if fully failed
        if alpha == 0
            u.u4 = 0;
        end

        % log virtual control inputs
        u1_log(i) = u.u1;
        u2_log(i) = u.u2;
        u3_log(i) = u.u3;
        u4_log(i) = u.u4;

        % allocation
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

    % all logs
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

end