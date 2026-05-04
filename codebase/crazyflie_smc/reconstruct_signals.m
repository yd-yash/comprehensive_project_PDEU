function logs = reconstruct_signals(t, X, p)

    N = length(t);
    dt = t(2) - t(1);

    % Commanded virtual inputs (raw SMC output, before fault)
    U1_cmd_log = zeros(N, 1);
    U2_cmd_log = zeros(N, 1);
    U3_cmd_log = zeros(N, 1);
    U4_cmd_log = zeros(N, 1);

    % Actual virtual inputs (after effectiveness + saturation)
    U1_actual_log = zeros(N, 1);
    U2_actual_log = zeros(N, 1);
    U3_actual_log = zeros(N, 1);
    U4_actual_log = zeros(N, 1);

    % Per-rotor effectiveness (alpha1..alpha4 over time)
    alpha_log = ones(N, 4);

    % Rotor-level signals
    Omega_log   = zeros(N, 4);
    Omega_r_log = zeros(N, 1);
    T_log       = zeros(N, 4);
    T_total_log = zeros(N, 1);

    % Sliding surfaces
    s_z_log     = zeros(N, 1);
    s_x_log     = zeros(N, 1);
    s_y_log     = zeros(N, 1);
    s_phi_log   = zeros(N, 1);
    s_theta_log = zeros(N, 1);
    s_psi_log   = zeros(N, 1);

    % Attitude commands
    phi_cmd_log   = zeros(N, 1);
    theta_cmd_log = zeros(N, 1);

    % Reference trajectories
    x_ref   = arrayfun(p.x_des,   t);
    y_ref   = arrayfun(p.y_des,   t);
    z_ref   = arrayfun(p.z_des,   t);
    psi_ref = arrayfun(p.psi_des, t);

    phi_ref   = zeros(N, 1);
    theta_ref = zeros(N, 1);

    % Errors
    ex     = x_ref   - X(:, 1);
    ey     = y_ref   - X(:, 2);
    ez     = z_ref   - X(:, 3);
    ephi   = phi_ref   - X(:, 4);
    etheta = theta_ref - X(:, 5);
    epsi   = psi_ref   - X(:, 6);

    % Replay 
    Uxy     = [0; 0];
    Omega_r = 0;

    cmd_mem.phi_cmd       = 0;
    cmd_mem.phi_cmd_dot   = 0;
    cmd_mem.theta_cmd     = 0;
    cmd_mem.theta_cmd_dot = 0;

    for i = 1:N

        % SMC nominal output
        [U_cmd, Uxy, cmd_mem, surf] = smc_controller(t(i), X(i,:)', Uxy, Omega_r, cmd_mem, dt, p);

        % Per-rotor effectiveness at this timestep
        alpha_vec = fault_model(t(i), p);

        % Control mixing: invert -> scale by alpha -> clamp -> back-compute
        [Omega, Omega_r, T_motors, T_total, U_actual] = control_mixing(U_cmd, alpha_vec, p);

        % Log commanded virtual inputs
        U1_cmd_log(i) = U_cmd(1);
        U2_cmd_log(i) = U_cmd(2);
        U3_cmd_log(i) = U_cmd(3);
        U4_cmd_log(i) = U_cmd(4);

        % Log actual delivered virtual inputs
        U1_actual_log(i) = U_actual(1);
        U2_actual_log(i) = U_actual(2);
        U3_actual_log(i) = U_actual(3);
        U4_actual_log(i) = U_actual(4);

        % Per-rotor effectiveness
        alpha_log(i, :) = alpha_vec';

        % Rotor signals
        Omega_log(i,:)  = Omega';
        Omega_r_log(i)  = Omega_r;
        T_log(i,:)      = T_motors';
        T_total_log(i)  = T_total;

        % Sliding surfaces and attitude commands
        s_z_log(i)       = surf.s_z;
        s_x_log(i)       = surf.s_x;
        s_y_log(i)       = surf.s_y;
        s_phi_log(i)     = surf.s_phi;
        s_theta_log(i)   = surf.s_theta;
        s_psi_log(i)     = surf.s_psi;
        phi_cmd_log(i)   = surf.phi_cmd;
        theta_cmd_log(i) = surf.theta_cmd;

    end

    % output struct
    logs.U1_cmd_log    = U1_cmd_log;
    logs.U2_cmd_log    = U2_cmd_log;
    logs.U3_cmd_log    = U3_cmd_log;
    logs.U4_cmd_log    = U4_cmd_log;
    logs.U1_actual_log = U1_actual_log;
    logs.U2_actual_log = U2_actual_log;
    logs.U3_actual_log = U3_actual_log;
    logs.U4_actual_log = U4_actual_log;
    logs.alpha_log     = alpha_log;
    logs.Omega_log     = Omega_log;
    logs.Omega_r_log   = Omega_r_log;
    logs.T_log         = T_log;
    logs.T_total_log   = T_total_log;
    logs.s_z           = s_z_log;
    logs.s_x           = s_x_log;
    logs.s_y           = s_y_log;
    logs.s_phi         = s_phi_log;
    logs.s_theta       = s_theta_log;
    logs.s_psi         = s_psi_log;
    logs.phi_cmd       = phi_cmd_log;
    logs.theta_cmd     = theta_cmd_log;
    logs.x_ref         = x_ref;
    logs.y_ref         = y_ref;
    logs.z_ref         = z_ref;
    logs.psi_ref       = psi_ref;
    logs.phi_ref       = phi_ref;
    logs.theta_ref     = theta_ref;
    logs.ex            = ex;
    logs.ey            = ey;
    logs.ez            = ez;
    logs.ephi          = ephi;
    logs.etheta        = etheta;
    logs.epsi          = epsi;

end