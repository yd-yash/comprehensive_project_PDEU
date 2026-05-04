clc; clear; close all;

p = parameters();
p = fault_params(p);
p = smc_params(p);
p = reference_trajectory(p);

dt = 0.01; % 100 Hz
Tf = 40;
x0 = zeros(12, 1);

t_grid = (0 : dt : Tf)';
N      = length(t_grid);

opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8, 'MaxStep', dt);

X_log       = zeros(N, 12);
X_log(1, :) = x0';

x       = x0;
Uxy     = [0; 0];
Omega_r = 0;

cmd_mem.phi_cmd       = 0;
cmd_mem.phi_cmd_dot   = 0;
cmd_mem.theta_cmd     = 0;
cmd_mem.theta_cmd_dot = 0;

fprintf('Running simulation...\n');

for i = 1 : N-1

    [U_cmd, Uxy, cmd_mem] = smc_controller(t_grid(i), x, Uxy, Omega_r, cmd_mem, dt, p);

    alpha_vec = fault_model(t_grid(i), p);

    [~, Omega_r, ~, ~, U_actual] = control_mixing(U_cmd, alpha_vec, p);

    dist = disturbance_model(t_grid(i));

    odefun = @(t_ode, x_ode) quad_dynamics(t_ode, x_ode, U_actual, dist, Omega_r, p);
    [~, x_hist] = ode45(odefun, [t_grid(i), t_grid(i+1)], x, opts);

    x = x_hist(end, :)';
    X_log(i+1, :) = x';

end

fprintf('Simulation complete.\n');

logs = reconstruct_signals(t_grid, X_log, p);
performance_metrics(t_grid, logs.ex, logs.ey, logs.ez, logs.ephi, logs.etheta, logs.epsi);

% quadrotor_animate(t_grid, X_log, p);

plot_results(t_grid, X_log, logs, p);

