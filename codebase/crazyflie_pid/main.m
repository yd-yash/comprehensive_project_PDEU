% Quadrotor 3D Simulation - Main Script
% Dynamics: 6DOF rigid body
% Trajectory: Helix
% Control: Cascaded PID (swap controller function to change strategy)
% Fault: Rotor T1 actuator fault with reduced control allocation

clc; clear; close all;

% parameters 
params = parameters();

% reference trajectory 
params = ref_trajectory(params);

% initial conditions
% State: [x y z phi theta psi x_dot y_dot z_dot phi_dot theta_dot psi_dot
%         e_int_x e_int_y e_int_z e_int_phi e_int_theta e_int_psi]
x0 = zeros(18, 1);

% simulation parameters
T     = 30;
tspan = linspace(0, T, 3000);

fprintf('Running simulation...\n');
[t, x] = ode45(@(t, x) quadrotor3D_dynamics(t, x, params, @pid_controller), ...
               tspan, x0);
fprintf('Simulation complete.\n');

% post processing of signals
[logs] = reconstruct_signals(t, x, params, @pid_controller);

% animation
% quadrotor3D_animate(t, x);

% performance metrics
performance_metrics(t, logs.ex, logs.ey, logs.ez, ...
                       logs.ephi, logs.etheta, logs.epsi);

% plots
plot_results(t, x, logs, params);