% =========================================================================
% main_stsmc.m — Quadrotor 3D Simulation: STSMC Controller
% =========================================================================
% Trajectory : 3D Lemniscate  (rt_thesis.m)
% Controller : Super-Twisting SMC  (stsmc_controller.m)
% Plots      : plots_thesis_stsmc.m  (7 figures, Fig7 = sliding variables)
% Metrics    : pm_thesis_stsmc.m    (13 metrics + sliding reach time)
%
% Files required in the same directory:
%   parameters.m, rt_thesis.m, quadrotor3D_dynamics.m
%   stsmc_controller.m, reconstruct_signals_stsmc.m
%   control_allocation.m, inducing_fault.m
%   plots_thesis_stsmc.m, pm_thesis_stsmc.m
% =========================================================================

clc; clear; close all;

% ---- Parameters & trajectory --------------------------------------------
params = parameters();
params = rt_thesis(params);

% ---- Initial conditions -------------------------------------------------
% [x y z φ θ ψ ẋ ẏ ż φ̇ θ̇ ψ̇ ∫ex ∫ey ∫ez ∫eφ ∫eθ ∫eψ]
x0 = zeros(18, 1);

% ---- Simulation ---------------------------------------------------------
T     = 50;                         % simulation duration (s)
tspan = linspace(0, T, 5000);       % 5000 pts for smoother ST dynamics

fprintf('Running STSMC simulation...\n');
[t, x] = ode45(@(t, x) quadrotor3D_dynamics(t, x, params, @stsmc_controller), ...
               tspan, x0);
fprintf('Simulation complete.\n');

% ---- Post-processing ----------------------------------------------------
logs = reconstruct_signals_stsmc(t, x, params, @stsmc_controller);

% ---- Performance metrics ------------------------------------------------
pm_thesis_stsmc(t, logs.ex, logs.ey, logs.ez, ...
                logs.ephi, logs.etheta, logs.epsi, logs);

% ---- Plots (7 figures) --------------------------------------------------
plots_thesis_stsmc(t, x, logs, params);