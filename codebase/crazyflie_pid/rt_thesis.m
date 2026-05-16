% =========================================================================
% rt_thesis.m — Reference Trajectory for Thesis (3D Lemniscate)
% =========================================================================
% Trajectory definition:
%   x_d(t) = A * sin(w*t)
%   y_d(t) = A * sin(2*w*t)
%   z_d(t) = z0 + h * sin(w*t)
%
% Parameters: A = 0.5 m,  w = 0.5 rad/s,  z0 = 1.0 m,  h = 0.2 m
%
% Derivatives (analytical):
%   x_dot   =  A*w   * cos(w*t)
%   y_dot   =  2*A*w * cos(2*w*t)
%   z_dot   =  h*w   * cos(w*t)
%
%   x_ddot  = -A*w^2   * sin(w*t)
%   y_ddot  = -4*A*w^2 * sin(2*w*t)
%   z_ddot  = -h*w^2   * sin(w*t)
%
%   x_dddot = -A*w^3   * cos(w*t)       [needed for phi_dot_des, theta_dot_des]
%   y_dddot = -8*A*w^3 * cos(2*w*t)
%
% INPUTS:
%   params — params struct (from parameters.m)
% OUTPUTS:
%   params — params struct with trajectory fields appended
%
% REQUIRED FIELDS (consumed by pid_controller.m and reconstruct_signals.m):
%   x_des, y_des, z_des
%   x_dot_des, y_dot_des, z_dot_des
%   x_ddot_des, y_ddot_des, z_ddot_des
%   x_dddot_des, y_dddot_des
%   psi_des, psi_ref
% =========================================================================

function params = rt_thesis(params)

    % --- Trajectory parameters -------------------------------------------
    A  = 0.5;    % amplitude (m)
    w  = 0.5;    % angular frequency (rad/s)  =>  period = 2*pi/w ~ 12.6 s
    z0 = 1.0;    % mean hover altitude (m)
    h  = 0.2;    % altitude oscillation amplitude (m)
    % ---------------------------------------------------------------------

    % Position
    params.x_des = @(t)  A * sin(w * t);
    params.y_des = @(t)  A * sin(2 * w * t);
    params.z_des = @(t)  z0 + h * sin(w * t);

    % Velocity (1st derivative)
    params.x_dot_des = @(t)   A * w       * cos(w * t);
    params.y_dot_des = @(t)   2 * A * w   * cos(2 * w * t);
    params.z_dot_des = @(t)   h * w       * cos(w * t);

    % Acceleration (2nd derivative)
    params.x_ddot_des = @(t)  -A * w^2       * sin(w * t);
    params.y_ddot_des = @(t)  -4 * A * w^2   * sin(2 * w * t);
    params.z_ddot_des = @(t)  -h * w^2       * sin(w * t);

    % Jerk (3rd derivative) — used for phi_dot_des, theta_dot_des feedforward
    params.x_dddot_des = @(t)  -A * w^3       * cos(w * t);
    params.y_dddot_des = @(t)  -8 * A * w^3   * cos(2 * w * t);

    % Yaw — held at zero throughout
    params.psi_des = @(t)  zeros(size(t));
    params.psi_ref = @(t)  zeros(size(t));

end