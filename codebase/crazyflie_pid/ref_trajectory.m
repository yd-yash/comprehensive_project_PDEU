% Reference Trajectory Definition — Crazyflie 2.1 Indoor Scale

% HOW TO SWITCH TRAJECTORY:
%   1. Comment out the currently active trajectory block
%   2. Uncomment the trajectory block you want
%   Only ONE block should be active at a time.

% AVAILABLE TRAJECTORIES:
%   1. Hover Waypoints — slow step references, good for initial gain tuning
%   2. Slow Circle    — 0.5 m radius, indoor-scale circular trajectory [ACTIVE]
%   3. Helix          — climbing helix, 0.5 m radius
%   4. Figure-8       — Lissajous, 0.4 m amplitude

% NOTE: Crazyflie flies indoors. Keep position amplitudes <= 0.5–1.0 m and
%       velocities <= 0.3 m/s for realistic simulation. The original 4 m
%       ellipse would require ~14 deg bank, which is at the edge of the
%       small-angle assumption; it will work but expect larger errors.

% INPUTS:
%   params - params struct
% OUTPUTS:
%   params - params struct with trajectory fields appended

function params = ref_trajectory(params)

% TRAJECTORY 1: Hover Waypoints (step to 0.5 m, hold)
%   Good for initial gain validation. Once hover is clean, switch to circular.
%   x(t) = 0.3 m constant
%   y(t) = 0   m constant
%   z(t) = 0.5 m constant

    % params.x_des = @(t)  0.3 * ones(size(t));
    % params.y_des = @(t)  zeros(size(t));
    % params.z_des = @(t)  0.5 * ones(size(t));
    % 
    % params.x_dot_des   = @(t)  zeros(size(t));
    % params.y_dot_des   = @(t)  zeros(size(t));
    % params.z_dot_des   = @(t)  zeros(size(t));
    % 
    % params.x_ddot_des  = @(t)  zeros(size(t));
    % params.y_ddot_des  = @(t)  zeros(size(t));
    % params.z_ddot_des  = @(t)  zeros(size(t));
    % 
    % params.x_dddot_des = @(t)  zeros(size(t));
    % params.y_dddot_des = @(t)  zeros(size(t));
    % 
    % params.psi_des = @(t)  zeros(size(t));
    % params.psi_ref = @(t)  zeros(size(t));


% TRAJECTORY 2: Slow Circle (indoor scale)  [ACTIVE]
%   Radius 0.5 m, omega = 0.3 rad/s  =>  period ~ 21 s
%   Max speed = r*omega = 0.15 m/s — well within Crazyflie envelope
%   Max centripetal accel = r*omega^2 = 0.045 m/s^2 => theta_des ~ 0.26 deg  [tiny]
%   x(t) = 0.5*cos(0.3*t)
%   y(t) = 0.5*sin(0.3*t)
%   z(t) = 0.5  (constant hover altitude)

    r     = 0.5;     % radius (m)
    omega = 0.3;     % angular rate (rad/s)
    z0    = 0.5;     % hover altitude (m)

    params.x_des = @(t)  r * cos(omega*t);
    params.y_des = @(t)  r * sin(omega*t);
    params.z_des = @(t)  z0 * ones(size(t));

    params.x_dot_des = @(t) -r*omega *  sin(omega*t);
    params.y_dot_des = @(t)  r*omega *  cos(omega*t);
    params.z_dot_des = @(t)  zeros(size(t));

    params.x_ddot_des = @(t) -r*omega^2 * cos(omega*t);
    params.y_ddot_des = @(t) -r*omega^2 * sin(omega*t);
    params.z_ddot_des = @(t)  zeros(size(t));

    % Jerk
    %   x_dddot = d/dt(-r*omega^2*cos(omega*t)) =  r*omega^3*sin(omega*t)
    %   y_dddot = d/dt(-r*omega^2*sin(omega*t)) = -r*omega^3*cos(omega*t)
    params.x_dddot_des = @(t)  r*omega^3 * sin(omega*t);
    params.y_dddot_des = @(t) -r*omega^3 * cos(omega*t);

    params.psi_des = @(t)  zeros(size(t));
    params.psi_ref = @(t)  zeros(size(t));


% TRAJECTORY 3: Helix (indoor scale)
%   r=0.5 m, omega=0.3 rad/s, z climbs at 0.02 m/s
%
    % r     = 0.5;
    % omega = 0.3;
    % 
    % params.x_des = @(t)  r * cos(omega*t);
    % params.y_des = @(t)  r * sin(omega*t);
    % params.z_des = @(t)  0.3 + 0.02*t;
    % 
    % params.x_dot_des = @(t) -r*omega * sin(omega*t);
    % params.y_dot_des = @(t)  r*omega * cos(omega*t);
    % params.z_dot_des = @(t)  0.02 * ones(size(t));
    % 
    % params.x_ddot_des = @(t) -r*omega^2 * cos(omega*t);
    % params.y_ddot_des = @(t) -r*omega^2 * sin(omega*t);
    % params.z_ddot_des = @(t)  zeros(size(t));
    % 
    % params.x_dddot_des = @(t)  r*omega^3 * sin(omega*t);
    % params.y_dddot_des = @(t) -r*omega^3 * cos(omega*t);
    % 
    % params.psi_des = @(t)  zeros(size(t));
    % params.psi_ref = @(t)  zeros(size(t));


% TRAJECTORY 4: Figure-8 (indoor Lissajous)
%   Ax = Ay = 0.4 m, omega = 0.2 rad/s  =>  period ~ 31 s
%
    % Ax    = 0.4;
    % Ay    = 0.4;
    % omega = 0.2;
    % 
    % params.x_des = @(t)  Ax * sin(2*omega*t);
    % params.y_des = @(t)  Ay * sin(  omega*t);
    % params.z_des = @(t)  0.5 * ones(size(t));
    % 
    % params.x_dot_des = @(t)  2*Ax*omega * cos(2*omega*t);
    % params.y_dot_des = @(t)    Ay*omega * cos(  omega*t);
    % params.z_dot_des = @(t)  zeros(size(t));
    % 
    % params.x_ddot_des = @(t) -4*Ax*omega^2 * sin(2*omega*t);
    % params.y_ddot_des = @(t) -  Ay*omega^2 * sin(  omega*t);
    % params.z_ddot_des = @(t)  zeros(size(t));
    % 
    % params.x_dddot_des = @(t) -8*Ax*omega^3 * cos(2*omega*t);
    % params.y_dddot_des = @(t) -  Ay*omega^3 * cos(  omega*t);
    % 
    % params.psi_des = @(t)  zeros(size(t));
    % params.psi_ref = @(t)  zeros(size(t));

end