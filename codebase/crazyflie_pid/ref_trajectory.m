% Reference Trajectory Definition

% HOW TO SWITCH TRAJECTORY:
%   1. Comment out the currently active trajectory block
%   2. Uncomment the trajectory block you want
%   Only ONE block should be active at a time.

% AVAILABLE TRAJECTORIES:
%   1. Helix        [ACTIVE]  — circular helix, climbing Z
%   2. Straight Line          — constant velocity along X, climbing Z
%   3. Ellipse                — flat ellipse, horizontal axis >> vertical (4:1)
%   4. Figure-8               — Lissajous curve in XY, gentle Z climb

% NOTE: All position, velocity, and acceleration handles must be
%       analytically consistent. Verify with symbolic differentiation
%       if you add a new trajectory.

% INPUTS:
%   params - params struct (from get_params.m)

% OUTPUTS:
%   params - params struct with trajectory fields appended

function params = ref_trajectory(params)

% TRAJECTORY 1: Helix  [ACTIVE]
%   x(t) =  2*cos(0.3*t)
%   y(t) =  2*sin(0.3*t)
%   z(t) =  0.5 + 0.1*t
    % % Position
    % params.x_des = @(t)  2*cos(0.3*t);
    % params.y_des = @(t)  2*sin(0.3*t);
    % params.z_des = @(t)  0.5 + 0.1*t;
    % 
    % % Velocity (first derivatives)
    % params.x_dot_des = @(t) -0.6*sin(0.3*t);
    % params.y_dot_des = @(t)  0.6*cos(0.3*t);
    % params.z_dot_des = @(t)  0.1*ones(size(t));
    % 
    % % Acceleration (second derivatives)
    % params.x_ddot_des = @(t) -0.18*cos(0.3*t);
    % params.y_ddot_des = @(t) -0.18*sin(0.3*t);
    % params.z_ddot_des = @(t)  zeros(size(t));
    % 
    % % Jerk (third derivatives) — used for exact phi_dot_des, theta_dot_des in PID
    % % x_dddot = d/dt(-0.18*cos(0.3t)) =  0.054*sin(0.3t)
    % % y_dddot = d/dt(-0.18*sin(0.3t)) = -0.054*cos(0.3t)
    % params.x_dddot_des = @(t)  0.054*sin(0.3*t);
    % params.y_dddot_des = @(t) -0.054*cos(0.3*t);
    % 
    % % Yaw reference
    % params.psi_des = @(t)  zeros(size(t));
    % params.psi_ref = @(t)  zeros(size(t));

% TRAJECTORY 2: Straight Line
%   x(t) =  0.3*t
%   y(t) =  0
%   z(t) =  0.5 + 0.1*t
    % % Position  
    % params.x_des = @(t)  0.3*t;
    % params.y_des = @(t)  zeros(size(t));
    % params.z_des = @(t)  0.5 + 0.1*t;
    % 
    % % Velocity (first derivatives)  
    % params.x_dot_des = @(t)  0.3*ones(size(t));
    % params.y_dot_des = @(t)  zeros(size(t));
    % params.z_dot_des = @(t)  0.1*ones(size(t));
    % 
    % % Acceleration (second derivatives)  
    % params.x_ddot_des = @(t)  zeros(size(t));
    % params.y_ddot_des = @(t)  zeros(size(t));
    % params.z_ddot_des = @(t)  zeros(size(t));
    % 
    % % Jerk (third derivatives) — straight line: all jerks are zero
    % params.x_dddot_des = @(t)  0;
    % params.y_dddot_des = @(t)  0;
    % 
    % % Yaw reference  
    % params.psi_des = @(t)  zeros(size(t));
    % params.psi_ref = @(t)  zeros(size(t));

% TRAJECTORY 3: Ellipse (horizontal axis >> vertical axis, 4:1 ratio)
%   Semi-major axis (X): a = 4 m
%   Semi-minor axis (Y): b = 1 m
%   Angular rate: omega = 0.3 rad/s
%   x(t) =  4*cos(0.3*t)
%   y(t) =  1*sin(0.3*t)
%   z(t) =  1.5  (constant altitude)

    a     = 4;      % semi-major axis (m) — horizontal (X)
    b     = 1;      % semi-minor axis (m) — lateral   (Y)
    omega = 0.3;    % angular rate (rad/s)
    z0    = 5;    % constant altitude (m)

    % Position  
    params.x_des = @(t)  a * cos(omega*t);
    params.y_des = @(t)  b * sin(omega*t);
    params.z_des = @(t)  z0 * ones(size(t));

    % Velocity (first derivatives)  
    params.x_dot_des = @(t) -a*omega *  sin(omega*t);
    params.y_dot_des = @(t)  b*omega *  cos(omega*t);
    params.z_dot_des = @(t)  zeros(size(t));

    % Acceleration (second derivatives)  
    params.x_ddot_des = @(t) -a*omega^2 * cos(omega*t);
    params.y_ddot_des = @(t) -b*omega^2 * sin(omega*t);
    params.z_ddot_des = @(t)  zeros(size(t));

    % Jerk (third derivatives)
    %   x_dddot = d/dt(-a*omega^2*cos(omega*t)) =  a*omega^3*sin(omega*t)
    %   y_dddot = d/dt(-b*omega^2*sin(omega*t)) = -b*omega^3*cos(omega*t)
    params.x_dddot_des = @(t)  a*omega^3 * sin(omega*t);
    params.y_dddot_des = @(t) -b*omega^3 * cos(omega*t);

    % Yaw reference  
    params.psi_des = @(t)  zeros(size(t));
    params.psi_ref = @(t)  zeros(size(t));


% TRAJECTORY 4: Figure-8 (Lissajous curve in XY, gentle Z climb)
  % Ax = 2 m, Ay = 2 m, omega = 0.2 rad/s  =>  period ~ 31 s
  % x(t) =  2*sin(2*0.2*t) = 2*sin(0.4*t)
  % y(t) =  2*sin(  0.2*t)
  % z(t) =  0.5 + 0.05*t

    % Ax    = 2;      % X amplitude (m)
    % Ay    = 2;      % Y amplitude (m)
    % omega = 0.2;    % base angular rate (rad/s)
    % 
    % % Position  
    % params.x_des = @(t)  Ax * sin(2*omega*t);
    % params.y_des = @(t)  Ay * sin(  omega*t);
    % params.z_des = @(t)  0.5 + 0.05*t;
    % 
    % % Velocity (first derivatives)  
    % params.x_dot_des = @(t)  2*Ax*omega * cos(2*omega*t);
    % params.y_dot_des = @(t)    Ay*omega * cos(  omega*t);
    % params.z_dot_des = @(t)  0.05*ones(size(t));
    % 
    % % Acceleration (second derivatives)  
    % params.x_ddot_des = @(t) -4*Ax*omega^2 * sin(2*omega*t);
    % params.y_ddot_des = @(t) -  Ay*omega^2 * sin(  omega*t);
    % params.z_ddot_des = @(t)  zeros(size(t));
    % 
    % Jerk (third derivatives)
    %   x_dddot = d/dt(-4*Ax*omega^2*sin(2*omega*t)) = -8*Ax*omega^3*cos(2*omega*t)
    %   y_dddot = d/dt(  -Ay*omega^2*sin(  omega*t)) =   -Ay*omega^3*cos(  omega*t)
    % params.x_dddot_des = @(t) -8*Ax*omega^3 * cos(2*omega*t);
    % params.y_dddot_des = @(t) -  Ay*omega^3 * cos(  omega*t);
    % % Yaw reference  
    % params.psi_des = @(t)  zeros(size(t));
    % params.psi_ref = @(t)  zeros(size(t));

end