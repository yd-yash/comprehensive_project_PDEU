% Quadrotor Physical Parameters & Fault Settings
% OUTPUTS:
%   params - struct containing all physical and fault parameters

function params = parameters()

    % % parameters
    % params.m = 0.5;
    % params.g = 9.81;
    % params.Ixx = 0.0023;
    % params.Iyy = 0.0023;
    % params.Izz = 0.004;
    % params.l  = 0.25;      % arm length (m)
    % params.d =     % parameters

    params.m = 0.034;
    params.g = 9.81;
    params.Ixx = 16.571710e-6;
    params.Iyy = 16.655602e-6;
    params.Izz = 29.261652e-6;
    params.l  = 0.046;    
    params.d = 0.006;     



    % rotor saturations
    params.T_min = 0;       % minimum rotor thrust (N)
    params.T_max = params.m * params.g;     % maximum rotor thrust (N)

    % fault settings
    params.fault_enable = 0;  % 0 = healthy | 1 = fault active
    params.fault_mode   = 1;  % 1 = pre-existing (from t=0)
                              % 2 = inject during flight
    params.fault_time   = 0;  % fault injection time (sec) [fault_mode 2 only]
    params.alpha_T1     = 1;  % T1 effectiveness:    % parameters
    params.m = 0.5;
    params.g = 9.81;
    params.Ixx = 0.0023;
    params.Iyy = 0.0023;
    params.Izz = 0.004;
    params.l  = 0.25;      % arm length (m)
    params.d = 0.03;      % torque 0 = fully failed, 1 = healthy

end