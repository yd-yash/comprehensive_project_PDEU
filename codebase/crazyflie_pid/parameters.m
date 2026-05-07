% Quadrotor Physical Parameters & Fault Settings
% OUTPUTS:
%   params - struct containing all physical and fault parameters

function params = parameters()

    params.m = 0.034;
    params.g = 9.81;
    params.Ixx = 16.571710e-6;
    params.Iyy = 16.655602e-6;
    params.Izz = 29.261652e-6;
    params.l  = 0.046;    
    params.d = 0.006;

    % rotor saturations
    params.T_min = 0;       % minimum rotor thrust (N)
    % params.T_max = params.m * params.g;     % maximum rotor thrust (N)
    params.T_max = 0.16;

    % fault settings
    params.fault_enable = 0;  % 0 = healthy | 1 = fault active
    params.fault_mode   = 1;  % 1 = pre-existing (from t=0)
                              % 2 = inject during flight
    params.fault_time   = 0;  % fault injection time (sec) [fault_mode 2 only]
    params.alpha_T1     = 1;  % T1 effectiveness:    % parameters
end