function p = parameters()
% Physical parameters: Bitcraze Crazyflie 2.1
% Inertia:  Förster (2015), ETH Zurich system identification
% b, d:     Förster (2015) rotor speed parameterisation
%             T = b * Omega^2,  tau_drag = d * Omega^2
% Ω range:  ~314 – 2200 rad/s  (3000 – 21000 RPM)
% T_max:    0.16 N/rotor  =>  b*Omega_max^2 = 2.88e-8*2357^2 ≈ 0.16 N  ✓
% hover Ω:  sqrt(m*g/(4*b)) = sqrt(0.034*9.81/(4*2.88e-8)) ≈ 1705 rad/s  ✓

    p.m   = 0.034;          % mass (kg)
    p.g   = 9.81;           % gravity (m/s^2)
    p.l   = 0.046;          % arm length, centre to motor (m)
    p.Ixx = 16.571710e-6;   % MOI about x-axis (kg·m^2)
    p.Iyy = 16.655602e-6;   % MOI about y-axis (kg·m^2)
    p.Izz = 29.261652e-6;   % MOI about z-axis (kg·m^2)
    p.Ir  = 6.0e-9;         % rotor axial inertia (kg·m^2)  — Förster 2015

    % Rotor aerodynamic coefficients (Omega-squared parameterisation)
    p.b   = 2.88e-8;        % thrust coefficient      (N·s^2/rad^2)
    p.d   = 7.24e-10;       % drag/torque coefficient (N·m·s^2/rad^2)

    % Rotor thrust saturation
    p.T_min = 0;            % (N)
    p.T_max = 0.16;         % (N) per rotor  [total max = 4*0.16 = 0.64 N]
                            % hover per rotor = m*g/4 = 0.034*9.81/4 = 0.0834 N

    % Aerodynamic drag (translational)
    % CF2.1 drag is small at indoor speeds. Set to zero initially;
    % re-enable from Förster 2015 if drag effects become relevant.
    p.Kx  = 0.0;
    p.Ky  = 0.0;
    p.Kz  = 0.0;

    % Aerodynamic drag (rotational)
    p.Kphi   = 0.0;
    p.Ktheta = 0.0;
    p.Kpsi   = 0.0;

    % Mixing matrix: maps [Omega1^2 ... Omega4^2] -> [U1; U2; U3; U4]
    % X-configuration motor layout (top view):
    %   M3 --- M1    M1,M3 CW  (+d); M2,M4 CCW (-d)
    %    \     /     Omega_r = Omega1 - Omega2 + Omega3 - Omega4
    %    /     \
    %   M2 --- M4
    l_eff = p.l / sqrt(2);

    p.A_mix = [ p.b,          p.b,          p.b,          p.b;
               -p.b*l_eff,   -p.b*l_eff,    p.b*l_eff,    p.b*l_eff;
                p.b*l_eff,   -p.b*l_eff,   -p.b*l_eff,    p.b*l_eff;
               -p.d,          p.d,          -p.d,           p.d];

end

    % 
    % p.Kx  = 0.01; % drag along x (N·s/m)
    % p.Ky  = 0.01; % drag along y (N·s/m)
    % p.Kz  = 0.01; % drag along z (N·s/m)
    % 
    % p.Kphi   = 5e-4; % roll drag (N·m·s^2/rad^2)
    % p.Ktheta = 5e-4; % pitch drag (N·m·s^2/rad^2)
    % p.Kpsi   = 5e-4; % yaw drag (N·m·s^2/rad^2)