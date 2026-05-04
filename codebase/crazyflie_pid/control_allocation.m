% Control Allocation with Fault Handling

% Maps virtual control inputs [u1, u2, u3, u4] to individual rotor
% thrusts [T1, T2, T3, T4] and computes actual forces/torques after
% applying actuator fault (via alpha).

% THREE CASES based on alpha:
%   alpha = 1          : healthy — full 4-rotor allocation (pinv)
%   0 < alpha < 1      : partial fault — full allocation computed, then
%                        T1 is physically scaled by alpha. Remaining
%                        rotors are NOT recomputed (open-loop disturbance).
%                        Controller will reject via feedback over time.
%   alpha = 0          : complete failure — T1 forced to 0, reduced
%                        3-rotor allocation used (sacrifice yaw)

% INPUTS:
%   u      - control input struct (from controller function)
%              u.u1 = total thrust cmd, u.u2 = roll torque cmd,
%              u.u3 = pitch torque cmd, u.u4 = yaw torque cmd
%   alpha  - T1 actuator effectiveness (from get_alpha)
%   p      - params struct

% OUTPUTS:
%   alloc  - struct with fields:
%              alloc.T1, T2, T3, T4       - actual rotor thrusts after fault (N)
%              alloc.total_thrust         - actual total thrust (N)
%              alloc.tau_phi              - actual roll torque  (Nm)
%              alloc.tau_theta            - actual pitch torque (Nm)
%              alloc.tau_psi              - actual yaw torque   (Nm)

function alloc = control_allocation(u, alpha, p)

    % Full control allocation matrix (X-frame)
    A = [1,              1,             1,             1;
        -p.l/sqrt(2),   -p.l/sqrt(2),   p.l/sqrt(2),  p.l/sqrt(2);
         p.l/sqrt(2),   -p.l/sqrt(2),  -p.l/sqrt(2),  p.l/sqrt(2);
         p.d,           -p.d,           p.d,          -p.d];

    if alpha == 0
        % CASE: Complete rotor failure — reduced 3-rotor allocation
        % T1 is forced to 0; yaw must be sacrificed (u4 ignored)
        T1 = 0;
        A_red = [1,             1,            1;
                -p.l/sqrt(2),   p.l/sqrt(2),  p.l/sqrt(2);
                -p.l/sqrt(2),  -p.l/sqrt(2),  p.l/sqrt(2)];
        U_red = [u.u1; u.u2; u.u3];
        T_red = A_red \ U_red;
        T2 = T_red(1);
        T3 = T_red(2);
        T4 = T_red(3);

    else
        % CASE: Healthy (alpha=1) or partial fault (0 < alpha < 1)
        % Full 4-rotor allocation is computed first (as if healthy).
        % Then T1 is physically scaled by alpha to model the fault.
        % T2, T3, T4 are unchanged — controller rejects disturbance via feedback.
        U = [u.u1; u.u2; u.u3; u.u4];
        T = pinv(A) * U;

        T1 = alpha * T(1);   % apply fault: T1_actual = alpha * T1_commanded
        T2 = T(2);           % T2, T3, T4 unaffected by T1 fault
        T3 = T(3);
        T4 = T(4);
    end

    % Rotor saturation
    T_vec = max(p.T_min, min([T1; T2; T3; T4], p.T_max));
    T1 = T_vec(1);  T2 = T_vec(2);
    T3 = T_vec(3);  T4 = T_vec(4);

    % Actual forces/torques after fault and saturation
    actual_forces = A * [T1; T2; T3; T4];

    % Pack outputs
    alloc.T1           = T1;
    alloc.T2           = T2;
    alloc.T3           = T3;
    alloc.T4           = T4;
    alloc.total_thrust = actual_forces(1);
    alloc.tau_phi      = actual_forces(2);
    alloc.tau_theta    = actual_forces(3);
    alloc.tau_psi      = actual_forces(4);

end