function [Omega, Omega_r, T_motors, T_total, U_actual] = control_mixing(U_cmd, alpha_vec, p)

    failed_idx = find(alpha_vec == 0);   % indices of dead rotors

    if isempty(failed_idx)

        Omega2_cmd = p.A_mix \ U_cmd;

        T_cmd = p.b * Omega2_cmd;

        T_eff = alpha_vec .* T_cmd;

        T_motors = max(p.T_min, min(T_eff, p.T_max));

        Omega2 = T_motors / p.b;

        U_actual = p.A_mix * Omega2;

    elseif isscalar(failed_idx)
        
        failed  = failed_idx(1);
        healthy = setdiff(1:4, failed); % failed = 1 -> [2 3 4]

        A_red = p.A_mix(1:3, healthy);       

        U_red = U_cmd(1:3);                

        Omega2_red = A_red \ U_red;

        T_red_cmd = p.b * Omega2_red;

        T_red_eff = alpha_vec(healthy) .* T_red_cmd;

        T_red = max(p.T_min, min(T_red_eff, p.T_max));

        Omega2_red_clamped = T_red / p.b;

        Omega2 = zeros(4, 1);
        Omega2(healthy) = Omega2_red_clamped;

        T_motors  = zeros(4, 1);
        T_motors(healthy) = T_red;

        U_actual = p.A_mix * Omega2;

    else
        warning(['control_mixing: %d rotors have alpha = 0 (rotors %s).' ...
                'Controlled flight is not possible. Falling back to full 4 by 4 mixing.'], ...
                length(failed_idx), num2str(failed_idx(:)'));

        Omega2_cmd = p.A_mix \ U_cmd;
        T_cmd      = p.b * Omega2_cmd;
        T_eff      = alpha_vec .* T_cmd;
        T_motors   = max(p.T_min, min(T_eff, p.T_max));
        Omega2     = T_motors / p.b;
        U_actual   = p.A_mix * Omega2;
    end

    Omega = sqrt(max(Omega2, 0));   

    Omega_r = Omega(1) - Omega(2) + Omega(3) - Omega(4);

    T_total = sum(T_motors);

end