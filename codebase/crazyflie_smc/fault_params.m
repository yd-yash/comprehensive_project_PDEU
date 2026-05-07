function p = fault_params(p)

    p.fault_enable = 0;     % 0 = healthy (all alphas forced to 1)
                            % 1 = fault active

    p.fault_mode   = 1;     % 1 = pre-existing (from t = 0, entire simulation)
                            % 2 = inject during [fault_start, fault_end],
                            %     healthy outside that window

    p.fault_start  = 5;     % fault onset time (s)   — must be >= 0
    p.fault_end    = 25;    % fault recovery time (s) — must be > fault_start

    p.alpha1 = 1.0;     % rotor 1 (M1, front-right) effectiveness
    p.alpha2 = 1.0;     % rotor 2 (M2, rear-left)   effectiveness
    p.alpha3 = 1.0;     % rotor 3 (M3, rear-right)  effectiveness
    p.alpha4 = 1.0;     % rotor 4 (M4, front-left)  effectiveness

end