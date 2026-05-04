% Returns actuator effectiveness alpha for T1 at time t
% Called by both dynamics and reconstruct_signals (single source of truth)
% INPUTS:
%   t  - current time (scalar)
%   p  - params struct
%
% OUTPUT:
%   alpha - effectiveness [0, 1]

function alpha = inducing_fault(t, p)
     
    % Default: healthy
    alpha = 1;
 
    % Guard: only apply fault if explicitly enabled
    if p.fault_enable ~= 1
        return;   % fault_enable = 0 exits here — alpha stays 1, always
    end
 
    % Fault is enabled: apply based on mode
    if p.fault_mode == 1
        % Pre-existing fault: active from t = 0
        alpha = p.alpha_T1;
 
    elseif p.fault_mode == 2
        % Injected during flight: active after fault_time
        % Safety check: warn if fault_time = 0 (fault active from t=0, same as mode 1)
        if p.fault_time == 0
            warning('get_alpha: fault_time = 0 with fault_mode = 2. Fault is active from t=0. Use fault_mode = 1 instead.');
        end
        if t >= p.fault_time
            alpha = p.alpha_T1;
        end
 
    else
        warning('get_alpha: unknown fault_mode = %d. No fault applied.', p.fault_mode);
    end

end

