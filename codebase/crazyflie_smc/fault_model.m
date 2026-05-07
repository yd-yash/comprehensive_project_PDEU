function alpha_vec = fault_model(t, p)

    alpha_vec = ones(4, 1);

    if p.fault_enable ~= 1
        return;
    end

    fault_active = false;

    if p.fault_mode == 1
        fault_active = true;

    elseif p.fault_mode == 2
        if p.fault_start < 0
            warning('fault_model: fault_start = %.2f is negative. Clamping to 0.', p.fault_start);
            p.fault_start = 0;
        end
        if p.fault_end <= p.fault_start
            warning('fault_model: fault_end (%.2f) must be > fault_start (%.2f). No fault applied.', ...
                    p.fault_end, p.fault_start);
        else
            fault_active = (t >= p.fault_start && t <= p.fault_end);
        end

    else
        warning('fault_model: unknown fault_mode = %d. No fault applied.', p.fault_mode);
    end

    if fault_active
        alpha_vec = [p.alpha1; p.alpha2; p.alpha3; p.alpha4];
    end

end