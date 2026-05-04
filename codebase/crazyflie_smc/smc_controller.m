function [U, Uxy, cmd_mem, surfaces] = smc_controller(t, x, Uxy, Omega_r, cmd_mem, dt, p)

    x_pos = x(1);   x_dot = x(7);
    y_pos = x(2);   y_dot = x(8);
    z = x(3);   z_dot = x(9);
    phi = x(4); phi_dot = x(10);
    theta = x(5); theta_dot = x(11);
    psi = x(6); psi_dot = x(12);

    phi_cmd_prev       = cmd_mem.phi_cmd;
    phi_cmd_dot_prev   = cmd_mem.phi_cmd_dot;
    theta_cmd_prev     = cmd_mem.theta_cmd;
    theta_cmd_dot_prev = cmd_mem.theta_cmd_dot;

    % Reference signals
    % XY position references (time-only, from reference_trajectory.m)
    Xd   = p.x_des(t);
    Yd   = p.y_des(t);
 
    % XY velocity and acceleration via PD outer-loop cascade (state-dependent).
    % This matches the ST-SMC reference structure:
    %   xd_dot  = kpx * (xd - x_pos)      position error -> velocity ref
    %   xd_ddot = kdx * (xd_dot - x_dot)  velocity error -> accel ref
    Xd_dot  = p.kpx * (Xd - x_pos);
    Xd_ddot = p.kdx * (Xd_dot - x_dot);
 
    Yd_dot  = p.kpy * (Yd - y_pos);
    Yd_ddot = p.kdy * (Yd_dot - y_dot);

    Zd = p.z_des(t); Zd_dot = p.zd_dot_des(t); Zd_ddot = p.zd_ddot_des(t);
    psid = p.psi_des(t); psid_dot = p.psid_dot_des(t); psid_ddot = p.psid_ddot_des(t);

    % U1
    e_z = Zd - z;
    e_z_dot = Zd_dot - z_dot;
    s_z = e_z_dot + p.c_z * e_z;

    U1 = (p.m / (cos(phi)*cos(theta)))*(p.g + (p.Kz/p.m)*z_dot + Zd_ddot + p.c_z*e_z_dot + p.ks_z*sat(s_z, p.sat_bl) + p.kl_z*s_z );

    % Ux
    e_x = Xd - x_pos;
    e_x_dot = Xd_dot - x_dot;
    s_x = e_x_dot + p.c_x * e_x;

    Ux = (p.m/U1) * ( (p.Kx/p.m)*x_dot + Xd_ddot ...
                         + p.c_x*e_x_dot ...
                         + p.ks_x*sat(s_x, p.sat_bl) + p.kl_x*s_x );
    Ux = max(-1, min(1, Ux)); % clamp to [-1, 1] (reference: f_sat(...,-1,1)/u1)
    Ux_dot = (Ux - Uxy(1)) / dt; % finite difference

    % Uy
    e_y = Yd - y_pos;
    e_y_dot = Yd_dot - y_dot;
    s_y = e_y_dot + p.c_y * e_y;

    Uy = (p.m/U1)*( (p.Ky/p.m)*y_dot + Yd_ddot + p.c_y*e_y_dot + p.ks_y*sat(s_y, p.sat_bl) + p.kl_y*s_y );
    Uy = max(-1, min(1, Uy)); % clamp to [-1, 1]
    Uy_dot = (Uy - Uxy(2)) / dt; % finite difference

    %  phi_cmd = asin( Ux*sin(psi_d) - Uy*cos(psi_d) )
    %  theta_cmd = asin( (Ux*cos(psi_d) + Uy*sin(psi_d)) / cos(phi_cmd) 
    phi_cmd = asin_norm(Ux*sin(psid) - Uy*cos(psid));
    phi_cmd_dot = wrap(phi_cmd - phi_cmd_prev) / dt;
    phi_cmd_ddot = (phi_cmd_dot - phi_cmd_dot_prev) / dt;

    % Guard against cos(phi_cmd) = 0 (singular at ±90 deg roll)
    % cos_phi_cmd = cos(phi_cmd);
    % if abs(cos_phi_cmd) < 1e-4
    %     cos_phi_cmd = sign(cos_phi_cmd + 1e-10) * 1e-4;
    % end

    % theta_cmd = asin_norm((Ux*cos(psid) + Uy*sin(psid)) / cos_phi_cmd);
    theta_cmd = asin_norm((Ux*cos(psid) + Uy*sin(psid)) / cos(psid));
    theta_cmd_dot = wrap(theta_cmd - theta_cmd_prev) / dt;
    theta_cmd_ddot = (theta_cmd_dot - theta_cmd_dot_prev) / dt;

    % U2
    e_phi = wrap(phi_cmd - phi);
    e_phi_dot = wrap( phi_cmd_dot - phi_dot );
    s_phi = e_phi_dot + p.c_phi * e_phi;

    U2 = p.Ixx * (-((p.Iyy - p.Izz)/p.Ixx)*theta_dot*psi_dot + (p.Kphi/p.Ixx)*phi_dot*abs(phi_dot) + (p.Ir/p.Ixx)*Omega_r*theta_dot + phi_cmd_ddot + p.c_phi*e_phi_dot + p.ks_phi*sat(s_phi, p.sat_bl) + p.kl_phi*s_phi);

    % U3
    e_theta     = wrap( theta_cmd     - theta     );
    e_theta_dot = wrap( theta_cmd_dot - theta_dot );
    s_theta     = e_theta_dot + p.c_theta * e_theta;

    U3 = p.Iyy*(-((p.Izz - p.Ixx)/p.Iyy)*phi_dot*psi_dot+ (p.Ktheta/p.Iyy)*theta_dot*abs(theta_dot) - (p.Ir/p.Iyy)*Omega_r*phi_dot + theta_cmd_ddot + p.c_theta*e_theta_dot + p.ks_theta*sat(s_theta, p.sat_bl) + p.kl_theta*s_theta);

    % U4
    e_psi     = wrap( psid     - psi     );
    e_psi_dot = wrap( psid_dot - psi_dot );
    s_psi     = e_psi_dot + p.c_psi * e_psi;

    U4 = p.Izz*(-((p.Ixx - p.Iyy)/p.Izz)*phi_dot*theta_dot + (p.Kpsi/p.Izz)*psi_dot*abs(psi_dot) + psid_ddot + p.c_psi*e_psi_dot + p.ks_psi*sat(s_psi, p.sat_bl) + p.kl_psi*s_psi);

    U   = [U1; U2; U3; U4];
    Uxy = [Ux; Uy];

    % Update attitude command memory for next step
    cmd_mem.phi_cmd        = phi_cmd;
    cmd_mem.phi_cmd_dot    = phi_cmd_dot;
    cmd_mem.theta_cmd      = theta_cmd;
    cmd_mem.theta_cmd_dot  = theta_cmd_dot;

    surfaces.s_z      = s_z;
    surfaces.s_x      = s_x;
    surfaces.s_y      = s_y;
    surfaces.s_phi    = s_phi;
    surfaces.s_theta  = s_theta;
    surfaces.s_psi    = s_psi;
    surfaces.phi_cmd  = phi_cmd;
    surfaces.theta_cmd = theta_cmd;

end

function y = asin_norm(u)
    if     u >  1;   y =  pi/2;
    elseif u < -1;   y = -pi/2;
    else;            y =  asin(u);
    end
end

function y = wrap(x)
    y = atan2(sin(x), cos(x));
end

function out = sat(s, bl)
    if bl <= 0
        out = sign(s);
    else
        out = max(-1, min(1, s / bl));
    end
end