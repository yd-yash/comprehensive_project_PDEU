% 3D Quadrotor Rigid Body Dynamics (6DOF)
% State vector (18x1):
%   x(1:3)   - position       [x, y, z]
%   x(4:6)   - attitude       [phi, theta, psi]
%   x(7:9)   - linear vel     [x_dot, y_dot, z_dot]
%   x(10:12) - angular vel    [phi_dot, theta_dot, psi_dot]
%   x(13:15) - pos integrators [e_int_x, e_int_y, e_int_z]
%   x(16:18) - att integrators [e_int_phi, e_int_theta, e_int_psi]
% INPUTS:
%   t              - current time
%   x              - state vector (18x1)
%   p              - params struct
%   ctrl_func      - function handle to controller (e.g. @pid_controller)
% OUTPUTS:
%   dx             - state derivative (18x1)

function dx = quadrotor3D_dynamics(t, x, p, ctrl_func)

    % states 
    x_pos     = x(1);   y_pos     = x(2);   z_pos     = x(3);
    phi       = x(4);   theta     = x(5);   psi       = x(6);
    x_pos_dot = x(7);   y_pos_dot = x(8);   z_pos_dot = x(9);
    phi_dot   = x(10);  theta_dot = x(11);  psi_dot   = x(12);
    e_int_x   = x(13);  e_int_y   = x(14);  e_int_z   = x(15);
    % e_int_phi, e_int_theta, e_int_psi used inside controller via x vector

    % integrator derivatives (feed into next step)
    e_int_x_dot = p.x_des(t) - x_pos;
    e_int_y_dot = p.y_des(t) - y_pos;
    e_int_z_dot = p.z_des(t) - z_pos;

    % call controller
    u = ctrl_func(t, x, p);

    % fault logic: determine actuator effectiveness
    alpha = inducing_fault(t, p);

    % override yaw if rotor fully failed
    if alpha == 0
        u.u4 = 0;  % sacrifice yaw control
    end

    % integrator error for attitude (used in next step)
    e_int_phi_dot   = u.phi_des   - phi;
    e_int_theta_dot = u.theta_des - theta;
    e_int_psi_dot   = u.psi_des   - psi;

    % control allocation
    alloc = control_allocation(u, alpha, p);

    total_thrust = alloc.total_thrust;
    tau_phi      = alloc.tau_phi;
    tau_theta    = alloc.tau_theta;
    tau_psi      = alloc.tau_psi;

    % translational dynamics (accelerations)
    x_ddot = (total_thrust/p.m) * (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi));
    y_ddot = (total_thrust/p.m) * (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi));
    z_ddot = (total_thrust/p.m) * (cos(phi)*cos(theta)) - p.g;

    % rotational dynamics (Euler equations)
    phi_ddot   = ((p.Iyy - p.Izz)/p.Ixx) * theta_dot * psi_dot + tau_phi   / p.Ixx;
    theta_ddot = ((p.Izz - p.Ixx)/p.Iyy) * phi_dot   * psi_dot + tau_theta / p.Iyy;
    psi_ddot   = ((p.Ixx - p.Iyy)/p.Izz) * phi_dot   * theta_dot + tau_psi / p.Izz;

    % state derivative
    dx = [x_pos_dot; y_pos_dot; z_pos_dot;
          phi_dot; theta_dot; psi_dot;
          x_ddot; y_ddot; z_ddot;
          phi_ddot; theta_ddot; psi_ddot;
          e_int_x_dot; e_int_y_dot; e_int_z_dot;
          e_int_phi_dot; e_int_theta_dot; e_int_psi_dot];

end