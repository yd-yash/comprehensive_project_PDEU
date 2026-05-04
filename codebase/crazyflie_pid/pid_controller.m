% Cascaded PID Controller for 3D Quadrotor
% To implement a DIFFERENT controller (e.g. LQR, SMC, MPC):
%   1. Create a new file: your_controller.m with the SAME signature
%   2. In main.m, replace @pid_controller with @your_controller
%   Nothing else needs to change.
% INPUTS:
%   t      - current time (scalar)
%   x      - state vector (18x1)
%   p      - params struct
% OUTPUTS:
%   u      - control input struct with fields:
%              u.u1   - total thrust command (N)
%              u.u2   - roll torque command  (Nm)
%              u.u3   - pitch torque command (Nm)
%              u.u4   - yaw torque command   (Nm)
%              u.phi_des   - desired roll angle (rad)
%              u.theta_des - desired pitch angle (rad)
%              u.psi_des   - desired yaw angle (rad)

function u = pid_controller(t, x, p)

    % states 
    x_pos     = x(1);   y_pos    = x(2);   z_pos    = x(3);
    phi       = x(4);   theta    = x(5);   psi      = x(6);
    x_pos_dot = x(7);   y_pos_dot= x(8);   z_pos_dot= x(9);
    phi_dot   = x(10);  theta_dot= x(11);  psi_dot  = x(12);
    e_int_x   = x(13);  e_int_y  = x(14);  e_int_z  = x(15);
    e_int_phi = x(16);  e_int_theta = x(17); e_int_psi = x(18);

    % reference trajectory 
    x_des     = p.x_des(t);
    y_des     = p.y_des(t);
    z_des     = p.z_des(t);
    x_dot_des = p.x_dot_des(t);
    y_dot_des = p.y_dot_des(t);
    z_dot_des = p.z_dot_des(t);

    % position errors 
    e_x     = x_des - x_pos;
    e_x_dot = x_dot_des - x_pos_dot;
    e_y     = y_des - y_pos;
    e_y_dot = y_dot_des - y_pos_dot;
    e_z     = z_des - z_pos;
    e_z_dot = z_dot_des - z_pos_dot;


    % PID gains
    Kp_x = 4;   Ki_x = 0.5;   Kd_x = 5;
    Kp_y = 4;   Ki_y = 0.5;   Kd_y = 5;
    Kp_z = 12;  Ki_z = 2;     Kd_z = 6;

    Kp_phi   = 0.01;  Ki_phi   = 0.0005;  Kd_phi   = 0.0008;
    Kp_theta = 0.010;  Ki_theta = 0.0005;  Kd_theta = 0.0008;
    Kp_psi   = 0.018;  Ki_psi   = 0.0008;   Kd_psi   = 0.0013;

    % outer loop: desired (commanded) acceleration (position PID) 
    x_ddot_cmd = Kp_x * e_x + Kd_x * e_x_dot + Ki_x * e_int_x;
    y_ddot_cmd = Kp_y * e_y + Kd_y * e_y_dot + Ki_y * e_int_y;
    z_ddot_cmd = Kp_z * e_z + Kd_z * e_z_dot + Ki_z * e_int_z;

    % desired roll & pitch from outer loop (small angle approx)
    phi_des   = -(1/p.g) * (y_ddot_cmd);
    theta_des =  (1/p.g) * (x_ddot_cmd);
    psi_des   = p.psi_des(t);

    % desired roll & pitch rates — feedforward jerk from reference trajectory
    % phi_dot_des   = -(1/g) * d(y_ddot_cmd)/dt
    % theta_dot_des =  (1/g) * d(x_ddot_cmd)/dt
    % Only the feedforward (reference jerk) contribution is used here.
    % The feedback contribution requires actual jerk of the quadrotor
    % (not available as a state), so it is intentionally omitted.
    % This is exact for the feedforward path and standard in cascaded PID design.
    phi_dot_des   = -(1/p.g) * p.y_dddot_des(t);
    theta_dot_des =  (1/p.g) * p.x_dddot_des(t);
    psi_dot_des   = 0;  
    % psi_des = 0 always, so psi_dot_des = 0 exactly
    
    % total thrust (u1)
    u1 = p.m * (p.g + z_ddot_cmd);

    % clamp u1 to physical rotor limits (4 rotors)
    u1 = max(0, min(u1, 4 * p.T_max));
 

    % inner loop: attitude PID (torques) 
    u2 = Kp_phi   * (phi_des   - phi)   + Kd_phi   * (phi_dot_des - phi_dot)   + Ki_phi   * e_int_phi;
    u3 = Kp_theta * (theta_des - theta) + Kd_theta * (theta_dot_des - theta_dot) + Ki_theta * e_int_theta;
    u4 = Kp_psi   * (psi_des   - psi)   + Kd_psi   * (psi_dot_des - psi_dot)   + Ki_psi   * e_int_psi;

    % outputs 
    u.u1        = u1;
    u.u2        = u2;
    u.u3        = u3;
    u.u4        = u4;
    u.phi_des   = phi_des;
    u.theta_des = theta_des;
    u.psi_des   = psi_des;

end