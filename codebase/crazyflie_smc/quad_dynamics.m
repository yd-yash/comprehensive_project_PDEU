function X_dot = quad_dynamics(t, x, U_faulty, dist, Omega_r, p)

    x_dot = x(7);
    y_dot = x(8);
    z_dot = x(9);
    phi = x(4); phi_dot = x(10);
    theta = x(5); theta_dot = x(11);
    psi = x(6); psi_dot = x(12);

    U1 = U_faulty(1);
    U2 = U_faulty(2);
    U3 = U_faulty(3);
    U4 = U_faulty(4);

    dTx = dist(1); dTy = dist(2); dTz = dist(3);
    dRx = dist(4); dRy = dist(5); dRz = dist(6);

    % dynamics
    x_ddot = (U1/p.m)*(cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi))- (p.Kx/p.m)*x_dot + dTx/p.m;

    y_ddot = (U1/p.m)*(cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi)) - (p.Ky/p.m)*y_dot + dTy/p.m;

    z_ddot = (U1/p.m)*(cos(phi)*cos(theta)) - p.g - (p.Kz/p.m)*z_dot + dTz/p.m;

    phi_ddot = ((p.Iyy - p.Izz)/p.Ixx)*theta_dot*psi_dot - (p.Kphi/p.Ixx)*phi_dot*abs(phi_dot) - (p.Ir/p.Ixx)*Omega_r*theta_dot + U2/p.Ixx + dRx/p.Ixx;

    theta_ddot = ((p.Izz - p.Ixx)/p.Iyy)*phi_dot*psi_dot - (p.Ktheta/p.Iyy)*theta_dot*abs(theta_dot) + (p.Ir/p.Iyy)*Omega_r*phi_dot + U3/p.Iyy + dRy/p.Iyy;

    psi_ddot = ((p.Ixx - p.Iyy)/p.Izz)*phi_dot*theta_dot - (p.Kpsi/p.Izz)*psi_dot*abs(psi_dot) + U4/p.Izz + dRz/p.Izz;

    X_dot = [x_dot; y_dot; z_dot;
             phi_dot; theta_dot; psi_dot;
             x_ddot; y_ddot; z_ddot;
             phi_ddot; theta_ddot; psi_ddot];

end