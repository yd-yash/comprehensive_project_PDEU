function a_dot = quad_discrete_function(t, a)

global m Jy Jx Jz g ez sz u1 ephi sphi u2 etheta stheta u3 ...
    epsi spsi u4 ex sx ey sy

% system variables
phi = a(1);
phi_dot = a(4);
theta = a(2);
theta_dot = a(5);
psi = a(3);
psi_dot = a(6);
x = a(7);
x_dot = a(10);
y = a(8);
y_dot = a(11);
z = a(9);
z_dot = a(12);


% Disturbance model
% Select ONE block by uncommenting it; keep all others commented out.

% 1. No disturbance (default)
% dTx = 0;  dTy = 0;  dTz = 0;
% dRx = 0;  dRy = 0;  dRz = 0;

% 2. Sinusoidal disturbance
% ATx = 0.5;  fTx = 0.5;   ATy = 0.5;  fTy = 0.5;   ATz = 0.3;  fTz = 0.3;
% ARx = 0.01; fRx = 0.7;   ARy = 0.01; fRy = 0.7;   ARz = 0.01; fRz = 0.5;
% dTx = ATx*sin(2*pi*fTx*t);  dTy = ATy*sin(2*pi*fTy*t);  dTz = ATz*sin(2*pi*fTz*t);
% dRx = ARx*sin(2*pi*fRx*t);  dRy = ARy*sin(2*pi*fRy*t);  dRz = ARz*sin(2*pi*fRz*t);

% 3. Random (noise) disturbance — original reference values
% dTx = 0.1 + 0.1*randn;  dTy = 0.1 + 0.1*randn;  dTz = 0.5*sin(t) + 0.1*randn;
% dRx = 0;                 dRy = 0;                 dRz = 0;

% dynamics
% phi_ddot   = (Jy-Jz)/Jx * psi_dot*theta_dot   + u2/Jx + dRx/Jx;
% theta_ddot = (Jz-Jx)/Jy * phi_dot*psi_dot     + u3/Jy + dRy/Jy;
% psi_ddot   = (Jx-Jy)/Jz * phi_dot*theta_dot   + u4/Jz + dRz/Jz;
% 
% x_ddot = (u1/m)*(cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi)) + dTx/m;
% y_ddot = (u1/m)*(cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi)) + dTy/m;
% z_ddot = (u1/m)*(cos(phi)*cos(theta)) - g + dTz/m;

phi_ddot = (Jy-Jz)/Jx*psi_dot*theta_dot+u2/Jx+0;
theta_ddot = (Jz-Jx)/Jy*phi_dot*psi_dot+u3/Jy+0;
psi_ddot = (Jx-Jy)/Jz*phi_dot*theta_dot+u4/Jz+0;
z_ddot = (u1/m)*(cos(phi)*cos(theta))-g;  %+ 0.5*sin(t) +0.1*randn(1);
x_ddot = (u1/m)*(cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)); %+0.1+0.1*randn(1);
y_ddot = (u1/m)*(cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi)); %+0.1+0.1*randn(1);

% Interal terms
I_u1dot  = sign(sz);
Is_zdot  = ez;

I_u2dot    = sign(sphi);
Is_phidot  = ephi;

I_u3dot     = sign(stheta);
Is_thetadot = etheta;

I_u4dot   = sign(spsi);
Is_psidot = epsi;

I_uxdot = sign(sx);
Is_xdot = ex;

I_uydot = sign(sy);
Is_ydot = ey;

a_dot = [phi_dot theta_dot psi_dot phi_ddot theta_ddot psi_ddot ...
         x_dot y_dot  z_dot x_ddot y_ddot z_ddot I_u1dot Is_zdot ...
         I_u3dot Is_thetadot I_u2dot Is_phidot I_u4dot Is_psidot ...
         I_uxdot Is_xdot I_uydot Is_ydot]';
end
