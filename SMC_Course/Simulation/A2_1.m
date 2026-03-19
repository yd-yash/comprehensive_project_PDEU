% Q. 1
clear; clc; close all;

m   = 4;                        
c   = 1;                        
eta = 2;                        
k_func = @(t) 1.5 + 0.4*sin(2*t);  

ode_smc = @(t, x) dynamics(t, x, m, c, eta, k_func);

x0     = [2; 0.5];             
tspan  = [0 10];
opts   = odeset('MaxStep', 0.001, 'RelTol', 1e-6);
[t, X] = ode45(ode_smc, tspan, x0, opts);

s_arr = c*X(:,1) + X(:,2);
u_arr = zeros(length(t),1);
for i = 1:length(t)
    k = k_func(t(i));
    x2 = X(i,2);
    s  = s_arr(i);
    u_arr(i) = k*x2*abs(x2) - m*c*x2 - m*eta*sign(s);
end

figure('Name','Q1 - SMC Underwater Vehicle','Position',[100 100 1000 700]);

subplot(2,2,1)
plot(t, s_arr, 'b', 'LineWidth', 1.5); hold on
yline(0,'k--','LineWidth',0.8);
xlabel('Time (s)'); ylabel('s'); title('Sliding Variable s(t)');
grid on;

subplot(2,2,2)
plot(t, u_arr, 'r', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('u [N]'); title('Control Input u(t)');
grid on;

subplot(2,2,3)
plot(t, X(:,1), 'g', 'LineWidth', 1.5); hold on
yline(0,'k--','LineWidth',0.8);
xlabel('Time (s)'); ylabel('x_1 [m]'); title('Position x_1(t)');
grid on;

subplot(2,2,4)
plot(t, X(:,2), 'm', 'LineWidth', 1.5); hold on
yline(0,'k--','LineWidth',0.8);
xlabel('Time (s)'); ylabel('x_2 [m/s]'); title('Velocity x_2(t)');
grid on;

function dxdt = dynamics(t, x, m, c, eta, k_func)
    x1 = x(1); x2 = x(2);
    k  = k_func(t);
    s  = c*x1 + x2;
    u  = k*x2*abs(x2) - m*c*x2 - m*eta*sign(s);
    dx1 = x2;
    dx2 = (u - k*x2*abs(x2)) / m;
    dxdt = [dx1; dx2];
end