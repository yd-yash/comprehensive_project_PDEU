clc; clear; close all;

m = 4;
c = 2;              
K = 8;              
k1 = 1.5;           

x0 = [2; 0.5];

tspan = [0 20];


eps = 0.05;

[t2, x2_sat] = ode45(@(t,x) dynamics_sat_q2(t,x,m,c,K,k1,eps), tspan, x0);

x1 = x(:,1);
x2 = x(:,2);

s = zeros(length(t),1);
u = zeros(length(t),1);

for i = 1:length(t)
    k = 1.5 + 0.4*sin(2*t(i)); 
    s(i) = c*x1(i) + x2(i);
    
    u(i) = k1*x2(i)*abs(x2(i)) - m*c*x2(i) - K*sign(s(i));
end

figure;

subplot(2,2,1);
plot(t, x1, 'LineWidth',1.5);
xlabel('Time'); ylabel('x_1');
title('Position');

subplot(2,2,2);
plot(t, x2, 'LineWidth',1.5);
xlabel('Time'); ylabel('x_2');
title('Velocity');

subplot(2,2,3);
plot(t, s, 'LineWidth',1.5);
xlabel('Time'); ylabel('s');
title('Sliding Variable');

subplot(2,2,4);
plot(t, u, 'LineWidth',1.5);
xlabel('Time'); ylabel('u');
title('Control Input');


function dx = dynamics_sat_q2(t, x, m, c, K, k1, eps)

x1 = x(1);
x2 = x(2);

k = 1.5 + 0.4*sin(2*t);

s = c*x1 + x2;

% Saturation function
if abs(s) > eps
    sat_s = sign(s);
else
    sat_s = s/eps;
end

u = k1*x2*abs(x2) - m*c*x2 - K*sat_s;

dx = zeros(2,1);
dx(1) = x2;
dx(2) = (1/m)*(u - k*x2*abs(x2));

end
