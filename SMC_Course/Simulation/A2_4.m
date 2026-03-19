% Q. 4
clear; clc; close all;

f = @(x) x(1)^2 + 2*x(2);
g = @(x) 2;
d_func = @(t) 1.5 + 0.5*cos(t);

p    = 5;
q    = 3;
beta = 1;                   
eta  = 5;                   
d_nom = 1.5;                

function dxdt = ntsmc_ode(t, x, f_func, g_func, d_func, p, q, beta, eta, d_nom)
    x1 = x(1); x2 = x(2);
    d  = d_func(t);
    fx = f_func(x);
    gx = g_func(x);

    s = x1 + (1/beta) * sign(x2) * abs(x2)^(p/q);
    coeff = (beta * q) / (p);   % coefficient from surface derivative
    u = -(1/gx) * (fx + coeff * abs(x2)^(2 - p/q) + eta*sign(s)) - d_nom;

    dxdt = [x2;
            fx + gx*(u + d)];
end

x0    = [2; 3.5];
tspan = [0 10];
opts  = odeset('MaxStep',0.001,'RelTol',1e-7,'AbsTol',1e-8);

[t, X] = ode45(@(t,x) ntsmc_ode(t,x,f,g,d_func,p,q,beta,eta,d_nom), tspan, x0, opts);

s_arr = X(:,1) + (1/beta)*sign(X(:,2)).*abs(X(:,2)).^(p/q);
u_arr = zeros(length(t),1);
d_arr = zeros(length(t),1);
for i = 1:length(t)
    xi = X(i,:)';
    fx = f(xi); gx = g(xi);
    di = d_func(t(i));
    d_arr(i) = di;
    si = s_arr(i);
    coeff = (beta*q)/p;
    u_arr(i) = -(1/gx)*(fx + coeff*abs(xi(2))^(2-p/q) + eta*sign(si)) - d_nom;
end

% State trajectories and sliding variable
figure('Name','NTSMC','Position',[100 100 1100 800]);

subplot(2,2,1)
plot(t, X(:,1),'b','LineWidth',2); hold on;
yline(0,'k--','LineWidth',0.8);
xlabel('Time (s)'); ylabel('x_1 [m]'); title('State x_1(t)'); grid on;

subplot(2,2,2)
plot(t, X(:,2),'m','LineWidth',2); hold on;
yline(0,'k--','LineWidth',0.8);
xlabel('Time (s)'); ylabel('x_2 [m/s]'); title('State x_2(t)'); grid on;

subplot(2,2,3)
plot(t, s_arr,'r','LineWidth',1.8); hold on;
yline(0,'k--','LineWidth',0.8);
xlabel('Time (s)'); ylabel('s'); title('Sliding Variable s(t)'); grid on;

subplot(2,2,4)
plot(t, u_arr,'Color',[0.1 0.6 0.1],'LineWidth',1.5);
xlabel('Time (s)'); ylabel('u'); title('Control Input u(t)'); grid on;


% Phase portrait x2 vs x1
figure('Name','Phase Portrait','Position',[200 200 600 500]);
plot(X(:,1), X(:,2),'b','LineWidth',2); hold on;
plot(X(1,1),  X(1,2),  'go','MarkerSize',10,'MarkerFaceColor','g','DisplayName','Start');
plot(X(end,1),X(end,2),'rs','MarkerSize',10,'MarkerFaceColor','r','DisplayName','End');

% Plot sliding manifold (s=0 curve): x1 = -(1/beta)*sign(x2)*|x2|^(p/q)
x2_range = linspace(-4, 4, 400);
x1_manifold = -(1/beta)*sign(x2_range).*abs(x2_range).^(p/q);
plot(x1_manifold, x2_range,'r--','LineWidth',1.5,'DisplayName','Sliding manifold s=0');

xlabel('x_1'); ylabel('x_2'); title('Phase Portrait');
legend('Trajectory','Start','End','s=0 manifold','Location','best');
grid on;
