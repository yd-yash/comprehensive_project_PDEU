clear; clc; close all;

f = @(x) x(1)^2 + 2*x(2);
g = @(x) 2;
d_func = @(t) 1.5 + 0.5*cos(t);

p    = 5;
q    = 3;
beta = 1;                   
eta  = 5;                   
d_nom = 1.5;                

% Convergence time analysis:
% On the sliding surface: x1_dot + (p/(beta*q))*x2^(p/q-1)*x2_dot = 0
% => x1 + (1/beta)*x2^(p/q) = 0 (sliding manifold)
% Finite-time convergence guaranteed by the terminal attractor
% 
% fprintf('=== NTSMC Design Parameters ===\n');
% fprintf('p/q = %d/%d = %.4f  (must be in (1,2))\n', p, q, p/q);
% fprintf('beta = %.2f,  eta = %.2f\n', beta, eta);
% fprintf('Sliding surface: s = x1 + (1/%.1f)*x2^(%.4f)\n', beta, p/q);

%% ODE: NTSMC
function dxdt = ntsmc_ode(t, x, f_func, g_func, d_func, p, q, beta, eta, d_nom)
    x1 = x(1); x2 = x(2);
    d  = d_func(t);
    fx = f_func(x);
    gx = g_func(x);

    % Sliding variable
    s = x1 + (1/beta) * sign(x2) * abs(x2)^(p/q);

    % NTSMC control law (non-singular: avoids x2=0 singularity via sign(x2))
    % u_eq cancels f(x) and drives sliding surface dynamics
    % u_sw provides robustness against disturbance
    coeff = (beta * q) / (p);   % coefficient from surface derivative
    u = -(1/gx) * (fx + coeff * abs(x2)^(2 - p/q) + eta*sign(s)) - d_nom;

    dxdt = [x2;
            fx + gx*(u + d)];
end

%% Simulation
x0    = [2; 3.5];
tspan = [0 10];
opts  = odeset('MaxStep',0.001,'RelTol',1e-7,'AbsTol',1e-8);

[t, X] = ode45(@(t,x) ntsmc_ode(t,x,f,g,d_func,p,q,beta,eta,d_nom), tspan, x0, opts);

%% Post-process
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

%% Convergence time: s reaches 0
% idx_s = find(abs(s_arr) < 0.01, 1);
% idx_x = find(abs(X(:,1)) < 0.01 & abs(X(:,2)) < 0.01, 1);
% if ~isempty(idx_s); fprintf('\ns reaches ~0 at t = %.4f s\n', t(idx_s)); end
% if ~isempty(idx_x); fprintf('States reach ~0 at t = %.4f s\n', t(idx_x)); end
% fprintf('Final: x1=%.6f,  x2=%.6f\n', X(end,1), X(end,2));

%% ---- Theoretical Convergence Time ----
% On sliding surface s=0: x1 = -(1/beta)*sign(x2)*|x2|^(p/q)
% Substituting into x1_dot = x2:
%   x2_dot_slide = -(beta*q/p)*|x2|^(2-p/q)
% This is a finite-time stable ODE. Convergence time from x2(t_r):
%   T_conv = (p/(beta*q*(p/q-1))) * |x2(t_r)|^(p/q-1)
% % (once on sliding surface)
% s0 = abs(x0(1) + (1/beta)*sign(x0(2))*abs(x0(2))^(p/q));
% t_reach_theory = s0 / eta;
% fprintf('\nTheoretical reaching time bound: t_r <= %.4f s\n', t_reach_theory);


% Figure 1: State trajectories and sliding variable
figure('Name','Q4 - NTSMC','Position',[100 100 1100 800]);

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


% Figure 2: Phase portrait x2 vs x1
figure('Name','Q4 - Phase Portrait','Position',[200 200 600 500]);
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


% % Figure 3: Convergence detail (zoom first 5s)
% figure('Name','Q4 - Convergence Detail','Position',[300 100 1100 400]);
% subplot(1,3,1)
% idx5 = t<=5;
% plot(t(idx5), X(idx5,1),'b','LineWidth',1.8); hold on;
% yline(0,'k--'); xlabel('Time (s)'); ylabel('x_1'); title('x_1(t) — first 5s'); grid on;
% 
% subplot(1,3,2)
% plot(t(idx5), X(idx5,2),'m','LineWidth',1.8); hold on;
% yline(0,'k--'); xlabel('Time (s)'); ylabel('x_2'); title('x_2(t) — first 5s'); grid on;
% 
% subplot(1,3,3)
% plot(t(idx5), s_arr(idx5),'r','LineWidth',1.8); hold on;
% yline(0,'k--'); xlabel('Time (s)'); ylabel('s'); title('s(t) — first 5s'); grid on;
% 
% sgtitle('Q4 — Convergence Detail (0–5s)','FontWeight','bold');
% 
% %% ======== Stability Proof Summary (printed) ========
% fprintf('\n=== Stability Analysis Summary ===\n');
% fprintf('Lyapunov function: V = 0.5*s^2\n');
% fprintf('V_dot = s*s_dot\n');
% fprintf('      = s*[x2 + (p/(beta*q))*|x2|^(p/q-1)*x2_dot]\n');
% fprintf('      = s*[x2 + (p/(beta*q))*|x2|^(p/q-1)*(f+g*(u+d))]\n');
% fprintf('With u_NTSMC: V_dot = s*[-eta*sign(s) + g*(d-d_nom)]\n');
% fprintf('            <= -eta*|s| + |g|*|d-d_nom|*|s|\n');
% fprintf('            <= -(eta - g*delta)*|s|   where delta = max|d-d_nom|\n');
% fprintf('Since eta=%.1f > g*delta=%.1f: V_dot < 0  => s->0 in finite time\n', eta, g([0;0])*0.5);
% fprintf('Once on s=0: states converge in finite time (terminal attractor)\n');