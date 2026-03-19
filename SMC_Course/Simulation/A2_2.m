% Q. 2
clear; clc; close all;

m      = 4;
c      = 1;
eta    = 2;
k_func = @(t) 1.5 + 0.4*sin(2*t);
x0     = [2; 0.5];
tspan  = [0 10];
opts   = odeset('MaxStep', 0.001, 'RelTol', 1e-6);
eps_vals = [0.1, 0.5];         

% 1. Ideal sign
[t_s, X_s] = ode45(@(t,x) dyn(t,x,m,c,eta,k_func,'sign',0),   tspan, x0, opts);

% 2. Sigmoid and Saturation for each epsilon
results = struct();
results(1).label = 'sign(s)';        results(1).t = t_s; results(1).X = X_s; results(1).type='sign'; results(1).eps=0;
idx = 2;
for eps = eps_vals
    [t_sig, X_sig] = ode45(@(t,x) dyn(t,x,m,c,eta,k_func,'sigmoid',eps), tspan, x0, opts);
    [t_sat, X_sat] = ode45(@(t,x) dyn(t,x,m,c,eta,k_func,'sat',   eps), tspan, x0, opts);
    results(idx).label = sprintf('sigmoid \\epsilon=%.1f',eps); results(idx).t=t_sig; results(idx).X=X_sig; results(idx).type='sigmoid'; results(idx).eps=eps; idx=idx+1;
    results(idx).label = sprintf('sat \\epsilon=%.1f',   eps); results(idx).t=t_sat; results(idx).X=X_sat; results(idx).type='sat';     results(idx).eps=eps; idx=idx+1;
end

% comparison at eps=0.1
cols  = {'k','b','r'};
types = {'sign','sigmoid','sat'};
lbls  = {'sign(s)', 'sigmoid \epsilon=0.1', 'sat \epsilon=0.1'};

figure('Name','Comparison eps=0.1','Position',[100 100 1100 750]);
vars  = {'s(t)','u(t)','x_1(t)','x_2(t)'};
for vi = 1:4
    subplot(2,2,vi); hold on;
    for ri = [1 2 4]   % sign, sigmoid_0.1, sat_0.1
        r = results(ri);
        s_arr = c*r.X(:,1) + r.X(:,2);
        u_arr = compute_u(r.t, r.X, m, c, eta, k_func, r.type, r.eps);
        data  = {s_arr, u_arr, r.X(:,1), r.X(:,2)};
        ls = '--'; if ri>1; ls='-'; end
        plot(r.t, data{vi}, 'Color', cols{find([1 2 4]==ri)}, 'LineWidth',1.5,'LineStyle',ls,'DisplayName',lbls{find([1 2 4]==ri)});
    end
    yline(0,'Color',[0.6 0.6 0.6],'LineWidth',0.5);
    xlabel('Time (s)'); ylabel(vars{vi}); title(vars{vi}); legend('Location','best','FontSize',8); grid on;
end

% effect of epsilon on u(t)
figure('Name','epsilon effect','Position',[100 100 1200 500]);
ep_cols = {'b','g'};
for pi = 1:2
    subplot(1,2,pi);
    appr = {'sigmoid','sat'}; ttl = {'Sigmoid','Saturation'};
    % ideal sign
    u_sign = compute_u(t_s, X_s, m, c, eta, k_func, 'sign', 0);
    plot(t_s, u_sign,'k--','LineWidth',1.5,'DisplayName','sign(s)'); hold on;
    for ei = 1:length(eps_vals)
        ri = 1 + (ei-1)*2 + (pi-1) + 1;  % index into results
        r  = results(ri);
        u_a = compute_u(r.t, r.X, m, c, eta, k_func, r.type, r.eps);
        plot(r.t, u_a,'Color',ep_cols{ei},'LineWidth',1.2,'DisplayName',sprintf('\\epsilon=%.1f',r.eps)); hold on;
    end
    xlabel('Time (s)'); ylabel('u [N]'); title(ttl{pi}); legend('Location','best'); grid on;
end

function dxdt = dyn(t, x, m, c, eta, k_func, type, eps)
    x1=x(1); x2=x(2); k=k_func(t);
    s = c*x1 + x2;
    switch type
        case 'sign',    sgn = sign(s);
        case 'sigmoid', sgn = s/(abs(s)+eps);
        case 'sat',     sgn = sat_func(s,eps);
    end
    u    = k*x2*abs(x2) - m*c*x2 - m*eta*sgn;
    dxdt = [x2; (u - k*x2*abs(x2))/m];
end

function u_arr = compute_u(t, X, m, c, eta, k_func, type, eps)
    u_arr = zeros(length(t),1);
    for i=1:length(t)
        x2=X(i,2); k=k_func(t(i)); s=c*X(i,1)+x2;
        switch type
            case 'sign',    sgn=sign(s);
            case 'sigmoid', sgn=s/(abs(s)+eps);
            case 'sat',     sgn=sat_func(s,eps);
        end
        u_arr(i) = k*x2*abs(x2) - m*c*x2 - m*eta*sgn;
    end
end

function v = sat_func(s, eps)
    if s > eps;      v =  1;
    elseif s < -eps; v = -1;
    else;            v = s/eps;
    end
end