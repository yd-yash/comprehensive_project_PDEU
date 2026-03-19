clear; clc; close all;

A = [0 1 0;
     0 0 1;
     1 3 5];

B = [2 1;
     1 0;
     1 2];

d_func = @(t) [2*sin(t)+cos(t); 1+cos(2*t)];

%  u = -K*x 
desired_poles = [-3, -4, -5];
K_sf = place(A, B, desired_poles);   
eigs_cl = eig(A - B*K_sf);

% SMC 
C     = [1 4 0; 0 1 4];              
CB    = C * B;
CB_inv = inv(CB);
K_eq   = CB_inv * C * A;        

% Disturbance bound -> choose eta
d_max = [3; 2];
eta   = max(norm(CB_inv) * norm(C) * norm(d_max) + 1, 15);

x0    = [2; 5; 3];
tspan = [0 12];
opts  = odeset('MaxStep', 0.001, 'RelTol', 1e-6);

% Run all four controllers
[t1, X1] = ode45(@(t,x) ode_q3(t,x,A,B,K_sf,K_eq,CB_inv,C,eta,d_func,'sf',     0),    tspan, x0, opts);
[t2, X2] = ode45(@(t,x) ode_q3(t,x,A,B,K_sf,K_eq,CB_inv,C,eta,d_func,'sign',   0),    tspan, x0, opts);
[t3, X3] = ode45(@(t,x) ode_q3(t,x,A,B,K_sf,K_eq,CB_inv,C,eta,d_func,'sat',    0.01), tspan, x0, opts);
[t4, X4] = ode45(@(t,x) ode_q3(t,x,A,B,K_sf,K_eq,CB_inv,C,eta,d_func,'sigmoid',0.1),  tspan, x0, opts);

T  = {t1,t2,t3,t4};
Xs = {X1,X2,X3,X4};
labels = {'State Feedback u=-Kx','SMC sign(s)','SMC sat(s,0.01)','SMC sigmoid(s,0.1)'};
cols   = {'b','k','r','g'};
ls_arr = {'--','-','-','-'};


% states
figure('Name','States','Position',[50 50 1100 850]);
state_names = {'x_1(t)','x_2(t)','x_3(t)'};
for si = 1:3
    subplot(3,1,si); hold on;
    for ci = 1:4
        plot(T{ci}, Xs{ci}(:,si), 'Color',cols{ci},'LineWidth',1.8,'LineStyle',ls_arr{ci},'DisplayName',labels{ci});
    end
    yline(0,'Color',[0.6 0.6 0.6],'LineWidth',0.5,'LineStyle',':');
    ylabel(state_names{si},'FontSize',12); grid on;
    legend('Location','best','FontSize',8);
    if si==3; xlabel('Time (s)','FontSize',11); end
end

% Sliding Variables
figure('Name','Sliding Variables','Position',[50 50 1200 700]);
for ci = 1:4
    subplot(2,2,ci); hold on;
    s_vals = (C * Xs{ci}')';   
    plot(T{ci}, s_vals(:,1),'b','LineWidth',1.5,'DisplayName','s_1');
    plot(T{ci}, s_vals(:,2),'r','LineWidth',1.5,'DisplayName','s_2');
    yline(0,'Color',[0.6 0.6 0.6],'LineWidth',0.5,'LineStyle',':');
    title(labels{ci},'FontSize',10); xlabel('Time (s)'); ylabel('s');
    legend('Location','best','FontSize',9); grid on;
end

% Control Inputs 
figure('Name','Q3 - Control Inputs','Position',[50 50 1200 700]);
ctrl_types = {'sf','sign','sat','sigmoid'};
eps_list   = [0, 0, 0.01, 0.1];
for ci = 1:4
    subplot(2,2,ci); hold on;
    u1_arr = zeros(length(T{ci}),1);
    u2_arr = zeros(length(T{ci}),1);
    for k=1:length(T{ci})
        x = Xs{ci}(k,:)';
        s = C*x;
        u = compute_u_q3(x,s,K_sf,K_eq,CB_inv,eta,ctrl_types{ci},eps_list(ci));
        u1_arr(k)=u(1); u2_arr(k)=u(2);
    end
    plot(T{ci}, u1_arr,'Color','red','LineWidth',1.2,'DisplayName','u_1');
    plot(T{ci}, u2_arr,'Color','blue','LineWidth',1.2,'DisplayName','u_2');
    yline(0,'Color',[0.6 0.6 0.6],'LineWidth',0.5,'LineStyle',':');
    title(labels{ci},'FontSize',10); xlabel('Time (s)'); ylabel('u');
    legend('Location','best','FontSize',9); grid on;
end

function dxdt = ode_q3(t, x, A, B, K_sf, K_eq, CB_inv, C, eta, d_func, type, eps)
    d = d_func(t);
    s = C * x;
    u = compute_u_q3(x, s, K_sf, K_eq, CB_inv, eta, type, eps);
    dxdt = A*x + B*(u + d);
end

function u = compute_u_q3(x, s, K_sf, K_eq, CB_inv, eta, type, eps)
    switch type
        case 'sf'
            u = -K_sf * x;
        case 'sign'
            u = -(K_eq*x) - CB_inv*(eta*sign(s));
        case 'sat'
            u = -(K_eq*x) - CB_inv*(eta*sat_vec(s,eps));
        case 'sigmoid'
            u = -(K_eq*x) - CB_inv*(eta*(s./(abs(s)+eps)));
    end
end

function v = sat_vec(s, eps)
    v = zeros(size(s));
    for i = 1:length(s)
        if s(i) > eps;       v(i) =  1;
        elseif s(i) < -eps;  v(i) = -1;
        else;                 v(i) = s(i)/eps;
        end
    end
end