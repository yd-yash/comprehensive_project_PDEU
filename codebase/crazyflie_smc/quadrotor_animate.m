function quadrotor_animate(t, X, p)
% QUADROTOR_ANIMATE  Real-time 3-D animation of the quadrotor trajectory.
%
%   quadrotor_animate(t, X, p)
%
%   Inputs:
%     t – N×1  time vector (s)
%     X – N×12 state history
%           X(:,1..3) = [x_pos, y_pos, z]       (m)
%           X(:,4..6) = [phi,   theta, psi]      (rad)
%     p – params struct (uses p.l for arm geometry)
%
%   Motor layout (X-configuration, top view):
%       M3 ──── M1
%         \    /
%         /    \
%       M2 ──── M4
%
%   Arms are drawn diagonally (±45° from body x-axis).
%   Frame is skipped every 5 steps for real-time speed.

    l_eff  = p.l / sqrt(2);   % projected arm length (m)
    r_prop = l_eff * 0.6;     % propeller disc display radius

    % Unit body-frame arm directions for X-config
    body_dirs = [ 1,  1,  0;    % M1  front-right
                 -1, -1,  0;    % M2  rear-left
                 -1,  1,  0;    % M3  rear-right
                  1, -1,  0];   % M4  front-left
    body_dirs = body_dirs ./ vecnorm(body_dirs, 2, 2);

    t_circle = linspace(0, 2*pi, 40);   % points for propeller disc circle

    % Axis limits with padding
    pad = 1.2 * p.l;
    xlims = [min(X(:,1)) - pad,  max(X(:,1)) + pad];
    ylims = [min(X(:,2)) - pad,  max(X(:,2)) + pad];
    zlims = [min(X(:,3)) - pad,  max(X(:,3)) + pad];

    figure('Color', 'w');
    hold on;   grid on;
    xlabel('X (m)');   ylabel('Y (m)');   zlabel('Z (m)');
    title('Quadrotor 3D Animation');
    axis equal;   view(45, 30);
    xlim(xlims);   ylim(ylims);   zlim(zlims);

    for i = 1 : 5 : length(t)   % skip every 5 frames for speed

        % State at frame i
        phi   = X(i, 4);
        theta = X(i, 5);
        psi   = X(i, 6);
        P0    = X(i, 1:3)';    % centre-of-mass position

        % ZYX rotation matrix (psi → theta → phi)
        Rx = [1,      0,       0;
              0,  cos(phi), -sin(phi);
              0,  sin(phi),  cos(phi)];
        Ry = [ cos(theta), 0, sin(theta);
                        0, 1,         0;
              -sin(theta), 0, cos(theta)];
        Rz = [cos(psi), -sin(psi), 0;
              sin(psi),  cos(psi), 0;
                     0,         0, 1];
        R  = Rz * Ry * Rx;

        % Motor hub positions in world frame
        motors = zeros(3, 4);
        for k = 1:4
            motors(:, k) = P0 + R * (l_eff * body_dirs(k,:)');
        end

        % Draw diagonal arms: M1–M2 and M3–M4
        h_a1 = plot3(motors(1,[1,2]), motors(2,[1,2]), motors(3,[1,2]), ...
                     'k-', 'LineWidth', 3);
        h_a2 = plot3(motors(1,[3,4]), motors(2,[3,4]), motors(3,[3,4]), ...
                     'k-', 'LineWidth', 3);

        % Draw propeller discs (circles in body-xy plane projected to world)
        h_props = gobjects(4, 1);
        for k = 1:4
            cx = motors(1,k) + r_prop*(cos(t_circle)*R(1,1) + sin(t_circle)*R(1,2));
            cy = motors(2,k) + r_prop*(cos(t_circle)*R(2,1) + sin(t_circle)*R(2,2));
            cz = motors(3,k) + r_prop*(cos(t_circle)*R(3,1) + sin(t_circle)*R(3,2));
            h_props(k) = plot3(cx, cy, cz, 'b-', 'LineWidth', 1.5);
        end

        % Motor hubs and body-centre marker
        h_hubs = plot3(motors(1,:), motors(2,:), motors(3,:), ...
                       'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 5);
        h_body = plot3(P0(1), P0(2), P0(3), ...
                       'rs', 'MarkerFaceColor', 'r', 'MarkerSize', 8);

        % Trajectory trail (all past positions)
        h_trail = plot3(X(1:i,1), X(1:i,2), X(1:i,3), 'b-', 'LineWidth', 1.2);

        title(sprintf('Quadrotor 3D Animation    t = %.2f s', t(i)));

        drawnow;

        % Delete dynamic objects before drawing next frame
        delete(h_a1);   delete(h_a2);
        delete(h_props);
        delete(h_hubs);   delete(h_body);
        delete(h_trail);

    end

end
