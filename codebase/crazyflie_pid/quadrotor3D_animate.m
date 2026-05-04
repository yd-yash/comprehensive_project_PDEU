% 3D Animation of Quadrotor Flight
% INPUTS:
%   t - time vector
%   x - state matrix (Nx18)

function quadrotor3D_animate(t, x)

    figure('Color', 'w');
    axis equal;
    axis([-5, 12, -3, 6, -5, 5]);
    view(45, 25);
    grid on;
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('Quadrotor 3D');
    hold on;

    % Plot full trajectory trace
    plot3(x(:, 1), x(:, 2), x(:, 3), 'b--', 'LineWidth', 1);

    % Animated handles
    h_arm1   = plot3([0 0], [0 0], [0 0], 'k-',  'LineWidth', 2);
    h_arm2   = plot3([0 0], [0 0], [0 0], 'k-',  'LineWidth', 2);
    h_center = plot3(0, 0, 0, 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');
    h_time   = text(1, 1.5, 3.5, '', 'FontSize', 10);

    L = 0.3;  % visual arm length

    for i = 1:5:length(t)
        pos   = x(i, 1:3);
        phi   = x(i, 4);
        theta = x(i, 5);
        psi   = x(i, 6);

        R = eul2rotm([psi, theta, phi], 'ZYX');

        arm_x = R * [L; 0; 0];
        arm_y = R * [0; L; 0];

        p1a = pos' + arm_x;
        p1b = pos' - arm_x;
        p2a = pos' + arm_y;
        p2b = pos' - arm_y;

        set(h_arm1,   'XData', [p1a(1), p1b(1)], 'YData', [p1a(2), p1b(2)], 'ZData', [p1a(3), p1b(3)]);
        set(h_arm2,   'XData', [p2a(1), p2b(1)], 'YData', [p2a(2), p2b(2)], 'ZData', [p2a(3), p2b(3)]);
        set(h_center, 'XData', pos(1),            'YData', pos(2),            'ZData', pos(3));
        set(h_time,   'String', sprintf('t = %.2f s', t(i)));

        drawnow;
    end

end