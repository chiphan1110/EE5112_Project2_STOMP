%% Figure for the report/presentation: the UR5 at its ZERO configuration with its six screw
% axes (joint axes) drawn as arrows, as found by computeScrewAxes.m.
% Arrow k starts at q_k (a point on joint k's axis, the origin of the body frame of joint k)
% and points along omega_k. The table on the right lists S_k = [omega_k; v_k], v_k = -omega_k x q_k.
%   Output: Task3_screw_axes.png
robot_struct = loadrobot('universalUR5');
robot        = loadrobot('universalUR5','DataFormat','column');
ax = computeScrewAxes(robot_struct);

f = figure('Position', [100 100 1400 650], 'Color', 'w'); theme(f, 'light');   % light even in dark mode
tl = tiledlayout(f, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

% left (2 tiles): robot and arrows
a1 = nexttile(tl, [1 2]); hold(a1, 'on');
show(robot, zeros(6,1), 'Parent', a1, 'PreservePlot', false, 'Frames', 'off');
set(findobj(a1, 'Type', 'patch'), 'FaceColor', [0.70 0.70 0.70], 'FaceAlpha', 0.25, 'EdgeColor', 'none');   % see-through arm
cols = lines(6); L = 0.22;
for k = 1:6
    w = ax.Slist(1:3,k); q = ax.Mlist{k}(1:3,4);
    quiver3(a1, q(1), q(2), q(3), L*w(1), L*w(2), L*w(3), 0, 'Color', cols(k,:), 'LineWidth', 3.5, 'MaxHeadSize', 0.6);
    plot3(a1, q(1), q(2), q(3), 'o', 'Color', 'k', 'MarkerFaceColor', cols(k,:), 'MarkerSize', 8);
    tp = q + 1.18*L*w + [0; 0; 0.03];
    text(a1, tp(1), tp(2), tp(3), sprintf('S_%d', k), 'Color', cols(k,:), 'FontSize', 15, 'FontWeight', 'bold');
end
% base frame {s}
quiver3(a1, 0,0,0, 0.15,0,0, 0, 'r', 'LineWidth', 2); quiver3(a1, 0,0,0, 0,0.15,0, 0, 'Color', [0 0.6 0], 'LineWidth', 2);
quiver3(a1, 0,0,0, 0,0,0.15, 0, 'b', 'LineWidth', 2);
text(a1, -0.06, -0.06, -0.05, '\{s\}', 'FontSize', 14);
daspect(a1, [1 1 1]); axis(a1, [-0.2 1.0 -0.25 0.45 -0.15 0.35]); view(a1, -35, 25);
grid(a1, 'on'); box(a1, 'on'); camlight(a1, 'headlight');
xlabel(a1, 'x (m)'); ylabel(a1, 'y (m)'); zlabel(a1, 'z (m)');
title(a1, 'UR5 at the zero configuration: arrow = \omega_k, dot = q_k', 'FontSize', 13);

% right (1 tile): table of the screw axes
a2 = nexttile(tl); axis(a2, 'off'); xlim(a2, [0 1]); ylim(a2, [0 1]);
text(a2, 0.0, 0.97, 'Screw axes S_k = [\omega_k ; v_k],  v_k = -\omega_k \times q_k', 'FontSize', 13, 'FontWeight', 'bold');
hdr = {'', '\omega_x', '\omega_y', '\omega_z', 'v_x', 'v_y', 'v_z'};
xs = [0.00 0.13 0.24 0.35 0.50 0.69 0.88];          % narrow columns for omega (0/±1), wide ones for v
for c = 1:7, text(a2, xs(c), 0.88, hdr{c}, 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'left'); end
for k = 1:6
    y = 0.88 - 0.075*k;
    text(a2, xs(1), y, sprintf('S_%d', k), 'FontSize', 12, 'FontWeight', 'bold', 'Color', cols(k,:));
    for c = 1:6
        val = ax.Slist(c,k); if abs(val) < 5e-5, val = 0; end
        if c <= 3, s = sprintf('%d', round(val)); else, s = sprintf('%.4f', val); end
        text(a2, xs(c+1), y, s, 'FontSize', 11, 'FontName', 'Courier');
    end
end
Mee = ax.Mee; Mee(abs(Mee) < 5e-5) = 0;
text(a2, 0.0, 0.30, 'End-effector (tool0) pose at zero, M:', 'FontSize', 12, 'FontWeight', 'bold');
for r = 1:4
    text(a2, 0.05, 0.30 - 0.055*r, sprintf('%8.4f %8.4f %8.4f %8.4f', Mee(r,:)), 'FontSize', 11, 'FontName', 'Courier');
end
text(a2, 0.0, 0.0, 'units: m (v, M translation); \omega dimensionless', 'FontSize', 10, 'Color', [0.4 0.4 0.4]);

title(tl, 'Task 3: screw axes of the UR5 used by the Product-of-Exponentials forward kinematics', 'FontSize', 15, 'FontWeight', 'bold');
exportgraphics(f, 'Task3_screw_axes.png', 'Resolution', 200);
