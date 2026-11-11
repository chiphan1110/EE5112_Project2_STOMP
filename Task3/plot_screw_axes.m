%% Figure for the report/presentation: the UR5 at its zero (home) position with its six screw
% axes S_i drawn as arrows, as found by computeScrewAxes.m.
% Arrow i starts at q_i (a point on joint i's axis, the origin of the frame of the link moved by
% joint i) and points along omega_i. The fixed frame {s} is drawn at the base.
% The numerical values of S_i and M are printed by verify_task3.m.
%   Output: Task3_screw_axes.png
robot_struct = loadrobot('universalUR5');
robot        = loadrobot('universalUR5','DataFormat','column');
ax = computeScrewAxes(robot_struct);

f = figure('Position', [100 100 1000 650], 'Color', 'w'); theme(f, 'light');   % light even in dark mode
a1 = axes(f); hold(a1, 'on');
show(robot, zeros(6,1), 'Parent', a1, 'PreservePlot', false, 'Frames', 'off');
set(findobj(a1, 'Type', 'patch'), 'FaceColor', [0.70 0.70 0.70], 'FaceAlpha', 0.25, 'EdgeColor', 'none');   % see-through arm

% screw axes: arrow along omega_i from q_i
cols = lines(6); L = 0.22;
for k = 1:6
    w = ax.Slist(1:3,k); q = ax.Mlist{k}(1:3,4);
    quiver3(a1, q(1), q(2), q(3), L*w(1), L*w(2), L*w(3), 0, 'Color', cols(k,:), 'LineWidth', 3.5, 'MaxHeadSize', 0.6);
    plot3(a1, q(1), q(2), q(3), 'o', 'Color', 'k', 'MarkerFaceColor', cols(k,:), 'MarkerSize', 8);
    tp = q + 1.18*L*w + [0; 0; 0.03];
    text(a1, tp(1), tp(2), tp(3), sprintf('$\\mathcal{S}_%d$', k), 'Interpreter', 'latex', ...
        'Color', cols(k,:), 'FontSize', 22);
end

% fixed frame {s} at the base
xyz = {[0.85 0.1 0.1], [0 0.6 0], [0.1 0.25 0.9]};
for c = 1:3
    d = 0.15 * double((1:3)' == c);
    quiver3(a1, 0, 0, 0, d(1), d(2), d(3), 0, 'Color', xyz{c}, 'LineWidth', 2.2, 'MaxHeadSize', 0.5);
end
text(a1, -0.07, -0.07, -0.06, '$\{s\}$', 'Interpreter', 'latex', 'FontSize', 20);

daspect(a1, [1 1 1]); axis(a1, [-0.2 1.0 -0.25 0.45 -0.15 0.35]); view(a1, -35, 25);
grid(a1, 'on'); box(a1, 'on'); camlight(a1, 'headlight');
xlabel(a1, 'x (m)'); ylabel(a1, 'y (m)'); zlabel(a1, 'z (m)');
exportgraphics(f, 'Task3_screw_axes.png', 'Resolution', 250);
