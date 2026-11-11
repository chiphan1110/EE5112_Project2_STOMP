% Make the Task 4 figures for the current scenario (run right after UR5_STOMP_Path_Planning.m;
% uses its workspace):
%   Task4_<name>_overlay.png : straight-line initial guess vs STOMP path. The UR5 is drawn at
%                              several waypoints (light -> dark purple = start -> goal; magenta =
%                              pose in collision), end-effector path in black, obstacles in the
%                              scenario colours.
%   Task4_<name>_cost.png    : total cost Q(theta) per iteration (iteration 0 = initial guess) of
%                              the attempt that produced the final path
% Added for the report only.

%% Initial guess (straight line in joint space) and its collisions
th_init = zeros(numJoints, nDiscretize);
for k = 1:numJoints, th_init(k,:) = linspace(q0(k), qT(k), nDiscretize); end
collideInit = false(1, nDiscretize);
for i = 1:nDiscretize
    collideInit(i) = checkCollision(robot, th_init(:,i), world, 'IgnoreSelfCollision','on');
end
collideFinal = inCollision(:)';                     % from helperSTOMP (last attempt)
paths    = {th_init, theta};
collides = {collideInit, collideFinal};
attemptTxt = '';
if nAttempts > 1, attemptTxt = sprintf(', attempt %d of %d', nAttempts, maxRestarts); end
titles   = {sprintf('Initial guess (straight line): %d of %d waypoints collide', nnz(collideInit), nDiscretize), ...
            sprintf('After STOMP (%d iterations%s): %d of %d waypoints collide', iter, attemptTxt, nnz(collideFinal), nDiscretize)};
scTitle  = strrep(sc.name, '_', ' ');

%% Figure 1: overlay of intermediate configurations
showIdx  = round(linspace(1, nDiscretize, 7));     % start, 5 intermediate, goal
ghostCol = interp1([0 1], [0.82 0.76 0.94; 0.33 0.15 0.55], linspace(0, 1, numel(showIdx)));
collCol  = [0.95 0.10 0.65];                        % magenta: differs from all obstacle colours
f = figure('Position', [50 50 1500 720], 'Color', 'w'); theme(f, 'light');  % light even in dark mode
tl = tiledlayout(f, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for p = 1:2
    ax = nexttile(tl); hold(ax, 'on');
    % robot ghosts, coloured by time (magenta if that pose collides)
    for n = 1:numel(showIdx)
        j = showIdx(n);
        before = findobj(ax, 'Type', 'patch');
        show(robot, paths{p}(:,j), 'Parent', ax, 'PreservePlot', true, 'Frames', 'off', 'FastUpdate', false);
        newPatches = setdiff(findobj(ax, 'Type', 'patch'), before);
        col = ghostCol(n,:);  if collides{p}(j), col = collCol; end
        alpha = 0.30;  if n == 1 || n == numel(showIdx), alpha = 0.65; end
        set(newPatches, 'FaceColor', col, 'FaceAlpha', alpha, 'EdgeColor', 'none');
    end
    % obstacles in the scenario colours
    for i = 1:numel(world)
        [~, pObj] = show(world{i}, 'Parent', ax);
        set(pObj, 'FaceColor', sc.obstacles(i).color, 'FaceAlpha', 0.55, 'EdgeColor', 0.6*sc.obstacles(i).color, 'LineWidth', 0.8);
    end
    % end-effector path
    ee = zeros(3, nDiscretize);
    for t = 1:nDiscretize, T = getTransform(robot, paths{p}(:,t), char(endEffector)); ee(:,t) = T(1:3,4); end
    hPath  = plot3(ax, ee(1,:), ee(2,:), ee(3,:), 'k.-', 'LineWidth', 2, 'MarkerSize', 14);
    hStart = plot3(ax, ee(1,1),   ee(2,1),   ee(3,1),   'o', 'MarkerSize', 11, 'MarkerFaceColor', [0.15 0.70 0.25], 'MarkerEdgeColor', 'k');
    hGoal  = plot3(ax, ee(1,end), ee(2,end), ee(3,end), 's', 'MarkerSize', 12, 'MarkerFaceColor', [0.85 0.15 0.15], 'MarkerEdgeColor', 'k');
    axis(ax, sc.axisLim); daspect(ax, [1 1 1]); view(ax, sc.view(1), sc.view(2));
    grid(ax, 'on'); box(ax, 'on'); camlight(ax, 'headlight'); lighting(ax, 'gouraud');
    xlabel(ax, 'x (m)'); ylabel(ax, 'y (m)'); zlabel(ax, 'z (m)');
    title(ax, titles{p}, 'FontSize', 13);
    % legend (dummy handles so the robot meshes do not appear in it)
    hGhost = patch(ax, NaN, NaN, ghostCol(end,:), 'FaceAlpha', 0.5, 'EdgeColor', 'none');
    hColl  = patch(ax, NaN, NaN, collCol, 'FaceAlpha', 0.5, 'EdgeColor', 'none');
    legend(ax, [hStart hGoal hPath hGhost hColl], ...
        {'start', 'goal', 'end-effector path', 'robot (light \rightarrow dark = time)', 'robot pose in collision'}, ...
        'Location', 'northwest', 'FontSize', 10);
end
title(tl, sprintf('Task 4, scenario %s: UR5 path planned with STOMP', scTitle), 'FontSize', 15, 'FontWeight', 'bold');
exportgraphics(f, sprintf('Task4_%s_overlay.png', sc.name), 'Resolution', 200);

%% Figure 2: total cost per iteration (iteration 0 = initial guess)
% (the smoothness term of Q is almost constant, so the change in Q is the change in the obstacle cost)
[~, Q0] = stompTrajCost(robot_struct, th_init, R, voxel_world);
Qall = [Q0 Q_time];
its  = 0:numel(Q_time);
f2 = figure('Position', [100 100 640 420], 'Color', 'w'); theme(f2, 'light');
ax2 = axes(f2);
plot(ax2, its, Qall, 'o-', 'LineWidth', 2, 'Color', [0.00 0.45 0.74], 'MarkerFaceColor', 'w', 'MarkerSize', 6);
grid(ax2, 'on'); box(ax2, 'on'); xlim(ax2, [0 its(end)]);
if its(end) <= 20, xticks(ax2, its); end
xlabel(ax2, 'STOMP iteration (0 = initial guess)'); ylabel(ax2, 'Total cost Q(\theta)');
title(ax2, sprintf('Scenario %s: converged after %d iterations%s', scTitle, iter, attemptTxt));
exportgraphics(f2, sprintf('Task4_%s_cost.png', sc.name), 'Resolution', 200);
