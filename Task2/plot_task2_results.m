% Make the Task 2 figures (run right after UR5_STOMP_Path_Planning.m; uses its workspace):
%   Task2_path_overlay.png : straight-line initial guess vs STOMP path. The UR5 is drawn at
%                            several waypoints (light -> dark blue = start -> goal; red = pose
%                            in collision), end-effector path in black.
%   Task2_cost.png         : total cost Q(theta) per iteration (iteration 0 = initial guess)
%   Task2_spheres.png      : the sphere model STOMP uses for the obstacle cost (start pose and
%                            the waypoint with the smallest sphere clearance)
% Added for the report only.

%% Initial guess (straight line in joint space) and its collisions
th_init = zeros(numJoints, nDiscretize);
for k = 1:numJoints, th_init(k,:) = linspace(q0(k), qT(k), nDiscretize); end
collideInit = false(1, nDiscretize);
for i = 1:nDiscretize
    collideInit(i) = checkCollision(robot, th_init(:,i), world, 'IgnoreSelfCollision','on');
end
collideFinal = inCollision(:)';                     % from helperSTOMP
paths    = {th_init, theta};
collides = {collideInit, collideFinal};
titles   = {sprintf('Initial guess (straight line): %d of %d waypoints collide', nnz(collideInit), nDiscretize), ...
            sprintf('After STOMP (%d iterations): %d of %d waypoints collide', iter, nnz(collideFinal), nDiscretize)};

%% Figure 1: overlay of intermediate configurations
showIdx = round(linspace(1, nDiscretize, 7));      % start, 5 intermediate, goal
ghostCol = interp1([0 1], [0.62 0.78 0.95; 0.10 0.25 0.60], linspace(0, 1, numel(showIdx)));
f = figure('Position', [50 50 1500 720], 'Color', 'w'); theme(f, 'light');  % light even in dark mode
tl = tiledlayout(f, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for p = 1:2
    ax = nexttile(tl); hold(ax, 'on');
    % robot ghosts, coloured by time (red if that pose collides)
    for n = 1:numel(showIdx)
        j = showIdx(n);
        before = findobj(ax, 'Type', 'patch');
        show(robot, paths{p}(:,j), 'Parent', ax, 'PreservePlot', true, 'Frames', 'off', 'FastUpdate', false);
        newPatches = setdiff(findobj(ax, 'Type', 'patch'), before);
        col = ghostCol(n,:);  if collides{p}(j), col = [0.90 0.25 0.20]; end
        alpha = 0.30;  if n == 1 || n == numel(showIdx), alpha = 0.65; end
        set(newPatches, 'FaceColor', col, 'FaceAlpha', alpha, 'EdgeColor', 'none');
    end
    % obstacles
    for i = 1:numel(world)
        [~, pObj] = show(world{i}, 'Parent', ax);
        set(pObj, 'FaceColor', [0.95 0.55 0.15], 'FaceAlpha', 0.55, 'EdgeColor', [0.55 0.30 0.05], 'LineWidth', 1);
    end
    % end-effector path
    ee = zeros(3, nDiscretize);
    for t = 1:nDiscretize, T = getTransform(robot, paths{p}(:,t), char(endEffector)); ee(:,t) = T(1:3,4); end
    hPath  = plot3(ax, ee(1,:), ee(2,:), ee(3,:), 'k.-', 'LineWidth', 2, 'MarkerSize', 14);
    hStart = plot3(ax, ee(1,1),   ee(2,1),   ee(3,1),   'o', 'MarkerSize', 11, 'MarkerFaceColor', [0.15 0.70 0.25], 'MarkerEdgeColor', 'k');
    hGoal  = plot3(ax, ee(1,end), ee(2,end), ee(3,end), 's', 'MarkerSize', 12, 'MarkerFaceColor', [0.85 0.15 0.15], 'MarkerEdgeColor', 'k');
    axis(ax, [-0.4 0.9 -0.7 0.7 -0.1 0.7]); daspect(ax, [1 1 1]); view(ax, 135, 25);
    grid(ax, 'on'); box(ax, 'on'); camlight(ax, 'headlight'); lighting(ax, 'gouraud');
    xlabel(ax, 'x (m)'); ylabel(ax, 'y (m)'); zlabel(ax, 'z (m)');
    title(ax, titles{p}, 'FontSize', 13);
    % legend (dummy handles so the robot meshes do not appear in it)
    hGhost = patch(ax, NaN, NaN, ghostCol(end,:), 'FaceAlpha', 0.5, 'EdgeColor', 'none');
    hColl  = patch(ax, NaN, NaN, [0.90 0.25 0.20], 'FaceAlpha', 0.5, 'EdgeColor', 'none');
    hObs   = patch(ax, NaN, NaN, [0.95 0.55 0.15], 'FaceAlpha', 0.55, 'EdgeColor', [0.55 0.30 0.05]);
    legend(ax, [hStart hGoal hPath hGhost hColl hObs], ...
        {'start', 'goal', 'end-effector path', 'robot (light \rightarrow dark = time)', 'robot pose in collision', 'obstacle'}, ...
        'Location', 'northwest', 'FontSize', 10);
end
title(tl, 'Task 2: UR5 avoiding a box obstacle with STOMP', 'FontSize', 15, 'FontWeight', 'bold');
exportgraphics(f, 'Task2_path_overlay.png', 'Resolution', 200);

%% Figure 2: total cost per iteration (iteration 0 = initial guess)
% (the smoothness term of Q is almost constant during the optimisation, so the change in Q
%  is essentially the change in the obstacle cost)
[~, Q0] = stompTrajCost(robot_struct, th_init, R, voxel_world);
Qall = [Q0 Q_time];
its = 0:numel(Q_time);
f2 = figure('Position', [100 100 640 420], 'Color', 'w'); theme(f2, 'light');
ax2 = axes(f2);
plot(ax2, its, Qall, 'o-', 'LineWidth', 2, 'Color', [0.00 0.45 0.74], 'MarkerFaceColor', 'w', 'MarkerSize', 7);
grid(ax2, 'on'); box(ax2, 'on'); xlim(ax2, [0 its(end)]); xticks(ax2, its);
xlabel(ax2, 'STOMP iteration (0 = initial guess)'); ylabel(ax2, 'Total cost Q(\theta)');
title(ax2, sprintf('Task 2 (UR5): total cost, converged after %d iterations', iter));
exportgraphics(f2, 'Task2_cost.png', 'Resolution', 200);

%% Figure 3: sphere model that STOMP uses for the obstacle cost
% Left: start configuration. Right: the waypoint of the STOMP path with the smallest sphere clearance (what the cost sees; the real meshes may come closer elsewhere).
% Clearance = signed distance (sEDT map) at a sphere centre minus the sphere radius, i.e. the gap
% between the sphere surface and the obstacle; STOMP penalises it when it is below 5 cm.
clearance = zeros(1, nDiscretize);
for t = 1:nDiscretize
    [X, ~] = updateJointsWorldPosition(robot_struct, theta(:,t));
    [c, rad] = stompRobotSphere(X);
    idx = ceil((c - voxel_world.Env_size(1,:)) ./ voxel_world.voxel_size);
    idx = min(max(idx, 1), voxel_world.world_size);
    d = voxel_world.sEDT(sub2ind(voxel_world.world_size, idx(:,1), idx(:,2), idx(:,3)));
    clearance(t) = min(d - rad);
end
[~, tClose] = min(clearance);
showT  = [1 tClose];
ttl    = {'Start configuration', sprintf('Waypoint %d: smallest sphere clearance', tClose)};
[sx, sy, sz] = sphere(16);
f3 = figure('Position', [50 50 1400 650], 'Color', 'w'); theme(f3, 'light');
tl3 = tiledlayout(f3, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for p = 1:2
    ax = nexttile(tl3); hold(ax, 'on');
    show(robot, theta(:,showT(p)), 'Parent', ax, 'PreservePlot', false, 'Frames', 'off');
    set(findobj(ax, 'Type', 'patch'), 'FaceColor', [0.70 0.70 0.70], 'FaceAlpha', 0.35, 'EdgeColor', 'none');
    [X, ~] = updateJointsWorldPosition(robot_struct, theta(:,showT(p)));
    [c, rad] = stompRobotSphere(X);
    for i = 1:size(c,1)
        surf(ax, c(i,1) + rad(i)*sx, c(i,2) + rad(i)*sy, c(i,3) + rad(i)*sz, ...
            'FaceColor', [0.10 0.40 0.90], 'FaceAlpha', 0.35, 'EdgeColor', 'none');
    end
    plot3(ax, X(:,1), X(:,2), X(:,3), 'ko', 'MarkerSize', 7, 'MarkerFaceColor', 'y');   % joint frames + end-effector
    for i = 1:numel(world)
        [~, pObj] = show(world{i}, 'Parent', ax);
        set(pObj, 'FaceColor', [0.95 0.55 0.15], 'FaceAlpha', 0.55, 'EdgeColor', [0.55 0.30 0.05], 'LineWidth', 1);
    end
    axis(ax, [-0.4 0.9 -0.7 0.7 -0.1 0.7]); daspect(ax, [1 1 1]); view(ax, 135, 25);
    grid(ax, 'on'); box(ax, 'on'); camlight(ax, 'headlight'); lighting(ax, 'gouraud');
    xlabel(ax, 'x (m)'); ylabel(ax, 'y (m)'); zlabel(ax, 'z (m)');
    title(ax, {ttl{p}, sprintf('%d spheres, r = %.0f cm, sphere clearance %.1f cm', size(c,1), 100*rad(1), 100*clearance(showT(p)))}, 'FontSize', 13);
    hR = patch(ax, NaN, NaN, [0.70 0.70 0.70], 'FaceAlpha', 0.5, 'EdgeColor', 'none');
    hS = patch(ax, NaN, NaN, [0.10 0.40 0.90], 'FaceAlpha', 0.5, 'EdgeColor', 'none');
    hJ = plot3(ax, NaN, NaN, NaN, 'ko', 'MarkerSize', 7, 'MarkerFaceColor', 'y');
    hO = patch(ax, NaN, NaN, [0.95 0.55 0.15], 'FaceAlpha', 0.55, 'EdgeColor', [0.55 0.30 0.05]);
    legend(ax, [hR hS hJ hO], {'UR5 (real geometry)', 'collision spheres', 'joint frames / end-effector', 'obstacle'}, ...
        'Location', 'northwest', 'FontSize', 10);
end
title(tl3, 'Task 2: sphere model of the UR5 used by the STOMP obstacle cost', 'FontSize', 15, 'FontWeight', 'bold');
exportgraphics(f3, 'Task2_spheres.png', 'Resolution', 200);
