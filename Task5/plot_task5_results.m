%% Task 5 figures: each scenario planned WITHOUT and WITH the end-effector orientation constraint
% Both plans use the same random seed, so the only difference is the constraint cost.
% Run from the Task5 folder:  plot_task5_results
%   (optional, before running: p5scenarios = [1 2 3]; p5seed = 0;)   takes about 1-2 min, no videos
% Output per scenario:
%   Task5_<scenario>_before_after.png : robot at several waypoints (light -> dark purple = start ->
%                                       goal) and the constrained end-effector axis drawn as orange
%                                       arrows at every waypoint
%   Task5_<scenario>_alignment_error.png : alignment error per waypoint = angle between the
%                                       constrained tool axis and its target direction
% and Task5_results.txt with the numbers of all scenarios.
if ~exist('p5scenarios','var'), p5scenarios = [1 2 3]; end
if ~exist('p5seed','var'), p5seed = 0; end
p5lines = {sprintf('Task 5 results (seed %d), %s', p5seed, char(datetime('now'))), ''};

for p5s = p5scenarios
    %% plan twice: without (p5c = 1) and with (p5c = 2) the constraint
    p5res = struct();
    for p5c = 1:2
        clearvars -except p5scenarios p5seed p5lines p5s p5c p5res
        close all; clear functions
        scenarioID = p5s; useOrientationConstraint = (p5c == 2);
        displayAnimation = 0; enableVideoTraining = 0; enableVideo = 0; rng(p5seed);
        evalc('run(''UR5_STOMP_Path_Planning.m'')');
        tilt = zeros(1, nDiscretize); ee = zeros(3, nDiscretize); ax3 = ee;
        for i = 1:nDiscretize
            [~, T] = updateJointsWorldPosition(robot_struct, theta(:,i));
            ee(:,i)  = T{end}(1:3,4);
            ax3(:,i) = T{end}(1:3,eeCon.axis);                                  % constrained axis in the world
            tilt(i)  = acosd(min(1, max(-1, ax3(:,i)' * eeCon.target(:))));     % alignment error = angle to its target direction
        end
        p5res(p5c).theta = theta; p5res(p5c).tilt = tilt; p5res(p5c).ee = ee; p5res(p5c).ax3 = ax3;
        p5res(p5c).iters = attemptIters; p5res(p5c).iter = iter; p5res(p5c).nAttempts = nAttempts;
        p5res(p5c).maxIter = maxIter; p5res(p5c).inColl = inCollision(:)'; p5res(p5c).time = planningTime;
        p5res(p5c).len = sum(vecnorm(diff(ee, 1, 2)));
    end
    % (robot, world, sc, eeCon, nDiscretize are from the last run; identical for both runs)

    axName = 'xyz'; axName = axName(eeCon.axis);
    tgtTxt = sprintf('[%g %g %g]', eeCon.target);
    scTitle = strrep(sc.name, '_', ' ');
    runName = {'Without orientation constraint', sprintf('With orientation constraint (tool %s-axis \\rightarrow %s)', axName, tgtTxt)};
    for p5c = 1:2
        r = p5res(p5c);
        capTxt = ''; if r.iter > r.maxIter, capTxt = ' (iteration cap)'; end
        attTxt = ''; if r.nAttempts > 1, attTxt = sprintf(', %d attempts', r.nAttempts); end
        p5res(p5c).sub = sprintf('%d iterations%s%s, %d of %d waypoints collide, max alignment error %.1f\\circ', ...
            r.iter, capTxt, attTxt, nnz(r.inColl), nDiscretize, max(r.tilt));
    end

    %% Figure 1: before / after overlay with the constrained axis as arrows
    showIdx  = round(linspace(1, nDiscretize, 7));     % start, 5 intermediate, goal
    ghostCol = interp1([0 1], [0.82 0.76 0.94; 0.33 0.15 0.55], linspace(0, 1, numel(showIdx)));
    collCol  = [0.95 0.10 0.65];
    arrowCol = [1.00 0.55 0.00];
    f = figure('Position', [50 50 1500 720], 'Color', 'w'); theme(f, 'light');
    tl = tiledlayout(f, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for p5c = 1:2
        r = p5res(p5c);
        ax = nexttile(tl); hold(ax, 'on');
        for n = 1:numel(showIdx)
            j = showIdx(n);
            before = findobj(ax, 'Type', 'patch');
            show(robot, r.theta(:,j), 'Parent', ax, 'PreservePlot', true, 'Frames', 'off', 'FastUpdate', false);
            newPatches = setdiff(findobj(ax, 'Type', 'patch'), before);
            col = ghostCol(n,:);  if r.inColl(j), col = collCol; end
            alpha = 0.25;  if n == 1 || n == numel(showIdx), alpha = 0.55; end
            set(newPatches, 'FaceColor', col, 'FaceAlpha', alpha, 'EdgeColor', 'none');
        end
        for i = 1:numel(world)
            [~, pObj] = show(world{i}, 'Parent', ax);
            set(pObj, 'FaceColor', sc.obstacles(i).color, 'FaceAlpha', 0.45, 'EdgeColor', 0.6*sc.obstacles(i).color, 'LineWidth', 0.8);
        end
        hPath  = plot3(ax, r.ee(1,:), r.ee(2,:), r.ee(3,:), 'k.-', 'LineWidth', 1.8, 'MarkerSize', 12);
        L = 0.10;
        hArrow = quiver3(ax, r.ee(1,:), r.ee(2,:), r.ee(3,:), L*r.ax3(1,:), L*r.ax3(2,:), L*r.ax3(3,:), 0, ...
            'Color', arrowCol, 'LineWidth', 2.2, 'MaxHeadSize', 0.4);
        hStart = plot3(ax, r.ee(1,1),   r.ee(2,1),   r.ee(3,1),   'o', 'MarkerSize', 11, 'MarkerFaceColor', [0.15 0.70 0.25], 'MarkerEdgeColor', 'k');
        hGoal  = plot3(ax, r.ee(1,end), r.ee(2,end), r.ee(3,end), 's', 'MarkerSize', 12, 'MarkerFaceColor', [0.85 0.15 0.15], 'MarkerEdgeColor', 'k');
        axis(ax, sc.axisLim); daspect(ax, [1 1 1]); view(ax, sc.view(1), sc.view(2));
        grid(ax, 'on'); box(ax, 'on'); camlight(ax, 'headlight'); lighting(ax, 'gouraud');
        xlabel(ax, 'x (m)'); ylabel(ax, 'y (m)'); zlabel(ax, 'z (m)');
        title(ax, {runName{p5c}, r.sub}, 'FontSize', 13);
        hGhost = patch(ax, NaN, NaN, ghostCol(end,:), 'FaceAlpha', 0.5, 'EdgeColor', 'none');
        hColl  = patch(ax, NaN, NaN, collCol, 'FaceAlpha', 0.5, 'EdgeColor', 'none');
        legend(ax, [hStart hGoal hPath hArrow hGhost hColl], ...
            {'start', 'goal', 'end-effector path', sprintf('tool %s-axis (constrained)', axName), ...
             'robot (light \rightarrow dark = time)', 'robot pose in collision'}, 'Location', 'northwest', 'FontSize', 10);
    end
    title(tl, sprintf('Task 5, scenario %s: before and after the orientation constraint (tool %s-axis kept at %s)', ...
        scTitle, axName, tgtTxt), 'FontSize', 15, 'FontWeight', 'bold');
    exportgraphics(f, sprintf('Task5_%s_before_after.png', sc.name), 'Resolution', 200);

    %% Figure 2: alignment error of the constrained axis along the path
    f2 = figure('Position', [100 100 680 430], 'Color', 'w'); theme(f2, 'light');
    ax2 = axes(f2); hold(ax2, 'on');
    plot(ax2, 1:nDiscretize, p5res(1).tilt, 'o-', 'LineWidth', 2, 'Color', [0.55 0.55 0.55], 'MarkerFaceColor', 'w', 'MarkerSize', 6);
    plot(ax2, 1:nDiscretize, p5res(2).tilt, 's-', 'LineWidth', 2, 'Color', [0.00 0.45 0.74], 'MarkerFaceColor', 'w', 'MarkerSize', 6);
    grid(ax2, 'on'); box(ax2, 'on'); xlim(ax2, [1 nDiscretize]); xticks(ax2, 1:nDiscretize);
    xlabel(ax2, 'waypoint'); ylabel(ax2, sprintf('alignment error of tool %s-axis to %s (deg)', axName, tgtTxt));
    legend(ax2, {sprintf('without constraint (max %.1f\\circ)', max(p5res(1).tilt)), ...
                 sprintf('with constraint (max %.1f\\circ)', max(p5res(2).tilt))}, 'Location', 'northeast');
    ylim(ax2, [0 1.15*max([p5res(1).tilt p5res(2).tilt 1])]);   % headroom above the peak
    title(ax2, sprintf('Scenario %s: end-effector alignment error along the path', scTitle));
    exportgraphics(f2, sprintf('Task5_%s_alignment_error.png', sc.name), 'Resolution', 200);

    %% numbers
    p5lines{end+1} = sprintf('Scenario %s: constrained axis = tool %s-axis, target %s, weight %g', sc.name, axName, tgtTxt, eeCon.weight);
    lbl = {'without', 'with   '};
    for p5c = 1:2
        r = p5res(p5c);
        p5lines{end+1} = sprintf('  %s constraint: iterations per attempt %s (cap %d), collision-free %d, STOMP time %.2f s, max alignment error %.2f deg, mean alignment error %.2f deg, end-effector path %.3f m', ...
            lbl{p5c}, mat2str(r.iters), r.maxIter + 1, ~any(r.inColl), r.time, max(r.tilt), mean(r.tilt), r.len); %#ok<SAGROW>
        p5lines{end+1} = sprintf('    alignment error per waypoint (deg): %s', mat2str(round(r.tilt, 1))); %#ok<SAGROW>
    end
    p5lines{end+1} = ''; %#ok<SAGROW>
    close all
end

p5txt = strjoin(p5lines, newline);
fprintf('%s\n', p5txt);
p5fid = fopen('Task5_results.txt', 'w'); fprintf(p5fid, '%s\n', p5txt); fclose(p5fid);
