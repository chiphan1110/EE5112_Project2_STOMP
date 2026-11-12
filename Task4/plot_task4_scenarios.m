function plot_task4_scenarios
% Export scenario views: start configuration, obstacles and expected EE path.
% Uses saved trajectories without running or modifying the planner.
% Saves PNG images in this Task4 folder and Group12_Report/figs.
codeDir = fileparts(mfilename('fullpath'));
outDir = fullfile(codeDir, '..', '..', 'Group12_Report', 'figs');
if ~isfolder(outDir), mkdir(outDir); end
robot = loadrobot('universalUR5', 'DataFormat', 'column');
names = {'Cluttered table', 'Wall and ceiling', 'Kitchen shelf'};
for id = 1:3
    sc = scenarioUR5(id);
    files = dir(fullfile(codeDir, ['Theta_UR5_' sc.name '_nDisc20_nPaths_*.mat']));
    assert(numel(files) == 1, 'Expected one saved trajectory for %s.', sc.name);
    data = load(fullfile(files(1).folder, files(1).name), 'theta');
    theta = data.theta;
    ee = zeros(3, size(theta, 2));
    for k = 1:size(theta, 2)
        T = getTransform(robot, theta(:, k), 'tool0');
        ee(:, k) = T(1:3, 4);
    end
    assert(norm(ee(:,1)-sc.taskInit(1:3,4)) < 1e-3);
    assert(norm(ee(:,end)-sc.taskFinal(1:3,4)) < 1e-3);
    f = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 660 700]);
    theme(f, 'light');
    ax = axes(f, 'Position', [0.10 0.19 0.82 0.76]);
    show(robot, theta(:,1), 'Parent', ax, 'Frames', 'off', 'PreservePlot', false);
    hold(ax, 'on');
    set(findobj(ax, 'Type', 'patch'), 'FaceColor', [0.43 0.33 0.60], ...
        'FaceAlpha', 1, 'EdgeColor', 'none');
    for k = 1:numel(sc.obstacles)
        o = sc.obstacles(k);
        if strcmp(o.type, 'box')
            obj = collisionBox(o.size(1), o.size(2), o.size(3));
        else
            obj = collisionCylinder(o.size(1), o.size(2));
        end
        obj.Pose = trvec2tform(o.center);
        [~, h] = show(obj, 'Parent', ax);
        set(h, 'FaceColor', o.color, 'FaceAlpha', 0.45, ...
            'EdgeColor', 0.65*o.color, 'LineWidth', 0.8);
    end
    hp = plot3(ax, ee(1,:), ee(2,:), ee(3,:), 'k-', 'LineWidth', 2.8);
    hs = plot3(ax, ee(1,1), ee(2,1), ee(3,1), 'o', 'MarkerSize', 12, ...
        'MarkerFaceColor', [0.1 0.75 0.25], 'MarkerEdgeColor', 'k');
    hg = plot3(ax, ee(1,end), ee(2,end), ee(3,end), 's', 'MarkerSize', 12, ...
        'MarkerFaceColor', [0.95 0.15 0.15], 'MarkerEdgeColor', 'k');
    axis(ax, sc.axisLim); daspect(ax, [1 1 1]); view(ax, sc.view);
    grid(ax, 'on'); box(ax, 'on');
    set(ax, 'FontName', 'Times New Roman', 'FontSize', 17);
    xlabel(ax, 'x (m)'); ylabel(ax, 'y (m)'); zlabel(ax, 'z (m)');
    title(ax, names{id}, 'FontSize', 21, 'FontWeight', 'normal');
    camlight(ax, 'headlight'); lighting(ax, 'gouraud');
    lg = legend(ax, [hs hg hp], {'Start', 'Goal', 'Expected path'}, ...
        'Orientation', 'horizontal', 'FontSize', 17, 'Box', 'off');
    lg.Units = 'normalized'; lg.Position = [0.08 0.025 0.84 0.05];
    % show(robot,...) configures the camera; restore automatic fitting and
    % reserve margins for the labels and the separate legend row.
    ax.CameraViewAngleMode = 'auto';
    ax.Units = 'normalized';
    ax.PositionConstraint = 'outerposition';
    ax.OuterPosition = [0.02 0.14 0.96 0.80];
    drawnow;
    imageName = ['Task4_' sc.name '_scenario.png'];
    localImage = fullfile(codeDir, imageName);
    exportgraphics(f, localImage, 'Resolution', 250);
    copyfile(localImage, fullfile(outDir, imageName));
    fprintf('Exported %s: %d waypoints.\n', sc.name, size(theta,2));
    close(f);
end
end
