%% task5_extra_study: reproducible numbers for the Task 5 report (run from the Task5 folder)
% Part 1: Scenario C, seeds 0-5, constraint OFF and ON (w = 100): max alignment error, attempts, collision
% Part 2: Scenario C, seed 0, weight sweep w = 1, 3, 10, 30, 100, 300
% Part 3: why do constrained runs reach the iteration limit? For every run above that stopped
%         at the limit, report the final obstacle cost and the last changes |dQ| of the total
%         cost (the loop continues while |dQ| > 0.1 OR obstacle cost > 0).
% Alignment error = angle between the constrained tool axis and its target direction (deg).
% The weight is changed through `stompOverrides` (see UR5_STOMP_Path_Planning.m). No videos are
% recorded, and the saved Theta_UR5_*.mat files of the normal runs are restored at the end.
% Output: printed and saved to Task5_extra_study.txt   (takes a few minutes)
thetaBak_ = tempname; mkdir(thetaBak_);
if ~isempty(dir('Theta_UR5_*.mat')), copyfile('Theta_UR5_*.mat', thetaBak_); end
R_ = struct('part',{},'scenario',{},'seed',{},'w',{},'constraint',{},'iters',{},'coll',{}, ...
            'maxTilt',{},'meanTilt',{},'atLimit',{},'finalObst',{},'lastdQ',{});
runs = {};
for seed = 0:5, runs(end+1,:) = {1, 3, seed, 100, false}; runs(end+1,:) = {1, 3, seed, 100, true}; end %#ok<SAGROW>
for w = [1 3 10 30 300], runs(end+1,:) = {2, 3, 0, w, true}; end %#ok<SAGROW>   % w = 100 is in part 1
runs(end+1,:) = {3, 1, 0, 100, true};  runs(end+1,:) = {3, 2, 0, 100, true};      % A and B with the constraint
for r = 1:size(runs,1)
    [part_, sid_, seed_, w_, on_] = runs{r,:};
    keep_ = {'R_','runs','r','part_','sid_','seed_','w_','on_','keep_','thetaBak_'};
    clearvars('-except', keep_{:}); close all; clear functions
    scenarioID = sid_; displayAnimation = 0; useOrientationConstraint = on_;
    enableVideoTraining = 0; enableVideo = 0;
    sc0_ = scenarioUR5(sid_); ec_ = sc0_.eeCon; ec_.weight = w_;
    stompOverrides = struct('eeCon', ec_);
    rng(seed_);
    evalc('run(''UR5_STOMP_Path_Planning.m'')');
    tilt = zeros(1, nDiscretize);
    ax_ = sc0_.eeCon.axis; tg_ = sc0_.eeCon.target(:);
    for i = 1:nDiscretize
        [~, T] = updateJointsWorldPosition(robot_struct, theta(:,i));
        tilt(i) = acosd(min(1, max(-1, T{end}(1:3,ax_)' * tg_)));
    end
    dQ = abs(diff(Q_time));
    R_(end+1) = struct('part', part_, 'scenario', sc.name, 'seed', seed_, 'w', w_, 'constraint', on_, ...
        'iters', attemptIters, 'coll', isTrajectoryInCollision, 'maxTilt', max(tilt), 'meanTilt', mean(tilt), ...
        'atLimit', iter > maxIter, 'finalObst', obstacleCost, 'lastdQ', dQ(max(1,end-4):end)); %#ok<SAGROW>
end
close all
L = {sprintf('Task 5 extra study (%s)', char(datetime('now')))};
titles = {'Part 1: Scenario C, seeds 0-5, without and with the constraint (w = 100)', ...
          'Part 2: Scenario C, seed 0, weight sweep (w = 100 is in part 1)', ...
          'Part 3: Scenarios A and B, seed 0, with the constraint'};
for p = 1:3
    L{end+1} = ''; L{end+1} = titles{p}; %#ok<SAGROW>
    for k = find([R_.part] == p)
        x = R_(k);
        L{end+1} = sprintf('  %-20s seed %d  w %-4g constraint %d | iterations %-12s collision %d | max alignment error %6.2f deg, mean %5.2f deg | at limit %d, final obstacle cost %.3f, last |dQ| %s', ...
            x.scenario, x.seed, x.w, x.constraint, mat2str(x.iters), x.coll, x.maxTilt, x.meanTilt, x.atLimit, x.finalObst, mat2str(round(x.lastdQ,3))); %#ok<SAGROW>
    end
end
lim = R_([R_.atLimit]);
L{end+1} = ''; L{end+1} = sprintf('Runs that stopped at the iteration limit: %d. Of these: final obstacle cost > 0 in %d; last |dQ| > 0.1 in %d.', ...
    numel(lim), sum([lim.finalObst] > 0), sum(arrayfun(@(x) x.lastdQ(end) > 0.1, lim)));
txt = strjoin(L, newline);
fprintf('%s\n', txt);
fid = fopen('Task5_extra_study.txt', 'w'); fprintf(fid, '%s\n', txt); fclose(fid);
% restore the saved paths of the normal runs
delete('Theta_UR5_*.mat');
if ~isempty(dir(fullfile(thetaBak_, '*.mat'))), copyfile(fullfile(thetaBak_, '*.mat'), pwd); end
rmdir(thetaBak_, 's');
