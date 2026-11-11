%% difficulty_table: compare how hard Scenarios A, B, C are (run from the Task4 folder)
% For each scenario:
%   - collWp     : waypoints of the straight-line first guess that collide (MATLAB meshes)
%   - obstCost0  : STOMP obstacle cost of the first guess (sum over waypoints of 1000*q_o):
%                  how deep and how fast the arm moves inside obstacles; 0 = no sphere within the margin
%   - rng(0) run : attempts, iterations of the successful attempt, collision-free?,
%                  planTime_s = STOMP loop time of the successful attempt (videos off)
%   - minClear   : smallest real gap (mesh) between robot and obstacles over the INTERIOR waypoints
%   - maxDev     : largest distance between the gripper on the final path and on the first guess
%                  at the same waypoint (how far STOMP had to move the path)
% Output: printed and saved to Task4_difficulty_table.txt
rows = {};
for sid = 1:3
    clearvars -except sid rows; close all; clear functions
    scenarioID = sid; displayAnimation = 0; enableVideoTraining = 0; enableVideo = 0; rng(0);
    evalc('run(''UR5_STOMP_Path_Planning.m'')');
    th0 = zeros(numJoints, nDiscretize);
    for k = 1:numJoints, th0(k,:) = linspace(q0(k), qT(k), nDiscretize); end
    collWp = 0;  for i = 1:nDiscretize, collWp = collWp + checkCollision(robot, th0(:,i), world, 'IgnoreSelfCollision','on'); end
    S0 = stompTrajCost(robot_struct, th0, R, voxel_world);  obstCost0 = sum(S0);
    minClear = inf;
    for i = 2:nDiscretize-1
        [~, sp] = checkCollision(robot, theta(:,i), world, 'IgnoreSelfCollision','on', 'Exhaustive','on');
        minClear = min(minClear, min(sp(:)));
    end
    maxDev = 0;
    for i = 1:nDiscretize
        [~, Ta] = updateJointsWorldPosition(robot_struct, th0(:,i));
        [~, Tb] = updateJointsWorldPosition(robot_struct, theta(:,i));
        maxDev = max(maxDev, norm(Ta{end}(1:3,4) - Tb{end}(1:3,4)));
    end
    rows(end+1,:) = {sc.name, numel(sc.obstacles), collWp, obstCost0, nAttempts, iter, ~isTrajectoryInCollision, planningTime, minClear, maxDev}; %#ok<SAGROW>
end
close all
T = cell2table(rows, 'VariableNames', {'scenario','nObstacles','collWp_of20','obstCost0','attempts','iterations','collisionFree','planTime_s','minClear_m','maxDev_m'});
txt = regexprep(evalc('disp(T)'), '</?strong>', '');   % remove MATLAB's bold tags
fprintf('%s', txt);
fid = fopen('Task4_difficulty_table.txt','w');
fprintf(fid, 'Task 4 difficulty table (rng(0) run per scenario), %s\n\n%s', char(datetime('now')), txt);
fclose(fid);
