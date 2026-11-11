%% Task 3 check: does the PoE planner give the SAME path as the getTransform planner of Task 2,
% and how much faster is it?
% Runs the Task 2 planner (../Task2, forward kinematics with getTransform) and the Task 3 planner
% (this folder, PoE forward kinematics), both with rng(0), so both draw the same random samples.
% If the two forward-kinematics functions agree, the planned paths must be identical up to rounding.
% planningTime = time of the STOMP optimisation loop only (measured in helperSTOMP).
% Run from the Task3 folder:  compare_task2_task3
% Output: printed, and saved to Task3_compare_STOMP.txt
cmpHere  = fileparts(mfilename('fullpath'));
cmpTask2 = fullfile(fileparts(cmpHere), 'Task2');
assert(isfolder(cmpTask2), 'The Task2 folder must be next to the Task3 folder.');
enableVideoTraining = 0; enableVideo = 0;          % no videos for this check

% 1) Task 2 planner: getTransform
cd(cmpTask2); clear updateJointsWorldPosition; rng(0);
run(fullfile(cmpTask2, 'UR5_STOMP_Path_Planning.m'));
cmp.theta2 = theta; cmp.iter2 = iter; cmp.Q2 = Qtheta; cmp.time2 = planningTime; cmp.coll2 = isTrajectoryInCollision;
close all

% 2) Task 3 planner: Product of Exponentials
cd(cmpHere); clear updateJointsWorldPosition; rng(0);
run(fullfile(cmpHere, 'UR5_STOMP_Path_Planning.m'));
close all

cmpLines = {
    sprintf('==== Task 3 (PoE) vs Task 2 (getTransform), both rng(0)  (%s) ====', char(datetime('now')))
    sprintf('Task 2 (getTransform): iterations = %d, final cost Q = %.4f, in collision = %d, STOMP planning time = %.2f s', cmp.iter2, cmp.Q2, cmp.coll2, cmp.time2)
    sprintf('Task 3 (PoE)         : iterations = %d, final cost Q = %.4f, in collision = %d, STOMP planning time = %.2f s', iter, Qtheta, isTrajectoryInCollision, planningTime)
    sprintf('max |theta(Task3) - theta(Task2)| = %.2e rad', max(abs(theta(:) - cmp.theta2(:))))
    sprintf('planning speed-up (Task 2 time / Task 3 time) = %.1fx', cmp.time2 / planningTime)
    };
cmpTxt = strjoin(cmpLines, newline);
fprintf('\n%s\n', cmpTxt);
fid = fopen(fullfile(cmpHere, 'Task3_compare_STOMP.txt'), 'w'); fprintf(fid, '%s\n', cmpTxt); fclose(fid);
