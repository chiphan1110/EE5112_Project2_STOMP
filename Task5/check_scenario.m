%% check_scenario: sanity checks of a scenario BEFORE planning (run: scenarioID = 1; check_scenario)
%   - IK succeeds for start and goal, and the poses are reached
%   - the robot at the start and at the goal is NOT in collision (MATLAB meshes)
%   - the straight-line initial guess IS in collision (the brief forbids a trivial initialisation)
%   - minimum sphere clearance at start/goal (spheres closer than radius + 5 cm margin keep a constant cost)
if ~exist('scenarioID','var'), scenarioID = 1; end
sc = scenarioUR5(scenarioID);
robot = loadrobot('universalUR5','DataFormat','column'); robot_struct = loadrobot('universalUR5');
clear updateJointsWorldPosition
numJoints = 6; endEffector = "tool0";
ik = inverseKinematics('RigidBodyTree', robot); ik.SolverParameters.AllowRandomRestart = false; w = ones(1,6);
[q0, i0] = ik(endEffector, sc.taskInit,  w, sc.ikSeed); q0 = wrapToPi(q0);
[qT, iT] = ik(endEffector, sc.taskFinal, w, q0);        qT = wrapToPi(qT);
for k = 1:numJoints
    lim = robot.Bodies{k+2}.Joint.PositionLimits; qs = q0(k) + wrapToPi(qT(k)-q0(k));
    if qs >= lim(1) && qs <= lim(2), qT(k) = qs; end
end
tic; helperCreateObstaclesUR5; tObs = toc;
fprintf('Scenario %d (%s): %d obstacles, voxel world built in %.1f s\n', scenarioID, sc.name, numel(world), tObs);
fprintf('IK start: %s (pose err %.1e), IK goal: %s (pose err %.1e)\n', i0.Status, i0.PoseErrorNorm, iT.Status, iT.PoseErrorNorm);
fprintf('q0 = %s\nqT = %s\n', mat2str(q0',3), mat2str(qT',3));
cs = checkCollision(robot, q0, world, 'IgnoreSelfCollision','on');
cg = checkCollision(robot, qT, world, 'IgnoreSelfCollision','on');
fprintf('start in collision: %d, goal in collision: %d\n', cs, cg);
n = 20; c = false(1,n);
for i = 1:n, q = q0 + (i-1)/(n-1)*(qT-q0); c(i) = checkCollision(robot, q, world, 'IgnoreSelfCollision','on'); end
fprintf('straight-line initial guess: %d of %d waypoints collide (%s)\n', sum(c), n, mat2str(find(c)));
for qq = {q0, qT}
    [X,~] = updateJointsWorldPosition(robot_struct, qq{1}); [cc, rr] = stompRobotSphere(X);
    idx = min(max(ceil((cc - voxel_world.Env_size(1,:))./voxel_world.voxel_size),1), voxel_world.world_size);
    d = voxel_world.sEDT(sub2ind(voxel_world.world_size, idx(:,1), idx(:,2), idx(:,3)));
    fprintf('min sphere clearance (d - r): %.3f m\n', min(d - rr));
end
