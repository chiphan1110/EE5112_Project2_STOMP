%% sphere_coverage_test: how well do the collision spheres (r = 0.05 m) cover the real UR5?
% Run from the Task2 folder:  sphere_coverage_test
% 1) Runs the Task 2 planner with rng(0) (gives q0, qT, theta, world, voxel_world).
% 2) Measures the true mesh-to-box separation of the planned path.
% 3) Draws N = 1500 random configurations near the straight-line path
%       q = (1-a) q0 + a qT + 0.35 * randn(6,1),  a ~ U(0,1),  with rng(1)
%    and compares, for each, the sphere model (sphere clearance = sEDT(centre) - r) with
%    MATLAB's checkCollision on the real meshes.
% Output: printed and saved to Task2_sphere_coverage.txt
clear; close all; clear functions
enableVideoTraining = 0; enableVideo = 0;   % no videos for this test
rng(0);
evalc('run(''UR5_STOMP_Path_Planning.m'')');
close all

minSep = inf;
for i = 1:size(theta,2)
    [~, sep] = checkCollision(robot, theta(:,i), world, "IgnoreSelfCollision","on", "Exhaustive","on");
    minSep = min(minSep, min(sep(:)));
end

rng(1);
N = 1500; sphereClr = zeros(1,N); meshHit = false(1,N); meshSep = zeros(1,N);
for n = 1:N
    a = rand; q = (1-a)*q0 + a*qT + 0.35*randn(6,1);
    [X,~] = updateJointsWorldPosition(robot_struct, q); [c, rr] = stompRobotSphere(X);
    idx = min(max(ceil((c - repmat(voxel_world.Env_size(1,:), size(c,1), 1)) ./ voxel_world.voxel_size), 1), voxel_world.world_size);
    d = voxel_world.sEDT(sub2ind(voxel_world.world_size, idx(:,1), idx(:,2), idx(:,3)));
    sphereClr(n) = min(d - rr);
    [hit, sep] = checkCollision(robot, q, world, "IgnoreSelfCollision","on", "Exhaustive","on");
    meshHit(n) = hit; meshSep(n) = min(sep(:), [], 'omitnan'); if hit, meshSep(n) = 0; end
end
missed = meshHit & sphereClr > 0;

lines = {
    sprintf('Task 2 sphere-coverage test (%s)', char(datetime('now')))
    sprintf('Planned path (rng(0)): isTrajectoryInCollision = %d, min true mesh-to-box separation = %.4f m', isTrajectoryInCollision, minSep)
    sprintf('Random configurations (rng(1)): N = %d', N)
    sprintf('  real mesh collisions (checkCollision): %d', sum(meshHit))
    sprintf('  missed by the spheres (real collision, sphere clearance > 0): %d (%.1f%% of real collisions)', sum(missed), 100*sum(missed)/max(1,sum(meshHit)))
    sprintf('  of which the spheres report clearance > 5 cm: %d', sum(meshHit & sphereClr > 0.05))
    sprintf('  worst miss (largest sphere clearance among real collisions): %.4f m', max(sphereClr(meshHit)))
    sprintf('  spheres report contact (clearance <= 0) but meshes > 5 cm apart: %d', sum(sphereClr <= 0 & meshSep > 0.05))
    };
txt = strjoin(lines, newline);
fprintf('%s\n', txt);
fid = fopen('Task2_sphere_coverage.txt', 'w'); fprintf(fid, '%s\n', txt); fclose(fid);
