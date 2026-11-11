% Given a trajectory, calculate its cost  (Task 5: + end-effector orientation constraint)
%
% [Stheta, Qtheta, obstacleCost] = stompTrajCost(robot_struct, theta, R, voxel_world, eeCon)
%   Stheta       1 x nDiscretize local cost at each waypoint = 1000*q_o + q_c
%   Qtheta       total cost = sum(Stheta) + smoothness term 1/2 * theta' R theta
%   obstacleCost sum over waypoints of 1000*q_o only (used by the stopping rule)
%   eeCon        (optional) orientation constraint, struct with fields
%                  .enabled  true/false
%                  .axis     which end-effector axis to align: 1 = x, 2 = y, 3 = z
%                  .target   3x1 world direction it should point to, e.g. [0;0;1] (upright)
%                  .weight   weight w of the constraint cost
%                q_c(i) = w * || a_sb(i) - target ||_1   (brief p.5, option 2: l1 norm)
%                where a_sb = the chosen axis of the end-effector frame in world coordinates
%                (column eeCon.axis of the rotation of T{end}, from the PoE forward kinematics)
function [Stheta, Qtheta, obstacleCost] = stompTrajCost(robot_struct, theta, R, voxel_world, eeCon)
if nargin < 5 || isempty(eeCon), eeCon.enabled = false; end

[~, nDiscretize] = size(theta);
qo_cost = zeros(1, nDiscretize);   % obstacle cost
qc_cost = zeros(1, nDiscretize);   % constraint cost

for i = 1 : nDiscretize
    % forward kinematics: joint positions X and frames T (T{end} = end-effector, tool0)
    [X, T] = updateJointsWorldPosition(robot_struct, theta(:, i));
    [sphere_centers, radi] = stompRobotSphere(X);
    if i == 1
        vel = zeros(length(sphere_centers), 1);        % initial velocity of the spheres is 0
    else
        % speed of each sphere centre: finite difference of current and previous position
        vel = vecnorm(sphere_centers_prev - sphere_centers, 2, 2);
    end
    qo_cost(i) = stompObstacleCost(sphere_centers, radi, voxel_world, vel);
    sphere_centers_prev = sphere_centers;

    % ---- end-effector orientation constraint (the sample code had "qc_cost = 0;" here,
    % ---- which overwrote the whole vector and never set qc_cost(1); fixed) ----
    if eeCon.enabled
        a_sb = T{end}(1:3, eeCon.axis);                % e.g. y-axis of the end-effector in the world
        qc_cost(i) = eeCon.weight * norm(a_sb - eeCon.target(:), 1);
    end
end

%% Local trajectory cost: relative weights between the different costs
Stheta = 1000*qo_cost + qc_cost;
obstacleCost = sum(1000*qo_cost);

% sum over time and add the smoothness cost
theta = theta(:, 2:end-1);
Qtheta = sum(Stheta) + 1/2 * sum(theta * R * theta', "all");
end
