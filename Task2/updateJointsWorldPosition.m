%% forward kinematics (Task 2 version: works for robots whose first bodies are fixed, e.g. UR5)
% INPUT:
%       robot_struct: the Matlab robot structure object (rigidBodyTree, struct data format)
%       theta: the joint angles of the MOVING joints only (nMoving x 1)
% OUTPUT:
%       X: (nMoving+1) x 4 matrix. Row k (k = 1..nMoving) is the position
%          [x y z 1] of the frame of the k-th MOVING joint in the base frame;
%          the last row is the end-effector frame (the last body of the tree,
%          'tool0' for the UR5). The end-effector row lets the sphere model
%          also cover the last link, and gives T{end} for the end-effector
%          orientation constraint of Task 5.
%       T: 1 x (nMoving+1) cell of 4x4 homogeneous transforms (same ordering)
%
% Why not just use BodyNames{k}? For the Gen3 body k happened to be the k-th
% joint. For the UR5 bodies 1 and 2 are FIXED (base_link, base), so the first
% moving joint belongs to body 3. We therefore find the moving bodies once
% and cache their names.
function [X, T] = updateJointsWorldPosition(robot_struct, theta)

persistent cachedRobot movingBodyNames eeBodyName
if isempty(cachedRobot) || ~isvalid(cachedRobot) || cachedRobot ~= robot_struct   % rigidBodyTree is a handle: '~=' tests for the same object
    % ---- executed once per robot (slow Bodies{k} access is fine here) ----
    cachedRobot = robot_struct;
    home = robot_struct.homeConfiguration;            % one entry per MOVING joint
    jointNames = {home.JointName};
    movingBodyNames = cell(1, numel(jointNames));
    for k = 1:robot_struct.NumBodies
        j = find(strcmp(robot_struct.Bodies{k}.Joint.Name, jointNames), 1);
        if ~isempty(j)
            movingBodyNames{j} = robot_struct.BodyNames{k};   % body driven by joint j
        end
    end
    eeBodyName = robot_struct.BodyNames{end};         % end-effector frame (tool0)
end

% Update the robot configuration structure used by Matlab
% Refer to: https://www.mathworks.com/matlabcentral/answers/31273-how-to-update-struct-array-fields-with-mutiple-values
theta_cell = num2cell(theta);
tConfiguration = robot_struct.homeConfiguration;      % copy the home configuration struct
[tConfiguration.JointPosition] = theta_cell{:};       % update the joint positions using theta

nJoints = length(theta);
T = cell(1, nJoints+1);
X = zeros(nJoints+1, 4);

for k = 1:nJoints
    % pose of the k-th moving joint's frame in the base frame (built-in FK)
    T{k} = getTransform(robot_struct, tConfiguration, movingBodyNames{k});
    X(k,:) = T{k}(:,4)';
end
% end-effector frame
T{nJoints+1} = getTransform(robot_struct, tConfiguration, eeBodyName);
X(nJoints+1,:) = T{nJoints+1}(:,4)';

end
