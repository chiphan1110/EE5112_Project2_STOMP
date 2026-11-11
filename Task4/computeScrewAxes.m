%% computeScrewAxes: derive the screw axes of a robot ONCE, from its geometry
%
% Product-of-Exponentials (PoE) forward kinematics needs only three things
% (LN4 p10): M (pose of the end-effector at the zero position), the screw axes
% S_1..S_n of the joints expressed in the fixed base frame at the zero
% position, and the joint variables. This function finds the first two.
%
% METHOD (LN5 p26, brief p.5 "method 1"):
%   1. Put the robot at its ZERO configuration (all joint variables 0).
%   2. For each moving joint k, take the pose T_k of the body that this joint
%      drives, in the base frame:  T_k = getTransform(robot, zeroConfig, body_k).
%      (This pose is also M_k, the "home frame" of joint k, used to get joint positions.)
%   3. The joint axis is given in the body's OWN frame (URDF <axis>):
%      expressed in the base frame it is   a = R_k * axis_own.
%      The origin of the body frame lies on the joint axis:   q = p_k.
%   4. Revolute joint:  S_k = [ a ; v ],  v = -a x q  (= q x a).
%      Prismatic joint: S_k = [ 0 ; a ] (pure translation along a, LN3 p22 case omega = 0).
%   Joint twist order is [omega; v] (angular part first), as in the lecture.
%
% INPUT : robot_struct  rigidBodyTree loaded with the default (struct) data format
% OUTPUT: ax (struct)
%   ax.Slist     6 x n   screw axes in the base frame (column k = joint k)
%   ax.Mlist     1 x n   cell, 4x4 zero-position pose of the body of joint k
%   ax.Mee       4 x 4   zero-position pose of the end-effector frame (last body of the tree)
%   ax.jointType 1 x n   cell, 'revolute' or 'prismatic'
%   ax.bodyNames 1 x n   cell, name of the body moved by joint k
%   ax.eeBodyName        name of the end-effector body
function ax = computeScrewAxes(robot_struct)

home = robot_struct.homeConfiguration;               % struct array, one entry per MOVING joint
jointNames = {home.JointName};
n = numel(jointNames);

zeroConfig = home;                                   % PoE "home" = ALL joint variables zero,
[zeroConfig.JointPosition] = deal(0);                % which may differ from homeConfiguration

ax.Slist     = zeros(6, n);
ax.Mlist     = cell(1, n);
ax.jointType = cell(1, n);
ax.bodyNames = cell(1, n);

for k = 1:robot_struct.NumBodies
    body = robot_struct.Bodies{k};                   % (slow access, but done only once per robot)
    j = find(strcmp(body.Joint.Name, jointNames), 1);
    if isempty(j), continue; end                     % fixed joint: skip
    name = robot_struct.BodyNames{k};
    Tk   = getTransform(robot_struct, zeroConfig, name);
    a    = Tk(1:3,1:3) * body.Joint.JointAxis(:);    % joint axis in the base frame
    a    = a / norm(a);
    q    = Tk(1:3,4);                                % a point on the axis
    switch body.Joint.Type
        case 'revolute'
            ax.Slist(:,j) = [a; -cross(a, q)];       % [omega; v], v = -omega x q
        case 'prismatic'
            ax.Slist(:,j) = [0; 0; 0; a];            % omega = 0, v = unit axis
        otherwise
            error('computeScrewAxes: unsupported joint type %s', body.Joint.Type);
    end
    ax.Mlist{j}     = Tk;
    ax.jointType{j} = body.Joint.Type;
    ax.bodyNames{j} = name;
end

ax.eeBodyName = robot_struct.BodyNames{end};         % end-effector frame (tool0 for the UR5)
ax.Mee = getTransform(robot_struct, zeroConfig, ax.eeBodyName);
end
