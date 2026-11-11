%% *UR5 Manipulator Path Planning using STOMP (Task 5: end-effector orientation constraint)*
% Set useOrientationConstraint = false before running to plan WITHOUT the constraint ("before").
% Choose the scenario before running, e.g.:   scenarioID = 1; rng(0); run('UR5_STOMP_Path_Planning.m')
% Scenarios are defined in scenarioUR5.m. Forward kinematics: PoE (from Task 3).
if ~exist('scenarioID','var'), scenarioID = 1; end
sc = scenarioUR5(scenarioID);
% (only for task5_extra_study.m: fields of a struct `stompOverrides` in the workspace replace those
%  of sc, e.g. a different constraint weight; normal runs do not define it, so nothing changes)
if exist('stompOverrides','var'), fo = fieldnames(stompOverrides); for io = 1:numel(fo), sc.(fo{io}) = stompOverrides.(fo{io}); end, end
% End-effector orientation constraint (Task 5). Default: keep the end-effector y-axis upright
% (world +z), weight 100; a scenario can choose another axis/target in sc.eeCon (scenarioUR5.m).
if ~exist('useOrientationConstraint','var'), useOrientationConstraint = true; end
eeCon = struct('enabled', useOrientationConstraint, 'axis', 2, 'target', [0;0;1], 'weight', 100);
if isfield(sc,'eeCon'), fo = fieldnames(sc.eeCon); for io = 1:numel(fo), eeCon.(fo{io}) = sc.eeCon.(fo{io}); end, end
eeCon.enabled = useOrientationConstraint;
% This example shows how to plan a collision-free robot path from an initial 
% to a desired end-effector pose using STOMP (<https://ieeexplore.ieee.org/document/5980280 
% Stochastic trajectory optimization for motion planning>)[1].
%% Robot Description and Poses
% Load the UR5 rigid body tree (RBT) model.

robot_name = 'universalUR5';
robot = loadrobot(robot_name, 'DataFormat', 'column'); % Note there are different data formats for loading the robot (chosen in the 2nd argument)
%% 
% Get the number of joints.

numJoints = numel(homeConfiguration(robot));
%% 
% Specify the robot frame where the end-effector is attached.

endEffector = "tool0"; % UR5: last body of the tree (flange). Its z-axis points out of the flange.
%% 
% Specify initial and desired end-effector poses. Use inverse kinematics to 
% solve for the initial robot configuration given a desired pose.

% Initial end-effector pose
% (UR5 reach is ~0.85 m, shorter than the Gen3, so the poses are closer to the base.)
% Rotating by pi about y makes the flange z-axis point DOWN (gripper facing the table).
taskInit = sc.taskInit;

% Compute current robot joint configuration using inverse kinematics
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.AllowRandomRestart = false;
weights = [1 1 1 1 1 1];

% The UR5 home configuration (all zeros, arm stretched out) is a poor IK seed:
% IK started from there lands in a different arm branch for start and goal, and
% the straight-line joint path then swings the arm behind the robot. Seeding IK
% with a "bent forward, tool down" configuration keeps both solutions in the same branch.
ikSeed = sc.ikSeed;
[currentRobotJConfig, ikInfoInit] = ik(endEffector, taskInit, weights, ikSeed);
assert(strcmp(ikInfoInit.Status,'success'), 'IK failed for the initial pose');

% The IK solver respects joint limits, but for those joints with infinite
% range, they must be wrapped to a finite range on the interval [-pi, pi].
% Since the the other joints are already bounded within this range, it is
% sufficient to simply call wrapToPi on the entire robot configuration
% rather than only on the joints with infinite range.
currentRobotJConfig = wrapToPi(currentRobotJConfig);

% Final (desired) end-effector pose
taskFinal = sc.taskFinal;
anglesFinal = rotm2eul(taskFinal(1:3,1:3),'XYZ');
poseFinal = [taskFinal(1:3,4);anglesFinal']; % 6x1 vector for final pose: [x, y, z, phi, theta, psi]

% Final configuration
[finalRobotJConfig, ikInfoFinal] = ik(endEffector, taskFinal, weights, currentRobotJConfig);
assert(strcmp(ikInfoFinal.Status,'success'), 'IK failed for the final pose');
finalRobotJConfig = wrapToPi(finalRobotJConfig);

% Joint-angle wrap-around: an angle and the same angle +-2*pi give the same pose. If a joint's
% start and goal differ by more than pi, the straight-line initial guess would turn it the LONG
% way round. Move the goal angle by +-2*pi (same pose) when the joint limits allow it, so the
% initial path turns the short way.
for k = 1:numJoints
    lim = robot.Bodies{k+2}.Joint.PositionLimits;      % UR5: bodies 1-2 are fixed, joint k drives body k+2
    qShort = currentRobotJConfig(k) + wrapToPi(finalRobotJConfig(k) - currentRobotJConfig(k));
    if qShort >= lim(1) && qShort <= lim(2)
        finalRobotJConfig(k) = qShort;
    end
end

%% Collision Meshes and Obstacles
% MATLAB has its own collision object functions to check for and avoid collisions 
% (e.g., collisionSphere, collisionBox, etc). Here, we follow the STOMP paper 
% to set up the voxel world and using the signed Euclidean distance transform 
% (sEDT) function for this purpose, which is a common practice in robotics perception, 
% mapping, and collision avoidance. We shall still use the MATLAB built-in collision 
% object functions to visualize the object for the convenience. See more details 
% in the following helper functions.
helperCreateObstaclesUR5;

%% 
% Visualize the robot at the initial configuration. You should see the obstacles 
% in the environment as well.
x0 = [currentRobotJConfig', zeros(1,numJoints)];
helperInitialVisualizerUR5;

%% 
% Specify a safety distance away from the obstacles.
safetyDistance = 0.01; 

%% Path Planning using STOMP
% The following helper function contain the setup and execution of the STOMP 
% algorithm for collision-avoidance planning.

% Random restarts (Task 4 addition). STOMP is a local optimiser and can get stuck in a local
% minimum (seen in Scenario C). If the final path still collides, STOMP is run again from the
% SAME colliding straight-line initial guess; only the random samples differ (the random
% generator simply continues). maxRestarts = maximum number of attempts.
maxRestarts = 5; if isfield(sc,'maxRestarts'), maxRestarts = sc.maxRestarts; end
if ~exist('displayAnimation','var'), displayAnimation = 1; end
wantAnimation = displayAnimation; displayAnimation = 0;     % animate only the final result
attemptIters = [];
for attempt = 1:maxRestarts
    theta_animation = {};
    helperSTOMP;
    attemptIters(end+1) = iter; %#ok<SAGROW>
    if ~isTrajectoryInCollision, break; end
    fprintf('Attempt %d ended in collision: restarting STOMP with new random samples.\n', attempt);
end
nAttempts = attempt;
fprintf('STOMP attempts used: %d (iterations per attempt: %s)\n', nAttempts, mat2str(attemptIters));
displayAnimation = wantAnimation;
if displayAnimation
    for t = 1:size(theta,2)
        show(robot, theta(:,t), 'PreservePlot', false, 'Frames', 'on'); drawnow; pause(5/20);
    end
end
%% 
% [1] M. Kalakrishnan, S. Chitta, E. Theodorou, P. Pastor and S. Schaal, "STOMP: 
% Stochastic trajectory optimization for motion planning," _2011 IEEE International 
% Conference on Robotics and Automation_, 2011, pp. 4569-4574, doi: 10.1109/ICRA.2011.5980280.