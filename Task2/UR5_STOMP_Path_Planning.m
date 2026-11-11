%% *UR5 Manipulator Path Planning using STOMP (Task 2: a different robot)*
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
taskInit = trvec2tform([0.45 -0.35 0.25])*axang2tform([0 1 0 pi]);

% Compute current robot joint configuration using inverse kinematics
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.AllowRandomRestart = false;
weights = [1 1 1 1 1 1];

% The UR5 home configuration (all zeros, arm stretched out) is a poor IK seed:
% IK started from there lands in a different arm branch for start and goal, and
% the straight-line joint path then swings the arm behind the robot. Seeding IK
% with a "bent forward, tool down" configuration keeps both solutions in the same branch.
ikSeed = [0 -1.2 1.6 -0.4 -1.57 0]';
[currentRobotJConfig, ikInfoInit] = ik(endEffector, taskInit, weights, ikSeed);
assert(strcmp(ikInfoInit.Status,'success'), 'IK failed for the initial pose');

% The IK solver respects joint limits, but for those joints with infinite
% range, they must be wrapped to a finite range on the interval [-pi, pi].
% Since the the other joints are already bounded within this range, it is
% sufficient to simply call wrapToPi on the entire robot configuration
% rather than only on the joints with infinite range.
currentRobotJConfig = wrapToPi(currentRobotJConfig);

% Final (desired) end-effector pose
taskFinal = trvec2tform([0.45 0.35 0.25])*axang2tform([0 1 0 pi]);
anglesFinal = rotm2eul(taskFinal(1:3,1:3),'XYZ');
poseFinal = [taskFinal(1:3,4);anglesFinal']; % 6x1 vector for final pose: [x, y, z, phi, theta, psi]

% Final configuration
[finalRobotJConfig, ikInfoFinal] = ik(endEffector, taskFinal, weights, currentRobotJConfig);
assert(strcmp(ikInfoFinal.Status,'success'), 'IK failed for the final pose');
finalRobotJConfig = wrapToPi(finalRobotJConfig);

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

helperSTOMP;
%% 
% [1] M. Kalakrishnan, S. Chitta, E. Theodorou, P. Pastor and S. Schaal, "STOMP: 
% Stochastic trajectory optimization for motion planning," _2011 IEEE International 
% Conference on Robotics and Automation_, 2011, pp. 4569-4574, doi: 10.1109/ICRA.2011.5980280.