%% scenarioUR5: definition of the Task 4 collision-avoidance scenarios for the UR5
% sc = scenarioUR5(id) returns a struct with
%   sc.name       short name (used in file names)
%   sc.taskInit   4x4 initial end-effector (tool0) pose
%   sc.taskFinal  4x4 final end-effector pose
%   sc.ikSeed     6x1 joint seed for inverse kinematics (picks the arm branch)
%   sc.obstacles  struct array, one per obstacle:
%                   .type   'box' or 'cylinder' (axis-aligned; cylinder axis = world z)
%                   .size   box: [lx ly lz];  cylinder: [radius height]
%                   .center [x y z] of the obstacle centre (m)
%                   .color  RGB for plots
%   sc.axisLim    plot limits [xmin xmax ymin ymax zmin zmax];  sc.view [az el]
% All positions are in the UR5 base frame {s} (z up, base on the ground at the origin).
function sc = scenarioUR5(id)
toolDown = axang2tform([0 1 0 pi]);          % tool0 z-axis pointing down
% "Cup grasp": tool0 z-axis horizontal, pointing away from the robot (+x), tool0 y-axis UP (+z).
% Columns of the rotation = tool x, y, z axes in the base frame. (Used in Scenario C; it is
% also the pose Task 5 needs: carrying a cup upright.)
cupGrasp = [0 0 1 0; 1 0 0 0; 0 1 0 0; 0 0 0 1];
grey = [0.6 0.6 0.6]; red = [0.85 0.25 0.2]; blue = [0.25 0.45 0.85]; green = [0.3 0.7 0.35];
switch id
    case 1  % ---- Scenario A: cluttered table -----------------------------------
        sc.name = 'A_cluttered_table';
        sc.taskInit  = trvec2tform([0.50 -0.32 0.15]) * toolDown;   % pick point, left side of the table
        sc.taskFinal = trvec2tform([0.50  0.32 0.15]) * toolDown;   % place point, right side
        sc.ikSeed = [0 -1.2 1.6 -0.4 -1.57 0]';
        sc.obstacles = [ ...
            obs('box',      [0.80 0.90 0.04], [0.60  0.00 -0.02], grey)   % table top (surface at z = 0)
            obs('box',      [0.10 0.10 0.25], [0.48 -0.08  0.125], red)   % tall box
            obs('cylinder', [0.05 0.30],      [0.56  0.08  0.15], blue)   % bottle
            obs('box',      [0.12 0.08 0.16], [0.40  0.05  0.08], green)  % small box, near the robot
            ];
        sc.axisLim = [-0.3 1.0 -0.6 0.6 -0.1 0.8];  sc.view = [135 25];
    case 2  % ---- Scenario B: wall + low ceiling = "letterbox" ---------------------
        % A wall stands between the robot and the goal, and a ceiling plate hangs above it,
        % leaving a 35 cm horizontal slot (z 0.20-0.55). The arm must reach OVER the wall and
        % UNDER the ceiling, then down behind the wall. The wall is too long to go round
        % (its ends, at y = +-0.70, are about 0.86 m from the base, at the edge of the UR5's reach)
        % and a table below stops going underneath.
        % (A first design with the gap at the wall's near end was infeasible: that space is
        %  occupied by the robot's own upper arm. Without a table the arm also dived below z = 0.)
        sc.name = 'B_wall_and_ceiling';
        sc.taskInit  = trvec2tform([0.30 -0.25 0.15]) * toolDown;   % in front of the wall
        sc.taskFinal = trvec2tform([0.70  0.15 0.20]) * toolDown;   % behind the wall (z 0.20: at 0.10-0.15 the forearm itself touches the wall top)
        sc.ikSeed = [0 -1.2 1.6 -0.4 -1.57 0]';
        sc.obstacles = [ ...
            obs('box', [0.90 1.40 0.04], [0.60 0.00 -0.02], grey)    % table (surface z = 0), x 0.15-1.05
            obs('box', [0.04 1.40 0.20], [0.50 0.00  0.10], red)     % wall: x 0.48-0.52, y -0.7..0.7, z 0-0.20
            obs('box', [0.30 1.40 0.04], [0.50 0.00  0.57], blue)    % ceiling: x 0.35-0.65, z 0.55-0.59
            ];
        sc.axisLim = [-0.3 1.0 -0.7 0.7 -0.1 0.8];  sc.view = [-25 12];   % side view: wall, slot and ceiling seen in profile
    case 3  % ---- Scenario C: kitchen - table, pillar and shelf ----------------
        % Pick a cup on the table (left of the pillar) and place it INSIDE a shelf on the
        % right: dodge the tall pillar, then enter the shelf between its two boards. The
        % gripper holds the cup sideways with its y-axis up (cupGrasp) at start and goal.
        sc.name = 'C_kitchen_shelf';
        sc.taskInit  = trvec2tform([0.50 -0.35 0.12]) * cupGrasp;   % cup on the table
        sc.taskFinal = trvec2tform([0.62  0.40 0.25]) * cupGrasp;   % inside the shelf (the UR5 wrist sits ~0.1 m above the gripper)
        sc.ikSeed = [0 -1.2 1.6 -0.4 -1.57 0]';
        sc.obstacles = [ ...
            obs('box',      [0.90 1.40 0.04], [0.60  0.00 -0.02], grey)    % table (surface z = 0)
            obs('cylinder', [0.06 0.75],      [0.62  0.02  0.375], red)    % pillar, 0.75 m tall
            obs('box',      [0.32 0.34 0.03], [0.66  0.40  0.125], green)  % shelf bottom board (top at z 0.14)
            obs('box',      [0.32 0.34 0.03], [0.66  0.40  0.535], green)  % shelf top board (bottom at z 0.52): opening 0.38 m
            obs('box',      [0.03 0.34 0.44], [0.835 0.40  0.33], green)   % shelf back wall
            ];
        sc.axisLim = [-0.3 1.0 -0.7 0.7 -0.1 0.9];  sc.view = [-15 35];   % camera on the start side: start, pillar and shelf (in profile) all visible
        % STOMP settings for this harder scenario (sample-code defaults: 1.5, 20, 50). With the
        % defaults only 1 of 6 seeds was collision-free; with these, 4 of 6.
        sc.explorationScale = 3;  sc.nPaths = 40;  sc.maxIter = 150;
    otherwise
        error('scenarioUR5: unknown scenario %d', id);
end
end

function o = obs(type, sz, c, col)
o = struct('type', type, 'size', sz, 'center', c, 'color', col);
end
