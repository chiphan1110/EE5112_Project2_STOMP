%Parameters
% T = 5; 
nDiscretize = 20; % number of discretized waypoint
nPaths = 20; % number of sample paths
convergenceThreshold = 0.1; % convergence threshhold

% Initial guess of joint angles theta is just linear interpolation of q0
% and qT
q0 = currentRobotJConfig;
qT = finalRobotJConfig;
numJoints = length(q0);
theta=zeros(numJoints, nDiscretize);
for k=1:length(q0)
    theta(k,:) = linspace(q0(k), qT(k), nDiscretize);
end

% by default, it loads the robot with the structure data format
robot_struct = loadrobot(robot_name); 

% store sampled paths
theta_samples = cell(1,nPaths);

% Joint limits [min max] of the movable joints (continuous joints have -Inf/Inf).
% Used to keep the updated trajectory inside the limits (the sample code had no such safeguard).
% Reading robot.Bodies{i} advances MATLAB's random number generator, so the RNG state is saved
% and restored: reading the limits must not change the random samples (rng(0) results).
rngState = rng;
jointLimits = zeros(numJoints, 2);
kJ = 0;
for i = 1:robot.NumBodies
    if ~strcmp(robot.Bodies{i}.Joint.Type, 'fixed')
        kJ = kJ + 1;
        jointLimits(kJ,:) = robot.Bodies{i}.Joint.PositionLimits;
    end
end
rng(rngState);

%% for calculating the acceleration of theta
% Precompute
A_k = eye(nDiscretize - 1, nDiscretize - 1);
A = -2 * eye(nDiscretize, nDiscretize);
A(1:nDiscretize - 1, 2:nDiscretize) = A(1:nDiscretize - 1, 2:nDiscretize) + A_k;
A(2:nDiscretize, 1:nDiscretize - 1) = A(2:nDiscretize, 1:nDiscretize - 1) + A_k;
A = A(:, 2:end-1); 
R = A' * A;
Rinv = inv(R);
% The smoothing matrix M, normalized from Rinv by each column, no longer symmetric
M = 1 / nDiscretize * Rinv ./ max(Rinv, [], 1); 
% R inverse is normalized so that the exploration is controlled to have samples within the created voxel world
Rinv = 1.5*Rinv/sum(sum(Rinv)); 


%%
%Planner
Q_time = [];   % Trajectory cost Q(theta), t-vector
RAR_time = [];

[~, Qtheta] = stompTrajCost(robot_struct, theta, R, voxel_world);
QthetaOld = 0;

iter=0;
planTimer = tic;   % time of the STOMP optimisation loop only (no setup, video or animation)
while abs(Qtheta - QthetaOld) > convergenceThreshold
    iter=iter+1;
    % overall cost: Qtheta
    QthetaOld = Qtheta;
    % use tic and toc for printing out the running time
    tic
    
    %% Sample noisy trajectories
    % theta_samples{k}: sampled path k (nJoints x nDiscretize); em{j}: the noise of joint j (nPaths x nDiscretize)
    [theta_samples, em] = stompSamples(nPaths, Rinv, theta);


    %% Calculate Local trajectory cost for each sampled trajectory
    % variable declaration (holder for the cost):
    Stheta = zeros(nPaths, nDiscretize);
    for k = 1:nPaths
        Stheta(k,:) = stompTrajCost(robot_struct, theta_samples{k}, R, voxel_world);
    end
    
    %% Given the local traj cost, update local trajectory probability
    trajProb = stompUpdateProb(Stheta);
    
    %% Compute delta theta (aka gradient estimator, the improvement of the delta)
    dtheta = stompDTheta(trajProb, em);
    [theta, dtheta_smoothed] = stompUpdateTheta(theta, dtheta, M);
    % Joint-limit safeguard: clip the updated trajectory to the joint limits
    theta = min(max(theta, jointLimits(:,1)), jointLimits(:,2));

    %% Compute the cost of the new trajectory
    [~, Qtheta] = stompTrajCost(robot_struct, theta, R, voxel_world);
 
    toc

    Q_time = [Q_time Qtheta];
    % control cost
    RAR = 1/2 * sum(sum(theta(:, 2:nDiscretize-1) * R * theta(:, 2:nDiscretize-1)'));
    RAR_time = [RAR_time RAR];
    Qtheta % display overall cost
    RAR  % display control cost

    % Record the itermediate training trajectories for later animation
    theta_animation{iter}=theta;

    % Set the stopping iteration criteria:
    if iter > 50 
        disp('Maximum iteration (50) has reached.')
        break
    end

    if sum(dtheta_smoothed,'all') == 0
    disp('Estimated gradient is 0 and Theta is not updated: there could be no obstacle at all')
    break
    end

end

planningTime = toc(planTimer);
disp('STOMP Finished.');
fprintf('STOMP planning time: %.2f s (%d iterations)\n', planningTime, iter);






%% check collision
inCollision = false(nDiscretize, 1); % initialization the collision status vector
worldCollisionPairIdx = cell(nDiscretize,1); % Initialization: Provide the bodies that are in collision

for i = 1:nDiscretize

    [inCollision(i),sepDist] = checkCollision(robot,theta(:,i),world,"IgnoreSelfCollision","on","Exhaustive","on");


    [bodyIdx,worldCollisionObjIdx] = find(isnan(sepDist)); % Find collision pairs
    worldCollidingPairs = [bodyIdx,worldCollisionObjIdx];
    worldCollisionPairIdx{i} = worldCollidingPairs;

end
% Display whether there is collision:
isTrajectoryInCollision = any(inCollision)


%% Record the whole training/learning process in video file
% Videos are on by default; set enableVideoTraining = 0 / enableVideo = 0 before running to skip them.
if ~exist('enableVideoTraining','var'), enableVideoTraining = 1; end
if ~exist('enableVideo','var'), enableVideo = 1; end
theme(hgif,'light'); figure(hgif);  % record the scene figure, light background even in dark mode

if enableVideoTraining == 1
    v = VideoWriter('KinvaGen3_Training.mp4', 'MPEG-4');   % .mp4 plays on any laptop (.avi did not)
    v.FrameRate = 15;
    open(v);

    htext = text(-0.2,0.6,0.7,'Iteration = 0','HorizontalAlignment','left','FontSize',14);

    theta_animation_tmp = theta_animation(~cellfun('isempty',theta_animation));
    nTraining = length(theta_animation_tmp);
    % (the sample looped k = 0:5:nTraining and read theta_animation_tmp{k+1}, which goes one past
    %  the end when nTraining is a multiple of 5.) Now a few iterations are shown, always the
    %  first and the last: all if nTraining <= 5, otherwise every 5th/10th/20th/25th/50th
    %  iteration (the smallest step that shows at most 8), e.g. 10 -> 1,5,10; 70 -> 1,10,...,70.
    showIters = 1:nTraining;
    if nTraining > 5
        for step = [5 10 20 25 50 100]
            showIters = unique([1, step:step:nTraining, nTraining]);
            if numel(showIters) <= 8, break; end
        end
        % drop the step just before the last iteration if it is very close to it (e.g. 10 and 11)
        if numel(showIters) > 2 && nTraining - showIters(end-1) < step/2, showIters(end-1) = []; end
    end
    for k = showIters

        UpdatedText = ['Iteration = ',num2str(k)];
        set(htext,'String',UpdatedText)
        theta_tmp = theta_animation_tmp{k};

        for t=1:size(theta_tmp,2)
            show(robot, theta_tmp(:,t),'PreservePlot', false, 'Frames', 'on');
            %             drawnow;
            frame = getframe(gcf);
            writeVideo(v,frame);
%             pause(1/15);
            %     pause;
        end
        pause(1/15);
    end

    close(v);
end




%% Record planned trajectory to video files
if enableVideo == 1
    v = VideoWriter('KinvaGen3_planned_path.mp4', 'MPEG-4');
    v.FrameRate =2;
    open(v);

    for t=1:size(theta,2)
        show(robot, theta(:,t),'PreservePlot', false, 'Frames', 'on');
        drawnow;
        frame = getframe(gcf);
        writeVideo(v,frame);
        pause(5/20);
        %     pause;
    end
    close(v);
end
%% Show the planned trajectory
displayAnimation = 1;
if displayAnimation
    for t=1:size(theta,2)
        show(robot, theta(:,t),'PreservePlot', false, 'Frames', 'on');
        drawnow;
        pause(5/20);
        %     pause;
    end
end



%% save data
filename = ['Theta_nDisc', num2str(nDiscretize),'_nPaths_', num2str(nPaths), '.mat'];
save(filename,'theta')

