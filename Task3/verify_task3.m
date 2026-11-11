%% Task 3 verification: screw axes (3 methods), PoE forward kinematics vs getTransform, timing
% Run from the Task3 folder:  verify_task3
% Output is printed and saved to Task3_verification.txt
% (the body of this script runs inside evalc so that its printed output is also saved to Task3_verification.txt)
if ~exist('inEvalc','var')
    inEvalc = true;
    outTxt = evalc(mfilename);
    fprintf('%s', outTxt);
    fid = fopen(fullfile(fileparts(mfilename('fullpath')), 'Task3_verification.txt'), 'w'); fwrite(fid, outTxt); fclose(fid);
    clear inEvalc outTxt fid
    return
end
addpath('mr_library');                       % Modern Robotics code: only used here, for the checks
rng(0);
fprintf('==== Task 3 verification (%s) ====\n', datestr(now));
robot_struct = loadrobot('universalUR5');                        % struct format (used by STOMP)
robot        = loadrobot('universalUR5','DataFormat','column');  % column format (used by getTransform below)
clear updateJointsWorldPosition updateJointsWorldPosition_getTransform
n = 6; z0 = zeros(n,1);

%% 1) Screw axes: three independent methods
ax  = computeScrewAxes(robot_struct);        % method B (used by the planner): zero pose + joint axis, v = -w x q
SB  = ax.Slist;  Mee = ax.Mee;
% Method C: rotate ONE joint by 0.3 rad, take the end-effector pose, matrix logarithm, normalise
SC = zeros(6,n);
for k = 1:n
    qq = z0; qq(k) = 0.3;
    E  = getTransform(robot, qq, 'tool0') / Mee;         % = exp([S_k]*0.3) when only joint k moved
    V  = se3ToVec(MatrixLog6(E));                         % [w; v]*theta
    SC(:,k) = V / norm(V(1:3));
end
% Method A: the lecture table (LN4 p12) with L1,L2,W1,W2,H1,H2 read from the model's zero pose
H1 = ax.Mlist{1}(3,4); L1 = ax.Mlist{3}(1,4); L2 = ax.Mlist{4}(1,4)-L1;
W1 = ax.Mlist{5}(2,4); W2 = Mee(2,4)-W1;      H2 = H1 - Mee(3,4);
SA = [0 0 1 0 0 0;  0 1 0 -H1 0 0;  0 1 0 -H1 0 L1;  0 1 0 -H1 0 L1+L2;  0 0 -1 -W1 L1+L2 0;  0 1 0 H2-H1 0 L1+L2]';
MA = [-1 0 0 L1+L2; 0 0 1 W1+W2; 0 1 0 H1-H2; 0 0 0 1];
fprintf('\nUR5 dimensions read from the model (m): L1=%.4f L2=%.4f W1=%.4f W2=%.4f H1=%.4f H2=%.4f\n',L1,L2,W1,W2,H1,H2);
fprintf('Lecture M vs model zero-pose tool0: max abs diff = %.2e\n', max(abs(MA(:)-Mee(:))));
fprintf('\nScrew axes derived by computeScrewAxes (columns = joints 1..6; rows = wx wy wz vx vy vz):\n'); disp(round(SB,4));
fprintf('max |B - C| (zero-pose geometry vs matrix log) = %.2e\n', max(abs(SB(:)-SC(:))));
fprintf('max |B - A| (zero-pose geometry vs lecture table) = %.2e\n', max(abs(SB(:)-SA(:))));
fprintf('max |C - A| = %.2e\n', max(abs(SC(:)-SA(:))));

%% 2) PoE forward kinematics vs getTransform (random joint angles)
N = 1000; errX = 0; errT = 0; errLib = 0;
for t = 1:N
    q = (rand(n,1)-0.5)*2*pi;
    [X1,T1] = updateJointsWorldPosition(robot_struct, q);               % PoE (this task)
    [X2,T2] = updateJointsWorldPosition_getTransform(robot_struct, q);  % getTransform (Task 2)
    errX = max(errX, max(abs(X1(:)-X2(:))));
    for k = 1:n+1, errT = max(errT, max(abs(T1{k}(:)-T2{k}(:)))); end
    errLib = max(errLib, max(abs(reshape(FKinSpace(Mee,SB,q)-T1{n+1},[],1))));   % Modern Robotics FKinSpace
end
fprintf('\nPoE vs getTransform over %d random configurations:\n', N);
fprintf('  joint positions X : max abs error = %.2e\n', errX);
fprintf('  all 7 frames T    : max abs error = %.2e   (includes orientation)\n', errT);
fprintf('  PoE vs library FKinSpace (end-effector) : %.2e\n', errLib);
% zero configuration must reproduce M exactly
[~,Tz] = updateJointsWorldPosition(robot_struct, z0);
fprintf('  zero configuration: |T_ee - M| = %.2e\n', max(abs(Tz{n+1}(:)-Mee(:))));

%% 3) Extra test of the PRISMATIC branch (the UR5 has none): small made-up robot R-P-R
tree = rigidBodyTree;                                  % struct data format by default
b1 = rigidBody('b1'); j1 = rigidBodyJoint('j1','revolute');  j1.JointAxis = [0 0 1];
setFixedTransform(j1, trvec2tform([0 0 0.1])*axang2tform([1 0 0 pi/2])); b1.Joint = j1; addBody(tree,b1,'base');
b2 = rigidBody('b2'); j2 = rigidBodyJoint('j2','prismatic'); j2.JointAxis = [0 0 1];
setFixedTransform(j2, trvec2tform([0.3 0.05 0])*axang2tform([0 1 0 pi/2])); b2.Joint = j2; addBody(tree,b2,'b1');
b3 = rigidBody('b3'); j3 = rigidBodyJoint('j3','revolute');  j3.JointAxis = [0 1 0];
setFixedTransform(j3, trvec2tform([0.1 0 0.2])); b3.Joint = j3; addBody(tree,b3,'b2');
be = rigidBody('ee'); jf = rigidBodyJoint('jf','fixed'); setFixedTransform(jf, trvec2tform([0.1 0.02 0.05])*axang2tform([0 0 1 0.4])); be.Joint = jf; addBody(tree,be,'b3');
clear updateJointsWorldPosition
errP = 0; cfg = tree.homeConfiguration;
for t = 1:200
    q = [(rand-0.5)*2*pi; (rand-0.5)*0.4; (rand-0.5)*2*pi];
    [~,Tp] = updateJointsWorldPosition(tree, q);
    [cfg.JointPosition] = deal(q(1),q(2),q(3));
    Tg = getTransform(tree, cfg, 'ee');
    errP = max(errP, max(abs(Tp{end}(:)-Tg(:))));
end
fprintf('\nPrismatic-joint test (R-P-R robot, 200 random configs): end-effector max abs error = %.2e\n', errP);
clear updateJointsWorldPosition

%% 4) Timing: one forward-kinematics call (6 joint frames + end-effector)
Q = (rand(n,2000)-0.5)*2*pi;
updateJointsWorldPosition(robot_struct,Q(:,1)); updateJointsWorldPosition_getTransform(robot_struct,Q(:,1));  % warm up
tic; for i = 1:2000, [~,~] = updateJointsWorldPosition_getTransform(robot_struct,Q(:,i)); end; tGT  = toc/2000;
tic; for i = 1:2000, [~,~] = updateJointsWorldPosition(robot_struct,Q(:,i));              end; tPoE = toc/2000;
fprintf('\nTime per forward-kinematics call: getTransform %.3f ms | PoE %.3f ms | speed-up %.1fx\n', tGT*1e3, tPoE*1e3, tGT/tPoE);
