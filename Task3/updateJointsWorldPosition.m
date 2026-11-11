%% forward kinematics with the Product of Exponentials (PoE) formula  (Task 3)
%
%   T_k(theta) = e^[S_1]th_1 e^[S_2]th_2 ... e^[S_k]th_k  M_k        (LN4 p10, LN5 p21)
%
% Same inputs and outputs as the getTransform version of Task 2, so the rest of
% the STOMP code is unchanged.
% INPUT:
%       robot_struct: rigidBodyTree (struct data format)
%       theta:        the MOVING joint variables (n x 1)
% OUTPUT:
%       X: (n+1) x 4. Row k = [x y z 1] of the frame of the k-th moving joint in
%          the base frame; the last row is the end-effector frame (tool0).
%       T: 1 x (n+1) cell of 4x4 homogeneous transforms (same ordering)
%
% WHAT MAKES IT FAST
%   * The screw axes and zero-position poses are computed ONCE per robot
%     (computeScrewAxes) and cached; they are never recomputed per call.
%   * The running product P_k = P_(k-1) * e^[S_k]th_k is reused (LN5 p22), so one
%     pass over the joints gives every joint frame (no restart from joint 1).
%   * The matrix exponential uses the closed form of LN3 p27 directly:
%       revolute (|w| = 1):  R = I + sin(th) W + (1-cos th) W^2
%                            p = (I th + (1-cos th) W + (th - sin th) W^2) v
%       prismatic (w = 0):   R = I,  p = v th
%     with W = [w] (skew matrix). W, W^2, W*v and W^2*v are precomputed once.
function [X, T] = updateJointsWorldPosition(robot_struct, theta)

persistent cachedRobot n isRev Wm W2m Wv W2v Vlin MR Mp MeeR Meep
if isempty(cachedRobot) || ~isvalid(cachedRobot) || cachedRobot ~= robot_struct
    % ---- once per robot: derive screw axes and precompute constants ----
    ax = computeScrewAxes(robot_struct);
    cachedRobot = robot_struct;
    n = size(ax.Slist, 2);
    isRev = strcmp(ax.jointType, 'revolute');
    Wm = zeros(3,3,n); W2m = zeros(3,3,n); Wv = zeros(3,n); W2v = zeros(3,n);
    Vlin = ax.Slist(4:6, :);
    MR = zeros(3,3,n); Mp = zeros(3,n);
    for k = 1:n
        w = ax.Slist(1:3,k);
        W = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];   % skew matrix [w]
        Wm(:,:,k) = W; W2m(:,:,k) = W*W;
        Wv(:,k)  = W*Vlin(:,k);  W2v(:,k) = W*W*Vlin(:,k);
        MR(:,:,k) = ax.Mlist{k}(1:3,1:3);  Mp(:,k) = ax.Mlist{k}(1:3,4);
    end
    MeeR = ax.Mee(1:3,1:3);  Meep = ax.Mee(1:3,4);
end

X = zeros(n+1, 4);
T = cell(1, n+1);
Rp = eye(3);  pp = zeros(3,1);            % running product P = [Rp pp; 0 1]
for k = 1:n
    th = theta(k);
    if isRev(k)
        s = sin(th);  c = cos(th);
        Re = eye(3) + s*Wm(:,:,k) + (1-c)*W2m(:,:,k);
        pe = th*Vlin(:,k) + (1-c)*Wv(:,k) + (th-s)*W2v(:,k);
    else
        Re = eye(3);  pe = th*Vlin(:,k);
    end
    pp = Rp*pe + pp;                      % P <- P * e^[S_k]th_k
    Rp = Rp*Re;
    Rk = Rp*MR(:,:,k);  pk = Rp*Mp(:,k) + pp;     % frame of joint k:  P * M_k
    T{k} = [Rk, pk; 0 0 0 1];
    X(k,:) = [pk', 1];
end
Rk = Rp*MeeR;  pk = Rp*Meep + pp;                  % end-effector:  P * M_ee
T{n+1} = [Rk, pk; 0 0 0 1];
X(n+1,:) = [pk', 1];
end
