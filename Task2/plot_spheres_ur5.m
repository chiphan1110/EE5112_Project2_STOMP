% Figure for the report/presentation: the UR5 and the spheres (balls) that
% STOMP uses to represent it for collision checking.
% Standalone: loads the planned path saved by UR5_STOMP_Path_Planning.m
% (Theta_UR5_nDisc20_nPaths_20.mat), draws the real arm, the spheres, and the box.
%   Output: Task2_spheres.png
S = load('Theta_UR5_nDisc20_nPaths_20.mat'); theta = S.theta;
robot = loadrobot('universalUR5','DataFormat','column');
robot_struct = loadrobot('universalUR5'); clear updateJointsWorldPosition
box = collisionBox(0.16,0.16,0.16); box.Pose = trvec2tform([0.57 0 0.25]);

f = figure('Position',[100 100 1200 520],'Color','w');
cols = [1 11];                         % start and the middle of the planned path
ttl  = {'Start configuration','Middle of the STOMP path'};
[sx,sy,sz] = sphere(14);
for p = 1:2
    subplot(1,2,p); hold on;
    show(robot, theta(:,cols(p)), 'PreservePlot', false, 'Frames','off');
    [X,~] = updateJointsWorldPosition(robot_struct, theta(:,cols(p)));
    [c,rad] = stompRobotSphere(X);
    for i = 1:size(c,1)
        surf(c(i,1)+rad(i)*sx, c(i,2)+rad(i)*sy, c(i,3)+rad(i)*sz, ...
            'FaceColor',[0.1 0.4 0.9],'FaceAlpha',0.45,'EdgeColor','none');
    end
    [~,pObj] = show(box); pObj.LineStyle='none'; pObj.FaceColor=[0.9 0.2 0.2]; pObj.FaceAlpha=0.6;
    axis([-0.4 0.9 -0.7 0.7 -0.1 0.7]); daspect([1 1 1]); view(135,25); camlight; title(sprintf('%s (%d spheres)',ttl{p},size(c,1)));
end
exportgraphics(f,'Task2_spheres.png','Resolution',150);
