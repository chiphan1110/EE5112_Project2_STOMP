%% Construct the obstacles (Task 4: several boxes / cylinders from scenarioUR5)
% Every obstacle is added in TWO places that must agree:
%   1) world       : MATLAB collision objects, used for display and for the final checkCollision test
%   2) voxel_world : occupancy grid + signed distance field (sEDT), used by the STOMP cost
% Needs the struct `sc` (from scenarioUR5) in the workspace.

% Workspace (voxel world limits) and resolution
Env_size   = [-1, -1, -1; 2, 2, 2];   % [xmin ymin zmin] in the 1st row, xyz lengths in the 2nd row
voxel_size = [0.02, 0.02, 0.02];      % m
nVox = round(Env_size(2,:) ./ voxel_size);

% Centres of all voxels, indexed (ix, iy, iz) like binary_world. NOTE: ndgrid, not meshgrid:
% meshgrid would swap the x and y dimensions.
gx = Env_size(1,1) + ((1:nVox(1)) - 0.5) * voxel_size(1);
gy = Env_size(1,2) + ((1:nVox(2)) - 0.5) * voxel_size(2);
gz = Env_size(1,3) + ((1:nVox(3)) - 0.5) * voxel_size(3);
[Xw, Yw, Zw] = ndgrid(gx, gy, gz);

binary_world = false(nVox);
world = cell(1, numel(sc.obstacles));
for i = 1:numel(sc.obstacles)
    o = sc.obstacles(i);
    switch o.type
        case 'box'
            obj = collisionBox(o.size(1), o.size(2), o.size(3));
            inside = abs(Xw - o.center(1)) <= o.size(1)/2 & ...
                     abs(Yw - o.center(2)) <= o.size(2)/2 & ...
                     abs(Zw - o.center(3)) <= o.size(3)/2;
        case 'cylinder'
            obj = collisionCylinder(o.size(1), o.size(2));
            inside = (Xw - o.center(1)).^2 + (Yw - o.center(2)).^2 <= o.size(1)^2 & ...
                     abs(Zw - o.center(3)) <= o.size(2)/2;
        otherwise
            error('unknown obstacle type %s', o.type);
    end
    % thin objects (thinner than a voxel) must still occupy at least one voxel layer
    if ~any(inside(:))
        [~, ix] = min(abs(gx - o.center(1))); [~, iy] = min(abs(gy - o.center(2))); [~, iz] = min(abs(gz - o.center(3)));
        inside(ix, iy, iz) = true;
    end
    obj.Pose = trvec2tform(o.center);
    world{i} = obj;
    binary_world = binary_world | inside;
end
clear Xw Yw Zw inside

% Signed Euclidean distance (m) to the nearest obstacle: positive outside, negative inside
voxel_world.voxel_size = voxel_size;
voxel_world.voxel      = double(binary_world);
voxel_world.world_size = size(binary_world);
voxel_world.Env_size   = Env_size;
voxel_world.sEDT       = prod(voxel_size)^(1/3) * sEDT_3d(double(binary_world));
