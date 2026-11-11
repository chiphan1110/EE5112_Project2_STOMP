function cost = stompObstacleCost(sphere_centers,radius,voxel_world,vel)

safety_margin = 0.05; % the safety margin distance, unit: meter
cost = 0;

% signed distance function of the world
voxel_world_sEDT = voxel_world.sEDT;
world_size = voxel_world.world_size;

% calculate which voxels the sphere centers are in. idx is the grid xyz subscripts
% in the voxel world.
env_corner = voxel_world.Env_size(1,:); % [xmin, ymin, zmin] of the metric world
env_corner_vec = repmat(env_corner,length(sphere_centers),1); % copy it to be consistent with the size of sphere_centers
idx = ceil((sphere_centers-env_corner_vec)./voxel_world.voxel_size);

%% Complete the following code according to Eq (13) in the STOMP conference paper.
try
    % Spheres outside the voxel world are clamped to the nearest boundary voxel
    % (far from every obstacle, so their cost is 0).
    idx = min(max(idx, 1), world_size);
    lin_idx = sub2ind(world_size, idx(:,1), idx(:,2), idx(:,3));
    d = voxel_world_sEDT(lin_idx);   % signed distance (m) of each sphere centre; positive outside obstacles
    % Eq. (13): q_o = sum_b max(eps + r_b - d(x_b), 0) * ||xdot_b||
    cost_array = max(safety_margin + radius(:) - d(:), 0) .* vel(:);
    cost = sum(cost_array);
catch  % for debugging
    idx = ceil((sphere_centers-env_corner_vec)./voxel_world.voxel_size);
end