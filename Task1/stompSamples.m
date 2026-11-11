% Input: 
%   nSamplePaths: the number of sample paths 
%   sigma： sample covariance matrix
%   theta: mean trajectory from last iteration
% Output:
%   theta_paths: sampled trajectories
%   em: sampled zero mean Gaussian trajectory for each joint

function [theta_paths, em]=stompSamples(nSamplePaths,sigma,theta)
% Sample theta (joints angles) trajectory 

[nJoints, nDiscretize] = size(theta);

em = cell(1,nJoints);
ek = cell(1,nSamplePaths);

theta_paths = cell(1, nSamplePaths);
mu=zeros(1,length(sigma));
% inv() leaves tiny asymmetries; mvnrnd needs an exactly symmetric covariance
sigma = (sigma + sigma')/2;

%% Complete code of independent sampling for each joint
for m = 1 : nJoints
    % Each joint is sampled independently.
    % The starting q0 and final qT are fixed, so set the sample to 0.
    % Sample from multivariable Gaussian distribution, eps ~ N(0, Rinv),
    % for the nDiscretize-2 interior waypoints, then pad one zero at each end.
    interior = mvnrnd(mu, sigma, nSamplePaths);               % nSamplePaths x (nDiscretize-2)
    em{m} = [zeros(nSamplePaths,1), interior, zeros(nSamplePaths,1)]; % nSamplePaths x nDiscretize
end

%% Regroup it by samples
emk = [em{:}];
for k=1:nSamplePaths
    ek{k} = reshape(emk(k,:),nDiscretize, nJoints)';
    theta_paths{k} = theta + ek{k};    
end