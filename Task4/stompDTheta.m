% dtheta: estimated gradient
% em: 1 by nJoints cell, each cell is nSamples by nDiscretize matrix
function dtheta = stompDTheta(trajProb, em)

nJoints = length(em);
nDiscretize = size(trajProb, 2);

% variable declaration
dtheta = zeros(nJoints, nDiscretize);

% Estimated gradient (STOMP paper, Eq. 12 / LN5 p13): for each joint, weight
% the noise of every sampled path by its probability at each time step and sum
% over the samples.
for j = 1:nJoints
    dtheta(j,:) = sum(trajProb .* em{j}, 1); % (nSamples x nDisc) .* (nSamples x nDisc) -> 1 x nDisc
end

end
