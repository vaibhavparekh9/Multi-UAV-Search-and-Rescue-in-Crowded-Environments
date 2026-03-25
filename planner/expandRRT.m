function [rrtTree, bestPose, maxGain] = expandRRT(startPose, map, maxNodes, stepSize, g_min)
    % Expand an RRT and compute information gain
    rrtTree = startPose; % Initialize RRT with the start pose
    maxGain = 0;
    bestPose = startPose;

    for i = 1:maxNodes
        % Sample a random point
        randomPose = [randi(size(map.grid, 1)), randi(size(map.grid, 2)), ...
                      randi(size(map.grid, 3))];
        % Find the nearest node in the RRT
        [~, nearestIdx] = min(vecnorm(rrtTree(:, 1:3) - randomPose, 2, 2));
        nearestPose = rrtTree(nearestIdx, :);
        
        % Steer towards the random pose
        direction = (randomPose - nearestPose) / norm(randomPose - nearestPose);
        newPose = nearestPose + stepSize * direction;
        
        % Check if the new pose is collision-free
        if isCollisionFree(nearestPose, newPose, map)
            % Calculate information gain for the new pose
            gain = computeInformationGain(newPose, map, g_min);
            if gain > maxGain
                maxGain = gain;
                bestPose = newPose;
            end
            rrtTree = [rrtTree; newPose]; % Add new pose to the RRT
        end
    end
end
