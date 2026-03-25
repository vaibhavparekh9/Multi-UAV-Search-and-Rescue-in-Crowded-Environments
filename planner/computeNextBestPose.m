% computeNextBestPose.m
function [gain, bestPose] = computeNextBestPose(map)
    % Randomly simulate a next best pose and gain for simplicity

   
    bestPose = randi([1, 100], 1, 3);
    gain = 1000000 - sum(map(:)); % Example: Gain decreases as the map is explored

end
