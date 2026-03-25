function gain = computeInformationGain(pose, map, g_min)
    % Compute the information gain for a given pose dynamically

    % Define the sensing radius (in grid cells)
    radius = 5; 
    [x, y, z] = ndgrid(-radius:radius, -radius:radius, -radius:radius);

    % Calculate the neighborhood around the pose
    neighborhood = [x(:), y(:), z(:)] + round(pose);

    % Clamp indices within the grid bounds
    validIndices = neighborhood( ...
        all(neighborhood >= 1 & neighborhood <= size(map.grid), 2), :);

    % If no valid indices, return zero gain
    if isempty(validIndices)
        gain = 0;
        return;
    end

    % Convert valid indices to linear indices
    linearIndices = sub2ind(size(map.grid), ...
                            validIndices(:, 1), ...
                            validIndices(:, 2), ...
                            validIndices(:, 3));

    % Calculate unexplored and explored voxels
    unexploredVoxels = ~map.grid(linearIndices);
    exploredVoxels = map.grid(linearIndices);

    % Debug: Log voxel counts
    disp(['Unexplored voxels: ', num2str(sum(unexploredVoxels)), ...
          ', Explored voxels: ', num2str(sum(exploredVoxels))]);

    % Compute the gain as the number of newly uncovered voxels
    gain = sum(unexploredVoxels) - g_min * sum(exploredVoxels);

    % Ensure gain is non-negative
    gain = max(gain, 0);
end
