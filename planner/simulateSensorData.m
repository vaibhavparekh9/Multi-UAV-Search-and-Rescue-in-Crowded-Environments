% simulateSensorData.m
function sensorData = simulateSensorData(map, pose, range)
    % Simulate a random sensor sweep
    [X, Y, Z] = ndgrid(-range:range, -range:range, -range:range);
    sensorPoints = [X(:), Y(:), Z(:)];
    sensorPoints = bsxfun(@plus, sensorPoints, pose); % Translate to pose
    sensorPoints = max(min(sensorPoints, size(map)), 1); % Keep within map bounds

    % Randomly mark some points as obstacles
    obstacleIdx = randi(size(sensorPoints, 1), [1, floor(size(sensorPoints, 1) * 0.2)]);
    sensorData = struct('free', sensorPoints(setdiff(1:end, obstacleIdx), :), ...
                        'occupied', sensorPoints(obstacleIdx, :));
end
