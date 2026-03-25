function transformedPoints = transformPointCloud(sensorData, pose)
    % Check if pose includes orientation
    if length(pose) < 6
        % Default to zero orientation if not provided
        roll = 0; pitch = 0; yaw = 0;
    else
        roll = pose(4); pitch = pose(5); yaw = pose(6);
    end
    
    % Compute rotation matrix
    R = eul2rotm([yaw, pitch, roll]); % Note: Euler angles are in the order [yaw, pitch, roll]
    t = pose(1:3)'; % Translation vector

    % Apply transformation
    transformedPoints = (R * sensorData' + t)';
end
