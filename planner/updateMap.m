% updateMap.m
function map = updateMap(map, pose, sensorData)
    % Mark free voxels
    for i = 1:size(sensorData.free, 1)
        map(sensorData.free(i, 1), sensorData.free(i, 2), sensorData.free(i, 3)) = 1;
    end
    
    % Mark occupied voxels
    for i = 1:size(sensorData.occupied, 1)
        map(sensorData.occupied(i, 1), sensorData.occupied(i, 2), sensorData.occupied(i, 3)) = 2;
    end
end
