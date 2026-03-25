% visualizeMap.m
function visualizeMap(map, iteration)
    figure(1);
    clf;
    hold on;
    grid on;
    axis equal;
    view(3);
    
    % Plot unexplored voxels
    [ux, uy, uz] = ind2sub(size(map), find(map == 0));
    scatter3(ux, uy, uz, 5, [0.8, 0.8, 0.8], 'filled'); % Light gray for unexplored
    
    % Plot free voxels
    [fx, fy, fz] = ind2sub(size(map), find(map == 1));
    scatter3(fx, fy, fz, 5, 'b', 'filled'); % Blue for free space
    
    % Plot occupied voxels
    [ox, oy, oz] = ind2sub(size(map), find(map == 2));
    scatter3(ox, oy, oz, 10, 'r', 'filled'); % Red for obstacles
    
    % Title and labels
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title(['Exploration Map - Iteration ', num2str(iteration)]);
    legend({'Unexplored', 'Free', 'Occupied'}, 'Location', 'northeastoutside');
    drawnow;
end
