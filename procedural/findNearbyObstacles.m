function [nearby_obstacle_indices] = findNearbyObstacles(ee, link_length, min_length, obstacles)
    n_obstacles = size(obstacles,1);
    nearby_obstacle_indices = zeros(1,n_obstacles);
    n_nearby_obstacles = 0;
    for i=1:1:n_obstacles
        d = norm(ee-obstacles(i,1:2)) - (obstacles(i,4) + min_length);
        if d<link_length
            n_nearby_obstacles = n_nearby_obstacles +1;
            nearby_obstacle_indices(n_nearby_obstacles) = i;
        end
    end
    nearby_obstacle_indices(n_nearby_obstacles+1:end) = [];
end