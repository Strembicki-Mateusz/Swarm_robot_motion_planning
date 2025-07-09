function [Optimal_Path] = dijkstra(Rows, Columns, Obstacles, Target, Start)
% dijkstra - computes the shortest path from Start to Target
% considering obstacles. Uses the distance_a function to calculate distances.

% Initialization
distances = inf(Rows, Columns); % Distance matrix
visited = zeros(Rows, Columns); % Matrix of visited nodes
previous = cell(Rows, Columns); % Matrix of predecessor pointers

% Add obstacles
for i = 1:size(Obstacles, 1)
    visited(Obstacles(i,1), Obstacles(i,2)) = 1; % Mark obstacles as visited
end

% Initialize the start node
distances(Start(1), Start(2)) = 0;

% Dijkstra's algorithm implementation
while true
    % Find the closest unvisited node
    [minDist, current] = min_unvisited(distances, visited);
    if minDist == inf || (current(1) == Target(1) && current(2) == Target(2))
        break; % Stop if the target is reached or no nodes are available
    end
    
    visited(current(1), current(2)) = 1; % Mark the current node as visited

    % Check neighbors
    neighbors = get_neighbors(current, Rows, Columns);
    for k = 1:size(neighbors, 1)
        neighbor = neighbors(k, :);
        if visited(neighbor(1), neighbor(2)) == 0
            % Use the distance_a function to calculate the step distance
            step_distance = distance_a(current, neighbor);
            tentative_dist = distances(current(1), current(2)) + step_distance;
            if tentative_dist < distances(neighbor(1), neighbor(2))
                distances(neighbor(1), neighbor(2)) = tentative_dist;
                previous{neighbor(1), neighbor(2)} = current;
            end
        end
    end
end

% Reconstruct the path
Optimal_Path = [];
if distances(Target(1), Target(2)) < inf
    current = Target;
    while ~isempty(current)
        Optimal_Path = [current; Optimal_Path]; %#ok<AGROW>
        current = previous{current(1), current(2)};
    end
end

end


