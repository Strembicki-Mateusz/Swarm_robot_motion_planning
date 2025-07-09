function [minDist, minNode] = min_unvisited(distances, visited)
% Finds the closest unvisited node
minDist = inf;
minNode = [];
for i = 1:size(distances, 1)
    for j = 1:size(distances, 2)
        if visited(i, j) == 0 && distances(i, j) < minDist
            minDist = distances(i, j);
            minNode = [i, j];
        end
    end
end
end