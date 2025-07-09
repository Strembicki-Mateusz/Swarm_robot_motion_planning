function neighbors = get_neighbors(current, Rows, Columns)
% get_neighbors - function to get all 8 neighbors of a given node

% Define 8 directions (up, down, left, right, and diagonals)
directions = [-1 0; 1 0; 0 -1; 0 1; -1 -1; -1 1; 1 -1; 1 1];

neighbors = [];
for k = 1:size(directions, 1)
    i = directions(k, 1);
    j = directions(k, 2);
    
    % Calculate the new position
    new_pos = current + [i, j];
    
    % Check if the new position is within bounds
    if new_pos(1) > 0 && new_pos(1) <= Rows && new_pos(2) > 0 && new_pos(2) <= Columns
        neighbors = [neighbors; new_pos];
    end
end
end
