%% Function to find alternative position for collision avoidance
function alt_pos = find_alternative_position(A_PATHS, robot_positions, robot_idx, step)
    % Get current position of the robot
    x = A_PATHS(step, 2*robot_idx);
    y = A_PATHS(step, 2*robot_idx-1);
    
    % Define possible moves: left, right, up, down
    possible_moves = [x+1, y; x-1, y; x, y+1; x, y-1];
    
    % Check for an unoccupied position
    for m = 1:size(possible_moves, 1)
        if ~ismember(possible_moves(m, :), robot_positions, 'rows')
            alt_pos = possible_moves(m, :);
            return;
        end
    end
    
    % If no alternative position is found, return empty
    alt_pos = [];
end