%% Function to find perpendicular position for collision avoidance
function alt_pos = find_perpendicular_position(A_PATHS, robot_positions, robot_idx, step)
    % Get current and previous positions of the robot
    x = A_PATHS(step, 2*robot_idx);
    y = A_PATHS(step, 2*robot_idx-1);
    
    if step > 1
        x_prev = A_PATHS(step-1, 2*robot_idx);
        y_prev = A_PATHS(step-1, 2*robot_idx-1);
    else
        x_prev = x;
        y_prev = y;
    end
    
    % Determine movement direction
    if abs(x - x_prev) > abs(y - y_prev) % Moving in X direction
        possible_moves = [x, y+1; x, y-1]; % Try moving up or down
    else % Moving in Y direction
        possible_moves = [x+1, y; x-1, y]; % Try moving left or right
    end
    
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