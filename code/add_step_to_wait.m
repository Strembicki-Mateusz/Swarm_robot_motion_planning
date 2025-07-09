function A_PATHS = add_step_to_wait(A_PATHS, robot_index, current_step, maxLength)
current_position = [A_PATHS(current_step, 2*robot_index), A_PATHS(current_step, 2*robot_index-1)];
previous_position = [A_PATHS(current_step-1, 2*robot_index), A_PATHS(current_step-1, 2*robot_index-1)];

% Check if the path needs to be extended
if current_step == maxLength
    A_PATHS = [A_PATHS; zeros(1, size(A_PATHS, 2))];  % Add a new row at the end of A_PATHS
end

% Shift the remaining steps down by one to insert a new step
A_PATHS(current_step:maxLength+1, 2*robot_index-1:end) = A_PATHS(current_step-1:maxLength, 2*robot_index-1:end);

% Insert a new position for robot k at the current step

A_PATHS(current_step, 2*robot_index-1) = previous_position(2);
A_PATHS(current_step, 2*robot_index) = previous_position(1);
end