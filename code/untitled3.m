clear all;
close all;
clc;

%% Init
% Number of Robots
N = 5;

% Grid size (Minimum size of column is 5)
Rows = 20;
Columns = 50;

if Rows < 5
    Rows = 5;
end
if Columns < 5
    Columns = 5;
end
if N > Rows
    N = Rows - 2;
end
%% Create Begining and Target position on the map
BeginingPosition = [];
TargetPosition = [];

i = 1;
j = 1;
while i <= N
    BeginingPosition(i,:) = [randi([1 Rows]) , 1];
    TargetPosition(i,:) = [randi([1 Rows]), Columns];
    if i > 1
        for b = 1:(i-1)
            if ( BeginingPosition(b,1) == BeginingPosition(i,1) ) || ( TargetPosition(b,1) == TargetPosition(i,1) )
                i = i-1;
            end
        end
    end
    i = i+1;
end

%% Obstacles
Number_Obstacles = fix((abs(randi([3 Rows-2]) - randi([2 Columns])) * (Rows) * (Columns) * (abs(randi([Rows-6 Columns-3]) / N))) / N /5 );
if Number_Obstacles > (Rows*Columns)*3/5 || Number_Obstacles < max(Rows,Columns)
    Number_Obstacles = fix((Rows*Columns)*1/4);
end


ObstaclesPosition = [];
i = 1;
while i <= Number_Obstacles
    ObstaclesPosition(i,:) = [randi([1 Rows]), randi([3 Columns-2])];
    if i > 1
        for b = 1:(i-1)
            if (ObstaclesPosition(b,1) == ObstaclesPosition(i,1)) && (ObstaclesPosition(b,2) == ObstaclesPosition(i,2))
                i = i-1;
            end
        end
    end
    i = i+1;
end

ObstaclesPosition;
BeginingPosition;
TargetPosition;

%save f2.mat
%load lawica4.mat
%load f2.mat
%% Plot Map
% Plot Positions on Grid
figure(1); hold on; grid on;
xlabel('X');
ylabel('Y');
ax = gca; % current axes
ax.YLim = [0 Rows+1];
ax.XLim = [0 Columns+1];

for i = 1:N
    plot(BeginingPosition(i,2), BeginingPosition(i,1), 'bo');
    plot(TargetPosition(i,2), TargetPosition(i,1), 'go');
end

for i = 1:Number_Obstacles
    plot(ObstaclesPosition(i,2), ObstaclesPosition(i,1), 'ko');
end

%% Path Searching - Swarm Navigation with Dynamic Weight Modification (Gradual Increase of Target Weight)
maxIter = 1500;         % maximum number of iterations (time steps)
targetThreshold = 10;   % threshold below which the robot focuses only on the target
penaltyFactor = 5;    % penalty for a move that does not bring the robot closer to the target
stuckThreshold = 5;    % number of iterations without improvement after which the robot is considered "stuck"

% initial weight values
W_target_initial = 0.3;  % initial weight of attraction to the target
W_flock_initial  = 1;  % initial weight of maintaining the swarm

% Initialization of dynamic weights for each robot (column vectors)
W_target = repmat(W_target_initial, N, 1);
W_flock  = repmat(W_flock_initial, N, 1);


delta_W = 0.2;     % increase in target weight
delta_F = 0.1;     % decrease in flocking weight


A_PATHS = zeros(maxIter, 2*N);

% initial positions
for i = 1:N
    A_PATHS(1, 2*i-1) = BeginingPosition(i, 1);
    A_PATHS(1, 2*i)   = BeginingPosition(i, 2);
end

currentPositions = BeginingPosition;
iter = 1; 

% iteration counter without improvement for each robot
stuckCounter = zeros(N, 1);
prevDist = zeros(N,1);
for i = 1:N
    prevDist(i) = norm(currentPositions(i,:) - TargetPosition(i,:));
end


candidateDeltas = [0,0; -1,0; 1,0; 0,-1; 0,1; 1,1; -1,1; 1,-1; -1,-1];
numCandidates = size(candidateDeltas, 1);

while iter < maxIter
    newPositions = currentPositions;
    % Compute the center of the swarm based on current positions
    flockCenter = mean(currentPositions, 1);
    
    % Random update order of robots in this iteration to change priority
    order = randperm(N);
    
    for idx = 1:N
        i = order(idx);
        % If robot position is equal target then go through
        if isequal(currentPositions(i,:), TargetPosition(i,:))
            continue;
        end
        
        bestCost = Inf;
        bestMove = currentPositions(i,:);
        candidateCosts = [];  % for possible random selection
        
        currentDist = norm(currentPositions(i,:) - TargetPosition(i,:));
        
        for k = 1:numCandidates
            candidate = currentPositions(i,:) + candidateDeltas(k,:);
            
            
            if candidate(1) < 1 || candidate(1) > Rows || candidate(2) < 1 || candidate(2) > Columns
                continue;
            end
            
            
            if any(ismember(ObstaclesPosition, candidate, 'rows'))
                continue;
            end
            
            % Check for collisions: verify if a newly chosen position of another robot conflicts
            conflict = false;
            for j = 1:i-1
                if isequal(newPositions(j,:), candidate)
                    conflict = true;
                    break;
                end
            end
            if conflict
                continue;
            end
            
            % Compute movement cost with dynamic weights
            cost_target = norm(candidate - TargetPosition(i,:));
            if currentDist > targetThreshold
                cost_flock = norm(candidate - flockCenter);
            else
                cost_flock = 0;
            end
            totalCost = W_target(i)*cost_target + W_flock(i)*cost_flock;
            
            % Penalty if the move does not bring the robot closer to the target
            if norm(candidate - TargetPosition(i,:)) >= currentDist
                totalCost = totalCost + penaltyFactor;
            end
            
            candidateCosts(end+1) = totalCost;
            if totalCost < bestCost
                bestCost = totalCost;
                bestMove = candidate;
            end
        end
        
        % Optional: random selection among moves with similar costs
        tol = 1e-3;
        similarIdx = find(abs(candidateCosts - bestCost) < tol);
        if numel(similarIdx) > 1
            % Randomly choose one of the moves with a similar cost
            rIdx = similarIdx(randi(numel(similarIdx)));
            bestMove = currentPositions(i,:) + candidateDeltas(rIdx,:);
        end
        
        % Update stuck counter
        newDist = norm(bestMove - TargetPosition(i,:));
        if newDist >= currentDist
            stuckCounter(i) = stuckCounter(i) + 1;
        else
            stuckCounter(i) = 0;
        end
        
        % If the robot is "stuck," modify weights
        if stuckCounter(i) >= stuckThreshold
            W_target(i) = W_target(i) + delta_W;
            W_flock(i)  = max(0, W_flock(i) - delta_F);
            stuckCounter(i) = 0;  % reset counter
        end
        
        newPositions(i,:) = bestMove;
        prevDist(i) = norm(bestMove - TargetPosition(i,:));
    end
    
    iter = iter + 1;
    currentPositions = newPositions;
    
    % Save robot positions in the A_PATHS matrix
    for i = 1:N
        A_PATHS(iter, 2*i-1) = currentPositions(i,1);
        A_PATHS(iter, 2*i)   = currentPositions(i,2);
    end
    
    % Check if all robots have reached their targets
    reached = true;
    for i = 1:N
        if ~isequal(currentPositions(i,:), TargetPosition(i,:))
            reached = false;
            break;
        end
    end
    if reached
        break;
    end
end

% Trim the A_PATHS matrix to the actual number of iterations
maxLength = iter;
A_PATHS = A_PATHS(1:maxLength, :);


%% Checking collisions and updating paths
% Iterate over each time step
maxLength = size(A_PATHS, 1);  
numPaths = size(A_PATHS, 2) / 2;  % Number of robots


for j = 1:maxLength
    % Create a matrix to track positions of all robots at time step j
    % Each row will store x and y position of a robot at step j
    robot_positions = zeros(numPaths, 2);  

    for i = 1:numPaths
        robot_positions(i, :) = [A_PATHS(j, 2*i), A_PATHS(j, 2*i-1)];
    end

    for i = 1:numPaths
        for k = i+1:numPaths
            if isequal(robot_positions(i, :), robot_positions(k, :))


                % If robot i and robot k are at the same position, we have a collision.
                % Now, we will resolve the collision by adjusting the paths.
                % Robot i has higher priority, so we keep its path as is
                % Adjust robot k's path (make it wait)
                % Add a new step for robot k to avoid collision
                % Insert the new step by duplicating the last position of robot k

                A_PATHS = add_step_to_wait(A_PATHS, k, j, maxLength);

                % Adjust path length
                maxLength = size(A_PATHS, 1);
            end
        end
    end
end




for i = 1:size(A_PATHS,1)
    for j = 1:size(A_PATHS,2)
        if A_PATHS(i,j) == 0
            A_PATHS(i,j) = A_PATHS(i-1,j);
        end
    end
end










%% Animation
numPaths = N;
colors = lines(numPaths); 

% draw paths
for i = 1:numPaths
    x = A_PATHS(1:maxLength, 2*i);    
    y = A_PATHS(1:maxLength, 2*i-1);  
    plot(x, y, '--', 'Color', colors(i,:), 'LineWidth', 0.5);  
end

% pointer for points
h = gobjects(1, numPaths);

for i = 1:numPaths
    x = A_PATHS(1:maxLength, 2*i);    
    y = A_PATHS(1:maxLength, 2*i-1);  
    h(i) = plot(x(1), y(1), 'o', 'Color', colors(i,:), 'MarkerFaceColor', colors(i,:), 'MarkerSize', 8);
end

% animation core
for j = 1:maxLength
    for i = 1:numPaths
        x = A_PATHS(1:maxLength, 2*i);  
        y = A_PATHS(1:maxLength, 2*i-1);
        
        set(h(i), 'XData', x(j), 'YData', y(j));
    end
    drawnow;
    pause(0.5);  
end