%% Path Searching - Swarm Navigation with Dynamic Weight Modification (Gradual Increase of Target Weight)
maxIter = 1500;         % maximum number of iterations (time steps)
targetThreshold = 5;   % threshold below which the robot focuses only on the target
penaltyFactor = 10;    % penalty for a move that does not bring the robot closer to the target
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