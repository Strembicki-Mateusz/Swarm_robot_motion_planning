load test.mat


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

%% Path Searching - Swarm Navigation with Improved Collision Handling and Random Impulse
maxIter = 1500;         % maximum number of iterations (time steps)
targetThreshold = 10;   % threshold below which the robot focuses only on the target
penaltyFactor = 5;      % penalty for a move that does not bring the robot closer to the target
stuckThreshold = 5;     % number of iterations without improvement after which the robot is considered "stuck"

% initial weight values
W_target_initial = 0.3;  % initial weight of attraction to the target
W_flock_initial  = 1;    % initial weight of maintaining the swarm

% Initialization of dynamic weights for each robot (column vectors)
W_target = repmat(W_target_initial, N, 1);
W_flock  = repmat(W_flock_initial, N, 1);

delta_W = 0.2;     % increase in target weight
delta_F = 0.1;     % decrease in flocking weight

A_PATHS = zeros(maxIter, 2*N);

% initial positions
currentPositions = BeginingPosition;
for i = 1:N
    A_PATHS(1, 2*i-1) = currentPositions(i, 1);
    A_PATHS(1, 2*i)   = currentPositions(i, 2);
end

iter = 1;
stuckCounter = zeros(N, 1);
prevDist = vecnorm(currentPositions - TargetPosition, 2, 2);
candidateDeltas = [0,0; -1,0; 1,0; 0,-1; 0,1; 1,1; -1,1; 1,-1; -1,-1];
numCandidates = size(candidateDeltas, 1);

historyLength = 5; % długość historii posunięć robota
recentMoves = zeros(N, historyLength, 2); % zmienna do przechowywania ostatnich posunięć

while iter < maxIter
    newPositions = currentPositions;
    flockCenter = mean(currentPositions, 1);
    order = randperm(N);
    plannedPositions = currentPositions;
    
    for idx = 1:N
        i = order(idx);
        if isequal(currentPositions(i,:), TargetPosition(i,:))
            continue;
        end
        
        bestCost = Inf;
        bestMove = currentPositions(i,:);
        currentDist = norm(currentPositions(i,:) - TargetPosition(i,:));
        
        for k = 1:numCandidates
            candidate = currentPositions(i,:) + candidateDeltas(k,:);

            % Sprawdzenie, czy robot nie wychodzi poza granice mapy
            if candidate(1) < 1 || candidate(1) > Rows || candidate(2) < 1 || candidate(2) > Columns
                continue;
            end
            
            % Sprawdzenie, czy robot nie wpada na przeszkodę
            if any(ismember(ObstaclesPosition, candidate, 'rows'))
                continue;
            end
            
            % Sprawdzenie, czy robot nie wpada na innego robota (w tej samej iteracji)
            if any(ismember(plannedPositions, candidate, 'rows'))
                continue;
            end
            
            cost_target = norm(candidate - TargetPosition(i,:));
            cost_flock = norm(candidate - flockCenter) * (currentDist > targetThreshold);
            totalCost = W_target(i) * cost_target + W_flock(i) * cost_flock;
            
            if cost_target >= currentDist
                totalCost = totalCost + penaltyFactor;
            end
            
            if totalCost < bestCost
                bestCost = totalCost;
                bestMove = candidate;
            end
        end
        
        % Zaktualizuj historię ruchów robota
        recentMoves(i, :, :) = circshift(recentMoves(i, :, :), [0, 1]);
        recentMoves(i, 1, :) = bestMove;
        
        % Sprawdź, czy robot powtarza ostatnie ruchy (cykliczne ruchy)
        if any(all(bsxfun(@eq, reshape(recentMoves(i, :, :), [], 2), bestMove), 2))
            stuckCounter(i) = stuckCounter(i) + 1;
        else
            stuckCounter(i) = 0;
        end
        
        % Jeśli robot wykonał kilka powtarzających się posunięć, dodaj losowy impuls
        if stuckCounter(i) >= stuckThreshold
            % Dodaj losowy impuls
            randomImpulse = candidateDeltas(randi(numCandidates), :); 
            bestMove = currentPositions(i, :) + randomImpulse; % przesunięcie z impulsu
            
            % Sprawdzenie, czy losowy impuls prowadzi do wyjścia poza granice mapy
            if bestMove(1) < 1 || bestMove(1) > Rows || bestMove(2) < 1 || bestMove(2) > Columns
                continue;  % Jeśli impuls prowadzi poza granice, pominąć ten ruch
            end
            
            % Sprawdzenie, czy robot wpada na przeszkodę po dodaniu impulsu
            if any(ismember(ObstaclesPosition, bestMove, 'rows'))
                continue;  % Jeśli impuls prowadzi na przeszkodę, pominąć ten ruch
            end
            
            % Sprawdzenie, czy robot wpada na innego robota po dodaniu impulsu
            if any(ismember(plannedPositions, bestMove, 'rows'))
                continue;  % Jeśli impuls prowadzi na innego robota, pominąć ten ruch
            end
            
            stuckCounter(i) = 0; % resetuj licznik
        end
        
        plannedPositions(i, :) = bestMove;
    end
    
    iter = iter + 1;
    currentPositions = plannedPositions;
    for i = 1:N
        A_PATHS(iter, 2*i-1) = currentPositions(i,1);
        A_PATHS(iter, 2*i)   = currentPositions(i,2);
    end
    
    if all(ismember(currentPositions, TargetPosition, 'rows'))
        break;
    end
end

A_PATHS = A_PATHS(1:iter, :);



maxLength = iter;
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