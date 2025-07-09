clear all;
close all;
clc;

%% Init
% Number of Robots
N = 5;

% Grid size (Minimum size of column is 5)
Rows = 10;
Columns = 20;

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
BeginingPosition = zeros(N, 2);  % Matrix storing the initial positions
TargetPosition = zeros(N, 2);    % Matrix storing the target positions

i = 1;
while i <= N
    BeginingPosition(i,:) = [randi([1 Rows]), 1];
    TargetPosition(i,:) = [randi([1 Rows]), Columns];
    if i > 1
        for b = 1:(i-1)
            if (BeginingPosition(b,1) == BeginingPosition(i,1)) || (TargetPosition(b,1) == TargetPosition(i,1))
                i = i-1;
            end
        end
    end
    i = i + 1;
end

%% Obstacles
Number_Obstacles = fix((abs(randi([3 Rows-2]) - randi([2 Columns])) * (Rows) * (Columns) * (abs(randi([Rows-6 Columns-3]) / N))) / N /5 );
if Number_Obstacles > (Rows * Columns) * 3 / 5 || Number_Obstacles < max(Rows, Columns)
    Number_Obstacles = fix((Rows * Columns) * 1 / 4);
end

ObstaclesPosition = zeros(Number_Obstacles, 2);
i = 1;
while i <= Number_Obstacles
    ObstaclesPosition(i,:) = [randi([1 Rows]), randi([3 Columns-2])];
    if i > 1
        for b = 1:(i-1)
            if (ObstaclesPosition(b,1) == ObstaclesPosition(i,1)) && (ObstaclesPosition(b,2) == ObstaclesPosition(i,2))
                i = i - 1;
            end
        end
    end
    i = i + 1;
end

% Plot Map
figure(1); hold on; grid on;
xlabel('X');
ylabel('Y');
ax = gca; % current axes
ax.YLim = [0 Rows + 1];
ax.XLim = [0 Columns + 1];

for i = 1:N
    plot(BeginingPosition(i,2), BeginingPosition(i,1), 'bo');
    plot(TargetPosition(i,2), TargetPosition(i,1), 'go');
end

for i = 1:Number_Obstacles
    plot(ObstaclesPosition(i,2), ObstaclesPosition(i,1), 'ko');
end

%% FSS Algorithm (Fish School Search)

MaxIter = 100;  % Max number of iterations
Wmax = 1;       % Max weight
Wmin = 0.1;     % Min weight
StepInd = 0.1;   % Step for individual movement
StepVol = 0.1;   % Step for collective movement

% Initialize matrices for positions and weights
X = zeros(N, 2);  % Positions of the robots
W = ones(N, 1) * (Wmax / 2);  % Weights of the robots
Path = zeros(N, MaxIter, 2); % Path for each robot
F = zeros(N, 1);  % Fitness (cost) of the robots

% Initializing the positions of robots
for i = 1:N
    X(i,:) = [BeginingPosition(i,1), BeginingPosition(i,2)]; % Initial position
    Path(i, 1, :) = X(i,:);  % Store initial path
    F(i) = calculate_cost(X(i,:), TargetPosition(i,:), ObstaclesPosition); % Initial cost
end

% Fish School Search Algorithm Iterations
for iter = 1:MaxIter
    for i = 1:N
        % Individual movement
        R = rand(1, 2) - 0.5;  % Random direction
        Xp = X(i,:) + R * StepInd;  % Move individual

        % Check for collision with obstacles
        if ~check_collision(Xp, ObstaclesPosition)
            Fp = calculate_cost(Xp, TargetPosition(i,:), ObstaclesPosition); % Cost for new position
            if Fp < F(i)  % If the new position is better
                X(i,:) = Xp;  % Update position
                F(i) = Fp;  % Update cost
                Path(i, iter + 1, :) = X(i,:);  % Update path
            end
        end
        
        % Update weight based on improvement
        if F(i) < Fp
            W(i) = W(i) + (F(i) - Fp) / (MaxIter * StepInd) * Wmax;  % Increase weight
        else
            W(i) = W(i) - (Fp - F(i)) / (MaxIter * StepInd) * Wmax;  % Decrease weight
        end
        W(i) = max(W(i), Wmin);  % Ensure the weight is not too small
    end
    
    % Collective movement
    X_center = sum(W .* X) / sum(W);  % Weighted center of the fish school
    
    for i = 1:N
        X(i,:) = X(i,:) + (X_center - X(i,:)) * StepVol;  % Move towards the center
        Path(i, iter + 1, :) = X(i,:);  % Update path
    end
end

%% Path Display

% Plotting the final paths of robots
for i = 1:N
    plot(squeeze(Path(i, :, 2)), squeeze(Path(i, :, 1)), '--', 'LineWidth', 2);  % Plot path for each robot
end

%% Functions
% Function to calculate the cost (distance to target + penalty for obstacles)
function cost = calculate_cost(position, target, obstacles)
    dist_to_target = norm(position - target);  % Euclidean distance to target
    penalty = 0;
    
    for i = 1:size(obstacles,1)
        % Check if the position is too close to an obstacle
        if norm(position - obstacles(i,:)) < 1
            penalty = penalty + 100;  % Large penalty for collision
        end
    end
    
    cost = dist_to_target + penalty;  % Total cost
end

% Function to check if the position collides with obstacles
function is_collision = check_collision(position, obstacles)
    is_collision = false;
    for i = 1:size(obstacles,1)
        if norm(position - obstacles(i,:)) < 1
            is_collision = true;
            return;
        end
    end
end
