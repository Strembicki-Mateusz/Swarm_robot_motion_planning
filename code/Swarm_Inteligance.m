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
BeginingPosition = [];
TargetPosition = [];

i = 1;
j = 1;
while i <= N
    BeginingPosition(i,:) = [randi([1 Rows]) , randi([1 Columns])];
    TargetPosition(i,:) = [randi([1 Rows]), randi([1 Columns])];
    if i > 1
        for b = 1:(i-1)
            if ( BeginingPosition(b,1) == BeginingPosition(i,1) ) || ( TargetPosition(b,1) == TargetPosition(i,1) ) || (BeginingPosition(b,1) == TargetPosition(i,1))
                i = i-1;
            end
        end
    end
    i = i+1;
end

%% Obstacles
Number_Obstacles = fix((abs(randi([3 Rows-2]) - randi([2 Columns])) * (Rows) * (Columns) * (abs(randi([Rows-6 Columns-3]) / N))) / N /5/5);
if Number_Obstacles > (Rows*Columns)*3/5 || Number_Obstacles < max(Rows,Columns)
    Number_Obstacles = fix((Rows*Columns)*1/8);
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

%save w3.mat
load w2.mat
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

%% Path Searching
% A* algorithm
A_PATH = [];
A_PATHS = [];
maxLength = 0;  % Max lenght of track

for s = 1:N
    A_PATH = a_star(Rows, Columns, ObstaclesPosition, TargetPosition(s,:), BeginingPosition(s,:));
    %A_PATH = dijkstra(Rows, Columns, ObstaclesPosition, TargetPosition(s,:), BeginingPosition(s,:));

    m = size(A_PATH, 1);

    if m > maxLength
        maxLength = m; 
    end

    % Initializing A_PATHS
    if s == 1
        A_PATHS = A_PATH; 
    else

        % Extend A_PATHS
        if size(A_PATHS, 1) < m
            lastPoint = A_PATHS(end, :);
            A_PATHS(size(A_PATHS, 1) + 1:m, :) = repmat(lastPoint, m - size(A_PATHS, 1), 1);
        end

        % Extend A_PATH
        if m < maxLength
            lastPoint = A_PATH(m, :);
            A_PATH(m+1:maxLength, :) = repmat(lastPoint, maxLength - m, 1);
        end


        A_PATHS = [A_PATHS, A_PATH];  % Adding path to paths matrix
    end
end


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
% 
% % animation core
% for j = 1:maxLength
%     for i = 1:numPaths
%         x = A_PATHS(1:maxLength, 2*i);  
%         y = A_PATHS(1:maxLength, 2*i-1);
% 
%         set(h(i), 'XData', x(j), 'YData', y(j));
%     end
%     drawnow;
%     pause(0.5);  
% end