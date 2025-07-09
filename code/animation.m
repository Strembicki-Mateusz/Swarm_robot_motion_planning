close all;
clear all;
clc;

load lawica_dziala_3.mat

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