function score = evaluatePaths(paths, obstacles, goalPositions, numRobots)
    score = 0;
    
    % Przekształcenie tablicy paths do współrzędnych robotów
    for i = 1:numRobots
        robotPos = [paths(2*i-1), paths(2*i)]; % Współrzędne (x, y) i-tego robota

        % Odległość od celu
        distToGoal = norm(robotPos - goalPositions(i, :));
        score = score + distToGoal;

        % Dodaj koszt przeszkód
        for j = 1:size(obstacles, 1)
            distToObstacle = norm(robotPos - obstacles(j, :));
            if distToObstacle < 1 % Odległość minimalna od przeszkody
                score = score + 1000; % Duży koszt, jeśli robot zbliża się do przeszkody
            end
        end
    end

    % Unikanie kolizji między robotami
    for i = 1:numRobots
        for j = i+1:numRobots
            % Oblicz odległość między robotami
            distBetweenRobots = norm([paths(2*i-1), paths(2*i)] - [paths(2*j-1), paths(2*j)]);
            
            if distBetweenRobots < 1 % Zakładając, że roboty to punkty materialne
                score = score + 1000; % Duży koszt, jeśli roboty się zderzą
            end
        end
    end
end
