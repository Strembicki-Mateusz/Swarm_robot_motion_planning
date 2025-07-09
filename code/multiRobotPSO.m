function [bestPaths] = multiRobotPSO(numRobots, numCols, numRows, startPositions, goalPositions, obstacles)
    % Parametry PSO
    numParticles = 50; % Liczba cząsteczek
    numIterations = 100; % Liczba iteracji
    w = 0.7; % Współczynnik bezwładności
    c1 = 1.5; % Współczynnik przyciągania do najlepszych cząsteczek
    c2 = 1.5; % Współczynnik przyciągania do globalnej najlepszej cząsteczki

    % Inicjalizacja cząsteczek (człony cząsteczek to współrzędne robotów)
    particles = initializeParticles(numParticles, numRobots, numCols, numRows, startPositions, goalPositions);
    velocities = zeros(numParticles, numRobots * 2); % Prędkości cząsteczek (2 dla każdego robota)

    % Najlepsze ścieżki
    personalBestPositions = particles;
    personalBestScores = evaluatePaths(particles, obstacles, goalPositions, numRobots);
    [globalBestScore, globalBestIndex] = min(personalBestScores);
    globalBestPosition = particles(globalBestIndex, :);

    % Główna pętla PSO
    for iteration = 1:numIterations
        for i = 1:numParticles
            % Oblicz nową prędkość
            r1 = rand(1, numRobots * 2); % losowe wartości do aktualizacji
            r2 = rand(1, numRobots * 2); 

            velocities(i, :) = w * velocities(i, :) + ...
                               c1 * r1 .* (personalBestPositions(i, :) - particles(i, :)) + ...
                               c2 * r2 .* (globalBestPosition - particles(i, :));

            % Zaktualizuj pozycje cząsteczek
            particles(i, :) = particles(i, :) + velocities(i, :);

            % Oblicz ocenę nowej ścieżki
            currentScore = evaluatePaths(particles(i, :), obstacles, goalPositions, numRobots);
            
            % Zaktualizuj najlepsze ścieżki (osobiste i globalne)
            if currentScore < personalBestScores(i)
                personalBestScores(i) = currentScore;
                personalBestPositions(i, :) = particles(i, :);
            end
            
            if currentScore < globalBestScore
                globalBestScore = currentScore;
                globalBestPosition = particles(i, :);
            end
        end
        
        % Wyświetl postęp
        fprintf('Iteration %d: Global best score = %f\n', iteration, globalBestScore);
    end

    % Zwróć najlepsze ścieżki
    bestPaths = globalBestPosition;
end




