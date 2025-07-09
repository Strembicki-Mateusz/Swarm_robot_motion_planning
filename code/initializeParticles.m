% Funkcja do inicjalizacji cząsteczek (pozycje robotów)
function particles = initializeParticles(numParticles, numRobots, numCols, numRows, startPositions, goalPositions)
    particles = zeros(numParticles, numRobots * 2); % (numParticles, numRobots * 2)
    for i = 1:numParticles
        for j = 1:numRobots
            % Inicjalizuj losowe pozycje robotów między punktem startowym a końcowym
            particles(i, 2*j-1) = startPositions(j, 1) + rand * (goalPositions(j, 1) - startPositions(j, 1));
            particles(i, 2*j) = startPositions(j, 2) + rand * (goalPositions(j, 2) - startPositions(j, 2));
        end
    end
end