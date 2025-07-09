function A_PATHS = add_step_to_avoid(A_PATHS, k, j, maxLength)
    % Pobierz aktualną i poprzednią pozycję robota k
    x = A_PATHS(j, 2*k);
    y = A_PATHS(j, 2*k-1);
    if j > 1
        prev_x = A_PATHS(j-1, 2*k);
        prev_y = A_PATHS(j-1, 2*k-1);
    else
        prev_x = x;
        prev_y = y;
    end
    
    % Sprawdzenie kierunku ostatniego ruchu
    if abs(x - prev_x) > 0 % Ruch w poziomie, więc dodaj krok w pionie
        moves = [0 1; 0 -1]; % Góra, dół
    else % Ruch w pionie, więc dodaj krok w poziomie
        moves = [1 0; -1 0]; % Prawo, lewo
    end
    
    % Rozszerzenie A_PATHS jeśli trzeba
    if j + 1 > size(A_PATHS, 1)
        A_PATHS(j+1, :) = A_PATHS(j, :);
    end
    
    % Sprawdzenie, które pozycje są wolne
    for m = 1:size(moves, 1)
        new_x = x + moves(m, 1);
        new_y = y + moves(m, 2);
        
        % Sprawdzenie, czy nowa pozycja jest zajęta
        is_free = true;
        for r = 1:size(A_PATHS, 2)/2  % Iteracja po wszystkich robotach
            if r ~= k && j <= maxLength && isequal([new_x, new_y], [A_PATHS(j, 2*r), A_PATHS(j, 2*r-1)])
                is_free = false;
                break;
            end
        end
        
        % Jeśli znaleziono wolne miejsce, przypisz nowe współrzędne
        if is_free
            A_PATHS(j+1, :) = A_PATHS(j, :); % Skopiowanie poprzednich pozycji
            A_PATHS(j+1, 2*k) = new_x;
            A_PATHS(j+1, 2*k-1) = new_y;
            return;
        end
    end
    
    % Jeśli brak wolnego miejsca, robot pozostaje na swoim miejscu
    A_PATHS(j+1, :) = A_PATHS(j, :);
end