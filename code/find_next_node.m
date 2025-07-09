function NextNode = find_next_node(Rows, Columns, BackPointer, Target, CLOSED)
% find_next_node - function which have to calculate which neighbour is the best to
% move. It contains the position of robot and value of three heuristic
% functions in this way:
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Position_X | Position_Y | h(n) | g(n) | f(n) %
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

NextNode = [];
counter = 1;

for i = -1:1:1
    for j = -1:1:1
        if (i ~= j || i ~= 0)
        XY = BackPointer + [i,j];
        if (XY(:,1) > 0 && XY(:,1) <= Rows) && (XY(:,2) > 0 && XY(:,2) <= Columns)
            flag = 1;
            for k = 1:size(CLOSED,1)
                if XY(:,1) == CLOSED(k,1) && XY(:,2) == CLOSED(k,2)
                    flag = 0;
                end
            end
            if flag == 1
                fn = heur_Astar(BackPointer,Target,XY);
                NextNode(counter,1) = XY(:,1);
                NextNode(counter,2) = XY(:,2);
                NextNode(counter,3) = distance_a(BackPointer,XY)+fn(3,:);
                NextNode(counter,4) = distance_a(XY,Target);
                NextNode(counter,5) = fn(1,:);
                counter = counter + 1;
            end
        end
        end
    end
end