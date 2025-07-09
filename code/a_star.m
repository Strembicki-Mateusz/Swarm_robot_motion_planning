function [Optimal_Path] = a_star(Rows, Columns, Obstacles, Target, Start)
% a_star - heart of algorith. It compare values to create optimal way
% with minimal cost which the robot goes to the target.


OPEN = [];
CLOSED = [];
endOPEN = 1;
endCLOSED = 1;
BackPointer = Start;


%%
% Adding Obstacles to the Closed List

for i = 1:size(Obstacles,1)
    CLOSED(endCLOSED,:) = Obstacles(i,:);
    endCLOSED = endCLOSED + 1;
end

%%
% Adding Beginning position to the open list

fn = heur_Astar(Start, Target, BackPointer);
OPEN(endOPEN,:) = insert_open(Start,BackPointer,fn);

PATH = [];


while(BackPointer(:,1) ~= Target(:,1) || BackPointer(:,2) ~= Target(:,2))
    next_node = find_next_node(Rows,Columns,BackPointer,Target,CLOSED);
    for i = 1:size(next_node,1)
        push_it_to_OPEN = 0;
        for j = 1:size(OPEN,1)
            if (OPEN(j,2) == next_node(i,1) && OPEN(j,3) == next_node(i,2))
                OPEN(j,8) = min(next_node(i,5) , OPEN(j,8));
                if OPEN(j,8) == next_node(i,5)
                    OPEN(j,4) = BackPointer(:,1);
                    OPEN(j,5) = BackPointer(:,2);
                    OPEN(j,6) = next_node(i,3);
                    OPEN(j,7) = next_node(i,4);
                end
                push_it_to_OPEN = 1;
            end
        end
        if push_it_to_OPEN == 0
            endOPEN = endOPEN + 1;
            OPEN(endOPEN,:) = insert_open([next_node(i,1) , next_node(i,2)] , BackPointer , [next_node(i,3) ; next_node(i,4) ; next_node(i,5)]);
        end
    end



    index_min_node = min_index_fn(OPEN, endOPEN, Target);
    if index_min_node ~= -100
        BackPointer(:,1) = OPEN(index_min_node,2);
        BackPointer(:,2) = OPEN(index_min_node,3);
        %path_cost = OPEN(index_min_node,6); % update h(n) = value

        endCLOSED = endCLOSED+1;
        CLOSED(endCLOSED,1) = BackPointer(:,1);
        CLOSED(endCLOSED,2) = BackPointer(:,2);
        OPEN(index_min_node,1) = 0;
    end


end

Optimal_Path_Inverse = [];
i = 1;
Optimal_Path_Inverse(i,1) = CLOSED(endCLOSED,1);
Optimal_Path_Inverse(i,2) = CLOSED(endCLOSED,2);
i = i + 1;

if( CLOSED(endCLOSED,1) == Target(:,1) && CLOSED(endCLOSED,2) == Target(:,2))
    NODE = [OPEN(node_idx(OPEN, [CLOSED(endCLOSED,1), CLOSED(endCLOSED,2)]),4) , OPEN(node_idx(OPEN, [CLOSED(endCLOSED,1),CLOSED(endCLOSED,2)]),5)];
    Optimal_Path_Inverse(i,:) = NODE;
    i = i + 1;
    while ((NODE(1) ~= Start(1)) || (NODE(2) ~= Start(2)))
        idx_node = node_idx(OPEN, NODE);
        NODE = [ OPEN(idx_node,4) ; OPEN(idx_node,5)];
        Optimal_Path_Inverse(i,:) = NODE;
        i = i + 1;
    end
end


Optimal_Path = [];
j = size(Optimal_Path_Inverse,1);
for i = 1:size(Optimal_Path_Inverse,1)
    Optimal_Path(i,:) = Optimal_Path_Inverse(j-i+1,:);
end
