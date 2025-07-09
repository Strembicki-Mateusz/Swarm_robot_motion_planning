function n_idx = node_idx(OPEN, NODE)
% node_idx - function which give number of node indeks

    i = 1;
    while( OPEN(i,2) ~= NODE(1) || OPEN(i,3) ~= NODE(2) )
        i = i + 1;
    end
    n_idx = i;
end