function min_fn = min_index_fn(OPEN, OPENcounter, Target)
% min_index_fn - function which find out the smallest index of heuristic in
% OPEN list

tmp = [];
target_flag = 0;
target_idx = 0;
k = 1;

for i = 1:OPENcounter
    if OPEN(i,1) == 1
        tmp(k,:) = [OPEN(i,:), i];
        if ((OPEN(i,4) == Target(:,1)) && (OPEN(i,5) == Target(:,2)))
            target_flag = 1;
            target_idx = i;
        end
        k = k+1;
    end
end

if target_flag == 1
    min_fn = target_idx;
end

if size(tmp) ~= 0
    [x, minimum] = min(tmp(:,8));
    min_fn = tmp(minimum, 9);
else
    min_fn = -100;
end




end