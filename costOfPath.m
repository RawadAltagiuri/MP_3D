function cost = costOfPath(sp, path)
    cost = 0;
    for i = 1:(size(path, 2) - 1)
        cost = cost + calculateCost(sp, path{i}, path{i + 1});
        % cost = cost + calculateCost_old(path{i}, path{i + 1}, sp.home_base);
    end
end

