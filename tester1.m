for i = 1:size(path, 2)
    for j = i + 2:size(path, 2)
        for u = i:j
            curCost = calculateCost(sp, path{u}, path{j});
            impCost = calculateCost(sp, path{i}, path{j});
            if impCost < curCost
                dirPath = directExpansion(sp, realmax, path{i}, path{j});
                if ~isempty(dirPath)
                    DEBUG = 0;
                end
            end
        end
    end
end
