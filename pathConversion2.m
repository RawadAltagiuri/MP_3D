function cellPath = pathConversion2(path)
    cellPath = {};
    for i = 1:3:size(path, 2) - 2
        cellPath{end + 1} = path(:, i:i+2);
    end
end

