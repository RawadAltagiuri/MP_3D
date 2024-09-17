function cellPath = pathConversionM_C(path)
    cellPath = {};
    for i = 1:3:size(path, 2) - 2
        cellPath{end + 1} = path(:, i:i+2);
    end
end

