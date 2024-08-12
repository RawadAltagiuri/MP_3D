function firstWrong = checkPath(sp, path)
    firstWrong = 0;
    for i = 1:size(path, 2) - 1
        conf1 = path{i};
        conf2 = path{i + 1};
        diffConfig = abs(conf1 - conf2);
        % Check length.
        for j = 1:size(conf1, 1)
            if lesser(sp.stepSize(2), diffConfig(j, 3), 0.0001) || ...
                (conf2(j, 3) ~= 0 && ...
                    (lesser(sp.stepSize(1), diffConfig(j, 1), 0.0001) || lesser(sp.stepSize(1), diffConfig(j, 2), 0.0001)))
                firstWrong = i;
                return;
            end
        end
    end
end

