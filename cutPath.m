function [path, cost] = cutPath(sp, path)
    minLength = realmax;
    for i = 1:size(path, 2)
        conf = path{i};
        length = sum(conf(:, 3));
        if length < minLength
            minLength = length;
        end
    end

    for i = 1:size(path, 2)
        path{i} = trim(path{i}, minLength);
    end

    path = [pathToRetraction(sp, sp.start_conf, minLength), path, directExpansion(sp, realmax, path{end}, sp.goal_conf)];
    i = 1;
    while i < size(path, 2)
        if path{i} == path{i + 1}
            path(i) = [];
        else
            i = i + 1;
        end

    end

    cost = costOfPath(sp, path);
end

function conf = trim(conf, length)
    setZero = false;
    sumLength = 0;
    for i = 1:size(conf, 1)
        if setZero
            conf(i, :) = 0;
        else
            sumLength = sumLength + conf(i, 3);
            if sumLength > length
                conf(i, 3) = length - (sumLength - conf(i, 3));
                setZero = true;
            end
        end
    end
end

