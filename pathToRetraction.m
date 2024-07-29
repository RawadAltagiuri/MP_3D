function [path, cost] = pathToRetraction(sp, conf, finalLength)
    path = {conf};
    length = sum(conf(:, 3));
    if finalLength >= length
        cost = 0;
        return;
    end

    retRemainder = length - finalLength;
    while retRemainder > sp.stepSize(2)
        conf = retract(conf, sp.stepSize(2));
        path = [path, conf];
        retRemainder = retRemainder - sp.stepSize(2);
    end
    if retRemainder > 0
        conf = retract(conf, retRemainder);
        path = [path, conf];
    end

    cost = (length - finalLength) * sp.costArray(3);
end
