function neighbourSize = calculateNeighbourSize(tempConfig, targetConfig, sp)
    tempConfig(1:end, 1:end) = 0;
    for i = 1:size(tempConfig, 1)
        tempConfig(i, 3) = sp.design(i);
    end
    targetConfig = tempConfig;
    targetConfig(2, 1) = targetConfig(2, 1) + sp.stepSize(1);
    neighbourSize = calculateCost(tempConfig, targetConfig, sp.home_base);
end

