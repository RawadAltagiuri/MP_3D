function config = sampleInformed(sp, configA, configB)
    dim = sp.j - 1;

    chosenIndices = [];
    for i = 2:sp.j
        chosenIndex = 1;
        if rand > 0.5
            chosenIndex = 2;
        end

        chosenIndices(end + 1) = chosenIndex;
    end

    chosenAnglesA = zeros(length(chosenIndices), 1);
    for i = 1:length(chosenAnglesA)
        chosenAnglesA(i) = configA(i, chosenIndices(i));
    end

    chosenAnglesB = zeros(length(chosenIndices), 1);
    for i = 1:length(chosenAnglesB) 
        
        chosenAnglesB(i) = configA(i, chosenIndices(i));
    end

    diamondPoint = sampleDiamond(chosenAnglesA, chosenAnglesB);
    
    config = configA;
    for i = 1:size(config, 1)

    end

end

