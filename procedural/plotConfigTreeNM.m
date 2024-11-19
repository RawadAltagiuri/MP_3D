function plotConfigTreeNM(graphTree, sp)
    numConfigs = length(graphTree);
    src = {};
    tgt = {};
    allConfigs = cell(numConfigs, 1);
    distanceMatrix = zeros(numConfigs, numConfigs);

    for i = 1:numConfigs
        config = graphTree{i, 1};
        parentIndex = graphTree{i, 2};
        allConfigs{i} = mat2str(config);

        if parentIndex ~= 0
            parentConfig = mat2str(graphTree{parentIndex, 1});
            src{end+1} = parentConfig;
            tgt{end+1} = mat2str(config);
        end
    end

    for i = 1:numConfigs
        for j = 1:numConfigs
            if i ~= j
                distanceMatrix(i, j) = calculateCost(sp, eval(allConfigs{i}), eval(allConfigs{j}));
            end
        end
    end

    positions = mdscale(distanceMatrix, 2);

    startIdx = find(strcmp(allConfigs, mat2str(sp.start_conf)));
    goalIdx = find(strcmp(allConfigs, mat2str(sp.goal_conf)));
    finalChildIdx = []; 

    if isfield(sp, 'final_child')
        finalChildIdx = find(strcmp(allConfigs, mat2str(sp.final_child)));
    end

    nodeColors = ones(numConfigs, 1);

    if ~isempty(startIdx)
        nodeColors(startIdx) = 2; 
    else
        warning('Start configuration (%s) not found in graphTree.', mat2str(sp.start_conf));
    end

    if ~isempty(goalIdx)
        nodeColors(goalIdx) = 3;
    end

    if ~isempty(finalChildIdx)
        nodeColors(finalChildIdx) = 4;
    end


    G = digraph(src, tgt);
    figure;
    h = plot(G, 'XData', positions(:,1), 'YData', positions(:,2), 'MarkerSize', 6, 'NodeLabel', {});
    title('Robot Configuration Tree');
    xlabel('Configuration');
    ylabel('Parent Configuration');
    axis equal;
    h.NodeCData = nodeColors;
    colormap([0 0 1; 0 1 0; 1 0 0; 0.5 0 0.5]);
end