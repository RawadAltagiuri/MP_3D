function plotConfigTree(configMap, sp)
    % Add the sp.goal_conf to the map, its parent is itself
    goalNode.path = sp.goal_conf;
    goalNode.h = 0;
    goalNode.g = 0;
    goalNode.f = 0;
    configMap(mat2str(sp.goal_conf)) = goalNode;
    configs = keys(configMap);
    numConfigs = length(configs);

    % Initialize arrays for edges and configurations
    src = {};
    tgt = {};
    allConfigs = {};

    % Iterate through each configuration and extract the edges
    for i = 1:numConfigs
        config = configs{i};
        parentConfig = mat2str(configMap(config).path);

        % Add edge if there is a parent configuration
        if ~isempty(parentConfig)
            src{end+1} = char(parentConfig);
            tgt{end+1} = char(config);
        end

        % Store all configurations
        allConfigs{end+1} = config;
    end

    % Create a directed graph
    G = digraph(src, tgt);

    % Initialize the distance matrix
    distanceMatrix = zeros(numConfigs, numConfigs);

    % Calculate distances between configurations
    for i = 1:numConfigs
        for j = 1:numConfigs
            if i ~= j
                distanceMatrix(i, j) = calculateCost(eval(allConfigs{i}), eval(allConfigs{j}), sp.home_base);
            end
        end
    end

    % Use multidimensional scaling (MDS) to position the nodes
    positions = mdscale(distanceMatrix, 2);

    % Find indices of the start, goal, and final child configurations
    startIdx = find(strcmp(allConfigs, mat2str(sp.start_conf)));
    goalIdx = find(strcmp(allConfigs, mat2str(sp.goal_conf)));
    
    % Check if final_child is present
    if isfield(sp, 'final_child')
        finalChildIdx = find(strcmp(allConfigs, mat2str(sp.final_child)));
    else
        finalChildIdx = [];
    end

    % Check if goalIdx is empty (sp.goal_conf not found)
    if isempty(goalIdx)
        warning('Goal configuration (%s) not found in configMap.', mat2str(sp.goal_conf));
    end
    
    % Initialize node colors (default: 1 for blue)
    nodeColors = ones(numConfigs, 1);

    % Highlight start, goal, and final child nodes
    if ~isempty(startIdx)
        nodeColors(startIdx) = 2; % Start node color: 2 (green)
    else
        warning('Start configuration (%s) not found in configMap.', mat2str(sp.start_conf));
    end
    
    if ~isempty(goalIdx)
        nodeColors(goalIdx) = 3; % Goal node color: 3 (red)
    end
    
    if ~isempty(finalChildIdx)
        nodeColors(finalChildIdx) = 4; % Final child node color: 4 (purple)
    end

    % Plot the graph with edges and custom positions
    figure;
    h = plot(G, 'XData', positions(:,1), 'YData', positions(:,2), 'MarkerSize', 6, 'NodeLabel', {});
    title('Robot Configuration Tree');
    xlabel('Configuration');
    ylabel('Parent Configuration');
    axis equal;

    % Set node colors using NodeCData
    h.NodeCData = nodeColors;
    colormap([0 0 1; 0 1 0; 1 0 0; 0.5 0 0.5]); % Blue, Green, Red, Purple colormap for nodeColors
    colorbar('Ticks', [1, 2, 3, 4], 'TickLabels', {'Default', 'Start', 'Goal', 'Final Child'});
end
