function plotConfigTree(configMap, sp)
    % add the sp.goal_conf to the map, its parent is itself
    goalNode.path = sp.goal_conf;
    goalNode.h = 0;
    goalNode.g = 0;
    goalNode.f = 0;
    configMap(mat2str(sp.goal_conf)) = goalNode;
    configs = keys(configMap);
    numConfigs = length(configs);

    goalConfig = sp.goal_conf;
    startConfig = sp.start_conf;
    if isfield(sp, 'final_child')
        final_child = sp.final_child;
    else
        final_child = [];
    end

    % initialize arrays for edges and configurations
    src = {};
    tgt = {};
    allConfigs = {};

    % iterate through each configuration and extract the edges
    for i = 1:numConfigs
        config = configs{i};
        parentConfig = mat2str(configMap(config).path);

        if ~isempty(parentConfig)
            src{end+1} = char(parentConfig);
            tgt{end+1} = char(config);
        end

        allConfigs{end+1} = config;
    end

    % calculate positions for 3D plotting
    positions = zeros(numConfigs, 3);
    for i = 1:numConfigs
        config = allConfigs{i};
        matrix = eval(config);
        matrix = solveForwardKinematics_3D(matrix, sp.home_base, false);
        % Sum of absolute values of every column
        x_sum = sum(matrix(:, 1)); 
        y_sum = sum(matrix(:, 2)); 
        z_sum = sum(matrix(:, 3)); 
        positions(i, :) = [x_sum, y_sum, z_sum];
    end

 % PLOTTING STARTS HERE
figure;
hold on;
% plot the edges
for i = 1:length(src)
    srcPos = positions(strcmp(allConfigs, src{i}), :);
    tgtPos = positions(strcmp(allConfigs, tgt{i}), :);
    plot3([srcPos(1), tgtPos(1)], [srcPos(2), tgtPos(2)], [srcPos(3), tgtPos(3)], 'k');
end
% plot nodes
scatter3(positions(:, 1), positions(:, 2), positions(:, 3), 36, 'b', 'filled');

% highlight goal, start and final child nodes
goalConfigPos = positions(strcmp(allConfigs, mat2str(goalConfig)), :);
startConfigPos = positions(strcmp(allConfigs, mat2str(startConfig)), :);
if isfield(sp, 'final_child')
    finalChildPos = positions(strcmp(allConfigs, mat2str(final_child.path(:, end-2:end))), :);
end
h1 =scatter3(goalConfigPos(1), goalConfigPos(2), goalConfigPos(3), 100, 'r', 'filled');
h2 = scatter3(startConfigPos(1), startConfigPos(2), startConfigPos(3), 100, 'g', 'filled');
if isfield(sp, 'final_child')
    h3 = scatter3(finalChildPos(1), finalChildPos(2), finalChildPos(3), 100, 'm', 'filled');
end

xlabel('Sum of 1st Column');
ylabel('Sum of 2nd Column');
zlabel('Sum of 3rd Column');

% automatically adjust axes limits
axis auto;
% manually set axes limits
%xlim([-50 1100]);
%ylim([-50 1100]);
%zlim([-2000 2000]);

% legend
if isfield(sp, 'final_child')
    lgd = legend([h1, h2, h3], {'Goal Node', 'Start Node', 'Final Child Node'});
else
    lgd = legend([h1, h2], {'Goal Node', 'Start Node'});
end
lgd.FontSize = 8;
lgdPos = lgd.Position;
lgd.Position = [lgdPos(1) + 0, lgdPos(2), lgdPos(3), lgdPos(4)];


hold off;
end