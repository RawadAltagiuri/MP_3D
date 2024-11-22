function plotConfigTree(tree, sp, final_child)
    tree = [tree; {sp.goal_conf, -1, -1, sp.goal_conf}];

    for i=1:size(tree, 1)
        configs{i}= mat2str(tree{i, 1});
    end
    numConfigs = length(configs);

    % initialize arrays for edges and configurations
    src = {};
    tgt = {};
    allConfigs = {};

    % iterate through each configuration and extract the edges
    for i = 1:numConfigs
        config = configs{i};
        parent_idx = tree{i, 2};
        if parent_idx == 0
            parent_idx = 1;
        end
        if parent_idx == -1
            parent_idx = numConfigs;
        end
        parentConfig = mat2str(tree{parent_idx, 1});

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
plot3([positions(end-1, 1), positions(end, 1)], [positions(end-1, 2), positions(end, 2)], [positions(end-1, 3), positions(end, 3)])

% plot nodes
scatter3(positions(:, 1), positions(:, 2), positions(:, 3), 36, 'b', 'filled');

% highlight goal, start and final child nodes
h1 =scatter3(positions(end, 1), positions(end, 2), positions(end, 3), 100, 'r', 'filled');
h2 = scatter3(positions(1, 1), positions(1, 2), positions(1, 3), 100, 'g', 'filled');
if ~isempty(final_child)
    h3 = scatter3(positions(end-1, 1), positions(end-1, 2), positions(end-1, 3), 100, 'm', 'filled');
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
if ~isempty(final_child)
    lgd = legend([h1, h2, h3], {'Goal Node', 'Start Node', 'Final Child Node'});
else
    lgd = legend([h1, h2], {'Goal Node', 'Start Node'});
end
lgd.FontSize = 8;
lgdPos = lgd.Position;
lgd.Position = [lgdPos(1) + 0, lgdPos(2), lgdPos(3), lgdPos(4)];

hold off;
end