function testWriterRRTs(rrtConf, testCase, numOfTest, fileName, sheetName)
    sp = testCase;

    tempConfig = sp.start_conf;
    tempConfig(1:2) = 0;
    for i = 1:size(tempConfig, 1)
        tempConfig(i, 3) = sp.design(i);
    end

    targetConfig = tempConfig;
    targetConfig(2, 1) = targetConfig(2, 1) + sp.stepSize(1);
    rrtConf.neighbourSize = calculateCost(sp, tempConfig, targetConfig);

    rrtHeaders = {"Problem Name", "Nodes", "P of Goal", "Step Size", "Cost", "Time", "Tree Size"};
    rrtStarHeaders = {"Problem Name", "Nodes", "P of Goal", "Step Size", "Neighbour Size", "Cost", "Time", "Tree Size"};
    
    % RRT
    rrtCosts = [];
    rrtTimes = [];
    rrtTable = {};
    for i = 1:numOfTest
        tic;
        [path, cost, tree] = searchAlgorithmRRT(sp, rrtConf, false);
        time = toc;

        if ~isempty(path)
            rrtCosts = [rrtCosts, cost];
        end

        rrtTimes = [rrtTimes, time];
        rrtTable = [rrtTable; {sp.problemName, rrtConf.numOfNodes, rrtConf.pOfGoal, rrtConf.stepSize, cost, time, size(tree, 1)}];
    end

    % RRT*
    rrtStarCosts = [];
    rrtStarTimes = [];
    rrtStarTable = {};
    for i = 1:numOfTest
        tic;
        [path, cost, tree] = searchAlgorithmRRT_star(sp, rrtConf, false);
        time = toc;

        if ~isempty(path)
            rrtStarCosts = [rrtStarCosts, cost];
        end

        rrtStarTimes = [rrtStarTimes, time];
        rrtStarTable = [rrtStarTable; {sp.problemName, rrtConf.numOfNodes, rrtConf.pOfGoal, rrtConf.stepSize, rrtConf.neighbourSize, cost, time, size(tree, 1)}];
    end

    rrtTitle = cell(1, size(rrtTable, 2));
    rrtTitle(1:end) = {""};
    rrtTitle{1} = "RRT";

    rrtStarTitle = cell(1, size(rrtStarTable, 2));
    rrtStarTitle(1:end) = {""};
    rrtStarTitle{1} = "RRT*";

    % Titles of the algorithms
    titles = [rrtTitle, {"", ""}, rrtStarTitle, {"", ""}];

    % Headers of the algorithms (parameter and results)
    headers = [rrtHeaders, {"", ""}, rrtStarHeaders, {"", ""}];

    filler = cell(numOfTest, 2);
    filler(1:end) = {""};

    statsHeaders = {"AVR_COST", "AVR_TIME", "MIN_COST"};

    rrtStats = {sum(rrtCosts) / length(rrtCosts), sum(rrtTimes) / length(rrtTimes), min(rrtCosts)};
    rrtStarStats = {sum(rrtStarCosts) / length(rrtStarCosts), sum(rrtStarTimes) / length(rrtStarTimes), min(rrtStarCosts)};

    statsFillerRRT = cell(1, size(rrtTable, 2) - size(statsHeaders, 2) + size(filler, 2));
    statsFillerRRT(1:end) = {""};

    statsFillerRRTStar = cell(1, size(rrtStarTable, 2) - size(statsHeaders, 2) + size(filler, 2));
    statsFillerRRTStar(1:end) = {""};
   
    stats = [statsHeaders, statsFillerRRT, ... 
        statsHeaders, statsFillerRRTStar, ...
        ];
    stats = [stats; 
        rrtStats, statsFillerRRT, ...
        rrtStarStats, statsFillerRRTStar, ...
        ];

    rowFiller = cell(1, size(rrtTable, 2) + size(filler, 2) ... 
        + size(rrtStarTable, 2) + size(filler, 2) ...
        );

    completeTable = [
        titles;
        headers;
        [rrtTable, filler, rrtStarTable, filler];
        rowFiller;
        stats
        ];

    writecell(completeTable, fileName, 'Sheet', sheetName, 'Range', 'B2');
end

