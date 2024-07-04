function testWriter(testCases, fileName)
    rrtConf.numOfNodes = 1000;
    rrtConf.pOfGoal = 0.1;
    rrtConf.stepSize = 3;
    rrtConf.neighbourSize = 3;

    aStar.fringeSize = 1000;

    rrtHeaders = {"Nodes", "P of Goal", "Step Size", "Cost", "Time"};
    rrtStarHeaders = {"Nodes", "P of Goal", "Step Size", "Neighbour Size", "Cost", "Time"};
    aStarHeaders = {"Fringe Size", "Cost", "Time"};
    
    % RRT
    rrtCosts = 0;
    rrtTimes = 0;

    rrtTable = {};
    for i = 1:length(testCases)
        sp = testCases{i};
        tic;
        [path, cost] = searchAlgorithmRRT(sp, rrtConf, false);
        time = toc;

        rrtCosts = rrtCosts + cost;
        rrtTimes = rrtTimes + time;

        rrtTable = [rrtTable; {rrtConf.numOfNodes, rrtConf.pOfGoal, rrtConf.stepSize, cost, time}];
    end

    % RRT*
    rrtStarCosts = 0;
    rrtStarTimes = 0;
    rrtStarTable = {};
    for i = 1:length(testCases)
        sp = testCases{i};
        tic;
        [path, cost] = searchAlgorithmRRT_star(sp, rrtConf);
        time = toc;

        rrtStarCosts = rrtCosts + cost;
        rrtStarTimes = rrtTimes + time;

        rrtStarTable = [rrtStarTable; {rrtConf.numOfNodes, rrtConf.pOfGoal, rrtConf.stepSize, rrtConf.neighbourSize, cost, time}];
    end

    % A*
    aStarCosts = 0;
    aStarTimes = 0;

    aStarTable = {};
    for i = 1:length(testCases)
        sp = testCases{i};
        tic;
        solution = searchAlgorithm(sp, rrtConf.numOfNodes);
        time = toc;

        aStarCosts = rrtCosts + cost;
        aStarTimes = rrtTimes + time;

        aStarTable = [aStarTable; {aStar.fringeSize, solution.g, time}];
    end
    rrtTitle = cell(1, size(rrtTable, 2));
    rrtTitle(1:end) = {""};
    rrtTitle{1} = "RRT";

    rrtStarTitle = cell(1, size(rrtStarTable, 2));
    rrtStarTitle(1:end) = {""};
    rrtStarTitle{1} = "RRT*";

    aStarTitle = cell(1, size(aStarTable, 2));
    aStarTitle(1:end) = {""};
    aStarTitle{1} = "A*";

    % Titles of the algorithms
    titles = [rrtTitle, {"", ""}, rrtStarTitle, {"", ""}, aStarTitle];

    % Headers of the algorithms (parameter and results)
    headers = [rrtHeaders, {"", ""}, rrtStarHeaders, {"", ""}, aStarHeaders];

    filler = cell(length(testCases), 2);
    filler(1:end) = {""};

    statsHeaders = {"AVR_COST", "AVR_TIME"};

    rrtStats = {rrtCosts / length(testCases), rrtTimes / length(testCases)};
    rrtStarStats = {rrtStarCosts / length(testCases), rrtStarTimes / length(testCases)};
    aStarStats = {aStarCosts / length(testCases), aStarTimes / length(testCases)};

    statsFillerRRT = cell(1, size(rrtTable, 2) - size(statsHeaders, 2) + size(filler, 2));
    statsFillerRRT(1:end) = {""};

    statsFillerRRTStar = cell(1, size(rrtStarTable, 2) - size(statsHeaders, 2) + size(filler, 2));
    statsFillerRRTStar(1:end) = {""};

    statsFillerAStar = cell(1, size(aStarTable, 2) - size(statsHeaders, 2));
    statsFillerAStar(1:end) = {""};
   
    stats = [statsHeaders, statsFillerRRT, ... 
        statsHeaders, statsFillerRRTStar, ...
        statsHeaders, statsFillerAStar
        ];
    stats = [stats; 
        rrtStats, statsFillerRRT, ...
        rrtStarStats, statsFillerRRTStar, ...
        aStarStats, statsFillerAStar
        ];

    rowFiller = cell(1, size(rrtTable, 2) + size(filler, 2) ... 
        + size(rrtStarTable, 2) + size(filler, 2) ...
        + size(aStarTable, 2));

    completeTable = [
        titles;
        headers;
        [rrtTable, filler, rrtStarTable, filler, aStarTable];
        rowFiller
        stats];

    writecell(completeTable, fileName, 'Range', 'B2');
end

