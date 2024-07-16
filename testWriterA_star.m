function testWriterA_star(aStarConf, testCase, numOfTest, fileName, sheetName)
    sp = testCase;

    aStarHeader = {"Problem Name", "Fringe Size", "Cost", "Time"};

    aStarCosts = [];
    aStarTimes = [];
    aStarTable = {};
    for i = 1:numOfTest
        tic;
        solution = searchAlgorithm(sp, aStarConf.fringeSize);
        time = toc;

        if ~isempty(path)
            aStarCosts = [aStarCosts, solution.g];
        end

        aStarTimes = [aStarTimes, time];
        aStarTable = [aStarTable; {sp.problemName, aStarConf.fringeSize, solution.g, time}];
    end

    aStarTitle = cell(1, size(aStarTable, 2));
    aStarTitle(1:end) = {""};
    aStarTitle{1} = "A*";

    statsHeaders = {"AVR_COST", "AVR_TIME", "MIN_COST"};

    aStarStats = {sum(aStarCosts) / length(aStarCosts), sum(aStarTimes) / length(aStarTimes), min(aStarCosts)};

    statsFiller = cell(1, size(aStarTable, 2) - size(statsHeaders, 2));
    statsFiller(1:end) = {""};
    stats = [
        statsHeaders, statsFiller;
        aStarStats, statsFiller;
        ];


    rowFiller = cell(1, size(aStarTable, 2));

    completeTable = [
        aStarTitle
        aStarHeader;
        aStarTable;
        rowFiller;
        stats,
        ];

    writecell(completeTable, fileName, 'Sheet', sheetName, 'Range', 'B2');
end

