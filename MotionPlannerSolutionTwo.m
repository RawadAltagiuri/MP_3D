% Define the cases to test
cases = {'wall', 'wallWithEntrance', 'hole'};
numSuccessfulRuns = 1;

% Initialize results storage
results = struct('caseName', {}, 'solutionG', {}, 'expandedNodes', {}, 'time', {});

% Loop through each case
for caseIdx = 1:length(cases)
    caseName = cases{caseIdx};
    successfulRuns = 0;
    
    while successfulRuns < numSuccessfulRuns
        % Set up the scenario parameters
        switch caseName
            case 'wall'
                start.design = [50; 150; 175; 150; 200];
                start.matrix = [0 0 50; 0 0 150; 0 0 175; -30 0 150; -40 0 200];
                sp.steerBounds = [-40 40];
                sp.lengthMin = 5;
                sp.plane_z = 1000;
                sp.costArray = [1, 1, 1];
                sp.stepSize = [2.5, 30];
                sp.obstacles = [
                    -50 0 sp.plane_z 25 650;
                    -50 50 sp.plane_z 25 650;
                    -50 100 sp.plane_z 25 650;
                    -50 150 sp.plane_z 25 650;
                    -50 200 sp.plane_z 25 650;
                    -50 250 sp.plane_z 25 650;
                    -50 300 sp.plane_z 25 650;
                ];
                sp.goals = [
                    [0 0 50; 0 -25 150; 0 -20 150; 0 20 150; 0 0 150]
                ];
            case 'wallWithEntrance'
                start.design = [50; 150; 150; 150; 150];
                start.matrix = [0 0 50; 10 0 150; 10 0 150; 0 0 150; 0 0 150];
                sp.steerBounds = [-30 30];
                sp.lengthMin = 5;
                sp.plane_z = 1000;
                sp.costArray = [1, 1, 1];
                sp.stepSize = [2.5, 30];
                sp.obstacles = [
                    -50 -300 sp.plane_z 25 650;
                    -50 -250 sp.plane_z 25 650;
                    -50 -200 sp.plane_z 25 650;
                    -50 -150 sp.plane_z 25 650;
                    -50 0 sp.plane_z 25 650;
                    -50 50 sp.plane_z 25 650;
                    -50 100 sp.plane_z 25 650;
                    -50 150 sp.plane_z 25 650;
                    -50 200 sp.plane_z 25 650;
                    -50 250 sp.plane_z 25 650;
                    -50 300 sp.plane_z 25 650;
                ];
                sp.goals = [
                    [0 0 50; 0 -25 150; 0 -20 150; 0 20 150; 0 0 150]
                ];
            case 'hole'
                start.design = [50; 150; 175; 150; 200];
                start.matrix = [0 0 50; 0 0 150; 0 0 175; 0 0 150; 0 0 150];
                sp.steerBounds = [-40 30];
                sp.lengthMin = 5;
                sp.plane_z = 1000;
                sp.costArray = [1, 1, 1];
                sp.stepSize = [2.5, 30];
                sp.obstacles = [
                    0 100 450 25 100;
                    -50 100 450 25 100;
                    50 100 450 25 100;
                    50 100 sp.plane_z 25 430;
                    -50 100 sp.plane_z 25 430;
                    0 100 sp.plane_z 25 430;
                    -125 100 sp.plane_z 50 650;
                    125 100 sp.plane_z 50 650;
                ];
                sp.goals = [
                    [0 0 50; 0 -25 150; 0 -20 150; 0 20 150; 0 0 150]
                ];
        end
        
        sp.design = start.design;
        sp.baseRotate = false;
        sp.start_conf = start.matrix;
        sp.j = size(sp.start_conf, 1);
        sp.goal_conf = sp.goals(1:sp.j, 1:3);
        sp.home_base = [0, 0, 0];
        sp.weight = 0.1;
        sp.iterations = 300;
        sp.AcceptedEuclideanDistance = 100;
        
        % Run the search algorithm
        tic
        [solution, expandedNodes] = searchAlgorithm(sp);
        time = toc;
        
        % Check if the solution is valid
        if isempty(solution)
            continue;
        end
        
        % Store the results
        successfulRuns = successfulRuns + 1;
        results(end+1).caseName = caseName;
        results(end).solutionG = solution.g;
        results(end).expandedNodes = expandedNodes;
        results(end).time = time;
    end
end

% Convert the results to a table
resultsTable = struct2table(results);

% Write the results to an Excel file
writetable(resultsTable, 'octoberRRTStarResults.xlsx');