% Write the experiment results for a specific test environment.
% Runs first comb_num combinations for run_num times
function tabulation(sp, run_num, comb_num)
    % Get the combinations
    load combinations_RRT.mat
    load combinations_RRT_star.mat
    load combinations_Astar.mat

    % Setup the variable names and types for the table 
    sz = [3 run_num+1];
    varTypes = "string";
    for i=1:run_num
        varTypes = [varTypes, "double"];
    end
    varNames = "variables";
    varNames = [varNames, compose("run-%d", 1:run_num)];
    
    % Run for RRT
    idx = 3;
    for i=1:comb_num
        T = table('Size', sz, 'VariableTypes', varTypes, 'VariableNames',varNames);
        T(:, 1) = {"cost"; "runtime"; "tree size"};
        for j=2:run_num+1
            tic
            [~, cost, tree, ~] = searchAlgorithmRRT(sp, combinations_RRT{1, i}, false);
            time = toc;
            T(:, j) = {cost; time; size(tree, 1)};
        end
        filename = [sp.problemName, '.xlsx'];
        excel_cell_idx = ['F', num2str(idx)];
        writetable(T, filename, 'Sheet', 'RRT', 'Range', excel_cell_idx);

        T2 = struct2table(combinations_RRT{1, i});
        excel_cell_idx = ['B', num2str(idx)];
        writetable(T2, filename, 'Sheet', 'RRT', 'Range', excel_cell_idx)
        idx = idx+6;
        % T = table();
    end
    
    
    % Run for RRT*
    idx = 3;
    for i=1:comb_num
        T = table('Size', sz, 'VariableTypes', varTypes, 'VariableNames',varNames);
        T(:, 1) = {"cost"; "runtime"; "tree size"};
        combinations_RRT_star{1, i}.neighbourSize = calculateNeighbourSize(sp.start_conf, sp.goal_conf, sp);
        for j=2:run_num+1
            tic
            [~, cost, tree, ~] = searchAlgorithmRRT_star(sp, combinations_RRT_star{1, i}, false);
            time = toc;
            T(:, j) = {cost; time; size(tree, 1)};
        end
        filename = [sp.problemName, '.xlsx'];
        excel_cell_idx = ['G', num2str(idx)];
        writetable(T, filename, 'Sheet', 'RRT_star', 'Range', excel_cell_idx);

        T2 = struct2table(combinations_RRT_star{1, i});
        excel_cell_idx = ['B', num2str(idx)];
        writetable(T2, filename, 'Sheet', 'RRT_star', 'Range', excel_cell_idx);
        idx = idx+6;
        % T = table();
    end


    % Run for A*_det
    idx = 3;
    for i=1:comb_num
        T = table('Size', sz, 'VariableTypes', varTypes, 'VariableNames',varNames);
        T(:, 1) = {"cost"; "runtime"; "expanded nodes"};
        for j=2:run_num+1
            tic
            [solution, expandedNodes] = searchAlgorithm_Det(sp, combinations_Astar{1, i}.fringeSize);
            time = toc;
            T(:, j) = {solution.g; time; expandedNodes};
        end
        filename = [sp.problemName, '.xlsx'];
        excel_cell_idx = ['D', num2str(idx)];
        writetable(T, filename, 'Sheet', 'A_star_Det', 'Range', excel_cell_idx);

        T2 = struct2table(combinations_Astar{1, i});
        excel_cell_idx = ['B', num2str(idx)];
        writetable(T2, filename, 'Sheet', 'A_star_Det', 'Range', excel_cell_idx);
        idx = idx+6;
        % T = table();
    end

    % Run for A*_sto
    idx = 3;
    for i=1:comb_num
        T = table('Size', sz, 'VariableTypes', varTypes, 'VariableNames',varNames);
        T(:, 1) = {"cost"; "runtime"; "expanded nodes"};
        for j=2:run_num+1
            tic
            [solution, expandedNodes] = searchAlgorithm_Sto(sp, combinations_Astar{1, i}.fringeSize);
            time = toc;
            T(:, j) = {solution.g; time; expandedNodes};
        end
        filename = [sp.problemName, '.xlsx'];
        excel_cell_idx = ['D', num2str(idx)];
        writetable(T, filename, 'Sheet', 'A_star_Sto', 'Range', excel_cell_idx);

        T2 = struct2table(combinations_Astar{1, i});
        excel_cell_idx = ['B', num2str(idx)];
        writetable(T2, filename, 'Sheet', 'A_star_Sto', 'Range', excel_cell_idx);
        idx = idx+6;
        % T = table();
    end
end