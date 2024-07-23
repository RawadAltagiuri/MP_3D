function drawTree(graphTree, sp)
    for i = 2:size(graphTree, 1)
        drawConfig(graphTree{i, 1}, sp, 'b');
        drawConfig(graphTree{i, 4}, sp, 'k');
        pause;
    end
end