% Test script for isAncestor function

function TestingIsAncestor()
    % Define a simple tree structure
    nodes_map = containers.Map();
    sp.start_conf = [0, 0, 0];

    % Define nodes
    node1.path = [0, 0, 0];
    node2.path = [1, 0, 0];
    node3.path = [2, 0, 0];
    node4.path = [3, 0, 0];

    % Define parent-child relationships
    nodes_map(mat2str(node2.path)) = node1;
    nodes_map(mat2str(node3.path)) = node2;
    nodes_map(mat2str(node4.path)) = node3;

    % Test cases
    assert(isAncestor(node1, node4, nodes_map, sp) == true, 'Test Case 1 Failed');
    assert(isAncestor(node2, node4, nodes_map, sp) == true, 'Test Case 2 Failed');
    assert(isAncestor(node3, node4, nodes_map, sp) == true, 'Test Case 3 Failed');
    assert(isAncestor(node4, node1, nodes_map, sp) == false, 'Test Case 4 Failed');
    assert(isAncestor(node1, node1, nodes_map, sp) == false, 'Test Case 5 Failed');

    disp('All test cases passed.');
end


function is_ancestor = isAncestor(possible_ancestor, child_node, nodes_map, sp)
    %we iterate from the child node to the root node, if we find the possible ancestor, then it is an ancestor
    path = child_node.path;
    counter = 0;
    while ~isequal(path, sp.start_conf)
        parent = nodes_map(mat2str(path(:, end-2:end)));
        path = parent.path;
        counter = counter + 1;
        if isequal(path, possible_ancestor.path)
            is_ancestor = true;
            return;
        end
    end
    is_ancestor = false;
    
end