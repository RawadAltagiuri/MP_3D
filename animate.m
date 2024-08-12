function animate(sp, path)
    path = pathConversion1(path);
    % Calculate the number of submatrices you will create
    numSubMatrices = size(path, 2) / 3;

    % Preallocate the 3D array to store the submatrices
    formattedPathForAnimation = zeros(sp.j, 3, numSubMatrices);

    % Extract the submatrices and store them in the 3D array
    for i = 1:numSubMatrices
        formattedPathForAnimation(:, :, i) = path(:, (i-1)*3 + 1 : i*3);
    end

    growthCount = 0;
    retractCount = 0;
    steerCount = 0;
    for i=2:size(formattedPathForAnimation,3)
        [growthCount, retractCount, steerCount] = actionCounter(formattedPathForAnimation(:, :, i), formattedPathForAnimation(:, :, i-1), growthCount, retractCount, steerCount);
    end

    softRobot_animation(formattedPathForAnimation, [0,0,0,0,0], true, sp);
end

