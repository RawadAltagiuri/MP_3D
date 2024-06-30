function commands3D = mapKeysTo3DArray(map)
    % Extract all keys from the map
    keys = map.keys;
    
    % Assuming the first key gives us a good example of the size and format
    % Convert the first key back to a numerical array to determine size
    sampleConfig = str2num(keys{1}); % This assumes keys are stored in a format that str2num can parse
    [n, m] = size(sampleConfig);
    
    % Initialize the 3D array to hold all configurations
    commands3D = zeros(n, m, length(keys));
    
    % Fill the 3D array with configurations converted from keys
    for i = 1:length(keys)
        configMatrix = str2num(keys{i}); % Convert each key back to a numerical matrix
        commands3D(:, :, i) = configMatrix;
    end
end