function coordinates = solveForwardKinematics_3D(conf, base)
    % Number of transformations
    num_links = size(conf, 1);
    
    % Initialize coordinates matrix
    coordinates = zeros(num_links+1, 3);

    R = eye(3);
    
    angleX = base(4);
    angleY = base(5);
    growth = 0;

    [new_coordinates, R] = rotateTranslate(base,angleX,angleY,growth,R);

    % Store new position in coordinates matrix
    coordinates(1, :) = new_coordinates;

    for i = 1:num_links
        angleX = conf(i,1);
        angleY = conf(i,2);
        growth = conf(i,3);
        [new_coordinates,R] = rotateTranslate(coordinates(i,:),angleX,angleY,growth,R);
        % Store new position in coordinates matrix
        coordinates(i+1, :) = new_coordinates;
        
    end
end

function [new_coordinates, new_R] = rotateTranslate(coordinates,angleX,angleY,growth,R)
    x = coordinates(1);
    y = coordinates(2);
    z = coordinates(3);
    Rx = [1, 0, 0; 0, cosd(angleX), -sind(angleX); 0, sind(angleX), cosd(angleX)];
    Ry = [cosd(angleY), 0, sind(angleY); 0, 1, 0; -sind(angleY), 0, cosd(angleY)];
    new_R = R*Rx*Ry;
    tcp_coordinates = (new_R * [0 0 growth]')';
    new_coordinates = tcp_coordinates + [x, y, z];
end
