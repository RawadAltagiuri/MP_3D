classdef Configuration < handle
    % This class represents a configuration of a soft-robot.
    % As a configuration has different representations (in joint space or
    % cartasian), instance of this class has respective properties for
    % each representation.
    % Each representation is instantiated the first time it is used
    % (or computed) rather than in construction of the object itself.

    % Properties:
    % j_space, configuration in joint space.
    % c_space, configuration in cartasian space.
    properties (Access = private)
        j_space double
        c_space double
    end

    methods (Static)
        % Generates a random configuration using a specified method.
        function conf = randomConfig(randomMethod)

        end
    end
    
    methods
        % Constructor
        % Takes configuration in joint space.
        function obj = Configuration(j_space)
            obj.j_space = j_space;
        end

        % Solves forward kinematics (converts joint coordinates into
        % cartasian coordinates of each joint).
        function solveFK(obj, sp)

        end

        % Checks whether this configuration collides with given obstacles.
        %
        % Inputs:
        % obstacles, cylinders in [x, y, z, r, h] form.
        % 
        % Outputs:
        % isCollide, boolean flag indicating a collusion or not.
        function isCollide = collusionCheck(obj, obstacles)
            if isempty(obj.c_space)
                solveFK(sp);
            end
            iscollide();
        end

        % Retrieves the path to go to a target configuration with a limit.
        % This method basically runs the local planner to retrieve the
        % path. In case of a collusion, obstacle's relative index is
        % returned.
        % 
        % Inputs:
        % stepSize, number of steps to iterate to go to target configuration.
        % targetConfig, target configuration.
        % obstacles, obstacles in the way.
        %
        % Outputs:
        % p, path.
        % obstacle, the collided (in case p is empty) obstacles.
        %
        % Note: In case of a valid path, p is non-empty and obstacle is
        % empty. On the other hand if there is a collusion p is empty and
        % obstacle is non empty. That is, either 'p' is empty or 'obstacle'
        % and not both.
        function [p, obstacle] = directExpansion(obj, stepSize, targetConfig, obstacles)
            
        end
    end
end

