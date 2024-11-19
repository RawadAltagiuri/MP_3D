classdef RobotConfig < handle
    
    properties
        baseRotate
        design
        joints
        links
        matrix

    end
    
    methods
        function obj = RobotConfig(design)
            if nargin == 0
                design = [10, 10, 10];
            end
            setDesign(obj, obj.design);
        end       
        function setDesign(obj, design)
            obj.design = design;
            configureRobot(obj);
        end
        
        function printRobotConfigMatrix(obj)
            disp(obj.matrix);
        end
        
        function insertMat(obj, newPosition)
            if size(newPosition, 1) ~= size(obj.matrix, 1)
                %disp(size(obj.matrix, 1))
                error('The number of rows in the new position matrix does not match the number of rows in the robot configuration matrix.');
            end
            for i = 1:size(obj.matrix, 1)
                for j = 1:size(obj.matrix, 2)
                    if j == 1
                        obj.insertTethaX(i, newPosition(i, j));
                    elseif j == 2
                        obj.insertTethaY(i, newPosition(i, j));
                    elseif j == 3
                        obj.insertLength(i, newPosition(i, j));
                    end
                end
            end
        end
        
        function insertTethaX(obj, row, val)
            if row == 1 && obj.baseRotate == false
                obj.matrix(row, 1) = 0;
                return;
            end
%             if ~obj.joints(row).isRotating & val ~= 0
%                 fprintf('This joint is not rotating. The value will be set to 0.\n');
%                 obj.matrix(row, 1) = 0;
%             else
                obj.matrix(row, 1) = val;
                obj.joints(row).tethaX = val;
%             end
        end
        
        function insertTethaY(obj, row, val)
            if row == 1 && obj.baseRotate == false
                obj.matrix(row, 2) = 0;
                return;
            end
%             if ~obj.joints(row).isRotating & val ~= 0
%                 fprintf('This joint is not rotating. The value will be set to 0.\n');
%                 obj.matrix(row, 2) = 0;
%             else
                obj.matrix(row, 2) = val;
                obj.joints(row).tethaY = val;
%             end
        end
        
        function insertLength(obj, row, val)
            if (row > 1 && obj.links(row-1).length < obj.links(row-1).maxLength || val < 0)
                %disp(obj.links(row-1).length)
                obj.matrix(row, 3) = 0;
                return;
            end
            if val > obj.links(row).maxLength
                fprintf('The length of the link is too long. The value will be set to %f.\n', obj.links(row).maxLength);
                obj.matrix(row, 3) = obj.links(row).maxLength;
            else
                obj.matrix(row, 3) = val;
                obj.links(row).length = val;
            end
            if obj.matrix(row, 3) == obj.links(row).maxLength
                obj.joints(row+1).isRotating = true;
            end
        end
    end
    
    methods (Access = public)
        function configureRobot(obj)
            %disp(numel(obj.design));
            obj.matrix = zeros(numel(obj.design), 3);
            
            obj.links = struct('id', [], 'length', 0, 'maxLength', [], 'jointIds', []);
            obj.joints = struct('thetaX', [], 'thetaY', [], 'isRotating', false, 'id', []);
            
            for i = 1:numel(obj.design)
                obj.matrix(i, :) = [0, 0, 0];
                obj.joints(i).id = i;
                
                obj.links(i).id = i;
                obj.links(i).length = 0;
                obj.links(i).maxLength = obj.design(i);
                
                obj.joints(i).thetaX = obj.matrix(i, 1);
                obj.joints(i).thetaY = obj.matrix(i, 2);
                obj.links(i).length = obj.matrix(i, 3);
            end
            %disp(obj.matrix);
        end
    end
end
