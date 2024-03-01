% This function is responsible of shecking collision between the robot and
% the obstacles 
% INPUT:
% 'conf' n x 3 matrix that represents the current configuration of the
%       robot
% 'op' structure of data that represents the robot and its details passed
%       as a whole  through the function
function [intersected_obstacles,j] = collisionCheck(conf,op)

    %these two lines are to deal with obstacles as if they are bigger to
    %avoid getting too close
    op.obstacles(:, 4) = op.obstacles(:, 4) + 25; % increases the radius by 25 so it makes the range that should be considered as collision
    op.obstacles(:, 5) = op.obstacles(:, 5) + 10; % increasing the height by 10 so it makes the range that should be considered as collision
   
    for ee=1:op.j % setting the current end effector of the robot
        if conf(ee,3)==0
            break
        end
    end
    xyz = solveForwardKinematics_3D(conf, op.home_base, false);
    intersected_obstacles = false; % setting the aboolean value of intersection to false and begins to check each joint and link it it collides with the robot 
    for j=1:1:ee %%might cause an error
        p_start = xyz(j,:); % getting the starting joint that is being under the checking 
        p_end = xyz(j+1,:); % getting the current ending joint that is being under the checking
        link_length = norm(p_end(1:2)-p_start(1:2)); % identifying the link length of the current link
        nearby_obstacles = findNearbyObstacles(p_start(1:2),link_length,op.lengthMin, op.obstacles); % checks if there are any nearby obstacles
        n_nearby_obstacles = size(nearby_obstacles,2);
        if isequal(p_start, p_end)% the change in line 9 (j=1:1:ee-1 to j=1:1:ee causes an error when p_end and p_start are equal, so we should stop here)
            return;
        end
        for z=1:1:n_nearby_obstacles % this loop execludes the checking among the nearby obstacles that needs to be checked
            obs = op.obstacles(nearby_obstacles(z),:); 
            if p_start(3) < (obs(3)-obs(5)) && p_end(3) > (obs(3)-obs(5))
                [resultZ,int] = checkIfZObstacle(p_start,p_end,obs); % checking if the current joints and link collide with the obstacle by any height
                if resultZ == true
                    if intersectObstacle_2D([int(1:2); p_end(1:2)],op.obstacles(nearby_obstacles(z),:),false)  
                        intersected_obstacles = true;
                        break;
                    end
                end
            elseif p_end(3) >= (obs(3)-obs(5))
                if obs(3) == op.plane_z
                    if intersectObstacle_2D([p_start(1:2); p_end(1:2)],op.obstacles(nearby_obstacles(z),:),false)  
                        intersected_obstacles = true;
                        break;
                    end
                else
                    [resultBase, notUsed] = checkIfBaseObstacle(p_start,p_end,obs); % checking if the current joints and link collide with the base of the obstacle
                    if resultBase == true
                        if intersectObstacle_2D([notUsed(1:2); notUsed(1:2)],op.obstacles(nearby_obstacles(z),:),false)  
                            intersected_obstacles = true;
                            break;
                        end
                    end
                end
            else
                % no intersection 
            end
            
        end  
        if intersected_obstacles
            break;
        end
    end
end
% this function checks if the obstacle collides with the obstacle at the z
% coordinates
function [result,i] = checkIfZObstacle(p_start,p_end,obs)
    result = false;
    u = (p_end-p_start)/norm(p_end-p_start);
    N = p_start;
    
    R = GetRodriguesRotation(u',[0 0 1]');
    p_start_r = (R*p_start')';
    p_end_r = (R*p_end')';
    
    %This checks the intersection between the segment and the plane defined by the top of the cylinder obstacle
    n = [0 0 1];
    M = [obs(1) obs(2) obs(3)-obs(5)];
    i = line_plane_intersection(u,N,n,M);  
    if isempty(i)
        result = false;
        return;
    end
    i_r = (R*i')';    
    if p_start_r(3) <= i_r(3) && i_r(3) <= p_end_r(3)
        result = true;
    end
end
%this function checks the collision between the current segment and the
%base of the obstacle 
function [result,i] = checkIfBaseObstacle(p_start,p_end,obs)
    result = false;
    u = (p_end-p_start)/norm(p_end-p_start);
    N = p_start;
    
    R = GetRodriguesRotation(u',[0 0 1]');
    p_start_r = (R*p_start')';
    p_end_r = (R*p_end')';
    
    %This checks the intersection between the segment and the plane defined by the top of the cylinder obstacle
    n = [0 0 1];
    M = [obs(1) obs(2) obs(3)];
    i = line_plane_intersection(u,N,n,M);  
    if isempty(i)
        result = false;
        return;
    end
    i_r = (R*i')';    
    if p_start_r(3) <= i_r(3) && i_r(3) <= p_end_r(3)
        result = true;
    end
    %YOU SHOULD RETURN THE SEGMENT INSIDE
end

function [I,rc] = line_plane_intersection(u, N, n, M, verbose)
assert(nargin > 3,'Not enough input arguments.');
assert(nargin < 6,'Too many input arguments.');
if nargin < 5    
    verbose = true;    
else    
    assert(islogical(verbose) || isreal(verbose),'verbose must be of type either logical or real numeric.');    
end
assert(isequal(size(u),size(N),size(n),size(M)),'Inputs u, M, n, and M must have the same size.');
assert(isequal(numel(u),numel(N),numel(n),numel(M),3),'Inputs u, M, n, and M must have the same number of elements (3).');
assert(isequal(ndims(u),ndims(N),ndims(n),ndims(M)),'Inputs u, M, n, and M must have the same number of dimensions.');
%% Body
% Plane offset parameter
d = -dot(n,M);
% Specific cases treatment
if ~dot(n,u) % n & u perpendicular vectors
    if dot(n,N) + d == 0 % N in P => line belongs to the plane
        if verbose
            disp('(N,u) line belongs to the (M,n) plane. Their intersection is the whole (N,u) line.');
        end
        I = M;
        rc = 2;
    else % line // to the plane
        if verbose
            disp('(N,u) line is parallel to the (M,n) plane. Their intersection is the empty set.');
        end
        I = [];
        rc = 0;
    end
else
    
    % Parametric line parameter t
    t = - (d + dot(n,N)) / dot(n,u);
    
    % Intersection coordinates
    I = N + u*t;
    
    rc = 1;
    
end
end % line_plane_intersection

