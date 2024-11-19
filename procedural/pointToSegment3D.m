% Calculates distance point to segment in 3D (uses Voronoi distance defined in "Stroppa, F., Loconsole, C., & Frisoli, A. (2018). Convex polygon fitting in robot-based neurorehabilitation. Applied Soft Computing, 68, 609-625")
% Parameters 
% - pt: the point defined as [x y z]
% - v1: first vertex of the segment defined as [x y z] 
% - v2: second vertex of the segment defined as [x y z] 
% Output
% - d: distance 
% - inside: true if the point pt is within the segment, false otherwise
% - xp: ?
function [d, inside, xp] = pointToSegment3D(pt, v1, v2)
    
    u = [];
    mod = sqrt((v1(1)-v2(1))^2+(v1(2)-v2(2))^2+(v1(3)-v2(3))^2);
    u(1) = (v1(1)-v2(1)) / mod;
    u(2) = (v1(2)-v2(2)) / mod;
    u(3) = (v1(3)-v2(3)) / mod;
    
    R = getRodriguesRotation(u',[1 0 0]');
    v1_1 = (R*v1')';
    v2_1 = (R*v2')';
    pt_1 = (R*pt')';
    
    x2 = v2_1(1) - v1_1(1);
    xp = pt_1(1) - v1_1(1);
    
    if x2<0
        x2 = -x2;
        xp = -xp;
    end
    
    inside = false;
        
    if (xp >= 0 && xp <= x2)
        a = v1_1 - v2_1;
        b = pt_1 - v2_1;
        d = norm(cross(a,b)) / norm(a);
        inside = true;
    elseif xp < 0
        d = norm(pt-v1);
    else
        d = norm(pt-v2);
    end
    d = round(d,3);    
    
end