function R = GetRodriguesRotation(v1, v2)
% R*v1=v2
% v1 and v2 should be column vectors and 3x1

% 1. rotation vector
    
    if v1==-v2
        R = [cos(pi) -sin(pi) 0; sin(pi) cos(pi) 0; 0 0 1];
    else
        w= cross(v1,v2);
        if(w(1)==0 && w(2) == 0 && w(3) == 0)
            R = eye(3);
        else
            w= w/norm(w);
            w_hat = fcn_GetSkew(w);

            % 2. rotation angle
            cos_tht = v1'*v2/norm(v1)/norm(v2);
            tht = acos(cos_tht);

            % 3. rotation matrix, using Rodrigues' formula
            R = eye(size(v1,1)) + w_hat*sin(tht) + w_hat^2 * (1-cos(tht));
        end
    end
end

function x_skew=fcn_GetSkew(x)
x_skew = [0 -x(3) x(2);
          x(3) 0 -x(1);
         -x(2) x(1) 0];
end