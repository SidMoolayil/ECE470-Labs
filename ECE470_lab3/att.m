function tau = att(q,q2,myrobot)
    % initialize tau
    tau = zeros(1,6);
    % compute forward kinematics 
    Hs_init = forward(q,myrobot);
    Hs_final = forward(q2,myrobot);
    zeta = 0.01;
    Zs = ones(3,6);
    Os = ones(3,6);
    J = zeros(3,6);
    
    F_att = zeros(3,6);
    for i = 1:6
        
        % extract initial and desired joint positions
        O1(:,i) = Hs_init(1:3,4,i);
        O2(:,i) = Hs_final(1:3,4,i);
        
        % Compute attractive force 
        F_att(:,i) = -zeta * (O1(:,i) - O2(:,i) );
        
    end
    
    % compute the sum of tau
    for i = 1:6
        % compute new jacobian 
        for j = 1:6
            Zs(:,j) = Hs_init(1:3,3,j);
            Os(:,j) = Hs_init(1:3,4,j);
        end
    J(:,1) = cross([0;0;1], Os(:,i));
    for j = 2:i
        J(:,j) = cross(Zs(:,j-1), Os(:,i) - Os(:,j-1));
    end
    % add to summation
        tau = tau + (J' *F_att(:,i))';
    end
    
    % normalization
    if norm(tau) ~= 0
        tau = tau / norm(tau);
    end
end

