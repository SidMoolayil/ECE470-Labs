function tau = rep(q,myrobot,obs)
    %
    eta = 1;
    
    %
    tau = zeros(1,6);
    if length(obs) != 0
        Hs = forward(q,myrobot)

        for i = 1:6


            % J calculation
            % J column by column
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


            % F 
            h_idx = H(:,:i)
            o_i  = h_idx(1:3,4,i);
            % check obs
            R =  obs.R;
            c =  obs.c;
            rho0 = obs.rho0;
            type = obs.type;
        
            % sphere or cylinder
            if     type == 'sph'
                rho_vector = (o_i - obs.c) * ( 1 - obs.R / norm(o_i - obs.c));
        
            elseif type == 'cyl'
                rho_vector = [(o_i(1:2) - obs.c); 0] * ( 1 - obs.R / norm([(o_i(1:2) - obs.c); 0]) );
            end
        
            rho = norm(rho_vector);
            if rho <= rho0
                del_rho = rho_vector / rho;
                force = eta * (1/rho - 1/rho0) / rho^2 * del_rho;
            else
                force = zeros(3,1);
            end

            % tau 
            tau = tau + J' * F;
        end
    end
    
    if tau != zeros(1,6)
        tau = tau / norm(tau);
    end

    tau = tau*10^-6
end