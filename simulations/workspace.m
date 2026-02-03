function [output] = workspace(pos, phi_0, phi_min, phi_max, pos_fun)
    % A function that estimates reachable workspace on a predefined grid. 
    % The plot displays points that the effector can reach within a specified error margin.


    % Predefined error threshold
    err_thr = 1.5;  

    % Workspace constraints
    workspaceX_lim = [-500, -50];
    workspaceY_lim = [-500, 500];
    workspaceZ_lim = [30, 500];

    % Number of points in each dimension
    Nx = 25;   
    Ny = 30;  
    Nz = 25;     
    
    % Vectors of points
    vec_x = linspace(workspaceX_lim(1), workspaceX_lim(2), Nx);
    vec_y = linspace(workspaceY_lim(1), workspaceY_lim(2), Ny);
    vec_z = linspace(workspaceZ_lim(1), workspaceZ_lim(2), Nz);

    % Conversion of initial angle values from degrees to radians
    phi_0 = deg2rad(phi_0);
    phi1_bnd = deg2rad([phi_min(1), phi_max(1)]);
    phi2_bnd = deg2rad([phi_min(2), phi_max(2)]);
    phi3_bnd = deg2rad([phi_min(3), phi_max(3)]);
    phi4_bnd = deg2rad([phi_min(4), phi_max(4)]);
        
    % Rotation penalty function
    s = @(x, a, b) 1e3*(max(0, x - b)^2 + max(0, a - x)^2);
    s_all = @(x) (s(x(1), phi1_bnd(1), phi1_bnd(2)) + s(x(2), phi2_bnd(1), phi2_bnd(2)) + s(x(3), phi3_bnd(1), phi3_bnd(2)) + s(x(4), phi4_bnd(1), phi4_bnd(2)));

    % Total number of iterations
    Ntot = Nx*Ny*Nz;
    pts = nan(Ntot, 3);
    k = 0;
    
    w_smooth = 1e-2;

    tic
    for ix = 1:Nx
        for iy = 1:Ny
            for iz = 1:Nz
                pos = [vec_x(ix); vec_y(iy); vec_z(iz)];
    
                % A position error function defined as the difference between the effector position and the desired position.
                error_fun = @(phi) norm(pos_fun(phi) - pos)^2 + w_smooth*norm(phi - phi_0(:))^2;
               
                % Final cost function for optimization
                cost_fun = @(phi) error_fun(phi) + s_all(phi);

                % Optimization with fminsearch
                options = optimset('MaxFunEvals', 5000, 'MaxIter', 5000);
                phi_val = fminsearch(cost_fun, phi_0, options);

                % Starting next optimization from current position
                %phi_0 = phi_val;
                
                % Calculate error
                fk = pos_fun(phi_val);
                dist = norm(fk - pos);
                
                if dist < err_thr
                    k = k + 1;
                    pts(k,:) = pos.';
                end
            end
        end
    end
    toc
    
    % Plotting results
    pts = pts(1:k, :);

    figure; hold on; grid on; axis equal
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title(sprintf('Estimated workspace', k, err_thr))
    
    h = scatter3(pts(:,1), pts(:,2), pts(:,3), 18, 'filled');
    h.MarkerFaceAlpha = 0.25;  
    h.MarkerEdgeAlpha = 0.10;  
    
    view([-35 22]);  
    xlim(workspaceX_lim); ylim(workspaceY_lim); zlim(workspaceZ_lim);


end

