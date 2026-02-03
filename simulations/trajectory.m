function [output] = trajectory(pos, phi_0, phi_min, phi_max, pos_fun, A0, A1, A2, A3)
    % A function that determines an animated trajectory
    % the effector moves along a semicircle.
    syms phi0 phi1 phi2 phi3
    
    % Conversion of initial angle values from degrees to radians
    phi_0 = deg2rad(phi_0);
    phi1_bnd = deg2rad([phi_min(1), phi_max(1)]);
    phi2_bnd = deg2rad([phi_min(2), phi_max(2)]);
    phi3_bnd = deg2rad([phi_min(3), phi_max(3)]);
    phi4_bnd = deg2rad([phi_min(4), phi_max(4)]);

    % Rotation penalty function
    s = @(x, a, b) 1e3*(max(0, x - b)^2 + max(0, a - x)^2);
    s_all = @(x) (s(x(1), phi1_bnd(1), phi1_bnd(2)) + s(x(2), phi2_bnd(1), phi2_bnd(2)) + s(x(3), phi3_bnd(1), phi3_bnd(2)) + s(x(4), phi4_bnd(1), phi4_bnd(2)));

    % Calculation of semicircle
    r = 200;
    theta = linspace(pi/8 + pi/2, 3*pi/2 - pi/8, 50);
    z_height = 150;
    
    % Plot preparation
    figure; hold on
    grid on; axis equal
    xlabel('X'); 
    ylabel('Y');
    zlabel('Z'); 
    view([539 32])
    circle = [r*cos(theta(:)), r*sin(theta(:)), z_height*ones(numel(theta),1)];
    
    plot3(circle(:,1), circle(:,2), circle(:,3), 'o-');
    grid on; axis equal
    xlabel('X'); ylabel('Y'); zlabel('Z');
    for i = 1:length(theta)
        % Calculation of current point in semicircle
        pos = [r*cos(theta(i)), r*sin(theta(i)), z_height];
        
        % A position error function defined as the difference between the effector position and the desired position.
        error_fun = @(phi) norm(pos_fun(phi)-pos')^2;

        % Final cost function for optimization
        cost_fun = @(x) error_fun(x) + s_all(x);
        
        % Optimization with fminsearch
        phi_val = fminsearch(cost_fun, phi_0);
    
        % Starting next optimisation from current position
        phi_0 = phi_val;

        % Transformation matrices with calculated angle values
        A0_val = subs(A0, phi0, phi_val(1));
        A1_val = subs(A1, phi1, phi_val(2));
        A2_val = subs(A2, phi2, phi_val(3));
        A3_val = subs(A3, phi3, phi_val(4));
    
        T1 = A0_val;
        T2 = A0_val*A1_val;
        T3 = A0_val*A1_val*A2_val;
        T4 = A0_val*A1_val*A2_val*A3_val;

        % Points of individual joints
        P0 = [0,0,0];
        P1 = T1(1:3,4);
        P2 = T2(1:3,4);
        P3 = T3(1:3,4);
        P4 = T4(1:3,4);
        
        % plotting
        X = [P0(1), P1(1), P2(1), P3(1), P4(1)];
        Y = [P0(2), P1(2), P2(2), P3(2), P4(2)];
        Z = [P0(3), P1(3), P2(3), P3(3), P4(3)];
        h = plot3(X,Y,Z,'-ok','LineWidth',2,'MarkerSize',6);
        plot3(P4(1), P4(2), P4(3), 'ro', 'MarkerSize',6,'MarkerFaceColor','r')

        % Animation
        drawnow
        pause(0.2)
        
        % Keep only the latest robot rendering
        if i < length(theta)
            delete(h)
        end
    end
    hold off