function [output] = simulate_ik(pos, phi_0, phi_min, phi_max, pos_fun, A0, A1, A2, A3)
    %A simulation function runs the optimization function 
    % displays the result on a 3D plot together with the target point.    
    syms phi0 phi1 phi2 phi3
    
    % A position error function defined as the difference between the effector position and the desired position.
    error_fun = @(x) norm(pos_fun(x) - pos(:))^2;
    
    % Penalty function
    s = @(x, a, b) 1e3*(max(0, x - b)^2 + max(0, a - x)^2);
    
    % Conversion of initial angle values from degrees to radians
    phi_0 = deg2rad(phi_0);
    phi1_bnd = deg2rad([phi_min(1), phi_max(1)]);
    phi2_bnd = deg2rad([phi_min(2), phi_max(2)]);
    phi3_bnd = deg2rad([phi_min(3), phi_max(3)]);
    phi4_bnd = deg2rad([phi_min(4), phi_max(4)]);
    
    % Rotation penalty function
    s_all = @(x) (s(x(1), phi1_bnd(1), phi1_bnd(2)) + s(x(2), phi2_bnd(1), phi2_bnd(2)) + s(x(3), phi3_bnd(1), phi3_bnd(2)) + s(x(4), phi4_bnd(1), phi4_bnd(2)));    
    
    % Final cost function for optimization
    cost_fun = @(x) error_fun(x) + s_all(x);

    tic
    phi_val = fminsearch(cost_fun, phi_0);
    elapsedTime = toc;
    disp(['Elapsed time: ', num2str(elapsedTime), ' s'])

    % Display rotation angle in degrees
    phi_deg = rad2deg(phi_val);
    disp(['Rotation: ', num2str(phi_deg), ' deg'])

    % Transformation matrices with calculated angle values
    A0_val = subs(A0, [phi0], [phi_val(1)]);
    A1_val = subs(A1, [phi1], [phi_val(2)]);
    A2_val = subs(A2, [phi2], [phi_val(3)]);
    A3_val = subs(A3, [phi3], [phi_val(4)]);
    
    T1 = A0_val;
    T2 = A0_val*A1_val;
    T3 = A0_val*A1_val*A2_val;
    T4 = A0_val*A1_val*A2_val*A3_val;
    
    % Points of individual joints
    P0 = [0, 0, 0];
    P1 = T1(1:3,4);
    P2 = T2(1:3,4);
    P3 = T3(1:3,4);
    P4 = T4(1:3,4);
    
    % Position of effector
    final_pos = double(P4);
    
    % Position error in milimeters
    error = norm(pos(:) - final_pos(:));
    disp(['Error: ', num2str(error)])
   
    % Plotting results
    X = [P0(1), P1(1), P2(1), P3(1), P4(1)];
    Y = [P0(2), P1(2), P2(2), P3(2), P4(2)];
    Z = [P0(3), P1(3), P2(3), P3(3), P4(3)];
    figure; hold on
    plot3(X, Y, Z, '-ok', 'LineWidth', 2, 'MarkerSize', 6)
    plot3(X(1), Y(1), Z(1), '-og', 'LineWidth', 2, 'MarkerSize', 6)
    plot3(pos(1), pos(2), pos(3), '-ro', 'LineWidth', 2, 'MarkerSize', 6)
    grid on
    axis equal
    xlabel('X'); ylabel('Y'); zlabel('Z');
    view([538 28])

    output = 1;
end

