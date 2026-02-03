% A script creates a robot model with four degrees of freedom. 
% The Denavit–Hartenberg notation was used to construct the corresponding transformation matrices.
syms phi0 phi1 phi2 phi3

% Constant robot arm lengths
l0 = 120;
l1 = 110;
l2 = 140;  
l3 = 50;  

% Transformation matrices
A0 = DH(0, pi/2, l0, phi0);
A1 = DH(l1, 0, 0, phi1+pi/2);
A2 = DH(l2, 0, 0, phi2);
A3 = DH(l3, 0, 0, phi3);

% Transformation matrix of the effector
T = A0 * A1 * A2 * A3;

% Effector position
x = T(1, 4);
y = T(2, 4);
z = T(3, 4);

digits(10)

x = vpa(simplify(x));
y = vpa(simplify(y));
z = vpa(simplify(z));

% C code export for STM32
ccode(x)
ccode(y)
ccode(z)

% Function of effector position, used for optimisation
pos_fun_sym = matlabFunction([x; y; z], ...
    'Vars', {[phi0; phi1; phi2; phi3]});

pos_fun = @(phi) pos_fun_sym(phi(:));
