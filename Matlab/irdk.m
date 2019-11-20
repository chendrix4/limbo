L = 8.5; %cm
L1 = 7; %cm
L2 = 8.5; %cm

alpha_A = 1.3091*180/pi;
alpha_B = .7865*180/pi;
alpha_C = .7865*180/pi;

A = [0; ...
     L - L1*cos(alpha_A); ...
     L1*sin(alpha_A)];
 
B = [120; ...
     L - L1*cos(alpha_B); ...
     L1*sin(alpha_B)];

C = [-120; ...
     L - L1*cos(alpha_C); ...
     L1*sin(alpha_C)];

%AB cross BC
%A rectangular
A = [A(2)*cos(A(1)); ...
     A(2)*sin(A(1)); ...
     A(3)];
     
B = [B(2)*cos(B(1)); ...
     B(2)*sin(B(1)); ...
     B(3)];

C = [C(2)*cos(C(1)); ...
     C(2)*sin(C(1)); ...
     C(3)];

AB = [A(1) - B(1); A(2) - B(2); A(3) - B(3)];
AC = [A(1) - C(1); A(2) - C(2); A(3) - C(3)];
vec = cross(AB, AC);
unit_vector = vec / norm(vec)

% BA = [B(1) - A(1); B(2) - A(2); B(3) - A(3)];
% BC = [B(1) - C(1); B(2) - C(2); B(3) - C(3)];
% vec = cross(BA, BC);
% unit_vector = vec / norm(vec)