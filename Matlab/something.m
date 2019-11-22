% %%
% L = 8.5; %cm
% L1 = 7; %cm
% L2 = 8.5; %cm
% s = 6.842*2.54; %cm
% 
% syms alpha_A real
% syms alpha_B real
% syms alpha_C real
% syms x_a real
% syms z_a real
% syms x_b real
% syms z_b real
% syms x_c real
% syms z_c real
% 
% % alpha_A = 60;
% % alpha_B = 60;
% % alpha_C = 60;
% 
% equation1 = L2^2 == (x_a - (L-L1*cos(alpha_A)))^2 + (z_a - L1*sin(alpha_A))^2
% equation2 = L2^2 == 4*(x_b + .5*(L-L1*cos(alpha_B)))^2 + (z_b - L1*sin(alpha_B))^2
% equation3 = L2^2 == 4*(x_c + .5*(L-L1*cos(alpha_C)))^2 + (z_c - L1*sin(alpha_C))^2
% equation4 = s^2 == (x_a - x_b)^2 + 3*x_b^2 + (z_a - z_b)^2 %y_a = 0, y_b = sqrt(3)x_b
% equation5 = s^2 == (x_a - x_c)^2 + 3*x_c^2 + (z_a - z_c)^2 %y_a = 0, y_c = -sqrt(3)x_c
% equation6 = s^2 == (x_b - x_c)^2 + 3*(x_b - x_c)^2 + (z_b - z_c)^2 %y_b = sqrt(3)x_b, y_c = -sqrt(3)x_c
% 
% equations = [equation1, equation2, equation3, equation4, equation5, equation6];
% variables = [x_a, z_a, x_b, z_b, x_c, z_c]
% 
% solution = solve(equations, variables, 'MaxDegree', 4)
% 
%%
x0 = [60, 60, 60, 0, 0, 0, 0, 0, 0]; %alpha_A, alpha_B, alpha_C, x_a, z_a, x_b, z_b, x_c, z_c
[x, value] = fsolve(@func, x0)

function F = func(x)
    L = 8.5; %cm
    L1 = 7; %cm
    L2 = 8.5; %cm
    s = 6.842*2.54; %cm

    F = [(x(4) - (L-L1*cos(x(1))))^2 + (x(5) - L1*sin(x(1)))^2 - L2^2;
         4*(x(6) + .5*(L-L1*cos(x(2))))^2 + (x(7) - L1*sin(x(2)))^2 - L2^2;
         4*(x(8) + .5*(L-L1*cos(x(3))))^2 + (x(9) - L1*sin(x(3)))^2  - L2^2;
         (x(4) - x(6))^2 + 3*x(6)^2 + (x(5) - x(7))^2 - s^2;
         (x(4) - x(8))^2 + 3*x(8)^2 + (x(5) - x(9))^2 - s^2;
         (x(6) - x(8))^2 + 3*(x(6) - x(8))^2 + (x(7) - x(9))^2 - s^2
        ];
end