%%
%https://www.mathworks.com/help/symbolic/solve.html
L = 8.5; %cm
L1 = 7; %cm
L2 = 8.5; %cm
s = 6.842*2.54; %cm

syms alpha_A real
syms beta_A real
syms alpha_B real
syms beta_B real
syms alpha_C real
syms beta_C real

%% nonlinear
% A = [L - L1*cos(alpha_A) - L2*cos(alpha_A + beta_A); ...
%      0; ...
%      L1*sin(alpha_A) + L2*sin(alpha_A + beta_A)];
% 
% B = [-1/2*(L - L1*cos(alpha_B) - L2*cos(alpha_B * beta_B)); ...
%      sqrt(3)/2*(L - L1*cos(alpha_B) - L2*cos(alpha_B + beta_B)); ...
%      L1*sin(alpha_B) + L2*sin(alpha_B + beta_B)];
% 
% C = [-1/2*(L - L1*cos(alpha_C) - L2*cos(alpha_C * beta_C)); ...
%      sqrt(3)/2*(L - L1*cos(alpha_C) - L2*cos(alpha_C + beta_C)); ...
%      L1*sin(alpha_C) + L2*sin(alpha_C + beta_C)];

A = [L; ...
     0; ...
     L1*sin(alpha_A) + L2*sin(alpha_A + beta_A)];

B = [-1/2*L; ...
     sqrt(3)/2*L; ...
     L1*sin(alpha_B) + L2*sin(alpha_B + beta_B)];

C = [-1/2*L; ...
     -sqrt(3)/2*L; ...
     L1*sin(alpha_C) + L2*sin(alpha_C + beta_C)];

%% linear 2nd order
% m = .425;
% % cos(a + b) ~= m*(a + b - pi)^2 - 1
% % sin(a + b) ~= -m*(a + b - pi/2)^2 + 1
% % cos(a) ~= -m*(a)^2 + 1
% % sin(a) ~= -m*(a - pi/2)^2 + 1

% A = [L - L1*(-m*alpha_A^2+1) - L2*(m*(alpha_A+beta_A-pi)^2-1); ...
%      0; ...
%      L1*(-m*(alpha_A-pi/2)^2+1) + L2*(-m*(alpha_A+beta_A-pi/2)^2+1)];
% 
% B = [-1/2*(L - L1*(-m*alpha_B^2+1) - L2*(m*(alpha_B+beta_B-pi)^2-1)); ...
%      sqrt(3)/2*(L - L1*(-m*alpha_B^2+1) - L2*(m*(alpha_B+beta_B-pi)^2-1)); ...
%      L1*(-m*(alpha_B-pi/2)^2+1) + L2*(-m*(alpha_B+beta_B-pi/2)^2+1)];
% 
% C = [-1/2*(L - L1*(-m*alpha_C^2+1) - L2*(m*(alpha_C+beta_C-pi)^2-1)); ...
%      -sqrt(3)/2*(L - L1*(-m*alpha_C^2+1) - L2*(m*(alpha_C+beta_C-pi)^2-1)); ...
%      L1*(-m*(alpha_C-pi/2)^2+1) + L2*(-m*(alpha_C+beta_C-pi/2)^2+1)];

% A = [L; ...
%      0; ...
%      L1*(-m*(alpha_A-pi/2)^2+1) + L2*(-m*(alpha_A+beta_A-pi/2)^2+1)];
% 
% B = [-1/2*L; ...
%      sqrt(3)/2*L; ...
%      L1*(-m*(alpha_B-pi/2)^2+1) + L2*(-m*(alpha_B+beta_B-pi/2)^2+1)];
% 
% C = [-1/2*L; ...
%      -sqrt(3)/2*L; ...
%      L1*(-m*(alpha_C-pi/2)^2+1) + L2*(-m*(alpha_C+beta_C-pi/2)^2+1)];

%% linear 1st order
% m = 2 / pi;
% % cos(a + b) ~= -m*(a+b-pi/2)
% % sin(a + b) ~= -m*(a+b-pi)
% % cos(a) ~= -m*(a-pi/2)
% % sin(a) ~= m*a

% % Solution 1
% solution.alpha_A
% ans =
% -0.23635119597051373285856667621262
% solution.beta_A
% ans =
% -0.12961194617737849866437527405208
% solution.alpha_B
% ans =
% -0.23635119597051373285856667621262
% solution.beta_B
% ans =
% -0.12961194617737849866437527405208
% solution.alpha_C
% ans =
% -0.23635119597051373285856667621262
% solution.beta_C
% ans =
% -0.12961194617737849866437527405208

% A = [L - L1*(-m*(alpha_A-pi/2)) - L2*(-m*(alpha_A+beta_A-pi/2)); ...
%      0; ...
%      L1*(m*alpha_A) + L2*(-m*(alpha_A+beta_A-pi))];
% 
% B = [-1/2*(L - L1*(-m*(alpha_B-pi/2)) - L2*(-m*(alpha_B+beta_B-pi/2))); ...
%      sqrt(3)/2*(L - L1*(-m*(alpha_B-pi/2)) - L2*(-m*(alpha_B+beta_B-pi/2))); ...
%      L1*(m*alpha_B) + L2*(-m*(alpha_B+beta_B-pi))];
% 
% C = [-1/2*(L - L1*(-m*(alpha_C-pi/2)) - L2*(-m*(alpha_C+beta_C-pi/2))); ...
%      -sqrt(3)/2*(L - L1*(-m*(alpha_C-pi/2)) - L2*(-m*(alpha_C+beta_C-pi/2))); ...
%      L1*(m*alpha_C) + L2*(-m*(alpha_C+beta_C-pi))];

% A = [L; ...
%      0; ...
%      L1*(m*alpha_A) + L2*(-m*(alpha_A+beta_A-pi))];
% 
% B = [-1/2*L; ...
%      sqrt(3)/2*L; ...
%      L1*(m*alpha_B) + L2*(-m*(alpha_B+beta_B-pi))];
% 
% C = [-1/2*L; ...
%      -sqrt(3)/2*L; ...
%      L1*(m*alpha_C) + L2*(-m*(alpha_C+beta_C-pi))];

%%
equation1 = (A(1) - B(1))^2 + (A(2) - B(2))^2 + (A(3) - B(3))^2 == s^2; %(A - B)^2 = s^2
equation2 = (A(1) - C(1))^2 + (A(2) - C(2))^2 + (A(3) - C(3))^2 == s^2; %(A - C)^2 = s^2
equation3 = (B(1) - C(1))^2 + (B(2) - C(2))^2 + (B(3) - C(3))^2 == s^2; %(B - C)^2 = s^2
equations = [equation1, equation2, equation3]
variables = [alpha_A, beta_A, alpha_B, beta_B, alpha_C, beta_C]

solution = solve(equations, variables, 'MaxDegree', 4)