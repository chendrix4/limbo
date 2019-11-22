clear
clc

syms aA aB aC
syms bA bB bC

L  = 8.5;
L1 = 7;
L2 = 8.5;
s_sq  = (6.842*2.54)^2;

A = [               L - L1*cos(aA) - L2*cos(aA+bA)  ; ...
                    0                               ; ...
                        L1*sin(aA) + L2*sin(aA+bA) ];

B = [      -1/2 * ( L - L1*cos(aB) - L2*cos(aB+bB) ); ...
      sqrt(3)/2 * ( L - L1*cos(aB) - L2*cos(aB+bB) ); ...
                        L1*sin(aB) + L2*sin(aB+bB) ];

C = [      -1/2 * ( L - L1*cos(aC) - L2*cos(aC+bC) ); ...
     -sqrt(3)/2 * ( L - L1*cos(aC) - L2*cos(aC+bC) ); ...
                        L1*sin(aC) + L2*sin(aC+bC) ];
  
eq1 = (A(1) - B(1))^2 + (A(2) - B(2))^2 + (A(3) - B(3))^2 == s_sq;
eq2 = (A(1) - C(1))^2 + (A(2) - C(2))^2 + (A(3) - C(3))^2 == s_sq;
eq3 = (B(1) - C(1))^2 + (B(2) - C(2))^2 + (B(3) - C(3))^2 == s_sq;

s = solve([eq1, eq2, eq3], [bA, bB, bC])