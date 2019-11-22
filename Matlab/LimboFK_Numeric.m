clear
clc

%global L L1 L2 s
L  = 8.5;
L1 = 7;
L2 = 8.5;
s  = 6.842*2.54;

b0 = [0,0,0];
b  = fsolve(@(b)limbo(b0, [70,70,70].*(pi/180)), b0);

A = getA(80, b(1));
B = getB(60, b(2));
C = getC(40, b(3));
figure();
simulatorLimbo(b', 1/2*(A(3) - B(3)) + B(3), 1)
hold on
plot3(A(1), A(2), A(3), B(1), B(2), B(3), C(1), C(2), C(3), 'x')


function F = limbo(b, a)

%global s

%A = getA(a(1), b(1));
%B = getB(a(2), b(2));
%C = getC(a(3), b(3));

L  = 8.5;
L1 = 7;
L2 = 8.5;
s  = 6.842*2.54;

A = [L - L1*cos(a) - L2*cos(a+b)  ; ...
     0                            ; ...
         L1*sin(a) + L2*sin(a+b) ];
B = [      -1/2 * ( L - L1*cos(a) - L2*cos(a+b) ); ...
      sqrt(3)/2 * ( L - L1*cos(a) - L2*cos(a+b) ); ...
                        L1*sin(a) + L2*sin(a+b) ];
C = [      -1/2 * ( L - L1*cos(a) - L2*cos(a+b) ); ...
     -sqrt(3)/2 * ( L - L1*cos(a) - L2*cos(a+b) ); ...
                        L1*sin(a) + L2*sin(a+b) ];
  
F(1) = (A(1) - B(1))^2 + (A(2) - B(2))^2 + (A(3) - B(3))^2 - s^2;
F(2) = (A(1) - C(1))^2 + (A(2) - C(2))^2 + (A(3) - C(3))^2 - s^2;
F(3) = (B(1) - C(1))^2 + (B(2) - C(2))^2 + (B(3) - C(3))^2 - s^2;

end

function A = getA(a, b)
global L L1 L2
A = [L - L1*cos(a) - L2*cos(a+b)  ; ...
     0                            ; ...
         L1*sin(a) + L2*sin(a+b) ];
end

function B = getB(a, b)
global L L1 L2
B = [      -1/2 * ( L - L1*cos(a) - L2*cos(a+b) ); ...
      sqrt(3)/2 * ( L - L1*cos(a) - L2*cos(a+b) ); ...
                        L1*sin(a) + L2*sin(a+b) ];
end

function C = getC(a, b)
global L L1 L2
C = [      -1/2 * ( L - L1*cos(a) - L2*cos(a+b) ); ...
     -sqrt(3)/2 * ( L - L1*cos(a) - L2*cos(a+b) ); ...
                        L1*sin(a) + L2*sin(a+b) ];
end