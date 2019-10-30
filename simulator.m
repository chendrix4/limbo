clc
clear
close all

figure
hold on 
axis square
xlabel('x')
ylabel('y')
zlabel('z')
title('Limbo Simulator')

%reference plane
radius = 3;
theta = linspace(0, 2*pi);
x = radius*cos(theta);
y = radius*sin(theta);
z = 0*theta;
points = [x; y; z];
normal_original = [0; 0; 1];
normal_original = normal_original / norm(normal_original); %unit vector

patch(points(1,:), points(2,:), points(3,:),'b')
plot3([0, normal_original(1)], [0, normal_original(2)], [0, normal_original(3)], 'b')

%plane to rotate to
%plane orthogonal to normal_new: normal_new(1)*x + normal_new(2)*y + normal_new(3)*z = 0
normal_new = [1; 1; 1]; %desired plane rotation
normal_new = normal_new / norm(normal_new); %unit vector
% theta is the angle between normals
c = dot(normal_original, normal_new) / ( norm(normal_original) * norm(normal_new) ); %cos(theta)
s = sqrt(1 - c * c); %sin(theta)
u = cross(normal_original, normal_new) / ( norm(normal_original) * norm(normal_new) ); %rotation axis
u = u / norm(u); %unit vector
C = 1-c;
R = [u(1)^2*C+c, u(1)*u(2)*C-u(3)*s, u(1)*u(3)*C+u(2)*s
    u(2)*u(1)*C+u(3)*s, u(2)^2*C+c, u(2)*u(3)*C-u(1)*s
    u(3)*u(1)*C-u(2)*s, u(3)*u(2)*C+u(1)*s, u(3)^2*C+c];
new_points = R*points;

patch(new_points(1, :), new_points(2, :), new_points(3, :), 'k')
plot3([0, normal_new(1)], [0, normal_new(2)], [0, normal_new(3)], 'k')