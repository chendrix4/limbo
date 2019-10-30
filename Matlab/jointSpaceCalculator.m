clc
clear
close all

unit_vector_JS = []; %hold valid unit vector combinations
angle_JS = []; %hold valid angle combinations

limits_x = -2:.05:2;
limits_y = -2:.05:2;
limits_z = -2:.05:2;
limits_height = 4.5:.05:5.5;
num_combinations = size(limits_x, 2) * size(limits_y, 2) * size(limits_z, 2) * size(limits_height, 2);
fprintf('Number combinations: %d\n', num_combinations)
fprintf('Progress:\n')
iter = 0;
for unit_x = limits_x
   for unit_y = limits_y
       for unit_z = limits_z
           for unit_height = limits_height
               [alpha_1, alpha_2, alpha_3] = simulatorLimbo([unit_x; unit_y; unit_z], unit_height);
               if isreal(alpha_1) && ~isnan(alpha_1) && isreal(alpha_2) && ~isnan(alpha_2) && isreal(alpha_3) && ~isnan(alpha_3)
                   angle_JS = [angle_JS; alpha_1, alpha_2, alpha_3];
                   unit_vector_JS = [unit_vector_JS; unit_x, unit_y, unit_z];
               end
           end
       end
   end
   fprintf('%.2f%%\n', iter / size(limits_x, 2) * 100)
   iter = iter + 1;
end

successful_combinations = size(angle_JS, 1);
fprintf('Successful Combinations: %d\n', successful_combinations)
fprintf('Real Rate: %f\n', successful_combinations / num_combinations)

angle_JS = 180/pi*angle_JS; %convert from radians to degrees

figure
subplot(2, 2, 1)
hold on
xlabel('x (degrees)')
ylabel('y (degrees)')
zlabel('z (degrees)')
title('Joint Space')
plot3(angle_JS(:, 1), angle_JS(:, 2), angle_JS(:, 3))

subplot(2, 2, 2)
hold on
xlabel('x (degrees)')
ylabel('y (degrees)')
zlabel('z (degrees)')
title('Joint Space Mesh')
delauney_triangulation = delaunay(angle_JS(:, 1), angle_JS(:, 2));
trisurf(delauney_triangulation, angle_JS(:, 1), angle_JS(:, 2), angle_JS(:, 3),'EdgeColor','none')

subplot(2, 2, 3)
hold on
xlabel('x')
ylabel('y')
zlabel('z')
title('Unit Vector Space')
plot3(unit_vector_JS(:, 1), unit_vector_JS(:, 2), unit_vector_JS(:, 3))

subplot(2, 2, 4)
hold on
xlabel('x')
ylabel('y')
zlabel('z')
title('Unit Vector Space Mesh')
delauney_triangulation = delaunay(unit_vector_JS(:, 1), unit_vector_JS(:, 2));
trisurf(delauney_triangulation, unit_vector_JS(:, 1), unit_vector_JS(:, 2), unit_vector_JS(:, 3),'EdgeColor','none')