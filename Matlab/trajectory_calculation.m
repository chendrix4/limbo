clc
clear all
close all

x_limit = [72, 166, 259];
y_limit = [3, 80, 174];

%%
figure
hold on

x = x_limit;
y = [-20, 0, 20];

scatter(x, y, 'r')

xx = x_limit(1):1:x_limit(3);
P2 = polyfit(x, y, 2);
P3 = polyfit(x, y, 3);
plot(xx, polyval(P2, xx), 'b')
plot(xx, polyval(P3, xx), 'm')

yy = (2.5*10^-5)*(xx - x_limit(2)).^3;
plot(xx, yy, 'c')

yy = (2.5*10^-9)*(xx - x_limit(2)).^5;
plot(xx, yy, 'g')

grid on
legend('points', 'quadratic', 'cubic', 'custom cubic', 'custom quintic')
xlim([xx(1), xx(end)])
ylim([-35, 35])
xlabel('pixel')
ylabel('angle')
title('hold center - polyfit')
hold off

%%
figure
hold on
x = 72:1:259; 
y = 3:1:174;

[xx, yy] = meshgrid(x, y);
zz = (2.5*10^-5)*sqrt((xx-166).^2 + (yy-80).^2).^3;
plot3(xx, yy, zz)
[px, py] = gradient(zz, 20, 20);

xlim([x(1), x(end)])
ylim([y(1), y(end)])
zlim([0, 30])
title('Potential Field - center (merged)')
hold off

%%
figure
subplot(1, 2, 1)
hold on

x = [x_limit(1):1:x_limit(3)];
y = [y_limit(1):1:y_limit(3)];
[xx, yy] = meshgrid(x, y);
zz1 = ((2.5*10^-5)*(xx-x_limit(2)).^3);
zz2 = ((2.5*10^-5)*(yy-y_limit(2)).^3);
mesh(xx, yy, zz1);
mesh(xx, yy, zz2);
xlabel('ball_x (pixel)')
ylabel('ball_y (pixel)')
zlabel('roll/pitch angle (degrees)')
title('hold at center - input angle')
hold off
subplot(1, 2, 2)
hold on
zz1 = abs((2.5*10^-5)*(xx-x_limit(2)).^3);
zz2 = abs((2.5*10^-5)*(yy-y_limit(2)).^3);
mesh(xx, yy, zz1);
mesh(xx, yy, zz2);
xlabel('ball_x (pixel)')
ylabel('ball_y (pixel)')
zlabel('roll/pitch angle (degrees)')
title('hold at center - potential field')
hold off

close all
%%
figure
hold on
xx = -100:100;
yy1 = [-30*ones(1, 100), 0, 10*ones(1, 50), 30*ones(1, 50)];
yy2 = -flip(yy1);
plot(xx, yy1, 'b--')
plot(xx, yy2, 'r:')
legend('left to right', 'right to left')
hold off
% subplot(1, 2, 2)
% hold on
% yy1 = [-30*ones(1, 80), polyval(polyfit([80, 100], [-30, 0], 2), 81:100), 0, 10*ones(1, 50), 30*ones(1, 50)];
% yy2 = -flip(yy1);
% plot(xx, yy1, 'b--')
% plot(xx, yy2, 'r:')
% legend('left to right smooth', 'right to left smooth')
% hold off

% %%
% figure
% hold on
% theta = linspace(0, 2*pi, 20);
% for r = linspace(0, 10, 20)
%     plot3(cos(theta)*r, sin(theta)*r, r^3*ones(1, size(theta, 2)))
% end
% 
% r = 5;
% x = cos(theta)*r;
% y = sin(theta)*r;
% z = r^3*ones(1, size(theta, 2));
% point = plot3(x(1), y(1), z(1), 'o', 'MarkerFaceColor', 'red');
% 
% % input('start')
% for i = 2:length(x)
%    point.XData = x(i);
%    point.YData = y(i);
%    point.ZData = z(i);
%    drawnow
%    pause on
%    pause(.1)
% end
% 
% % hold off