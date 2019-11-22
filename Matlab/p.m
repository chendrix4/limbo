% clc
% clear all
% close all
% 
% x = [72, 166, 259];
% y = [-20, 0, 20]; %[3, 80, 174]
% 
%
% px = 0:1:300;
% P2 = polyfit(x, y, 2);
% P3 = polyfit(x, y, 3);
% 
% hold on
% scatter(x, y, 'r')
% plot(px, polyval(P2, px), 'b')
% plot(px, polyval(P3, px), 'm')
% 
% xx = 50:1:300;
% yy = (2.5*10^-5)*(xx - 166).^3;
% plot(xx, yy, 'c')
% 
% yy = (2.5*10^-9)*(xx - 166).^5;
% plot(xx, yy, 'g')
% 
% grid on
% legend('points', 'quadratic', 'cubic', 'custom cubic', 'custom quintic')
% xlim([xx(1), xx(end)])
% ylim([-35, 35])
% hold off
% 
% figure
% x = 72:1:259; 
% y = 3:1:174;
% 
% [xx, yy] = meshgrid(x, y);
% zz = 10*exp((-2.5*10^-5)*(xx-166).^3 - (2.5*10^-5)*(yy-80).^3);
% plot3(xx, yy, zz)
% % [px, py] = gradient(zz, .2, .2);
% 
% quiver(x, y, px, py)
% xlim([x(1), x(end)])
% ylim([y(1), y(end)])


x = -2:.1:2;
y = -2:.1:2;
[xx, yy] = meshgrid(x, y);
g = xx.^2 + yy.^2;
hold on
contour(xx, yy, g)
plot(2*xx + yy.^2, xx.^2 + 2*yy)
