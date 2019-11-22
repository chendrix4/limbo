clc
clear all
close all

x = [72, 166, 259];
y = [-20, 0, 20];

px = 0:1:300;
P2 = polyfit(x, y, 2);
P3 = polyfit(x, y, 3);

hold on
scatter(x, y, 'r')
plot(px, polyval(P2, px), 'b')
plot(px, polyval(P3, px), 'm')

xx = 50:1:300;
yy = (2.5*10^-5)*(xx - 166).^3;
plot(xx, yy, 'c')

yy = (2.5*10^-9)*(xx - 166).^5;
plot(xx, yy, 'g')

grid on
legend('points', 'quadratic', 'cubic', 'custom cubic', 'custom quintic')
xlim([xx(1), xx(end)])
ylim([-35, 35])
hold off