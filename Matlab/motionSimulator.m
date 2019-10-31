%Motion Simulation
for th = 0:pi/16:10*pi
    [a1,a2,a3] = simulatorLimbo([cos(th);sin(th);3],3.5+cos(th));
    fprintf('a1:%1.3f  a2:%1.3f  a3:%1.3f \n',a1*180/pi,a2*180/pi,a3*180/pi);
    pause(0.01);
end