clc
clear
close all

% Arduino->tools -> port and the baud rate (9600)
arduino = serial('COM14','BaudRate',9600);

try
    fopen(arduino);
catch err
    fclose(arduino);
    error('Make sure you select the correct COM Port where the Arduino is connected.');
end

for theta = 0:pi/16:10*pi
    %[alpha1, alpha2, alpha3] = simulatorLimbo([cos(theta); sin(theta); 3], 4)
    alpha1 = 2;
    alpha2 = 3;
    alpha3 = 4;
    fprintf(arduino,'%s', char(alpha1));
    fprintf(arduino,'%s', char(alpha2));
    fprintf(arduino,'%s', char(alpha3));
    pause(0.1);
end
fclose(arduino);