clc
clear
close all

MIN_BOUND = 65;
MAX_BOUND = 140;

% Init serial connection
arduino = serialport(serialportlist, 115200);
writeline(arduino, 'A80');
writeline(arduino, 'B80');
writeline(arduino, 'C80');

f = figure;
a = uicontrol('Parent', f, 'Style', 'slider');
b = uicontrol('Parent', f, 'Style', 'slider');
c = uicontrol('Parent', f, 'Style', 'slider');

% A Control
a.Position = [40,50,200,25];
a.Value = 0.5*(MIN_BOUND + MAX_BOUND);
a.Min = MIN_BOUND;
a.Max = MAX_BOUND;
a.String = 'A';
addlistener(a, 'Value', 'PostSet', @(~, x) update(arduino, ...
                                                  x.AffectedObject, ...
                                                  [a, b, c, ax]));

al1 = uicontrol('Parent',f,'Style', 'text', ...
                           'Position', [20,50,25,25],...
                           'String', a.Min);
al2 = uicontrol('Parent',f,'Style', 'text', ...
                           'Position', [240,50,25,25],...
                           'String', a.Max);
al3 = uicontrol('Parent',f,'Style', 'text', ...
                           'Position', [90,25,100,25],...
                           'String', a.String);

% B Control
b.Position = [40,200,200,25];
b.Value = 0.5*(MIN_BOUND + MAX_BOUND);
b.Min = MIN_BOUND;
b.Max = MAX_BOUND;
b.String = 'B';
addlistener(b, 'Value', 'PostSet', @(~, x) update(arduino, ...
                                                  x.AffectedObject, ...
                                                  [a, b, c, ax]));

bl1 = uicontrol('Parent',f,'Style', 'text', ...
                           'Position', [20,200,25,25],...
                           'String', b.Min);
bl2 = uicontrol('Parent',f,'Style', 'text', ...
                           'Position', [240,200,25,25],...
                           'String', b.Max);
bl3 = uicontrol('Parent',f,'Style', 'text', ...
                           'Position', [90,175,100,25],...
                           'String', b.String);

% C Control
c.Position = [40,350,200,25];
c.Value = 0.5*(MIN_BOUND + MAX_BOUND);
c.Min = MIN_BOUND;
c.Max = MAX_BOUND;
c.String = 'C';
addlistener(c, 'Value', 'PostSet', @(~, x) update(arduino, x.AffectedObject));

cl1 = uicontrol('Parent',f,'Style', 'text', ...
                           'Position', [20,350,25,25],...
                           'String', c.Min);
cl2 = uicontrol('Parent',f,'Style', 'text', ...
                           'Position', [240,350,25,25],...
                           'String', c.Max);
cl3 = uicontrol('Parent',f,'Style', 'text', ...
                           'Position', [90,325,100,25],...
                           'String', c.String);


function [] = update(arduino, control)

  % Write data to robot
  writeline(arduino, sprintf('%s%d', control.String, round(control.Value)));
  fprintf('%s%d\n', control.String, round(control.Value));
  
end