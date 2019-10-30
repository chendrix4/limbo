%GUI Simulation
f = figure;
p = uipanel(f,'Position',[0.1 0.1 0.35 0.65]);
c = uicontrol(p,'Style','slider');
c.Value = 0.5;