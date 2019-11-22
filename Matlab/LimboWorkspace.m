% Limbo Workspace Analysis

clear
clc

good = [];
O7z = 7.5:0.1:15;
psi_x = -30:1:30;
psi_y = -30:1:30;

O7z_plot = [];
psi_x_plot = [];
psi_y_plot = [];

for o7z = O7z
    for px = psi_x
        for py = psi_y
            th = LimboIK(o7z, px, py);
            if all(65 < abs(th(1,:))) && all(abs(th(1,:)) < 140)
                O7z_plot = [O7z_plot, o7z];
                psi_x_plot = [psi_x_plot; px];
                psi_y_plot = [psi_y_plot; py];
            end
        end
    end
end

delauney_triangulation = delaunay(psi_x_plot, psi_y_plot);
trisurf(delauney_triangulation, psi_x_plot, psi_y_plot, O7z_plot,'EdgeColor','none')