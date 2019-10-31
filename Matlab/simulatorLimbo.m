function [alpha1,alpha2,alpha3] = simulatorLimbo(inVec,height)

    %INPUTS 
    normVec = inVec/norm(inVec,height);

    %CONSTANTS
    B = 4;
    L = 3;
    U = 3.5;
    Hc = height;
    plateRadius = 5.5;

    %CALCULATIONS
    %arm 1
    f1 = [B;0;0;1];
    %arm 2
    f2 = rotateZ(f1,2*pi/3);
    %plot arm 3
    f3 = rotateZ(f1,-2*pi/3);

    %plot plate vectors
    perpVec1 = [normVec(3) ; 0 ; -1*normVec(1)];
    perpVec1 = plateRadius*perpVec1/norm(perpVec1);

    perpVec2 = rodrigues_rot(perpVec1,normVec,2*pi/3);
    perpVec2 = plateRadius*perpVec2/norm(perpVec2);

    perpVec3 = rodrigues_rot(perpVec1,normVec,-2*pi/3);
    perpVec3 = plateRadius*perpVec3/norm(perpVec3);

    %upper arm connection with plate
    p1 = [perpVec1(1);perpVec1(2);Hc+perpVec1(3);1];
    p2 = [perpVec2(1);perpVec2(2);Hc+perpVec2(3);1];
    p3 = [perpVec3(1);perpVec3(2);Hc+perpVec3(3);1];

    %calculate alphas (INVERSE KINEMATICS from 10/18)
    %alpha1
    r1 = norm(p1(1:3) - f1(1:3));
    des = p1 - f1;
    gamma = atan2(des(3),norm(des(1:2)));
    beta = acos((L^2+r1^2-U^2)/(2*L*r1));
    alpha1 = pi - (gamma + beta);
    %alpha2
    r2 = norm(p2(1:3) - f2(1:3));
    des = p2 - f2;
    gamma = atan2(des(3),norm(des(1:2)));
    beta = acos((L^2+r2^2-U^2)/(2*L*r2));
    alpha2 = pi - (gamma + beta);
    %alpha3
    r3 = norm(p3(1:3) - f3(1:3));
    des = p3 - f3;
    gamma = atan2(des(3),norm(des(1:2)));
    beta = acos((L^2+r3^2-U^2)/(2*L*r3));
    alpha3 = pi - (gamma + beta);


    %Recalculate lower arms
    e1 = f1 + [-1*L*cos(alpha1);0;L*sin(alpha1);1];
    e2 = rotateZ(f1 + [-1*L*cos(alpha2);0;L*sin(alpha2);1],2*pi/3);
    e3 = rotateZ(f1 + [-1*L*cos(alpha3);0;L*sin(alpha3);1],-2*pi/3);

% %PLOTTING
% theta = linspace(0,2*pi);
% %plot base
% hold off;
% plot3(B*cos(theta),B*sin(theta),0*theta,'k');
% hold on;
% %plot arm 1
% plot3( [f1(1) e1(1)] , [f1(2) e1(2)] , [f1(3) e1(3)],'b');
% %plot arm 2
% plot3( [f2(1) e2(1)] , [f2(2) e2(2)] , [f2(3) e2(3)],'b');
% %plot arm 3
% plot3( [f3(1) e3(1)] , [f3(2) e3(2)] , [f3(3) e3(3)],'b');
% %plot normVec
% plot3( [0 normVec(1)] , [0 normVec(2)] , [Hc (Hc+normVec(3))] ,'b');
% %plot plate vectors
% plot3( [0 perpVec1(1)] , [0 perpVec1(2)] , [Hc (Hc+perpVec1(3))] ,'g');
% plot3( [0 perpVec2(1)] , [0 perpVec2(2)] , [Hc (Hc+perpVec2(3))] ,'g');
% plot3( [0 perpVec3(1)] , [0 perpVec3(2)] , [Hc (Hc+perpVec3(3))] ,'g');
% %plot upper arms
% plot3([p1(1) e1(1)] , [p1(2) e1(2)] , [p1(3) e1(3)],'r');
% plot3([p2(1) e2(1)] , [p2(2) e2(2)] , [p2(3) e2(3)],'r');
% plot3([p3(1) e3(1)] , [p3(2) e3(2)] , [p3(3) e3(3)],'r');
% %plot plate border
% lastborder = rodrigues_rot(p1(1:3)-[0;0;Hc],normVec,0);
% lastborder = lastborder + [0;0;Hc];
% for th = pi/10:pi/10:2*pi
%     border = rodrigues_rot(p1(1:3)-[0;0;Hc],normVec,th);
%     border = border + [0;0;Hc];
%     plot3([lastborder(1) border(1)], ...
%           [lastborder(2) border(2)], ...
%           [lastborder(3) border(3)],'g');
%     lastborder = border;
%     
% end
% axis square;
% axis([-6 6 -6 6 0 8]);
end

%ROTATION FUNCTIONS
function out = rotateZ(in,theta)
    R = [cos(theta) -sin(theta) 0 0;sin(theta) cos(theta) 0 0;0 0 1 0;0 0 0 1];
    out = R*in;
end