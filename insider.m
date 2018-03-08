function B_inner = insider(A,B,C,distanceFromEdge,map)
% clc
% clear all
% A = [0 30];
% B = [0 0];
% C = [30 0];
robot = BotSim(map);

V1=[(A(1)-B(1)) (A(2)-B(2))];
V2=[(C(1)-B(1)) (C(2)-B(2))];
V1_unit = V1/norm(V1);
V2_unit = V2/norm(V2);

bisector = (V1_unit + V2_unit) / norm(V1_unit+V2_unit);
half_angle  = abs(atan2(bisector(2),bisector(1)) - atan2(V1_unit(2),V1_unit(1)));
hyp = distanceFromEdge/sin(half_angle);

new_point  = B + hyp*bisector;
    if robot.pointInsideMap(new_point) == 0
        bisector = - (V1_unit + V2_unit) / norm(V1_unit+V2_unit);
        half_angle  = abs(atan2(bisector(2),bisector(1)) - atan2(V1_unit(2),V1_unit(1)));
        new_point  = B + hyp*bisector;
    end
    
B_inner = new_point;

% hold on
% axis equal
% plot([C(1),B(1)],[C(2),B(2)],'Color','r','LineWidth',2)
% plot([A(1),B(1)],[A(2),B(2)],'Color','r','LineWidth',2)
% plot([B(1),B_inner(1)],[B(2),B_inner(2)],'Color','b','LineWidth',2)
clear robot
end