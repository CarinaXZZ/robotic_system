function [ angle, distance ] = pathplanning( start_point, end_point, external_boundaries, robot_size )
% Function which calculates and returns the length and angle of the path
% between points waypoints p1(x1, y1) and p2(x2, y2). This allows the
% robot's estimated path from start_point to end_point to be identified, so
% that the required translation and rotation can be performed by the robot.

%% Estimate a path from a given start point to a given end point
%tic % time taken to estimate a path
if robot_size > 5
    boundaries = boundary_inflation(external_boundaries, robot_size);
    coordinates = pathfinder(start_point, end_point, boundaries);
    %toc

    x1 = coordinates(1,1);
    y1 = coordinates(1,2);
    x2 = coordinates(2,1);
    y2 = coordinates(2,2);

    angle = atan2d(y2-y1,x2-x1) + 180;

    distance = sqrt ( ((y2-y1)^2) + ((x2-x1)^2) );
    if distance > 10
    distance = distance / 2;
    end
else
     botSim.move(move)    
     botSim.turn(turn);
     botEstimate.move(move);
     botEstimate.turn(turn);

end
end

