%%REFERENCE
%The function for path planning is reference from matlab community
%website: https://uk.mathworks.com/matlabcentral/fileexchange/37656-pathfinder-v2/content
function [botSim] = localise(botSim,map,target)
%This function returns botSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function
%% NXT Control
COM_CloseNXT all
h=COM_OpenNXT();
COM_SetDefaultNXT(h);
leftWheel = NXTMotor('A');  % to move the left wheel
rightWheel = NXTMotor('C'); % to move the right wheel
scannerMotor = NXTMotor('B'); % to move the motor in which the scanner is connected
scannerMotor.Power = 50;
OpenUltrasonic(SENSOR_3);
%% setup code
%you can modify the map to take account of your robots configuration space
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);

%generate some random particles inside the map
numParticles =600; % number of particles
% particles(num,1) = BotSim; %how to set up a vector of objects
% for i = 1:num
%     particles(i) = BotSim(modifiedMap);  %each particle should use the same map as the botSim object
%     particles(i).randomPose(0); %spawn the particles in random locations
% end
% botEstimate = BotSim;
% botEstimate = BotSim(modifiedMap);
maxNumOfIterations = 50;
scan = 40;

%% Localisation code
[botSim, botEstimate] = particlefilter(botSim, modifiedMap,numParticles, maxNumOfIterations, scan);
%% path plan
n=0;
epsilon = 3; % how close from target the robot should be
reachTarget =0;
maxForwardDistanceRobot = 5;
maxNumOfIterations = 30;
position = botEstimate.getBotPos();
angel = botEstimate.getBotAng();
botSim.setScanConfig(botSim.generateScanConfig(scan));
botEstimate.setScanConfig(botEstimate.generateScanConfig(scan));
robot_size = 6;
while(reachTarget == 0 && n < maxNumOfIterations) %% particle filter loop    
    % maybe we should break out of the loop if this is true

    if  (abs(position(1) - target(1)) <= epsilon) &&  (abs(position(2) - target(2)) <= epsilon)
        disp('reach');
        disp(position);
        reachTarget = 1;
    else
        n = n+1;
        [angle, distance] = pathplanning(position, target, map, robot_size);
        position = botEstimate.getBotAng();
        angleRadian = (angle) * pi / 180; 
        turn(leftWheel, rightWheel, pi-position+angleRadian);
        botEstimate.turn(pi-position+angleRadian);

        [distances, crossingPoints] = ultraRealScan(distances,scannerMotor, scan);
        %botScan = botSim.ultraScan();

        %Check the bot won't move through a wall, if so run particle filter
        %again
        if distances(1)<= distance;
            [botSim, botEstimate] = particlefilter(botSim, modifiedMap,numParticles, maxNumOfIterations, scan);
        else
            move(distance, leftWheel, rightWheel);
            %botSim.move(distance);
            botEstimate.move(distance);
        end
        [distances, crossingPoints] = ultraRealScan(distances,scannerMotor, scan);
        %botScan = botSim.ultraScan();
        botEstimateScan = botEstimate.ultraScan();

        %calculate the difference between the ghost robot and the real robot
        difference = (sum(botEstimateScan-distances)/scan);
        threshold = 3;

        %Run particle filter if the difference between the ultrasound values is
        %above the threshold
        if (abs(difference) > threshold)
            [botSim, botEstimate] = particlefilter(botSim, modifiedMap,numParticles, maxNumOfIterations, scan);
        end

        %Estimated robot location
        position = botEstimate.getBotPos();
    

%         path_planner = A_star(position,target,map);  % path is a nx2 matrix with all coords to reach target
%         angleForNextMove = atan2(path_planner(2,2)-path_planner(1,2),path_planner(2,1)-path_planner(1,1));
% %                 disp('a star')
%         turn = angleForNextMove - angel;
%         move = min(maxForwardDistanceRobot, ...
%         sqrt( (path_planner(2,1) - position(1))^2 + (path_planner(2,2) - position(2))^2));
%         botSim.move(move);
%         botSim.turn(turn);
%         botEstimate.move(move);
%         botEstimate.turn(turn);
        
    end
%     botScan = botSim.ultraScan();
%     botEstimateScan = botEstimate.ultraScan();
% %     disp(length(botScan));
% %     disp('scan');
% %     disp(scan);
% %     disp(length(botEstimateScan));
%     %calculate the difference between the ghost robot and the real robot
%     difference = (sum(botEstimateScan - botScan)/scan);
%     threshold = 3;
%     
%     %Run particle filter if the difference between the ultrasound values is
%     %above the threshold
%     if (abs(difference) > threshold)
%         [botSim, botEstimate] = particlefilter(botSim, modifiedMap,numParticles, maxNumOfIterations, scan);
%     end
% 
%     %Estimated robot location
%     position = botEstimate.getBotPos();
%     angel = botEstimate.getBotAng();

    
end
%% NXT Control
CloseSensor(SENSOR_3);
end
