function [botSim, botEstimate] = particlefilter(botSim, modifiedMap,numParticles, maxNumOfIterations, scan)
%% NXT Control
COM_CloseNXT all
h=COM_OpenNXT();
COM_SetDefaultNXT(h);
leftWheel = NXTMotor('A');  % to move the left wheel
rightWheel = NXTMotor('C'); % to move the right wheel
scannerMotor = NXTMotor('B'); % to move the motor in which the scanner is connected
scannerMotor.Power = 50;
OpenUltrasonic(SENSOR_3);

%% Localisation code
num = numParticles;
particles(num,1) = BotSim; %how to set up a vector of objects
for i = 1:num
    particles(i) = BotSim(modifiedMap);  %each particle should use the same map as the botSim object
    particles(i).randomPose(0); %spawn the particles in random locations 
end
%botEstimate = BotSim();
botEstimate = BotSim(modifiedMap);
botEstimate.setScanConfig(botSim.generateScanConfig(scan));
% modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);
maxNumOfIterations = 50;
n = 0;
converged =0;%The filter has not converged yet
varience = 80;
botSim.setScanConfig(botSim.generateScanConfig(scan));


while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
    n = n+1; %increment the current number of iterations
    botScan = botSim.ultraScan(); %get a scan from the real robot.
    while (botScan < 0)  %Catch invalid scan results
        botSim.turn(pi/2);
        for i=1:num
           particles(i).turn(pi/2); 
        end
        botScan = botSim.ultraScan(); %get a scan from the real robot.
    end
    %% Write code for updating your particles scans
    particleScan = zeros(scan,num);
    differences = zeros(scan,num);
    weightParticle = zeros(scan,1);
    weight = zeros(num,1);
    for i = 1:num
        if particles(i).insideMap() ==0
            particles(i).randomPose(0);
        end
        particles(i).setScanConfig(particles(i).generateScanConfig(scan));
        particleScan(:,i)=particles(i).ultraScan();
        for j = 1:scan
    %% Write code for scoring your particles
            p = circshift(particleScan(:,i),j);
            	
            weightParticle(j) = (1/sqrt(2*pi*varience))*exp(-((differences(j,i))^2/(2*varience)));
        end
        [max_weight, max_pos] = max(weightParticle);
        weight(i) = max_weight;
        angleParticle = particles(i).getBotAng() + max_pos*2*pi/scan;
        particles(i).setBotAng(mod(angleParticle, 2*pi));
    end 
    weights = weight./sum(weight);
    %% Write code for resampling your particles
    resample_weight = prctile(weights,80);
    resampleParticles = zeros(num,3);
    count = 1;
    for i = 1:num
        particles(i).setMotionNoise(0.5);
        particles(i).setTurningNoise(0.5);
        if weight(i) >= resample_weight
            for j = count : count + 4
                resampleParticles(j, 1:2) = particles(i).getBotPos();
                resampleParticles(j, 3) = particles(i).getBotAng();
            end
            count = count + 5;
        end
    end
    for i=1:num
        particles(i).setBotPos([resampleParticles(i,1), resampleParticles(i,2)]);
        particles(i).setBotAng(resampleParticles(i,3));
    end
    %% Write code to check for convergence
    position = zeros(num, 2);   
    angle = zeros(num,1);
   
    for i = 1:num
        position(i,:) = particles(i).getBotPos();
        angle(i)=particles(i).getBotAng();
    end
	convergeThreshold = 5;
    convergePosition = std(position);
    if convergePosition <= convergeThreshold
        break;
    end

    %% Write code to take a percentage of your particles and respawn in randomised locations (important for robustness)	
    mutation_rate=0.01;
    for i=1:mutation_rate*num
        particles(randi(num)).randomPose(0);
    end 
    
    %% Write code to decide how to move next
    % here they just turn in cicles as an example
    %[distance crossingPoint]  = max(botScan);
    %[distanceMax positionMax] = max(distance);
    %disp('distanceMax');
    %disp(distanceMax);
    %disp('distance')
    %disp(distance);
%     turn = 0.5;
%     move = 2;
%     botSim.turn(turn); %turn the real robot.  
%     botSim.move(move); %move the real robot. These movements are recorded for marking 
%     for i =1:num %for all the particles. 
%         particles(i).turn(turn); %turn the particle in the same way as the real robot
%         particles(i).move(move); %move the particle in the same way as the real robot
%     end
    [distance crossingPoint]  = max(botScan);%find maximum possible distance
    turn = (crossingPoint-1)*2*pi/scan; %orientate towards the max distance   
    move = 2*rand(1);
    botSim.turn(turn);
    botSim.move(move);
    for i =1:num %for all the particles. 
        particles(i).turn(turn); %turn the particle in the same way as the real robot
        particles(i).move(move); %move the particle in the same way as the real robot
    end
%     if rand()<0.7
%         move = distance*0.8*rand();
%         turn =(crossingPoint-1)*2*pi/scan;
%     else
%         index=randi(scan); % get random scan orientation
%         turn = (index-1) *2*pi/scan;  % face the scan orientation
%         move= botScan(index) * 0.9; % move less than the scan distance
%     end
%     %disp('positionMax');
%     %disp(positionMax);
%     botSim.move(move); %move the real robot. These movements are recorded for marking 
%     botSim.turn(turn);
%     if botSim.insideMap==0
%         botSim.move(-move);
%         botSim.turn(pi/2);
%     end
%     for i =1:num %for all the particles.
%         particles(i).move(move); %move the particle in the same way as the real robot
%         particles(i).turn(turn); %turn the particle in the same way as the real robot
%         if particles(i).insideMap()==0
%             particles(i).randomPose(0);
%         end
%     end
    
    %% Drawing
    %only draw if you are in debug mode or it will be slow during marking
%     if botSim.debug()
%         hold off; %the drawMap() function will clear the drawing when hold is off
%         botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
%         botSim.drawBot(30,'g'); %draw robot with line length 30 and green
%         for i =1:num
%             particles(i).drawBot(3); %draw particle with line length 3 and default color
%         end
%         drawnow;
%     end
end
%% estimate the position of the real robot

botScan = botSim.ultraScan();
for i = 1:num
    particles(i).setScanConfig(particles(i).generateScanConfig(scan));
    particleScan(:,i)=particles(i).ultraScan();
    for j = 1:scan
        p = circshift(particleScan(:,i),j);
        differences(j,i) = sqrt(sum((p-botScan).^2));
        weightParticle(j) = (1/sqrt(2*pi*varience))*exp(-((differences(j,i))^2/(2*varience)));
    end
    [max_weight, max_pos] = max(weightParticle);
    weight(i) = max_weight;
end
    [maxWeight maxPos]=max(weight);
%     disp('maxPos');
%     disp(maxPos)
    max_Pos = particles(maxPos).getBotPos();
    max_Ang = particles(maxPos).getBotAng();
%    disp(max_Pos);
    botEstimate.setBotPos(max_Pos);
    botEstimate.setBotAng(max_Ang);

BotEstimate1 = BotSim(modifiedMap);
BotEstimate1.setScanConfig(BotEstimate1.generateScanConfig(scan));
BotEstimate2 = BotSim(modifiedMap);
BotEstimate2.setScanConfig(BotEstimate2.generateScanConfig(scan));

BotEstimate1.setBotPos(mean(position));
BotEstimate1.setBotAng(mean(angle));
BotEstimate2.setBotPos(mode(position));
BotEstimate2.setBotAng(mean(angle));

botScan = botSim.ultraScan();
botmean= zeros(360,1);
botmode= zeros(360,1);
for i=1:360    %Check scans of mode and mean estimates at every angle
    BotEstimateScan1 = BotEstimate1.ultraScan();
    BotEstimateScan2 = BotEstimate2.ultraScan();
    botmean(i) = norm(BotEstimateScan1-botScan);
    botmode(i) = norm(BotEstimateScan2-botScan);
    BotEstimate1.setBotAng(i*pi/180);
    BotEstimate2.setBotAng(i*pi/180);
end

[averageError, averageInd] = min(botmean);
BotEstimate1.setBotAng(averageInd*pi/180); 
[mostError, mostInd]=min(botmode);
BotEstimate2.setBotAng(mostInd*pi/180);

if averageError < mostError %pick best from mean or mode estimates
    botEstimate = BotEstimate1;
else
    botEstimate = BotEstimate2;
end
end