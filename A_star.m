function path_planner = A_star(currentPos,target,map)

% [50 70],[230 70],[-30,0;-30,40;30,40;30,60;5,60;45,90;85,60;60,60;60,40;120,40;120,60;95,60;135,90;175,60;150,60;150,40;210,40;210,60;185,60;225,90;265,60;240,60;240,40;300,40;300,0]
% 
%% Initialise
robot = BotSim(map);
robot.setBotPos(currentPos)
margin = 5;
inside_map = makeReducedMap(map,margin);
robot2 = BotSim(inside_map);

while (robot2.pointInsideMap(currentPos) == 0 || robot2.pointInsideMap(target) == 0) && margin>=0 
    margin = margin -1;
    inside_map = makeReducedMap(map,margin);
    robot2 = BotSim(inside_map);
end

new_map = [currentPos; inside_map; target];
nodes = length(new_map);
distanceBetweenNodes = zeros(length(new_map),length(new_map));

%% Reachable nodes in the map
for i = 1:length(new_map)  
   for m = 1:length(new_map)
        
        x1 = new_map(i,1);    
        y1 = new_map(i,2);
        x2 = new_map(m,1);
        y2 = new_map(m,2);
        discretised_line = linepts([x1 y1],[x2 y2]);
        k=1;
        goodSegment = true;
        while k<=length(discretised_line)
            testPos = discretised_line(k,:);
            if robot.pointInsideMap(testPos) == 0
                goodSegment = false;
                break
            else
                k=k+1;
            end
        end
        
        if goodSegment == true            
            distanceBetweenNodes(i,m) = sqrt((x2-x1)^2+(y2-y1)^2); %find the euclidian distance between all ...
%             hold on;
%             line([x1 x2],[y1 y2],'color',[.5 .5 .5]);
        else           
            distanceBetweenNodes(i,m) = Inf;
        end        
   end
   distanceBetweenNodes(i,i) = Inf;  %set diagonal to Inf
end

% line(map(:,1), map(:,2),'color',[0.5 .5 0],'linewidth',2 );
clear x1 x2 y1 y2 goodSegment discretised_line testPos i m ii

%% Check if going directly to target from current position is possible
if distanceBetweenNodes(1,end)~= Inf
    solution = [1,nodes(end)];
else
    solution = 1;
end

%% Build path
distanceCovered = 0;
dummy_idx = [];
while solution(end) ~= nodes(end)
    
    % first generation
    parent1 = solution(end);
    children1 = find(distanceBetweenNodes(parent1,:)~=Inf);
    
    % check if any children has already been visited and remove them
    [~,occurrences] = ismember(children1,solution);
    dummy_idx = find(occurrences);
        
    if isempty(dummy_idx) == 0
    children1(dummy_idx(:)) = [];
    dummy_idx(:) = [];
    end
    
    % set branches
    for m = 1:length(children1)
        branches{m,:} = [solution children1(m)];
    end
    long = size(branches,1);
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    %second generation
    for i=1:long
        parent2 = branches{i,:};
        parent2 = parent2(end);
        children2 = find(distanceBetweenNodes(parent2,:)~=Inf);
        
        [~,occurrences] = ismember(children2, branches{i,:});
        for m=1:length(occurrences)
            if occurrences(m) ~= 0
                dummy_idx = [dummy_idx m];
            end
        end
        children2(dummy_idx(:)) = [];
        dummy_idx(:) = [];
        
        low = size(branches,1);
        for k = 1:length(children2)
            branches{low+k} = [branches{i,:} children2(k)];
        end
        branches{i,:} = {}; % clear parent
    end
    branches = branches(~cellfun('isempty',branches)); 
    cost = zeros(size(branches,1),1);
    clear i j k m
    
    % sort out best child
    for i = 1:size(branches,1)
       currentBranch = branches{i,:};
       airlineDistance = sqrt((target(1) - new_map(currentBranch(end),1))^2+...
           (target(2) - new_map(currentBranch(end),2))^2);

       for j = 1:length(currentBranch)-1
       distanceCovered =  distanceCovered + distanceBetweenNodes(currentBranch(j),currentBranch(j+1));
       end
       cost(i,1) = distanceCovered + airlineDistance;
       distanceCovered = 0;
    end
    [best_cost,idx] = min(cost(:));
    solution = branches{idx,:};

end
    path_planner = new_map(solution,:);
    
%     line(new_map(solution(:),1),new_map(solution(:),2),'color',[0 .5 0],'linewidth',2 );
%     h = scatter(new_map(solution(:),1),new_map(solution(:),2),'filled','MarkerFaceColor',[0 .5 0]);
%     set(h,'SizeData',96);    
%     h = scatter(new_map(solution(1),1),new_map(solution(1),2),'o','filled','MarkerFaceColor','r');
%     set(h,'SizeData',96);
end