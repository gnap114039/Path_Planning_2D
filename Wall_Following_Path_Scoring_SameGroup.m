function [start_node , dest_node , distanceFromStart , parent_node, flag ,route ] = Wall_Following_Path_Scoring_SameGroup(map,Algorithm_Selection,force_stop_flag,exploring_direction,target_point,startnode)



map(find(map==6))=2;
[nrows, ncols] = size(map);
% start_node = startnode;
% dest_node = sub2ind(size(map),target_point(1,1),target_point(1,2));
dest_node = startnode;
start_node = sub2ind(size(map),target_point(1,1),target_point(1,2));
distanceFromStart = Inf(size(map));
distanceFromStart(start_node) = 0; % For each grid cell this array holds the index of its parent
flagbreak = 0;

%%=== Initializing Gradient Map, Open List, Parent List %%%%%%%%%%%%%%%%%%%%%
[X, Y] = meshgrid (1:ncols, 1:nrows);
[A, B] = ind2sub(size(map), dest_node);
H = abs(Y - A) + abs(X - B);
H_weight = H + (H./100);
f = Inf(nrows,ncols);
f(start_node) = H(start_node);
parent_node = zeros(size(map));

% tic;
%%=== Main Loop %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while true
    
    %%=== Force Stop Detection %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    drawnow();
    if strcmp(force_stop_flag.Enable, 'off');
        flag = 7;
        force_stop_flag.Enable = 'on';
        start_node = []; dest_node = []; map = [];
        distanceFromStart = []; parent_node = []; time = []; route=[];
        return;
    end
    map(start_node) = 5;
    map(dest_node) = 6;
    
    
    %%=== Find the node with the minimum distance ( f = g+ h ) %%%%%%%%%%%%
    if Algorithm_Selection == 3 %% for A* Heuristic
        min_dist_value = min(f(:));
        min_dist_candidate = find(f == min_dist_value); %%% ��map���ȳ̤p��candidate�� index
        current_candidate = H(min_dist_candidate); %%% ���Heuristic����current�Կ諸�Ȫ��j�p
        [~ , index] = min(current_candidate(:));
        current = min_dist_candidate(index);
    else
        [~, current] = min(f(:));
    end
    
    min_dist = distanceFromStart(current);
    if ((current == dest_node) && ~isinf(min_dist))
        break;
    end;
    
    if isinf(f(current))
        flag = 8;
        start_node = []; dest_node = []; 
        distanceFromStart = []; parent_node = []; time = []; route=[];
        return;
    end
    
    %%=== Expand Map Cell Candidate &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    map(current) = 3;

    f(current) = Inf;
    [i, j] = ind2sub(size(distanceFromStart), current);
    if exploring_direction == 0
        neighbor = [i-1,j;... %%...�W
            i+1,j;... %%...�U
            i,j-1;... %%...�k
            i,j+1]; %%...��
        direction = [1:4]';
    else
        neighbor = [i-1,j;... %%...�W
            i+1,j;... %%...�U
            i,j-1;... %%...�k
            i,j+1;... %%...��
            i-1,j-1;... %%...���W
            i+1,j-1;... %%...���U
            i-1,j+1;... %%...�k�W
            i+1,j+1]; %%...�k�U
        direction = [1:8]';
    end
    
    %%=== To Check the Expand Candidate Make Sure in Range of Map %%%%%%%%%
    outRangetest = (neighbor(:,1)<1) + (neighbor(:,1)>nrows) +...
        (neighbor(:,2)<1) + (neighbor(:,2)>ncols ) ;
    
    locate = find(outRangetest>0);
    
    neighbor(locate,:)=[];
    direction(locate,:)=[];
    neighborIndex = sub2ind(size(map),neighbor(:,1),neighbor(:,2));
    
    
    %%=== Fill Number Into Map Matrix and Update Distance Matrix, Parent Node, Open List Matrix
    for i=1:length(neighborIndex)
        if (map(neighborIndex(i))==6)
            
            if (direction(i) < 5) && distanceFromStart(neighborIndex(i)) > min_dist + 1
                distanceFromStart(neighborIndex(i)) = min_dist+1;
                parent_node(neighborIndex(i)) = current; %% �����̵u�Z�������|
            elseif  distanceFromStart(neighborIndex(i)) > min_dist + 1.414
                distanceFromStart(neighborIndex(i)) = min_dist+1.414;
                parent_node(neighborIndex(i)) = current; %% �����̵u�Z�������|
            end
            flagbreak=1;
            
        elseif ((map(neighborIndex(i))~=1) && (map(neighborIndex(i))~=3) && (map(neighborIndex(i))~= 5) && (map(neighborIndex(i))~=7) && (map(neighborIndex(i))~=8))
            map(neighborIndex(i)) = 4;

            
            if (direction(i) < 5) && distanceFromStart(neighborIndex(i)) > min_dist + 1
                distanceFromStart(neighborIndex(i)) = min_dist+1;
                parent_node(neighborIndex(i)) = current; %% �����̵u�Z�������|
            elseif  distanceFromStart(neighborIndex(i)) > min_dist + 1.414
                distanceFromStart(neighborIndex(i)) = min_dist+1.414;
                parent_node(neighborIndex(i)) = current; %% �����̵u�Z�������|
            end
            
            switch Algorithm_Selection
                case 1 %%... Dijkstra
                    f(neighborIndex(i)) = distanceFromStart(neighborIndex(i));
                case 2 %%... A*
                    f(neighborIndex(i)) = H(neighborIndex(i))+distanceFromStart(neighborIndex(i));
                case 3 %%... A* Weight
                    f(neighborIndex(i)) = H(neighborIndex(i))+distanceFromStart(neighborIndex(i));
                case 4 %%... Greedy Apporach
                    f(neighborIndex(i)) = H(neighborIndex(i));
                case 5 %%... Heuristic Weight
                    f(neighborIndex(i)) = H_weight(neighborIndex(i))+distanceFromStart(neighborIndex(i));
            end
            
        end
        
        if flagbreak==1
            break;
        end
        
    end
    
    if flagbreak==1
        break;
    end
    
end



%%=== Foolproof and Get the Last Position of Destination %%%%%%%%%%%%%%%%%%
if (isinf(distanceFromStart(dest_node)))
    route = [];
else %% �������|
    route = [dest_node];
end

%%=== Construction the Optimize Path by Parent Node Matrix %%%%%%%%%%%%%%%%
while (parent_node(route(end)) ~= 0) %%(parent_node(route(1)) ~= 0)
%     route = [parent_node(route(1)), route];
    route = [route,parent_node(route(end))];
end   %% �O���Ҧ����|


% %%=== Recolor the Optimize Path from Red to Green %%%%%%%%%%%%%%%%%%%%%%%%%
% for k = 2:length(route) - 1
%     map(route(k)) = 9; %% route color
% end

% toc;
% time = toc;
flag = [];



end