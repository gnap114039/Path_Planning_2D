function [start_node , dest_node , distanceFromStart , parent_node, flag ,submap_shift, submap_route_in_area_upd] = Path_Scoring_Dynamic(submap_shift,map,Algorithm_Selection,force_stop_flag,exploring_direction)

[nrows, ncols] = size(submap_shift);
[nrows_map, ncols_map] = size(map);
start_node = find(submap_shift==5);
dest_node = find(submap_shift==6); % Initialize distance array
distanceFromStart = Inf(size(submap_shift));
distanceFromStart(start_node) = 0;
flagbreak = 0;
if nrows > nrows_map*1.2 && ncols > ncols_map*1.2
    warning = warndlg('Cannot find a path !!!','Warning !!','modal');
    flag == 9;
    start_node = []; dest_node = []; submap_shift = [];
    distanceFromStart = []; parent_node = []; time = []; submap_route_in_area_upd=[];
    return;
end

%%=== Initializing Gradient Map, Open List, Parent List %%%%%%%%%%%%%%%%%%%%%
[X, Y] = meshgrid (1:ncols, 1:nrows);
[A, B] = ind2sub(size(submap_shift), dest_node);
H = abs(Y - A) + abs(X - B);
H_weight = H + (H./100);
f = Inf(nrows,ncols);
f(start_node) = H(start_node);
parent_node = zeros(size(submap_shift));
p=1;
%%=== Main Loop %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while true
    
    %%=== Force Stop Detection %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    drawnow();
    if strcmp(force_stop_flag.Enable, 'off');
        flag = 7;
        force_stop_flag.Enable = 'on';
        start_node = []; dest_node = []; submap_shift = [];
        distanceFromStart = []; parent_node = []; time = []; submap_route_in_area_upd=[];
        return;
    end
    submap_shift(start_node) = 5;
    submap_shift(dest_node) = 6;
    
    
    %%=== Find the node with the minimum distance ( f = g+ h ) %%%%%%%%%%%%
    if Algorithm_Selection == 3 %% for A* Heuristic
        min_dist_value = min(f(:));
        min_dist_candidate = find(f == min_dist_value); %%% 給map中值最小的candidate的 index
        current_candidate = H(min_dist_candidate); %%% 比較Heuristic中的current候選的值的大小
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
        flag=8;
        start_node = []; dest_node = []; submap_shift = [];
        distanceFromStart = []; parent_node = []; time = []; submap_route_in_area_upd=[];
        return;
    end
    
    %%=== Expand Map Cell Candidate &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    submap_shift(current) = 3;
    f(current) = Inf;
    [i, j] = ind2sub(size(distanceFromStart), current);
    if exploring_direction == 0
        neighbor = [i-1,j;... %%...上
            i+1,j;... %%...下
            i,j-1;... %%...右
            i,j+1]; %%...左
    else
        neighbor = [i-1,j;... %%...上
            i+1,j;... %%...下
            i,j-1;... %%...右
            i,j+1;... %%...左
            i-1,j-1;... %%...左上
            i+1,j-1;... %%...左下
            i-1,j+1;... %%...右上
            i+1,j+1]; %%...右下
    end
    
    %%=== To Check the Expand Candidate Make Sure in Range of Map %%%%%%%%%
    outRangetest = (neighbor(:,1)<1) + (neighbor(:,1)>nrows) +...
        (neighbor(:,2)<1) + (neighbor(:,2)>ncols ) ;
    
    locate = find(outRangetest>0);
    
    neighbor(locate,:)=[];
    neighborIndex = sub2ind(size(submap_shift),neighbor(:,1),neighbor(:,2));
    

    %%=== Fill Number Into Map Matrix and Update Distance Matrix, Parent Node, Open List Matrix
    for i=1:length(neighborIndex)
        if (submap_shift(neighborIndex(i))==6)
            
            if (i < 5) && distanceFromStart(neighborIndex(i)) > min_dist + 1
                distanceFromStart(neighborIndex(i)) = min_dist+1;
                parent_node(neighborIndex(i)) = current; %% 紀錄最短距離的路徑
            elseif  distanceFromStart(neighborIndex(i)) > min_dist + 1.414
                distanceFromStart(neighborIndex(i)) = min_dist+1.414;
                parent_node(neighborIndex(i)) = current; %% 紀錄最短距離的路徑
            end
            flagbreak=1;
            
        elseif ((submap_shift(neighborIndex(i))~=1) && (submap_shift(neighborIndex(i))~=3) && (submap_shift(neighborIndex(i))~= 5) && (submap_shift(neighborIndex(i))~=7) && (submap_shift(neighborIndex(i))~=8) && (submap_shift(neighborIndex(i))~=10))
            submap_shift(neighborIndex(i)) = 4;
            
            if (i < 5) && distanceFromStart(neighborIndex(i)) > min_dist + 1
                distanceFromStart(neighborIndex(i)) = min_dist+1;
                parent_node(neighborIndex(i)) = current; %% 紀錄最短距離的路徑
            elseif  distanceFromStart(neighborIndex(i)) > min_dist + 1.414
                distanceFromStart(neighborIndex(i)) = min_dist+1.414;
                parent_node(neighborIndex(i)) = current; %% 紀錄最短距離的路徑
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
    submap_route_in_area_upd = [];
else %% 提取路徑
    submap_route_in_area_upd = [dest_node];
end

%%=== Construction the Optimize Path by Parent Node Matrix %%%%%%%%%%%%%%%%
while (parent_node(submap_route_in_area_upd(1)) ~= 0)
    submap_route_in_area_upd = [parent_node(submap_route_in_area_upd(1)), submap_route_in_area_upd];
end   %% 記錄所有路徑


% for k = 2:length(route_in_area_upd) - 1
%     submap_shift(route_in_area_upd(k)) = 9;
% end

flag = [];




end