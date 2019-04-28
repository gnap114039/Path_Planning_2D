function [route, map, k] = Dynamic_Obstacle(route,map,Algorithm_Selection,force_stop_flag,k,exploring_direction,area_range,handles)


[nrows, ncols] = size(map);
[current_x , current_y] = ind2sub( size(map) ,route(k));%% route color of position index in map by route(OBSTACLE)
expand_point = [current_x-((area_range-1)/2) , current_y-((area_range-1)/2)];
expand_area = [];

%%=== 找出submap內所有的element在map的實際位置 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for j=1:area_range
    for q=1:area_range
        expand_area = [expand_area ; expand_point(1)+(q-1) expand_point(2)+(j-1) ];
    end
end

outRangetest = (expand_area(:,1)<1) + (expand_area(:,1)>nrows) +...
    (expand_area(:,2)<1) + (expand_area(:,2)>ncols ) ;

locate = find(outRangetest>0);

expand_area(locate,:)=[];
% expand_area_shift(locate,:)=[];
submap_shift = zeros( (max(expand_area(:,1))-min(expand_area(:,1))+1) , (max(expand_area(:,2))-min(expand_area(:,2))+1) );



%%=== 把submap實際位置換成對應的 map in index %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
expand_area_by_indx = sub2ind(size(map),expand_area(:,1),expand_area(:,2));
expand_area_shift_by_indx = [linspace(1,length(expand_area_by_indx),length(expand_area_by_indx))]';


%%=== 把在submap內的route找出來 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
route_in_area=[];
for i = 1:length(expand_area_by_indx)
    route_temp =  find(route == expand_area_by_indx(i)); %%... route 裡面第幾個
    route_in_area = [route_in_area,route_temp]; %%... route 裡面第幾個的矩陣
end

% if map(route(k+1) == 1)|| map(route(k+1) == 7)|| map(route(k+1)== 8)|| map(route(k+1)== 10);
%     return;
% else
% obstacle = find ( map(route(k+1:length(route-1))) == 10);


%%=== 把submap內的route在實際route中排序最小的作為起始點，最大的作為終點 %%%%%%%
start_node_temp = route(k);
dest_node_temp_indx = max(route_in_area);
% start_node_temp_indx = min(route_in_area); %%... 被框起來的area裡面的最小index in route的位置
% dest_node_temp_indx = max(route_in_area); %%... 被框起來的area裡面的最大index in route的位置
% start_node_temp = route(start_node_temp_indx); %%... Global Map 的index位置
dest_node_temp = route(dest_node_temp_indx); %%... Global Map 的index位置

%%=== 把submap紅色，紫色，藍色的element全部換成白色 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:length(expand_area_by_indx)
    if ((map(expand_area_by_indx(i))==3) ||(map(expand_area_by_indx(i))==4))
        map(expand_area_by_indx(i)) = 2;
    end
end

submap_shift(expand_area_shift_by_indx) = map(expand_area_by_indx);
if isempty((find(map(expand_area_by_indx)==5))) && isempty((find(map(expand_area_by_indx)==6))) %%... no head no tail
    map(start_node_temp) = 5;
    map(dest_node_temp) = 6;
    submap_shift(find(map(expand_area_by_indx)==5)) = 5;
    submap_shift(find(map(expand_area_by_indx)==6)) = 6;
    %%=== 進行新的最短路徑演算法 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [~ , ~ , ~ , ~, flag ,submap_shift, submap_route_in_area_upd] = Path_Scoring_Dynamic(submap_shift,map,Algorithm_Selection,force_stop_flag,exploring_direction);
    if flag == 8
        area_range = 31+(area_range-1)
        map(start_node_temp) = 2;
        map(dest_node_temp) = 2;
        [route, map, k] = Dynamic_Obstacle(route,map,Algorithm_Selection,force_stop_flag,k,exploring_direction,area_range,handles);
    else
        map(expand_area_by_indx) = submap_shift(expand_area_shift_by_indx);
        map(start_node_temp) = 3;
        map(dest_node_temp) = 3;
        route = [route(1:k),...
            [expand_area_by_indx(submap_route_in_area_upd)]',...
            route((dest_node_temp_indx+1):length(route))];
        k = k;
    end
    
    
elseif isempty((find(map(expand_area_by_indx)==5))) && ~isempty((find(map(expand_area_by_indx)==6)))%%... no head yes tail
    map(start_node_temp) = 5;
    map(dest_node_temp) = 2;
    submap_shift(find(map(expand_area_by_indx)==5)) = 5;
    submap_shift(find(map(expand_area_by_indx)==6)) = 6;
    [~ , ~ , ~ , ~, flag ,submap_shift, submap_route_in_area_upd] = Path_Scoring_Dynamic(submap_shift,map,Algorithm_Selection,force_stop_flag,exploring_direction);
    if flag == 8
        area_range = 31+(area_range-1)
        map(start_node_temp) = 2;
        map(dest_node_temp) = 2;
        [route, map, k] = Dynamic_Obstacle(route,map,Algorithm_Selection,force_stop_flag,k,exploring_direction,area_range,handles);
    else
        map(expand_area_by_indx) = submap_shift(expand_area_shift_by_indx);
        map(start_node_temp) = 3;
        route = [route(1:k),...
            [expand_area_by_indx(submap_route_in_area_upd)]',...
            route((dest_node_temp_indx+1):length(route))];
        k = k;
    end
    
    
elseif ~isempty(find (expand_area_by_indx == route(k))) && isempty((find(map(expand_area_by_indx)==6))) %%... yes head no tail
    map(start_node_temp) = 2;
    map(dest_node_temp) = 6;
    submap_shift(find(map(expand_area_by_indx)==5)) = 5;
    submap_shift(find(map(expand_area_by_indx)==6)) = 6;
    [~ , ~ , ~ , ~, flag ,submap_shift, submap_route_in_area_upd] = Path_Scoring_Dynamic(submap_shift,map,Algorithm_Selection,force_stop_flag,exploring_direction);
    if flag == 8
        area_range = 31+(area_range-1)
        map(start_node_temp) = 2;
        map(dest_node_temp) = 2;
        [route, map, k] = Dynamic_Obstacle(route,map,Algorithm_Selection,force_stop_flag,k,exploring_direction,area_range,handles);
    else
        map(expand_area_by_indx) = submap_shift(expand_area_shift_by_indx);
        map(start_node_temp) = 3;
        route = [route(1:k),...
            [expand_area_by_indx(submap_route_in_area_upd)]',...
            route((dest_node_temp_indx+1):length(route))];
        k = k;
    end
    
    
elseif ~isempty(find (expand_area_by_indx == route(k))) && ~isempty((find(map(expand_area_by_indx)==6))) %%... yes head yes tail
    map(start_node_temp) = 2;
    map(dest_node_temp) = 2;
    submap_shift(find(map(expand_area_by_indx)==5)) = 5;
    submap_shift(find(map(expand_area_by_indx)==6)) = 6;
    [~ , ~ , ~ , ~, flag ,submap_shift, submap_route_in_area_upd] = Path_Scoring_Dynamic(submap_shift,map,Algorithm_Selection,force_stop_flag,exploring_direction);
    if flag == 8
        area_range = 31+(area_range-1)
        map(start_node_temp) = 2;
        map(dest_node_temp) = 2;
        [route, map, k] = Dynamic_Obstacle(route,map,Algorithm_Selection,force_stop_flag,k,exploring_direction,area_range,handles);
    else
        map(expand_area_by_indx) = submap_shift(expand_area_shift_by_indx);
        map(start_node_temp) = 3;
        route = [route(1:k),...
            [expand_area_by_indx(submap_route_in_area_upd)]',...
            route((dest_node_temp_indx+1):length(route))];
        k = k;
    end
end
 
end













% for i=1:length(obsta_x)
%     expand_point = [expand_point ; obsta_x(i)-5 obsta_y(i)-5];
% end