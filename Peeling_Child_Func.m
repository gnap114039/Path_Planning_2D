function [map,db_scan_peel_data_by_idx,current_boundary_point,current_boundary_point_by_idx,b4_route_end_node_by_idx,route,black_map_with_11,total_distance,keep_wallfollowing_data_for_update,stop_flag] = Peeling_Child_Func(map,route,start_node,algorithm_selection,exploring_direction,force_stop_flag,total_distance,black_map_with_11,keep_wallfollowing_data_for_update)


if  isempty(black_map_with_11)
    modifying_map = map;
    modifying_map(find(modifying_map==2)) = 0; %%% 把已經做的邊界變黑色
    logical_modifying_map = 99*logical(modifying_map); %%% 障礙物全部黑色的地圖
    
    dilation_index = keep_wallfollowing_data_for_update;
    wall_line_map = zeros(size(map));
    wall_line_map(dilation_index) = 1; %%% 把已經做的邊界變黑色
%     wall_line_map(find(wall_line_map==2))=0;
    logical_wall_line_map = logical(wall_line_map); %%% 只有 0 和 wall 的 1
else
    logical_modifying_map = 99*logical(black_map_with_11);
    
    dilation_index = find(black_map_with_11==11);
    wall_line_map = zeros(size(map));
    wall_line_map(dilation_index) = 1; %%% 把已經做的邊界變黑色
    logical_wall_line_map = logical(wall_line_map); %%% 只有 0 和 wall 的 1
end


db_scan_peel_data_by_idx = [];

peeling_width = 9;
se_peelingwidth = strel('rectangle',[peeling_width peeling_width]);
peeling_map_width = imdilate(logical_wall_line_map,se_peelingwidth);


peeling_layer = 3;
se_peelinglayer = strel('rectangle',[peeling_layer peeling_layer]);
peeling_map_layer = imdilate(peeling_map_width,se_peelinglayer);


three_logic_map_addition = logical_modifying_map + peeling_map_width + peeling_map_layer;
db_scan_peel_data_by_idx = find(three_logic_map_addition==1);

if isempty(db_scan_peel_data_by_idx)
    stop_flag=2;
    db_scan_peel_data_by_idx = []; current_boundary_point=[]; current_boundary_point_by_idx=[];b4_route_end_node_by_idx=[];
    return;
else
    stop_flag=0;
end

%%%% 判?是否?停

%%%%%%%


black_map_with_11 = three_logic_map_addition;
black_map_with_11 = logical(black_map_with_11);
black_map_with_11 = double(black_map_with_11);
black_map_with_11(db_scan_peel_data_by_idx)=11;

[db_scan_peel_data_x,db_scan_peel_data_y] = ind2sub(size(map),db_scan_peel_data_by_idx);
db_scan_peel_data = [db_scan_peel_data_x,db_scan_peel_data_y];

[start_node_knn_1,start_node_knn_2] = ind2sub(size(map),start_node);
start_node_knn = [start_node_knn_1,start_node_knn_2];
target_point_idx = knnsearch(db_scan_peel_data,start_node_knn);
target_point = db_scan_peel_data(target_point_idx,:);
[start_node_unjudge , dest_node_unjudge , map , distanceFromStart , ~  , flag , route_temp_unjudge] = Wall_Following_Path_Scoring(map,algorithm_selection,force_stop_flag,exploring_direction,target_point,start_node);
map(find(map==3))=2;
map(find(map==4))=2;

route = [route,route_temp_unjudge];
total_distance = total_distance + distanceFromStart(dest_node_unjudge);
[finish_index(1),finish_index(2)] = ind2sub(size(map),route(end));
b4_route_end_node = [finish_index(1),finish_index(2)-1; %%... left
    finish_index(1),finish_index(2)+1; %%... right
    finish_index(1)-1,finish_index(2); %%... up
    finish_index(1)+1,finish_index(2); %%... down
    finish_index(1)-1,finish_index(2)-1; %%... leftup
    finish_index(1)-1,finish_index(2)+1; %%... rightup
    finish_index(1)+1,finish_index(2)-1; %%... leftdown
    finish_index(1)+1,finish_index(2)+1]; %%... rightdown
b4_route_end_node_by_idx = sub2ind(size(map),b4_route_end_node(:,1),b4_route_end_node(:,2));
[boundary_start_point_i,boundary_start_point_j] = ind2sub(size(map),route(end));
current_boundary_point = [boundary_start_point_i,boundary_start_point_j];
current_boundary_point_by_idx = sub2ind(size(map),current_boundary_point(1),current_boundary_point(2));
db_scan_peel_data_by_idx(find(db_scan_peel_data_by_idx==route(end))) = [];




















% 
% peeling_map_boundary = 11*(peeling_map_width - logical_wall_map);
% new_begin_map_b4_dbscan = peeling_map_boundary + logical_wall_map;
% 
% %%=== DBSCAN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% boundary_pixel = find(new_begin_map_b4_dbscan==11);
% [boundary_detect_i, boundary_detect_j] = ind2sub(size(new_begin_map_b4_dbscan),boundary_pixel);
% boundary_detect = [boundary_detect_i, boundary_detect_j];
% DistMat = pdist2(boundary_detect,boundary_detect); %%... 每一點的相互距離
% Eps = 1.42; %%... DBSCAN探索半徑
% MinPts = 1; %%... DBSCAN探索半徑內的點群密度（幾個點？？）
% Clust = DBSCAN(DistMat,Eps,MinPts); %%... DBSCAN的分群結果（）
% boundary_gruop = sort(unique(Clust)); %%... 一共有幾群
% clust_amount = zeros(length(boundary_gruop),1);%%... 
% cluster_priority_group = zeros(length(boundary_gruop),1);
% for i=1:length(boundary_gruop)
%     clust_amount(i) = sum(Clust==i);
% end
% dbscan.raw_data = boundary_detect;
% dbscan.raw_data_by_idx = boundary_pixel;
% dbscan.raw_clust_idx = Clust;
% [dbscan.clust_amount_descend , dbscan.cluster_priority_group] = sort(clust_amount,'descend');
% 
% for i=1:length(dbscan.cluster_priority_group) %%%%% 群的節點數量與面積
% 
%     current_corres_data_point = dbscan.raw_data(find(dbscan.raw_clust_idx==i),:);
%     [x_min_value,x_min_idx] = min(current_corres_data_point(:,1));
%     [x_max_value,x_max_idx] = max(current_corres_data_point(:,1));
%     [y_min_value,y_min_idx] = min(current_corres_data_point(:,2));
%     [y_max_value,y_max_idx] = max(current_corres_data_point(:,2));
%     surface_area_estim = (x_max_value - x_min_value)*(y_max_value-y_min_value);
%     data_group_rearrange{i,1} = current_corres_data_point;
%     data_group_rearrange{i,2} = surface_area_estim;
% 
% end
% 
% 
% [peeling_candidate_value,peeling_candidate_group] = max(cell2mat(data_group_rearrange(:,2)));%%% 找個群眾的最大面積
% db_scan_peel_data = dbscan.raw_data(find(dbscan.raw_clust_idx==peeling_candidate_group),:);%%% wall following 點群候選節點【二維坐標點】
% db_scan_peel_data_by_idx = dbscan.raw_data_by_idx(find(dbscan.raw_clust_idx==peeling_candidate_group));%%% wall following 點群候選節點【以index表示】
% keep_wallfollowing_data_for_update = db_scan_peel_data_by_idx; %%%用來長新一層變黑用的
% db_scan_notwall_data_by_idx = dbscan.raw_data_by_idx;
% 
% for i=1:length(db_scan_peel_data_by_idx) 
%     db_scan_notwall_data_by_idx(find(db_scan_notwall_data_by_idx == db_scan_peel_data_by_idx(i)))=[];
% end
% 
% 
% new_begin_map_b4_dbscan(db_scan_notwall_data_by_idx) = 0;%%% 給下一次回圈用
% map(db_scan_notwall_data_by_idx) = 2; %%% 更新後要傳出去function
% map(db_scan_peel_data_by_idx)=11; %%%把這一次的wall 填色
% [start_node_knn_1,start_node_knn_2] = ind2sub(size(map),start_node);
% start_node_knn = [start_node_knn_1,start_node_knn_2];
% target_point_idx = knnsearch(db_scan_peel_data,start_node_knn);
% target_point = db_scan_peel_data(target_point_idx,:);
% [start_node_unjudge , dest_node_unjudge , map , distanceFromStart , ~  , flag , route_temp_unjudge] = Wall_Following_Path_Scoring(map,algorithm_selection,force_stop_flag,exploring_direction,target_point,start_node);
% map(find(map==3))=2;
% map(find(map==4))=2;
% 
% 
% if ~isempty(route_temp_unjudge)
%     start_node  = start_node_unjudge;
%     dest_node = dest_node_unjudge;
%     total_distance = total_distance + distanceFromStart(dest_node);
%     route = [route,route_temp_unjudge];
%     [finish_index(1),finish_index(2)] = ind2sub(size(map),route(end));
%     b4_route_end_node = [finish_index(1),finish_index(2)-1; %%... left
%             finish_index(1),finish_index(2)+1; %%... right
%             finish_index(1)-1,finish_index(2); %%... up
%             finish_index(1)+1,finish_index(2); %%... down
%             finish_index(1)-1,finish_index(2)-1; %%... leftup
%             finish_index(1)-1,finish_index(2)+1; %%... rightup
%             finish_index(1)+1,finish_index(2)-1; %%... leftdown
%             finish_index(1)+1,finish_index(2)+1]; %%... rightdown
%     b4_route_end_node_by_idx = sub2ind(size(map),b4_route_end_node(:,1),b4_route_end_node(:,2));    
%     [boundary_start_point_i,boundary_start_point_j] = ind2sub(size(map),route(end));
%     current_boundary_point = [boundary_start_point_i,boundary_start_point_j];
%     current_boundary_point_by_idx = sub2ind(size(map),current_boundary_point(1),current_boundary_point(2));
%     db_scan_peel_data_by_idx(find(db_scan_peel_data_by_idx==route(end))) = [];
% % 
% % if 
% %     stop_flag = 1;
% % end



end