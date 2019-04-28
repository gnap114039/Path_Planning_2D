function [final_dest_node , map , distanceFromStart,time,flag,route,total_distance] = Wall_Following_Setup_Brutal(map,handles,algorithm_selection,force_stop_flag,exploring_direction)


db_scan = handles.Fig.UserData.dbscan;
raw_data = db_scan.raw_data;
raw_data_by_idx = db_scan.raw_data_by_idx;
[nrows, ncols] = size(map);
start_node = find(map==5);
final_dest_node = find(map==6);
%%=== Avoid Weird Input %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if ( (isempty(start_node)) && isempty(final_dest_node) )
    flag = 1;
    start_node = []; final_dest_node = []; map = []; total_distance = [];
    distanceFromStart = []; parent_node = []; time = []; route = [];
    return;
elseif ( (length(start_node)>1) && (length(final_dest_node)>1) )
    flag = 2;
    start_node = []; final_dest_node = []; map = []; total_distance = [];
    distanceFromStart = []; parent_node = []; time = []; route=[];
    return;
elseif  length(start_node)>1
    flag = 3;
    start_node = []; final_dest_node = []; map = []; total_distance = [];
    distanceFromStart = []; parent_node = []; time = []; route=[];
    return;
elseif length(final_dest_node)>1
    flag = 4;
    start_node = []; final_dest_node = []; map = []; total_distance = [];
    distanceFromStart = []; parent_node = []; time = []; route=[];
    return;
elseif isempty(start_node)
    flag = 5;
    start_node = []; final_dest_node = []; map = []; total_distance = [];
    distanceFromStart = []; parent_node = []; time = []; route=[];
    return;
elseif isempty(final_dest_node)
    flag = 6;
    start_node = []; final_dest_node = []; map = []; total_distance = [];
    distanceFromStart = []; parent_node = []; time = []; route=[];
    return;
end





[start_node_knn_1,start_node_knn_2] = ind2sub(size(map),start_node);
start_node_knn = [start_node_knn_1,start_node_knn_2];
route = start_node;
total_distance = 0;

candidate_node_of_rawdata_by_idx = knnsearch(raw_data,start_node_knn);
candidate_node_of_rawdata = raw_data(candidate_node_of_rawdata_by_idx,:);

[start_node_unjudge,dest_node_unjudge,map,distanceFromStart_unjudge,~,flag,route_temp_unjudge] = Wall_Following_Path_Scoring(map,algorithm_selection,force_stop_flag,exploring_direction,candidate_node_of_rawdata,start_node);
map(find(map==3))=2;
% map(raw_data_by_idx) = 11;

if ~isempty(route_temp_unjudge)
    start_node  = start_node_unjudge;
    dest_node = dest_node_unjudge;
    total_distance = total_distance + distanceFromStart_unjudge(dest_node);
    route = [route,route_temp_unjudge];
    [boundary_start_point_i,boundary_start_point_j] = ind2sub(size(map),route(end));
    current_boundary_point = [boundary_start_point_i,boundary_start_point_j];
    raw_data_by_idx(find(raw_data_by_idx==route(end))) = [];
end

tic;
while true
    if isempty(raw_data_by_idx)
        break;
    end
    exploring_node = [current_boundary_point(1),current_boundary_point(2)-1; %%... left
        current_boundary_point(1),current_boundary_point(2)+1; %%... right
        current_boundary_point(1)-1,current_boundary_point(2); %%... up
        current_boundary_point(1)+1,current_boundary_point(2)]; %%... down
    
    exploring_direction_by_idx = sub2ind(size(map),exploring_node(:,1),exploring_node(:,2));
    is_member_of_corresponding_group = ismember(exploring_direction_by_idx,raw_data_by_idx);
    
    
    if is_member_of_corresponding_group(1)
        route = [route,exploring_direction_by_idx(1)];
        raw_data_by_idx(find(raw_data_by_idx==exploring_direction_by_idx(1))) = [];
        current_boundary_point = [current_boundary_point(1),current_boundary_point(2)-1];
        total_distance = total_distance + 1;
    elseif is_member_of_corresponding_group(2)
        route = [route,exploring_direction_by_idx(2)];
        raw_data_by_idx(find(raw_data_by_idx==exploring_direction_by_idx(2))) = [];
        current_boundary_point = [current_boundary_point(1),current_boundary_point(2)+1];
        total_distance = total_distance + 1;
    elseif is_member_of_corresponding_group(3)
        route = [route,exploring_direction_by_idx(3)];
        raw_data_by_idx(find(raw_data_by_idx==exploring_direction_by_idx(3))) = [];
        current_boundary_point = [current_boundary_point(1)-1,current_boundary_point(2)];
        total_distance = total_distance + 1;
    elseif is_member_of_corresponding_group(4)
        route = [route,exploring_direction_by_idx(4)];
        raw_data_by_idx(find(raw_data_by_idx==exploring_direction_by_idx(4))) = [];
        current_boundary_point = [current_boundary_point(1)+1,current_boundary_point(2)];
        total_distance = total_distance + 1;
    elseif length(raw_data_by_idx)>0
        [raw_data_i ,  raw_data_j]= ind2sub(size(map),raw_data_by_idx);
        raw_data = [raw_data_i ,  raw_data_j];
        
        %%%% 加 DBSCAN 判斷是否要進行拼接 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        DistMat = pdist2(raw_data,raw_data); %%... 每一點的相互距離
        Eps = 1.42; %%... DBSCAN探索半徑
        MinPts = 1; %%... DBSCAN探索半徑內的點群密度（幾個點？？）
        raw_data_clust_condition = DBSCAN(DistMat,Eps,MinPts); %%... DBSCAN的分群結果（）
        curr_raw_data_gruop_left = sort(unique(raw_data_clust_condition)); %%... 一共有幾群
        curr_raw_data_clust_amount_left = zeros(length(curr_raw_data_gruop_left),1);%%...
        %         curr_cluster_priority_group_left = zeros(length(curr_boundary_gruop_left),1);
        for i=1:length(curr_raw_data_gruop_left)
            curr_raw_data_clust_amount_left(i) = sum(raw_data_clust_condition==i);
        end
        [curr_raw_data_clust_amount_descend , curr_raw_data_cluster_priority_group] = sort(curr_raw_data_clust_amount_left,'descend');
        
        eliminate_useless_node = find(curr_raw_data_clust_amount_descend<10);
        
        if ~isempty(eliminate_useless_node)
            pending_to_delete = curr_raw_data_cluster_priority_group(eliminate_useless_node);
            for i=1:length(pending_to_delete)
                raw_data(find(raw_data_clust_condition==pending_to_delete(i)),:)=[];
                raw_data_by_idx(find(raw_data_clust_condition==pending_to_delete(i)))=[];
                raw_data_clust_condition(find(raw_data_clust_condition==pending_to_delete(i)))=[];
            end
        end
        
        start_node = route(end);
        [start_node_knn_1,start_node_knn_2] = ind2sub(size(map),start_node);
        start_node_knn = [start_node_knn_1,start_node_knn_2];
        
        if ~isempty(raw_data_by_idx)
            candidate_node_of_rawdata_by_idx = knnsearch(raw_data,start_node_knn);
            candidate_node_of_rawdata = raw_data(candidate_node_of_rawdata_by_idx,:);
            [start_node_unjudge,dest_node_unjudge,map,distanceFromStart_unjudge,~,flag,route_temp_unjudge] = Wall_Following_Path_Scoring(map,algorithm_selection,force_stop_flag,exploring_direction,candidate_node_of_rawdata,start_node);
            map(find(map==3))=2;
            map(raw_data_by_idx) = 11;
            
            if flag==8
                speedup_delete_raw_clustering = db_scan.raw_clust_idx(find(db_scan.raw_data_by_idx==raw_data_by_idx(candidate_node_of_rawdata_by_idx)));
                speedup_delete_idx = db_scan.raw_data_by_idx(find(db_scan.raw_clust_idx==speedup_delete_raw_clustering));
                for i=1:length(speedup_delete_idx)
                    raw_data_by_idx(find(raw_data_by_idx==speedup_delete_idx(i)))=[];
                end
%                 raw_data_by_idx(candidate_node_of_rawdata_by_idx) = [];
                
            else
                start_node  = start_node_unjudge;
                dest_node = dest_node_unjudge;
                total_distance = total_distance + distanceFromStart_unjudge(dest_node);
                route = [route,route_temp_unjudge];
                [boundary_start_point_i,boundary_start_point_j] = ind2sub(size(map),route(end));
                current_boundary_point = [boundary_start_point_i,boundary_start_point_j];
                raw_data_by_idx(find(raw_data_by_idx==route(end))) = [];
            end
        end
    end
end



map(find(map==3))=2;
map(find(map==4))=2;
map(find(map==5))=2;
map(find(map==6))=2;
map(route(end)) = 5;
map(final_dest_node) = 6;
[xxx,yyy] = ind2sub(size(map),route(end));
[~ , ~ , map , ~ , ~, time , ~ ,route_temp,total_distance] = Path_Scoring(map,algorithm_selection,force_stop_flag,exploring_direction)
route = [route,route_temp];




time = toc;
flag = [];
distanceFromStart = [];
map(db_scan.raw_data_by_idx) = 11;
map(find(map==3))=2;
map(find(map==4))=2;


end