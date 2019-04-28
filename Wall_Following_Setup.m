function [final_dest_node , map , distanceFromStart,time,flag,route,total_distance] = Wall_Following_Setup(map,handles,algorithm_selection,force_stop_flag,exploring_direction)

db_scan = handles.Fig.UserData.dbscan;
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
% dest_node = find(map==6); % Initialize distance array
how_many_group = length(db_scan.cluster_priority_group); %%... 目前進行的邊界路徑規劃
target_point_idx = zeros(length(db_scan.cluster_priority_group),1); %%... 存的是相對應點群裡面最靠近點的序號
total_distance = 0;



tic;
for pending_investigave_group = 1:how_many_group %%... 每一群節點的開始設定/前一群結束後下一群的開始設定
%     pending_investigave_group
    corresponding_boundarygroup_rawdata = db_scan.raw_data(find(db_scan.raw_clust_idx==db_scan.cluster_priority_group(pending_investigave_group)),:); %%... 目前邊界的rawdata（由dbscan.raw_data提供）
    target_point_idx(pending_investigave_group) = knnsearch(corresponding_boundarygroup_rawdata,start_node_knn);
    target_point = corresponding_boundarygroup_rawdata(target_point_idx(pending_investigave_group),:);%%...候選點（目前終點，candidate_node）
    [start_node_unjudge , dest_node_unjudge , map , distanceFromStart , ~  , flag , route_temp_unjudge] = Wall_Following_Path_Scoring(map,algorithm_selection,force_stop_flag,exploring_direction,target_point,start_node);
    map(find(map==3))=2; %%%%%%%%%%!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
%     map(db_scan.raw_data_by_idx) = 11;
    
    if pending_investigave_group==how_many_group
        flag=[];
    end
    
    if ~isempty(route_temp_unjudge)
        start_node  = start_node_unjudge;
        dest_node = dest_node_unjudge;
        total_distance = total_distance + distanceFromStart(dest_node);
        route = [route,route_temp_unjudge];
        [boundary_start_point_i,boundary_start_point_j] = ind2sub(size(map),route(end));
        current_boundary_point = [boundary_start_point_i,boundary_start_point_j];
        current_boundary_group_by_idx = db_scan.raw_data_by_idx(find(db_scan.raw_clust_idx==db_scan.cluster_priority_group(pending_investigave_group)));
        
        %%!#$@%^#&*$(!@#$%^&*()!@#$%^&*()!@#$%^&*()
        
        current_boundary_group_by_idx(find(current_boundary_group_by_idx==route(end))) = [];
        
        % for pending_investigave_group=1:length
        while true %%... 同一群一直探索
            if isempty(current_boundary_group_by_idx)
                break;
            end
            exploring_node = [current_boundary_point(1),current_boundary_point(2)-1; %%... left
                current_boundary_point(1),current_boundary_point(2)+1; %%... right
                current_boundary_point(1)-1,current_boundary_point(2); %%... up
                current_boundary_point(1)+1,current_boundary_point(2)]; %%... down
            
            exploring_direction_by_idx = sub2ind(size(map),exploring_node(:,1),exploring_node(:,2));
            is_member_of_corresponding_group = ismember(exploring_direction_by_idx,current_boundary_group_by_idx);
            
            
            %%=== 路徑拼接，同時將那一類已拼接的點刪除%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if is_member_of_corresponding_group(1)
                route = [route,exploring_direction_by_idx(1)];
                current_boundary_group_by_idx(find(current_boundary_group_by_idx==exploring_direction_by_idx(1))) = [];
                current_boundary_point = [current_boundary_point(1),current_boundary_point(2)-1];
                total_distance = total_distance + 1;
            elseif is_member_of_corresponding_group(2)
                route = [route,exploring_direction_by_idx(2)];
                current_boundary_group_by_idx(find(current_boundary_group_by_idx==exploring_direction_by_idx(2))) = [];
                current_boundary_point = [current_boundary_point(1),current_boundary_point(2)+1];
                total_distance = total_distance + 1;
            elseif is_member_of_corresponding_group(3)
                route = [route,exploring_direction_by_idx(3)];
                current_boundary_group_by_idx(find(current_boundary_group_by_idx==exploring_direction_by_idx(3))) = [];
                current_boundary_point = [current_boundary_point(1)-1,current_boundary_point(2)];
                total_distance = total_distance + 1;
            elseif is_member_of_corresponding_group(4)
                route = [route,exploring_direction_by_idx(4)];
                current_boundary_group_by_idx(find(current_boundary_group_by_idx==exploring_direction_by_idx(4))) = [];
                current_boundary_point = [current_boundary_point(1)+1,current_boundary_point(2)];
                total_distance = total_distance + 1;
            elseif length(current_boundary_group_by_idx)>0 %%... 同一群另一個地方還有節點還沒探索
                
                [current_boundary_group_i ,  current_boundary_group_j]= ind2sub(size(map),current_boundary_group_by_idx);
                current_boundary_group = [current_boundary_group_i ,  current_boundary_group_j];
                
                %%%% 加 DBSCAN 判斷是否要進行拼接 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                DistMat = pdist2(current_boundary_group,current_boundary_group); %%... 每一點的相互距離
                Eps = 1.42; %%... DBSCAN探索半徑
                MinPts = 1; %%... DBSCAN探索半徑內的點群密度（幾個點？？）
                curr_group_clust_condition = DBSCAN(DistMat,Eps,MinPts); %%... DBSCAN的分群結果（）
                curr_boundary_gruop_left = sort(unique(curr_group_clust_condition)); %%... 一共有幾群
                curr_clust_amount_left = zeros(length(curr_boundary_gruop_left),1);%%...
                curr_cluster_priority_group_left = zeros(length(curr_boundary_gruop_left),1);
                for i=1:length(curr_boundary_gruop_left)
                    curr_clust_amount_left(i) = sum(curr_group_clust_condition==i);
                end
                [curr_clust_amount_descend , curr_cluster_priority_group] = sort(curr_clust_amount_left,'descend');
                
                eliminate_useless_node = find(curr_clust_amount_descend<10);
                if ~isempty(eliminate_useless_node)
                    pending_to_delete = curr_cluster_priority_group(eliminate_useless_node);
                    for i=1:length(pending_to_delete)
                        current_boundary_group(find(curr_group_clust_condition==pending_to_delete(i)),:)=[];
                        curr_group_clust_condition(find(curr_group_clust_condition==pending_to_delete(i)))=[];
                    end
                end
                
                current_boundary_group_by_idx = sub2ind(size(map),current_boundary_group(:,1),current_boundary_group(:,2));
                
                if ~isempty(current_boundary_group_by_idx)
                    curr_bundry_group_nxt_closest_point_idx = knnsearch(current_boundary_group,current_boundary_point);
                    [target_point_i,target_point_j] = ind2sub(size(map),current_boundary_group_by_idx(curr_bundry_group_nxt_closest_point_idx));
                    target_point = [target_point_i,target_point_j];
                    [start_node , dest_node  , distanceFromStart , ~ , flag , route_temp] = Wall_Following_Path_Scoring_SameGroup(map,algorithm_selection,force_stop_flag,exploring_direction,target_point,route(end));
                    if flag==8
                        current_boundary_group_by_idx(curr_bundry_group_nxt_closest_point_idx) = [];
                    else
                        current_boundary_group_by_idx(find(current_boundary_group_by_idx==route(end))) = [];
                        route = [route,route_temp];
                        [current_boundary_point(1) , current_boundary_point(2)] = ind2sub(size(map),route_temp(end));
                        current_boundary_group_by_idx(curr_bundry_group_nxt_closest_point_idx) = [];
                        total_distance = total_distance + distanceFromStart(dest_node);
                    end
                end
                
            end
            
        end   
        start_node = route(end);
        [start_node_knn_1,start_node_knn_2] = ind2sub(size(map),start_node);
        start_node_knn = [start_node_knn_1,start_node_knn_2];
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




time=toc;
map(db_scan.raw_data_by_idx) = 11;
map(find(map==3))=2;
map(find(map==4))=2;






end