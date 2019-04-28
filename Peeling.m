function [dest_node , map , distanceFromStart,time,flag,route,total_distance] = Peeling(map,handles,algorithm_selection,exploring_direction,force_stop_flag)
time = 0;
db_scan = handles.Fig.UserData.dbscan;
loop_count=0;
stop_flag = 0;
black_map_with_11 = [];
total_distance = 0;
for i=1:length(db_scan.cluster_priority_group)

    current_corres_data_point = db_scan.raw_data(find(db_scan.raw_clust_idx==i),:);
    [x_min_value,x_min_idx] = min(current_corres_data_point(:,1));
    [x_max_value,x_max_idx] = max(current_corres_data_point(:,1));
    [y_min_value,y_min_idx] = min(current_corres_data_point(:,2));
    [y_max_value,y_max_idx] = max(current_corres_data_point(:,2));
    surface_area_estim = (x_max_value - x_min_value)*(y_max_value-y_min_value);
    data_group_rearrange{i,1} = current_corres_data_point;
    data_group_rearrange{i,2} = surface_area_estim;

end

[nrows, ncols] = size(map);
start_node = find(map==5); 
final_node = start_node;
if isempty(start_node)
    flag = 5;
    return;
end


[unconfirm_peeling_candidate_surface , uncomfirm_peeling_candidate_group] = sort(cell2mat(data_group_rearrange(:,2)),'descend'); %%% �٨S�T�w�̤j�s�O�_��Ϸ�U���_�l�I��F
[start_node_knn_1,start_node_knn_2] = ind2sub(size(map),start_node);
start_node_knn = [start_node_knn_1,start_node_knn_2];
route = start_node;



for i=1:length(uncomfirm_peeling_candidate_group) %%% �T�w�̤j�s�j��A�P�ɧP�_�{�b���_�l�I�O�_���F���j��O�_��
    uncomfirm_db_scan_peel_data = db_scan.raw_data(find(db_scan.raw_clust_idx==uncomfirm_peeling_candidate_group(i)),:);
    target_point_idx = knnsearch(uncomfirm_db_scan_peel_data,start_node_knn);
    target_point = uncomfirm_db_scan_peel_data(target_point_idx,:);
    [start_node_unjudge , dest_node_unjudge , map , distanceFromStart , ~  , flag , route_temp_unjudge] = Wall_Following_Path_Scoring(map,algorithm_selection,force_stop_flag,exploring_direction,target_point,start_node);
    map(find(map==3))=2;
    map(find(map==4))=2;
    if ~isempty(route_temp_unjudge)
        db_scan_peel_data = uncomfirm_db_scan_peel_data;
        db_scan_peel_data_by_idx = db_scan.raw_data_by_idx(find(db_scan.raw_clust_idx==uncomfirm_peeling_candidate_group(i)));
        keep_wallfollowing_data_for_update = db_scan_peel_data_by_idx; %%%�ΨӪ��s�@�h�ܶ¥Ϊ�
        db_scan_notwall_data_by_idx = db_scan.raw_data_by_idx;
        
        for i=1:length(db_scan_peel_data_by_idx)
            db_scan_notwall_data_by_idx(find(db_scan_notwall_data_by_idx == db_scan_peel_data_by_idx(i)))=[];
        end

        map(db_scan_notwall_data_by_idx) = 2;
        begin_map = map;

        start_node  = start_node_unjudge;
        dest_node = dest_node_unjudge;
        total_distance = total_distance + distanceFromStart(dest_node);
        route = [route,route_temp_unjudge];
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
        

        break;
    end
end





while true
%     if isempty(db_scan_peel_data_by_idx)
%         break;
%     end

%     length_of_current_data = length(db_scan_peel_data_by_idx)
    exploring_node = [current_boundary_point(1),current_boundary_point(2)-1; %%... left
        current_boundary_point(1),current_boundary_point(2)+1; %%... right
        current_boundary_point(1)-1,current_boundary_point(2); %%... up
        current_boundary_point(1)+1,current_boundary_point(2)]; %%... down
    
    exploring_direction_by_idx = sub2ind(size(map),exploring_node(:,1),exploring_node(:,2));
    is_member_of_corresponding_group = ismember(exploring_direction_by_idx,db_scan_peel_data_by_idx);
    is_member_of_boundary_closeloop_node = ismember(current_boundary_point_by_idx,b4_route_end_node_by_idx);
    
    
    if is_member_of_corresponding_group(1)
        route = [route,exploring_direction_by_idx(1)];
        db_scan_peel_data_by_idx(find(db_scan_peel_data_by_idx==exploring_direction_by_idx(1))) = [];
        current_boundary_point = [current_boundary_point(1),current_boundary_point(2)-1];
        current_boundary_point_by_idx = sub2ind(size(map),current_boundary_point(1),current_boundary_point(2));
        total_distance = total_distance + 1;
    elseif is_member_of_corresponding_group(2)
        route = [route,exploring_direction_by_idx(2)];
        db_scan_peel_data_by_idx(find(db_scan_peel_data_by_idx==exploring_direction_by_idx(2))) = [];
        current_boundary_point = [current_boundary_point(1),current_boundary_point(2)+1];
        current_boundary_point_by_idx = sub2ind(size(map),current_boundary_point(1),current_boundary_point(2));
        total_distance = total_distance + 1;
    elseif is_member_of_corresponding_group(3)
        route = [route,exploring_direction_by_idx(3)];
        db_scan_peel_data_by_idx(find(db_scan_peel_data_by_idx==exploring_direction_by_idx(3))) = [];
        current_boundary_point = [current_boundary_point(1)-1,current_boundary_point(2)];
        current_boundary_point_by_idx = sub2ind(size(map),current_boundary_point(1),current_boundary_point(2));
        total_distance = total_distance + 1;
    elseif is_member_of_corresponding_group(4)
        route = [route,exploring_direction_by_idx(4)];
        db_scan_peel_data_by_idx(find(db_scan_peel_data_by_idx==exploring_direction_by_idx(4))) = [];
        current_boundary_point = [current_boundary_point(1)+1,current_boundary_point(2)];
        current_boundary_point_by_idx = sub2ind(size(map),current_boundary_point(1),current_boundary_point(2));
        total_distance = total_distance + 1;
    elseif isempty(db_scan_peel_data_by_idx) %%% �S�I�F
        [map,db_scan_peel_data_by_idx,current_boundary_point,current_boundary_point_by_idx,b4_route_end_node_by_idx,route,black_map_with_11,total_distance,keep_wallfollowing_data_for_update,stop_flag] = Peeling_Child_Func(map,route,route(end),algorithm_selection,exploring_direction,force_stop_flag,total_distance,black_map_with_11,keep_wallfollowing_data_for_update);
%         loop_count = loop_count+1



    elseif ~isempty(db_scan_peel_data_by_idx) && is_member_of_boundary_closeloop_node %%%�٦��I,�w��
        [db_scan_peel_data_i ,  db_scan_peel_data_j]= ind2sub(size(map),db_scan_peel_data_by_idx);
        db_scan_peel_data = [db_scan_peel_data_i ,  db_scan_peel_data_j];
        
        %%%% �[ DBSCAN �P�_�O�_�n�i����� %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        DistMat = pdist2(db_scan_peel_data,db_scan_peel_data); %%... �C�@�I���ۤ��Z��
        Eps = 1.42; %%... DBSCAN�����b�|
        MinPts = 1; %%... DBSCAN�����b�|�����I�s�K�ס]�X���I�H�H�^
        curr_group_clust_condition = DBSCAN(DistMat,Eps,MinPts); %%... DBSCAN�����s���G�]�^
        curr_boundary_gruop_left = sort(unique(curr_group_clust_condition)); %%... �@�@���X�s
        curr_clust_amount_left = zeros(length(curr_boundary_gruop_left),1);%%...
        curr_cluster_priority_group_left = zeros(length(curr_boundary_gruop_left),1);
        for i=1:length(curr_boundary_gruop_left)
            curr_clust_amount_left(i) = sum(curr_group_clust_condition==i);
        end
        [curr_clust_amount_descend , curr_cluster_priority_group] = sort(curr_clust_amount_left,'descend');
        eliminate_useless_node = find(curr_clust_amount_descend<40);
        if ~isempty(eliminate_useless_node)
            pending_to_delete = curr_cluster_priority_group(eliminate_useless_node);
            for i=1:length(pending_to_delete)
                db_scan_peel_data(find(curr_group_clust_condition==pending_to_delete(i)),:)=[];
                curr_group_clust_condition(find(curr_group_clust_condition==pending_to_delete(i)))=[];
            end
        end
        
        db_scan_peel_data_by_idx = sub2ind(size(map),db_scan_peel_data(:,1),db_scan_peel_data(:,2));
        
        if ~isempty(db_scan_peel_data_by_idx)
            curr_bundry_group_nxt_closest_point_idx = knnsearch(db_scan_peel_data,current_boundary_point);
            [target_point_i,target_point_j] = ind2sub(size(map),db_scan_peel_data_by_idx(curr_bundry_group_nxt_closest_point_idx));
            target_point = [target_point_i,target_point_j];
            [start_node , dest_node  , distanceFromStart , ~ , flag , route_temp] = Wall_Following_Path_Scoring_SameGroup(map,algorithm_selection,force_stop_flag,exploring_direction,target_point,route(end));
            if flag==8
                db_scan_peel_data_by_idx(curr_bundry_group_nxt_closest_point_idx) = [];
            else
                db_scan_peel_data_by_idx(find(db_scan_peel_data_by_idx==route(end))) = [];
                route = [route,route_temp];
                [current_boundary_point(1) , current_boundary_point(2)] = ind2sub(size(map),route_temp(end));
                db_scan_peel_data_by_idx(curr_bundry_group_nxt_closest_point_idx) = [];
                total_distance = total_distance + distanceFromStart(dest_node);
            end
        else
            [map,db_scan_peel_data_by_idx,current_boundary_point,current_boundary_point_by_idx,b4_route_end_node_by_idx,route,black_map_with_11,total_distance,keep_wallfollowing_data_for_update,stop_flag] = Peeling_Child_Func(map,route,route(end),algorithm_selection,exploring_direction,force_stop_flag,total_distance,black_map_with_11,keep_wallfollowing_data_for_update);
        end

%         [map,db_scan_peel_data_by_idx,current_boundary_point,current_boundary_point_by_idx,b4_route_end_node_by_idx,route,black_map_with_11,total_distance,keep_wallfollowing_data_for_update,stop_flag] = Peeling_Child_Func(map,route,route(end),algorithm_selection,exploring_direction,force_stop_flag,total_distance,black_map_with_11,keep_wallfollowing_data_for_update);
%         loop_count = loop_count+1

    elseif ~isempty(db_scan_peel_data_by_idx) && ~is_member_of_boundary_closeloop_node %%%�٦��I,�S��
        [db_scan_peel_data_i ,  db_scan_peel_data_j]= ind2sub(size(map),db_scan_peel_data_by_idx);
        db_scan_peel_data = [db_scan_peel_data_i ,  db_scan_peel_data_j];

         %%%% �[ DBSCAN �P�_�O�_�n�i����� %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        DistMat = pdist2(db_scan_peel_data,db_scan_peel_data); %%... �C�@�I���ۤ��Z��
        Eps = 1.42; %%... DBSCAN�����b�|
        MinPts = 1; %%... DBSCAN�����b�|�����I�s�K�ס]�X���I�H�H�^
        curr_group_clust_condition = DBSCAN(DistMat,Eps,MinPts); %%... DBSCAN�����s���G�]�^
        curr_boundary_gruop_left = sort(unique(curr_group_clust_condition)); %%... �@�@���X�s
        curr_clust_amount_left = zeros(length(curr_boundary_gruop_left),1);%%...
        curr_cluster_priority_group_left = zeros(length(curr_boundary_gruop_left),1);
        for i=1:length(curr_boundary_gruop_left)
            curr_clust_amount_left(i) = sum(curr_group_clust_condition==i);
        end
        [curr_clust_amount_descend , curr_cluster_priority_group] = sort(curr_clust_amount_left,'descend');
        eliminate_useless_node = find(curr_clust_amount_descend< 40);
        if ~isempty(eliminate_useless_node)
            pending_to_delete = curr_cluster_priority_group(eliminate_useless_node);
            for i=1:length(pending_to_delete)
                db_scan_peel_data(find(curr_group_clust_condition==pending_to_delete(i)),:)=[];
                curr_group_clust_condition(find(curr_group_clust_condition==pending_to_delete(i)))=[];
            end
        end
        
        db_scan_peel_data_by_idx = sub2ind(size(map),db_scan_peel_data(:,1),db_scan_peel_data(:,2));
        
        
        if ~isempty(db_scan_peel_data_by_idx)
            same_group_target_point_idx = knnsearch(db_scan_peel_data,current_boundary_point);
            target_point = db_scan_peel_data(same_group_target_point_idx,:);
            [start_node , dest_node  , distanceFromStart , ~ , flag , route_temp] = Wall_Following_Path_Scoring_SameGroup(map,algorithm_selection,force_stop_flag,exploring_direction,target_point,route(end));
            map(find(map==3))=2;
            map(find(map==4))=2;
            if flag==8
                db_scan_peel_data_by_idx(same_group_target_point_idx) = [];
            else
                db_scan_peel_data_by_idx(find(db_scan_peel_data_by_idx==route(end))) = [];
                route = [route,route_temp];
                [current_boundary_point(1) , current_boundary_point(2)] = ind2sub(size(map),route_temp(end));
                db_scan_peel_data_by_idx(same_group_target_point_idx) = [];
                total_distance = total_distance + distanceFromStart(dest_node);
            end
        end
    end
%     if loop_count==3
%         break;
%     end    
    
    if stop_flag==2
        break;
    end

end

[target_point(1),target_point(2)] = ind2sub(size(map),final_node);
start_node = route(end);
[start_node_unjudge , dest_node_unjudge , map , distanceFromStart , ~  , flag , route_temp_unjudge] = Wall_Following_Path_Scoring(map,algorithm_selection,force_stop_flag,exploring_direction,target_point,start_node);
map(find(map==5)) = 2;
map(find(map==3))=2;
map(find(map==4))=2;
route = [route,route_temp_unjudge];








end