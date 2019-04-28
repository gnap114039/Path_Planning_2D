function Browse_Image(Browse_Button,data)


handles = get(Browse_Button,'UserData');
refresh_flag = Browse_Button.String;
boundary_flag = handles.Boundary_Detection_Check.Value;



if isempty(str2num(handles.Dangerous_Zone_Edit.String)) && isempty(str2num(handles.Save_Zone_Edit.String))
    dangerouszone_range = 5;
    savezone_range = 7;
elseif ~isempty(str2num(handles.Dangerous_Zone_Edit.String)) && isempty(str2num(handles.Save_Zone_Edit.String))
    dangerouszone_range = 1+2*str2num(handles.Dangerous_Zone_Edit.String);
    savezone_range = 7;
elseif isempty(str2num(handles.Dangerous_Zone_Edit.String)) && ~isempty(str2num(handles.Save_Zone_Edit.String))
    dangerouszone_range = 5;
    savezone_range = 1+2*str2num(handles.Save_Zone_Edit.String);
else
    dangerouszone_range = 1+2*str2num(handles.Dangerous_Zone_Edit.String);
    savezone_range = 1+2*str2num(handles.Save_Zone_Edit.String);
end



%%=== browse-->cancle || browse-->image || refresh %%%%%%%%%%%%%%%%%%%%%%%%%%         
if strcmp(refresh_flag,handles.Browse_Button.String) %%... refresh_flag 有可能是 Browse_Button 或是 Refresh_Button
    [filename, Pathname] = uigetfile({'*.jpg';'*.png'},'Select A Map'); %%... will popup a window for selecting a browse path
    if (filename==0)
        raw_ori_map = handles.Img.CData;
        return;
    else
        set(handles.Img,'CData',[]); %%... clean the previous data
        set(handles.Path_Arrow,'XData',[],'YData',[]);
        handles.Dangerous_Zone_Edit.UserData = {Pathname,filename};
        raw_ori_map = imread([handles.Dangerous_Zone_Edit.UserData{1},handles.Dangerous_Zone_Edit.UserData{2}]); %%... reading the raw map by corresponding path and filename
    end
else
    set(handles.Img,'CData',[]); %%... clean the previous data
    set(handles.Path_Arrow,'XData',[],'YData',[]);
    raw_ori_map = imread([handles.Dangerous_Zone_Edit.UserData{1},handles.Dangerous_Zone_Edit.UserData{2}]); %%... reading the raw map by corresponding path and filename
end





%%=== To Filter All Color Map Into Black and White
if (size(raw_ori_map, 3) ==3 )
    map_colorpass = rgb2gray(raw_ori_map);
    raw_ori_map = (map_colorpass > 200);
end

%%=== To Get Map Dangerous Zone and Save Zone %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
index_reverse_raw_ori_map =~ raw_ori_map;

se_dangerous = strel('rectangle',[dangerouszone_range dangerouszone_range]);
se_save = strel('rectangle',[savezone_range savezone_range]);
se_boundary = strel('rectangle',[3 3]);

dilated_bwhite_dangerous = imdilate( index_reverse_raw_ori_map , se_dangerous );
dilated_bwhite_save = imdilate( dilated_bwhite_dangerous , se_save );
dilated_bwhite_boundary = imdilate( dilated_bwhite_save , se_boundary );

dangerouszone_scaling = 5;
dangerouszone_img = ~dilated_bwhite_dangerous;
dangerouszone_area = logical(raw_ori_map) - dangerouszone_img;
dangerouszone_area = dangerouszone_scaling*dangerouszone_area;

savezone_scalling = 6;
savezone_img = ~dilated_bwhite_save;
savezone_area = dangerouszone_img - savezone_img;
savezone_area = savezone_scalling*savezone_area;

if boundary_flag
    boundary_scalling = 9;
    boundary_img = ~dilated_bwhite_boundary;
    boundary_area = savezone_img - boundary_img;
    boundary_area = boundary_scalling*boundary_area;
    map = 1+logical(raw_ori_map)+dangerouszone_area+savezone_area+boundary_area;
else
    map = 1+logical(raw_ori_map)+dangerouszone_area+savezone_area;
end

%%=== Final Starting Map %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
map_init = map;
set(handles.Img,'CData',map);
if ~isempty(handles.Dangerous_Zone_Edit.UserData)
    set(handles.Browse_Info,'String',sprintf('\n%s',handles.Dangerous_Zone_Edit.UserData{2}));
% else
%     set(handles.Browse_Info,'String',sprintf('\n%s',handles.Dangerous_Zone_Edit.UserData{2}));
end
parameters = handles.Fig.UserData;
parameters.map_init = map_init;
parameters.map_current = map_init;
handles.Fig.UserData = parameters;
% assignin('base','map_init',map_init); %%... to keep the original map and using by @Reset_Path
% assignin('base','boundary_pixel',boundary_pixel);



%%=== DBSCAN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
boundary_pixel = find(map==11);
[boundary_detect_i, boundary_detect_j] = ind2sub(size(map_init),boundary_pixel);
boundary_detect = [boundary_detect_i, boundary_detect_j];
DistMat = pdist2(boundary_detect,boundary_detect); %%... 每一點的相互距離
Eps = 1.42; %%... DBSCAN探索半徑
MinPts = 1; %%... DBSCAN探索半徑內的點群密度（幾個點？？）
Clust = DBSCAN(DistMat,Eps,MinPts); %%... DBSCAN的分群結果（）
boundary_gruop = sort(unique(Clust)); %%... 一共有幾群
clust_amount = zeros(length(boundary_gruop),1);%%... 
cluster_priority_group = zeros(length(boundary_gruop),1);
for i=1:length(boundary_gruop)
    clust_amount(i) = sum(Clust==i);
end
dbscan.raw_data = boundary_detect;
dbscan.raw_data_by_idx = boundary_pixel;
dbscan.raw_clust_idx = Clust;
[dbscan.clust_amount_descend , dbscan.cluster_priority_group] = sort(clust_amount,'descend');
parameters = handles.Fig.UserData;
parameters.dbscan = dbscan;
handles.Fig.UserData = parameters;
set(handles.Fig,'UserData',parameters);
    

end




% boundary_gruop = sort(unique(Clust));
% asdf = [];
% qwer = [];
% for i=1:length(boundary_gruop)
%     current_descending_idx = find(clust_amount==clust_amount_descend(i));
%     if length(current_descending_idx) == 1
%         cluster_priority_group(i) = current_descending_idx;
%     else
%         for j=1:length(current_descending_idx)
%             asd = find(cluster_priority_group==current_descending_idx(j));
%             asdf = [asdf;asd];
%         end
%         if isempty(asdf)
%             cluster_priority_group(i) = current_descending_idx(1);
%         else
%             for k=1:length(asdf)
%                 qwe = find(current_descending_idx==cluster_priority_group(asdf(k)))
%                 qwer = [qwer;qwe];
%             end
%             current_descending_idx(qwer) = 0;
%             cluster_priority_group(i) = max(current_descending_idx);
%             asdf = [];
%             qwer = [];
%         end
%     end
% end









