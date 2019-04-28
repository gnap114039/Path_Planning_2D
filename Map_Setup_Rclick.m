function Map_Setup_Rclick(menu,data)

    handles = get(menu,'UserData');  
    map_color_value_old = handles.Fig.UserData.map_color_value;
    map = handles.Img.CData;
    [nrows, ncols] = size(map);
    
    %%=== 右鍵選單觸發的顏色 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    switch menu.Label
        case 'Green'
            map_color_value = 1; 
            handles.Current_Color_Information.String = 'Green Node';
        case 'Yellow'
            map_color_value = 2;
            handles.Current_Color_Information.String = 'Yellow Node';
        case 'White'
            map_color_value = 3;
            handles.Current_Color_Information.String = 'White Node';
        case 'Dynamic Obstacle'
            map_color_value = 4;
            handles.Current_Color_Information.String = 'Dynamic Obstacle';
        case 'Static Obstacle'
            map_color_value = 5;
            handles.Current_Color_Information.String = 'Static Obstacle';
    end
    
    
    parameters = handles.Fig.UserData;
    parameters.map_color_value = map_color_value;
    handles.Fig.UserData = parameters;
    x = handles.Fig.UserData.click_pos;
    if isempty(x)
        return;
    end
%%=== 如果在黑色點上右鍵新的顏色選項，將黑色障礙物的boundary剔除    
    if map_color_value_old == 5
        if map_color_value ~= map_color_value_old
            if isempty(str2num(handles.Dangerous_Zone_Edit.String)) && isempty(str2num(handles.Save_Zone_Edit.String))
                dangerouszone_range = 5;
                savezone_range = 7;
                area_range = dangerouszone_range+savezone_range-1;
            elseif ~isempty(str2num(handles.Dangerous_Zone_Edit.String)) && isempty(str2num(handles.Save_Zone_Edit.String))
                dangerouszone_range = 1+2*str2num(handles.Dangerous_Zone_Edit.String);
                savezone_range = 7;
                area_range = dangerouszone_range+savezone_range-1;
            elseif isempty(str2num(handles.Dangerous_Zone_Edit.String)) && ~isempty(str2num(handles.Save_Zone_Edit.String))
                dangerouszone_range = 5;
                savezone_range = 1+2*str2num(handles.Save_Zone_Edit.String);
                area_range = dangerouszone_range+savezone_range-1;
            else
                dangerouszone_range = 1+2*str2num(handles.Dangerous_Zone_Edit.String);
                savezone_range = 1+2*str2num(handles.Save_Zone_Edit.String);
                area_range = dangerouszone_range+savezone_range-1;
            end
            expand_point = [x(2)-((area_range-1)/2) , x(1)-((area_range-1)/2)];
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
            submap_obstacle = zeros( (max(expand_area(:,1))-min(expand_area(:,1))+1) , (max(expand_area(:,2))-min(expand_area(:,2))+1) );
            expand_area_by_indx = sub2ind(size(map),expand_area(:,1),expand_area(:,2));
            expand_area_shift_by_indx = [linspace(1,length(expand_area_by_indx),length(expand_area_by_indx))]';
            for i = 1:length(expand_area_by_indx)
                if ((map(expand_area_by_indx(i))==7) ||(map(expand_area_by_indx(i))==8))
                    map(expand_area_by_indx(i)) = 2;
                end
            end
        end
    end
    
    
    
    
%%=== 將相對應的格點換成右鍵選單所觸發的顏色 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    switch map_color_value
        case 1
            map(x(2),x(1)) = 5; %%... Green
        case 2
            map(x(2),x(1)) = 6; %%... Yellow
        case 3
            map(x(2),x(1)) = 2; %%... White
        case 4
            map(x(2),x(1)) = 1; %%... Dynamic Obstacle
        case 5
            map(x(2),x(1)) = 1; %%... Static Obstacle
            if isempty(str2num(handles.Dangerous_Zone_Edit.String)) && isempty(str2num(handles.Save_Zone_Edit.String))
                dangerouszone_range = 5;
                savezone_range = 7;
                area_range = dangerouszone_range+savezone_range-1;
            elseif ~isempty(str2num(handles.Dangerous_Zone_Edit.String)) && isempty(str2num(handles.Save_Zone_Edit.String))
                dangerouszone_range = 1+2*str2num(handles.Dangerous_Zone_Edit.String);
                savezone_range = 7;
                area_range = dangerouszone_range+savezone_range-1;
            elseif isempty(str2num(handles.Dangerous_Zone_Edit.String)) && ~isempty(str2num(handles.Save_Zone_Edit.String))
                dangerouszone_range = 5;
                savezone_range = 1+2*str2num(handles.Save_Zone_Edit.String);
                area_range = dangerouszone_range+savezone_range-1;
            else
                dangerouszone_range = 1+2*str2num(handles.Dangerous_Zone_Edit.String);
                savezone_range = 1+2*str2num(handles.Save_Zone_Edit.String);
                area_range = dangerouszone_range+savezone_range-1;
            end
            
            expand_point = [x(2)-((area_range-1)/2) , x(1)-((area_range-1)/2)];
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
            submap_obstacle = zeros( (max(expand_area(:,1))-min(expand_area(:,1))+1) , (max(expand_area(:,2))-min(expand_area(:,2))+1) );
            expand_area_by_indx = sub2ind(size(map),expand_area(:,1),expand_area(:,2));
            expand_area_shift_by_indx = [linspace(1,length(expand_area_by_indx),length(expand_area_by_indx))]';
            for i = 1:length(expand_area_by_indx)
                if ((map(expand_area_by_indx(i))==2) ||(map(expand_area_by_indx(i))==3) || (map(expand_area_by_indx(i))==4) || (map(expand_area_by_indx(i))==5) || (map(expand_area_by_indx(i))==6) || (map(expand_area_by_indx(i))== 7) || (map(expand_area_by_indx(i))== 8) ||(map(expand_area_by_indx(i))== 9))
                    map(expand_area_by_indx(i)) = 0;
                else
                    map(expand_area_by_indx(i)) = 1;
                end
            end
            submap_obstacle(expand_area_shift_by_indx) = map(expand_area_by_indx);  
            se_dangerous = strel('rectangle',[dangerouszone_range dangerouszone_range]);
            se_save = strel('rectangle',[savezone_range savezone_range]);
            
            dilated_bwhite_dangerous = imdilate( submap_obstacle , se_dangerous );
            dilated_bwhite_save = imdilate( dilated_bwhite_dangerous , se_save );
            
            dangerouszone_scaling = 7;
            dangerouszone_img = ~dilated_bwhite_dangerous;
            dangerouszone_area = ~submap_obstacle - dangerouszone_img;
            dangerouszone_area = dangerouszone_scaling*dangerouszone_area;
            
            savezone_scalling = 8;
            savezone_img = ~dilated_bwhite_save;
            savezone_area = dangerouszone_img - savezone_img;
            savezone_area = savezone_scalling*savezone_area;
            submap_obstacle = submap_obstacle+dangerouszone_area+savezone_area;
            map(expand_area_by_indx) = submap_obstacle(expand_area_shift_by_indx);
    end


    set(handles.Img,'CData',map); %%... ready for calculation
    drawnow();
    map_current = map;
    parameters = handles.Fig.UserData;
    parameters.map_current = map_current;
    handles.Fig.UserData = parameters;
    handles.Fig.UserData.click_pos = [];
end