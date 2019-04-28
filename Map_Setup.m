%%%======================    Edition List  =============================%%%
%%%2017/08/07
%%%     1. 1st Edition of Map_Setup
%%%
%%%     by Stud.Pang
%%%
%%%2017/08/10
%%%     1. 3rd Edition of Map_Setup, passing Img.CData by structure handles
%%%
%%%2017/08/10
%%%     1. 4rd Edition of Map_Setup, passing Popup.Value by structure handles
%%%
%%%     by Stud.Pang
%%%=====================================================================%%%
function Map_Setup(Img,data)
handles = get(Img,'UserData');
map = handles.Img.CData;
[nrows, ncols] = size(map);
x = floor(data.IntersectionPoint(1:2));
parameters = handles.Fig.UserData;
parameters.click_pos = x;
handles.Fig.UserData = parameters;
map_color_value = handles.Fig.UserData.map_color_value;
dangerouszone_range = 1+2*str2num(handles.Dangerous_Zone_Edit.String);
savezone_range = 1+2*str2num(handles.Save_Zone_Edit.String);

%%=== �N�̪�k����ҿ諸�C���J����Q��쪺���I %%%%%%%%%%%%%%%%%%%%%%%%%%

if map(x(2),x(1))~=1 %%... �p�G�Q�I�쪺���I���O�¦�
    
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
            %%=== ��Xsubmap���Ҧ���element�bmap����ڦ�m %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
    end %%... color setup switch case

    
elseif map(x(2),x(1))==1 && map_color_value == 3 %%... �p�G�Q�I�쪺���I�ɶ¦�P�ɳ̪����C��O�զ�A�h�����N��ê�������զ�A�P�ɱN��ê����boundary�簣
    
    map(x(2),x(1)) = 2; %%... White
    
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
    %%=== ��Xsubmap���Ҧ���element�bmap����ڦ�m %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
elseif map(x(2),x(1))==1 && map_color_value ~= 3
    warning = warndlg('Please use white color to clean the obstacle','Warning !!','modal');
end





set(Img,'CData',map); %%... ready for calculation
drawnow();
map_current = map;
parameters = handles.Fig.UserData;
parameters.map_current = map_current;
handles.Fig.UserData = parameters;
%     set(handles.AXES,'UserData',map_current);

end






