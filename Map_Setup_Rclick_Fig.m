function Map_Setup_Rclick_Fig(menu,data)

handles = get(menu,'UserData');
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
% handles.Img.CData = map_color_value;
handles.Fig.UserData = parameters;
    
end