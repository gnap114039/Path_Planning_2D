function Demo_Kit(Demo_Kit_Popup,~)

handles = get(Demo_Kit_Popup,'UserData');
DemoDimension = get(Demo_Kit_Popup,'Value');
set(handles.Path_Arrow,'XData',[],'YData',[]);
switch DemoDimension
    case 1
        map = 2*ones(10);
        map_init = 2*ones(10);
        dimension = '10x10';
    case 2
        map = 2*ones(20);
        map_init = 2*ones(20);
        dimension = '20x20';
    case 3
        map = 2*ones(30);
        map_init = 2*ones(30);
        dimension = '30x30';
    case 4
        map = 2*ones(50);
        map_init = 2*ones(50);
        dimension = '50x50';
    case 5
        map = 2*ones(100);
        map_init = 2*ones(100);
        dimension = '100x100';
    case 6
        map = 2*ones(500);
        map_init = 2*ones(500);
        dimension = '500x500';
end

set(handles.Img,'CData',map);
set(handles.Browse_Info,'String',sprintf('\nDemo Kit %s',dimension));
parameters = handles.Fig.UserData;
parameters.map_init = map_init;
handles.Fig.UserData = parameters;
% assignin('base','map_init',map_init);


end