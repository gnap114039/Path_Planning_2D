%%%======================    Edition List  =============================%%%
%%%2017/08/07
%%%     1. 1st Edition of Clear_Path
%%%
%%%     by Stud.Pang
%%%
%%%2017/08/10
%%%     1. 2nd Edition of Clear_Path, combine with any grid map
%%%
%%%     by Stud.Pang
%%%
%%%=====================================================================%%%
function Clear_Path(Button_Clear,data)

% map_current = evalin('base','map_current'); %%... which assign in from @Map_Setup
handles = get(Button_Clear,'UserData');
map_current = handles.Img.CData;
% map_current = handles.Fig.UserData.map_current;
red_cell = find(map_current==3);
purple_cell = find(map_current==4);
blue_cell = find(map_current==9);
dynamic_obstacle = find(map_current==10);
map_current(red_cell) = 2;
map_current(purple_cell) = 2;
map_current(blue_cell) = 2;
map_current(dynamic_obstacle) = 2;
set(handles.Img,'CData',map_current);
set(handles.AXES,'UserData',map_current);
set(handles.Calculation_Info,'String',sprintf(''));
set(handles.Button_Start,'Enable','on');
set(Button_Clear,'Enable','off');
set(handles.Path_Arrow,'XData',[],'YData',[]);
% set(handles.Map_Color_Setup,'Value',1);
set(handles.Force_Stop,'Enable','off');
% set(handles.Arrow,'XData',0,'YData',0);
end