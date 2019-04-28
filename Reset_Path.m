function Reset_Path(Button_Clear,data)

% map_init = evalin('base','map_init'); %%... which assignin from @Browse_Image
handles = get(Button_Clear,'UserData');
map_init = handles.Fig.UserData.map_init;
set(handles.Img,'CData',map_init);
set(handles.Calculation_Info,'String',sprintf(''));
set(handles.Button_Start,'Enable','on');
set(handles.Path_Arrow,'XData',[],'YData',[]);
% set(handles.Map_Color_Setup,'Value',1);
set(handles.Force_Stop,'Enable','off');
% set(handles.Arrow,'XData',0,'YData',0);
end