 %%%======================    Edition List  =============================%%%
%%%2017/08/07
%%%     1. 1st Edition of Optimization_Path
%%%
%%%     by Stud.Pang
%%%
%%%2017/08/08
%%%     1. 2nd Edition of Optimization_Path, modelize some variable 
%%%
%%%     by Stud.Pang
%%%
%%%2017/08/10
%%%     1. 3rd Edition of Optimization_Path, passing Img.CData by structure handles 
%%%
%%%     by Stud.Pang
%%%
%%%2017/08/25
%%%     1. 4th Edition of Optimization_Path, Adding A* Algorithm 
%%%
%%%     by Stud.Pang
%%%
%%%2017/09/17
%%%     1. 5th Edition of Optimization_Path, Seperate A* Algorithm with UIControl 
%%%
%%%     by Stud.Pang
%%%=====================================================================%%%
function Drawing_Path(Button_Start,~)

handles = get(Button_Start,'UserData');

%%== Clear_Path %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% map_current = handles.Fig.UserData.map_current;
map_current = handles.Img.CData;
red_cell = find(map_current==3);
purple_cell = find(map_current==4);
blue_cell = find(map_current==9);
dynamic_obstacle = find(map_current==10);
map_current(red_cell) = 2;
map_current(purple_cell) = 2;
map_current(blue_cell) = 2;
map_current(dynamic_obstacle) = 2;
% map_size = size(map_current);
set(handles.Img,'CData',map_current);
set(handles.Calculation_Info,'String',sprintf(''));
set(handles.Path_Arrow,'XData',[],'YData',[]);
% set(handles.Arrow,'XData',0,'YData',0);




%%== Get the Initial Information %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
map = handles.Img.CData;
algorithm_selection = handles.Algorithm_Selection.Value;
mission_selection = handles.Mission_Selection.Value;
force_stop_flag = handles.Force_Stop;
set(handles.Calculation_Info,'String',sprintf('Calculating........'));
exploring_direction = handles.Exploring_Direction_Check.Value;





%%=== Disable all pushbutton %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
set(Button_Start,'Enable','off');
set(handles.Button_Clear,'Enable','off');
set(handles.Button_Reset,'Enable','off');
set(handles.Browse_Button,'Enable','off');
set(handles.Force_Stop,'Enable','on');
drawnow();



%%==Path Scoring Algorithm %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
switch algorithm_selection
    case 1
        Algthm = 'Dijkstra';
    case 2
        Algthm = 'A*';
    case 3 
        Algthm = 'A* Heuristic';
    case 4
        Algthm = 'Greedy Appoarch';
    case 5
        Algthm = 'Heuristic Weight';
end

switch mission_selection
    case 1
        [~ , dest_node , map , distanceFromStart , ~ , time , flag , route,total_distance] = Path_Scoring(map,algorithm_selection,force_stop_flag,exploring_direction);
    case 2
        exploring_direction = 1;
        [dest_node , map , distanceFromStart,time,flag,route,total_distance ]= Wall_Following_Setup(map,handles,algorithm_selection,force_stop_flag,exploring_direction);
    case 3    
        exploring_direction = 1;
        [dest_node , map , distanceFromStart,time,flag,route,total_distance] = Wall_Following_Setup_Brutal(map,handles,algorithm_selection,force_stop_flag,exploring_direction);
    case 4
        exploring_direction = 1;
        [dest_node , map , distanceFromStart,time,flag,route,total_distance] = Peeling(map,handles,algorithm_selection,exploring_direction,force_stop_flag);
end



%%=== Warning Messages %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if flag~=0
    switch flag
        case 1
            warning = warndlg('Please insert Start node and Destination Node','Warning !!','modal');
            set(handles.Calculation_Info,'String',sprintf(''));
            set(Button_Start,'Enable','on');
            set(handles.Button_Reset,'Enable','on');
            set(handles.Browse_Button,'Enable','on');
            set(handles.Force_Stop,'Enable','off');
            return;
        case 2
            warning = warndlg('Too many Start node and Destination Node','Warning !!','modal');
            set(handles.Calculation_Info,'String',sprintf(''));
            set(Button_Start,'Enable','on');
            set(handles.Button_Reset,'Enable','on');
            set(handles.Browse_Button,'Enable','on');
            set(handles.Force_Stop,'Enable','off');
            return;
        case 3
            warning = warndlg('Too many Start Node ','Warning !!','modal');
            set(handles.Calculation_Info,'String',sprintf(''));
            set(Button_Start,'Enable','on');
            set(handles.Button_Reset,'Enable','on');
            set(handles.Browse_Button,'Enable','on');
            set(handles.Force_Stop,'Enable','off');
            return;
        case 4
            warning = warndlg('Too many Destination Node','Warning !!','modal');
            set(handles.Calculation_Info,'String',sprintf(''));
            set(handles.Calculation_Info,'String',sprintf(''));
            set(Button_Start,'Enable','on');
            set(handles.Button_Reset,'Enable','on');
            set(handles.Browse_Button,'Enable','on');
            set(handles.Force_Stop,'Enable','off');
            return;
        case 5
            warning = warndlg('Please insert Start Node ','Warning !!','modal');
            set(handles.Calculation_Info,'String',sprintf(''));
            set(Button_Start,'Enable','on');
            set(handles.Button_Reset,'Enable','on');
            set(handles.Browse_Button,'Enable','on');
            set(handles.Force_Stop,'Enable','off');
            return;
        case 6
            warning = warndlg('Please insert Destiantion Node','Warning !!','modal');
            set(handles.Calculation_Info,'String',sprintf(''));
            set(Button_Start,'Enable','on');
            set(handles.Button_Reset,'Enable','on');
            set(handles.Browse_Button,'Enable','on');
            set(handles.Force_Stop,'Enable','off');
            return;
        case 7
            set(handles.Calculation_Info,'String',sprintf('Process path scoring has terminated~'));
            set(Button_Start,'Enable','on');
            set(handles.Button_Reset,'Enable','on');
            set(handles.Browse_Button,'Enable','on');
            set(handles.Force_Stop,'Enable','off');
            return;
        case 8
            warning = warndlg('Cannot find a path !!!','Warning !!','modal');
            set(Button_Start,'Enable','on');
            set(handles.Button_Reset,'Enable','on');
            set(handles.Browse_Button,'Enable','on');
            set(handles.Force_Stop,'Enable','off');
            return;
    end
end




%%% Path Scoring Result Panel Information %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if total_distance<Inf
    set(handles.Calculation_Info,'String',sprintf('Algorithm : %s\n\nPath Scoring Time : %d Sec\n\nDistance from start :  %d unit.',...
        Algthm,time,total_distance));%%¡C¡C¡CdistanceFromStart(dest_node)
else
    set(handles.Calculation_Info,'String',sprintf('Cannot find a path!!'));
end




%%===Drawing Arrow %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% [p,q]=ind2sub(map_size,route); %%... extracting the matrix position of optimized path
% arrow_matrix = [p;q]';
% [map_x,map_y] = size(map_size);
% if map_x>100 || map_y>100
%     arrow_scaling = 4;
% elseif map_x<100 || map_y<100
%     arrow_scaling = 2;
% elseif map_x<50 || map_y<50
%     arrow_scaling = 1;
% end    
% % arrow_scaling = 8;
% total_drawing_arrow = [];
% total_drawing_arrow_x = [];
% total_drawing_arrow_y = [];
% for i = 1:20:length(arrow_matrix)
%     if i < ( length(arrow_matrix)-2 )
%         if( (arrow_matrix(i,1)-arrow_matrix(i+1,1))==0 && (arrow_matrix(i,2)-arrow_matrix(i+1,2))==-1 ) %%right
%             drawing_arrow=[arrow_matrix(i,:) ; arrow_matrix(i,1)+arrow_scaling  arrow_matrix(i,2) ;  arrow_matrix(i,1)  arrow_matrix(i,2)+arrow_scaling ; arrow_matrix(i,1)-arrow_scaling  arrow_matrix(i,2); arrow_matrix(i,:)]+0.5;
%             total_drawing_arrow = [total_drawing_arrow;drawing_arrow];
%             total_drawing_arrow_x = [total_drawing_arrow_x,drawing_arrow(:,1)];
%             total_drawing_arrow_y = [total_drawing_arrow_y,drawing_arrow(:,2)];
%             set(handles.Path_Arrow,'XData',total_drawing_arrow_y,'YData',total_drawing_arrow_x,'FaceColor',[0.6 0.6 0.6],'EdgeColor','none','FaceAlpha',0.5);
% %             pause(0.3);
%             drawnow();
%         elseif ( (arrow_matrix(i,1)-arrow_matrix(i+1,1))==0 && (arrow_matrix(i,2)-arrow_matrix(i+1,2))==1 ) %% left
%             drawing_arrow=[arrow_matrix(i,:) ; arrow_matrix(i,1)+arrow_scaling  arrow_matrix(i,2) ;  arrow_matrix(i,1)  arrow_matrix(i,2)-arrow_scaling ; arrow_matrix(i,1)-arrow_scaling  arrow_matrix(i,2); arrow_matrix(i,:)]+0.5;
%             total_drawing_arrow = [total_drawing_arrow;drawing_arrow];
%             total_drawing_arrow_x = [total_drawing_arrow_x,drawing_arrow(:,1)];
%             total_drawing_arrow_y = [total_drawing_arrow_y,drawing_arrow(:,2)];
%             set(handles.Path_Arrow,'XData',total_drawing_arrow_y,'YData',total_drawing_arrow_x,'FaceColor',[0.6 0.6 0.6],'EdgeColor','none','FaceAlpha',0.5);
% %             pause(0.3);
%             drawnow();
%         elseif ( (arrow_matrix(i,1)-arrow_matrix(i+1,1))==1 && (arrow_matrix(i,2)-arrow_matrix(i+1,2))==0 ) %%up
%             drawing_arrow=[arrow_matrix(i,:) ; arrow_matrix(i,1)  arrow_matrix(i,2)+arrow_scaling ;  arrow_matrix(i,1)-arrow_scaling  arrow_matrix(i,2) ; arrow_matrix(i,1)  arrow_matrix(i,2)-arrow_scaling; arrow_matrix(i,:)]+0.5;
%             total_drawing_arrow = [total_drawing_arrow;drawing_arrow];
%             total_drawing_arrow_x = [total_drawing_arrow_x,drawing_arrow(:,1)];
%             total_drawing_arrow_y = [total_drawing_arrow_y,drawing_arrow(:,2)];
%             set(handles.Path_Arrow,'XData',total_drawing_arrow_y,'YData',total_drawing_arrow_x,'FaceColor',[0.6 0.6 0.6],'EdgeColor','none','FaceAlpha',0.5);
% %             pause(0.3);
%             drawnow();
%         elseif ( (arrow_matrix(i,1)-arrow_matrix(i+1,1))==-1 && (arrow_matrix(i,2)-arrow_matrix(i+1,2))==0 ) %%down
%             drawing_arrow=[arrow_matrix(i,:) ; arrow_matrix(i,1)  arrow_matrix(i,2)+arrow_scaling ;  arrow_matrix(i,1)+arrow_scaling  arrow_matrix(i,2) ; arrow_matrix(i,1)  arrow_matrix(i,2)-arrow_scaling; arrow_matrix(i,:)]+0.5;
%             total_drawing_arrow = [total_drawing_arrow;drawing_arrow];
%             total_drawing_arrow_x = [total_drawing_arrow_x,drawing_arrow(:,1)];
%             total_drawing_arrow_y = [total_drawing_arrow_y,drawing_arrow(:,2)];
%             set(handles.Path_Arrow,'XData',total_drawing_arrow_y,'YData',total_drawing_arrow_x,'FaceColor',[0.6 0.6 0.6],'EdgeColor','none','FaceAlpha',0.5);
% %             pause(0.3);
%             drawnow();
%         elseif ( (arrow_matrix(i,1)-arrow_matrix(i+1,1))== 1 && (arrow_matrix(i,2)-arrow_matrix(i+1,2))== 1 ) %% left up
%             drawing_arrow=[arrow_matrix(i,:) ; arrow_matrix(i,1)+arrow_scaling  arrow_matrix(i,2)-arrow_scaling ;  arrow_matrix(i,1)-arrow_scaling  arrow_matrix(i,2)-arrow_scaling ; arrow_matrix(i,1)-arrow_scaling  arrow_matrix(i,2)+arrow_scaling; arrow_matrix(i,:)]+0.5;
%             total_drawing_arrow = [total_drawing_arrow;drawing_arrow];
%             total_drawing_arrow_x = [total_drawing_arrow_x,drawing_arrow(:,1)];
%             total_drawing_arrow_y = [total_drawing_arrow_y,drawing_arrow(:,2)];
%             set(handles.Path_Arrow,'XData',total_drawing_arrow_y,'YData',total_drawing_arrow_x,'FaceColor',[0.6 0.6 0.6],'EdgeColor','none','FaceAlpha',0.5);
% %             pause(0.3);
%             drawnow();
%         elseif ( (arrow_matrix(i,1)-arrow_matrix(i+1,1))==-1 && (arrow_matrix(i,2)-arrow_matrix(i+1,2))==-1 ) %% right down
%             drawing_arrow=[arrow_matrix(i,:) ; arrow_matrix(i,1)+arrow_scaling  arrow_matrix(i,2)-arrow_scaling ;  arrow_matrix(i,1)+arrow_scaling  arrow_matrix(i,2)+arrow_scaling ; arrow_matrix(i,1)-arrow_scaling  arrow_matrix(i,2)+arrow_scaling; arrow_matrix(i,:)]+0.5;
%             total_drawing_arrow = [total_drawing_arrow;drawing_arrow];
%             total_drawing_arrow_x = [total_drawing_arrow_x,drawing_arrow(:,1)];
%             total_drawing_arrow_y = [total_drawing_arrow_y,drawing_arrow(:,2)];
%             set(handles.Path_Arrow,'XData',total_drawing_arrow_y,'YData',total_drawing_arrow_x,'FaceColor',[0.6 0.6 0.6],'EdgeColor','none','FaceAlpha',0.5);
% %             pause(0.3);
%             drawnow();
%         elseif ( (arrow_matrix(i,1)-arrow_matrix(i+1,1))== 1 && (arrow_matrix(i,2)-arrow_matrix(i+1,2))==-1 ) %% right up
%             drawing_arrow=[arrow_matrix(i,:) ; arrow_matrix(i,1)-arrow_scaling  arrow_matrix(i,2)-arrow_scaling ;  arrow_matrix(i,1)-arrow_scaling  arrow_matrix(i,2)+arrow_scaling ; arrow_matrix(i,1)+arrow_scaling  arrow_matrix(i,2)+arrow_scaling; arrow_matrix(i,:)]+0.5;
%             total_drawing_arrow = [total_drawing_arrow;drawing_arrow];
%             total_drawing_arrow_x = [total_drawing_arrow_x,drawing_arrow(:,1)];
%             total_drawing_arrow_y = [total_drawing_arrow_y,drawing_arrow(:,2)];
%             set(handles.Path_Arrow,'XData',total_drawing_arrow_y,'YData',total_drawing_arrow_x,'FaceColor',[0.6 0.6 0.6],'EdgeColor','none','FaceAlpha',0.5);
% %             pause(0.3);
%             drawnow();
%         elseif ( (arrow_matrix(i,1)-arrow_matrix(i+1,1))==-1 && (arrow_matrix(i,2)-arrow_matrix(i+1,2))== 1 ) %% left down
%             drawing_arrow=[arrow_matrix(i,:) ; arrow_matrix(i,1)-arrow_scaling  arrow_matrix(i,2)-arrow_scaling ;  arrow_matrix(i,1)+arrow_scaling  arrow_matrix(i,2)-arrow_scaling ; arrow_matrix(i,1)+arrow_scaling  arrow_matrix(i,2)+arrow_scaling; arrow_matrix(i,:)]+0.5;
%             total_drawing_arrow = [total_drawing_arrow;drawing_arrow];
%             total_drawing_arrow_x = [total_drawing_arrow_x,drawing_arrow(:,1)];
%             total_drawing_arrow_y = [total_drawing_arrow_y,drawing_arrow(:,2)];
%             set(handles.Path_Arrow,'XData',total_drawing_arrow_y,'YData',total_drawing_arrow_x,'FaceColor',[0.6 0.6 0.6],'EdgeColor','none','FaceAlpha',0.5);
% %             pause(0.3);
%             drawnow();
%         end
%     end
% end
% map(find(map==4))=2;
set(handles.Img,'CData',map);




%%=== Recolor the Optimize Path from Red to Green %%%%%%%%%%%%%%%%%%%%%%%%%
k=2;
% video_name = 'Closest_Group';
% V = VideoWriter(video_name,'MPEG-4');
% open(V);
area_range = 31;
% [f_updArr,handles.Arrow] = drawArrow2D([0 0],[0 0]);
while k <= length(route)
    map = get(handles.Img,'CData');
    map(route(k)) = 9;
    map(dest_node) = 6;
%     if k<length(route)  %%%%%% ºñ¦â½bÀY
%         [i,j] = ind2sub(size(map),[route(k) route(k+1)]);
%         updArrow ([j(1) i(1)]+0.5,[j(2) i(2)]+0.5,handles.Arrow);
%     end
    set(handles.Img,'CData',map);
    if rem(k,40)==0 || k==1 
        drawnow();
%         F = getframe(handles.AXES);
%         writeVideo(V,F);
    end
    if (k+1)<=length(route)
        if (map(route(k+1))==1 || map(route(k+1))==7 || map(route(k+1))==8 || map(route(k+1))== 10)
            [route, map, k_new]  = Dynamic_Obstacle(route,map,algorithm_selection,force_stop_flag,k,exploring_direction,area_range,handles);
            k = k_new;
        else
            k = k+1;
        end   
    else
        k = k+1;
    end
end
set(handles.Img,'CData',map);
drawnow();
% F = getframe(handles.AXES);
% writeVideo(V,F);
% close(V);



set(handles.Button_Start,'Enable','on');
set(handles.Button_Clear,'Enable','on');
set(handles.Button_Reset,'Enable','on');   
set(handles.Browse_Button,'Enable','on');
set(handles.Force_Stop,'Enable','off');
% set(handles.Map_Color_Setup,'Value',1);
% map(find(map==4))=2;




figure2mapcolor = [ 0        0       0; ... % 1 - black - clear cell
         1        1       1; ... % 2 - white - obstacle
         0.9804   0.4     0.4; ... % 3 - red - visited
         0.8      0.8     1; ... % 4 - light purple - on list
         0        1       0; ... % 5 - green - start
         1        1       0; ... % 6 - yellow - destination
         0.9686   0.8039  0.8039; ...% 7 - pink - dangerous zone
         0.8039   0.9804  0.9804;...% 8 - light green - save zone
         0        0       1;...% 9 - blue - final path
         0.412    0.412   0.412;...% 10 - grey - dynamic obstacle
         1        0.647   0.31]; % 11 - orange chocolate - boundary detection

fig_test = figure(2);
colormap(figure2mapcolor);
image(map);
axis image;
% % set(ax, 'Visible','off')
% 
% % set(findobj(gcf, 'type','axes'), 'Visible','off');
% % set(gca,'looseInset',[0 0 0 0]);
% 
set(findobj(gcf, 'type','axes'), 'Visible','on');
set(gca,'looseInset',get(gca,'TightInset'),'FontSize',18);


end











