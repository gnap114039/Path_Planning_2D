clear all;clc;close all;    

%%% set up color map for display 
cmap = [0 0 0; ... % 1 - black - clear cell 
        1 1 1; ... % 2 - white - obstacle 
        1 0 0; ... % 3 - red = visited 
        0 0 1; ... % 4 - blue - on list 
        0 1 0; ... % 5 - green - start 
        1 1 0; ... % 6 - yellow - destination
        1 0.6 0.6; ...% 7- pink - dangerous zone 
        0.8 1 0.8]; % 8 - light green - save zone

    


originalBW = imread('20170816_IMU_01_5892m_Obs.jpg');
index_reverse = ~originalBW;

se_dangerous = strel('rectangle',[5 5]);
se_save = strel('rectangle',[7 7]);

dilated_bwhite_dangerous = imdilate( index_reverse , se_dangerous );
dilated_bwhite_save = imdilate( dilated_bwhite_dangerous , se_save );

dangerouszone_img = ~dilated_bwhite_dangerous;
dangerouszone_area = logical(originalBW) - dangerouszone_img;
dangerouszone_area = 5*dangerouszone_area; %% 危險邊邊的矩陣

savezone_img = ~dilated_bwhite_save;
savezone_area = dangerouszone_img - savezone_img;
savezone_area = 6*savezone_area; %% 安全邊邊的矩陣

% ori_map = logical(originalBW)+dangerouszone_area+savezone_area;
% 
% 
% if (size(ori_map, 3) ==3 )
%     map_colorpass = rgb2gray(ori_map);
% else
%     map_colorpass = ori_map;
% end
% 
% % not_white_log =1 + map_colorpass;
% % map_init = double( not_white_log );
% % map = double( not_white_log );
% map_init = double( 1+ logical(map_colorpass) );
% map = double( 1+ logical(map_colorpass) );


figure ;
colormap(cmap); 
subplot(1 ,3 ,1)
image(1+logical(originalBW));
grid on;
subplot(1 ,3 ,2)
image((1+logical(originalBW)+dangerouszone_area));
% image(map,'AlphaData',0.5);
grid on;
subplot(1 ,3 ,3)
image((1+logical(originalBW)+dangerouszone_area+savezone_area));
% image(map,'AlphaData',0.5);
grid on;





