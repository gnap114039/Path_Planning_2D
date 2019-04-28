function [f_updArr,Arrow] = drawArrow2D(P1,P2)
d1 = 0.1;
d2 = 0.1;

% Vector = P2-P1;
% d = norm(Vector);
% theta = atan2(Vector(2), Vector(1));
% R = [ cos(theta), -sin(theta); ...
%      sin(theta), cos(theta)];
% c1 = [d+ceil(d1*d), 0];
% c2 = d2*d;
% arrowVect = [[0,0]; ...
%              c1; ...
%              c1+[-c2,c2]; ...
%              c1+[-c2,-c2]; ...
%              c1];
% 
% arrowP = (R*arrowVect')' + repmat(P1,5,1);

Vector = P2-P1;
d = norm(Vector);
nV = [-Vector(2), Vector(1)]/d;
c1 = P1+(1+d1)*Vector;
arrowP = [P1; ...
          c1; ...
          P2+ceil(d1*d)*nV; ...
          P2-ceil(d1*d)*nV; ...
          c1];

 Arrow = patch(arrowP(:,1), arrowP(:,2), 'g','EdgeColor','g',...
                'FaceAlpha', 1, 'LineWidth', 1);

% d3 = d2+0.05;
% text(P2(1)-d3*d, P2(2)-d3*d, S, 'Color', 'g', 'FontSize', 20);

% f_updArr = @(P1,P2) updArrow(P1, P2, Arrow);


end







