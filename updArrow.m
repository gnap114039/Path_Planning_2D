function updArrow(P1, P2, Arrow)
d1 = 4;
d2 = 2;
Vector = P2-P1;
d = norm(Vector);
v_d = Vector/d;
nV = [-Vector(2), Vector(1)]/d;
c1 = P1+(1+d2)*Vector;
arrowP = [P1-3*v_d; ...
          c1+ 3*v_d; ...
          P2+ceil(d1*d)*nV; ...
          P2-ceil(d1*d)*nV; ...
          c1+ 3*v_d];
set(Arrow,'XData',arrowP(:,1),...
            'YData',arrowP(:,2));

end