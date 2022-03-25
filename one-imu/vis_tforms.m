function vis_tforms(tforms,color,marker)
% Visualizes homogeneous transformations: the links, and the rotation axes
% at each point
% set up xyz
delta = 0.02;
xyz = zeros(3,length(tforms));
for i = 1:length(tforms)
    T = tforms{:,i};
    o = T(1:3,4);
    xyz(:,i) = o;
    % plot the individual coordinate frame
    R = T(1:3,1:3);  % \eye(3)?
    xcol = R(1:3,1);
    ycol = R(1:3,2);
    zcol = R(1:3,3);
    x = o + xcol*delta/norm(xcol);
    y = o + ycol*delta/norm(ycol);
    z = o + zcol*delta/norm(zcol);
    line('XData',[o(1),x(1)], ...
        'YData',[o(2),x(2)], ...
        'ZData',[o(3),x(3)], ...
        'Color','y')
    line('XData',[o(1),y(1)], ...
        'YData',[o(2),y(2)], ...
        'ZData',[o(3),y(3)], ...
        'Color','y')
    line('XData',[o(1),z(1)], ...
        'YData',[o(2),z(2)], ...
        'ZData',[o(3),z(3)], ...
        'Color','y')
end
plot3(xyz(1,:),xyz(2,:),xyz(3,:),'Color',color,'Marker',marker)
end