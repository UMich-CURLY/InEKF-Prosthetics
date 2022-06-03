function vis_axes(tforms,color,linestyle)

for i = 1:length(tforms)
    T = tforms{:,i};
    o = T(1:3,4);
    % plot the individual coordinate frame
    R = T(1:3,1:3);  % \eye(3)?
    xcol = R(1:3,1);
    ycol = R(1:3,2);
    zcol = R(1:3,3);
    x = o + xcol/norm(xcol);
    y = o + ycol/norm(ycol);
    z = o + zcol/norm(zcol);
    plot3(o(1),o(2),o(3),'Color',color,'Marker','*')
    hold on;
    axis equal;
    line('XData',[o(1),x(1)], ...
        'YData',[o(2),x(2)], ...
        'ZData',[o(3),x(3)], ...
        'Color',color)
    line('XData',[o(1),y(1)], ...
        'YData',[o(2),y(2)], ...
        'ZData',[o(3),y(3)], ...
        'Color',color)
    line('XData',[o(1),z(1)], ...
        'YData',[o(2),z(2)], ...
        'ZData',[o(3),z(3)], ...
        'Color',color,'LineStyle',linestyle)
    xlabel('x')
    ylabel('y')
    zlabel('z')

    % Make motion 'movie' again, but this time have pauses every 100 or so
    % timesteps that clearly show the axes, with the z axis standing out.
    % Also make sure axes stay consistent with movement
    % On second look, z axis is axis of rotation, pointing out.
end