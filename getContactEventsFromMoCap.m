% get contact events from motion capture
% Could potentially add second contact point with toe
contacts = zeros(height(fkTable),1);
zs = zeros(height(fkTable),1);
for i = 1:height(fkTable)
    % assumes "zup" transform from Osim
    point = cell2mat(fkTable{i,'calcn_r'});
    z = point(3,4);
    zs(i) = z;
    if z < 0.07  % may need to set this differently later
        contacts(i) = 1;
    end
end
plot(zs)
hold on
yline(0.008)  % 8

% Use this to plot Euler angles over time to determine radians or degrees,
% alongside the simulation being drawn (use subfigures?)
% Could be useful to have a visualization pipeline that runs ground truth,
% estimated, and individual stats altogether as a video