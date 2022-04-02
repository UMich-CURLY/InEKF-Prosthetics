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