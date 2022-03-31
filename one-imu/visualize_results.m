comp_log = zeros(3068,2);
for i = 2:3068
    % measure norm of Lie bracket for rotations
    gt = cell2mat(fkTable{i,'femur_r'});
    filt = log{i,2};
    R_gt = gt(1:3,1:3);
    R_filt = filt(1:3,1:3);
    rot_diff = norm(R_filt*R_gt - R_gt*R_filt);

    p_gt = gt(1:3,4);
    p_filt = filt(1:3,4);
    p_diff = norm(p_filt-p_gt);

    comp_log(i,:) = [rot_diff,p_diff];
end

figure(1)
plot(1:3068,comp_log(:,1))
title('Maximum singular value of Lie bracket\nof filter/ground truth rotations')

figure(2)
plot(1:3068,comp_log(:,2))
title('Norm of delta between\npredicted and actual femur point')