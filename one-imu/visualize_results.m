comp_log = zeros(length(log),6);
for i = 2:length(log)
    % measure norm of Lie bracket for rotations
    gt = cell2mat(fkTable{i,'femur_r'});
    filt = log{i,2};
    R_gt = gt(1:3,1:3);
    R_filt = filt(1:3,1:3);
    rot_diff = rotm2eul(R_filt)-rotm2eul(R_gt);

    p_gt = gt(1:3,4);
    p_filt = filt(1:3,4);
    p_diff = p_filt-p_gt;

    comp_log(i,:) = [rot_diff,p_diff'];
end

figure(1)
plot(1:length(log),comp_log(:,1:3))
legend('z','y','x')
title('Difference in Euler angles between ground truth & predicted')

figure(2)
plot(1:length(log),comp_log(:,4:6))
ylim([-10,10])
legend('x','y','z')
title('Delta between predicted and actual femur point')