comp_log = zeros(length(log),6);
bias_log = zeros(length(log),9);
gt_traj = zeros(length(log),3);
filt_traj = zeros(length(log),3);
for i = 1:length(log)
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
    bias = log{i,4};
    bias_log(i,:) = bias;

    gt_traj(i,:) = p_gt;
    filt_traj(i,:) = p_filt;
end

figure(1)
plot(1:length(log),comp_log(:,1:3))
legend('z','y','x')
title('Difference in Euler angles between ground truth & predicted')

figure(2)
plot(1:length(log),comp_log(:,4:6))
legend('x','y','z')
title('Delta between predicted and actual femur point')

figure(3)
plot3(gt_traj(:,1),gt_traj(:,2),gt_traj(:,3),'Color','g');
hold on; axis equal;
plot3(filt_traj(:,1),filt_traj(:,2),filt_traj(:,3),'Color','b');
% plot initial points
plot3(gt_traj(1,1),gt_traj(1,2),gt_traj(1,3),'Color','g','Marker','*')
plot3(filt_traj(1,1),filt_traj(1,2),filt_traj(1,3),'Color','b','Marker','*')

figure(4)
plot(1:length(log),bias_log(:,1:3))
legend('bgx','bgy','bgz')

figure(5)
plot(1:length(log),bias_log(:,4:6))
legend('ba1x','ba1y','ba1z')

figure(6)
plot(1:length(log),bias_log(:,7:9))
legend('ba2x','ba2y','ba2z')