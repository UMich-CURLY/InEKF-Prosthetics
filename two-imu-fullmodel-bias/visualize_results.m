len = length(log);
% len = 500;  % doesn't even really go anywhere by this point

comp_log = zeros(len,6);
gt_traj = zeros(len,3);
filt_traj = zeros(len,3);
foot_traj = {};
thigh2foot = {};
error_log = zeros(len,6);
accel_log = zeros(len,3);
dt = 0.005;
for i = 1:len
    % measure norm of Lie bracket for rotations
    gt = cell2mat(fkTable{i+initial-1,'femur_r'});
    filt = log{i,2};
    R_gt = gt(1:3,1:3);
    R_filt = filt(1:3,1:3);
    rot_diff = rotm2eul(R_filt)-rotm2eul(R_gt);

    p_gt = gt(1:3,4);
    if log{i,5}
        p_filt = filt(1:3,4);
    else
        % p_filt = [NaN; NaN; NaN];  % don't plot trajectory where we're not in contact, come back later and change the color
        p_filt = filt(1:3,4);
    end
    p_diff = p_filt-p_gt;
    % add: position T1to3 over time, see if it looks like a cyclic motion

    if size(filt,1) > 7
        p_foot = filt(1:3,8);
    else
        p_foot = [NaN; NaN; NaN];
    end

    p_foot_relative = log{i,6};
    % separate out steps of transforming back
    p_foot_relative = filt(1:3,1:3)*p_foot_relative;
    % p_foot_relative = filt(1:3,4) + p_foot_relative;

    accelerometer = imu_data{i,{'thigh_Accel_X','thigh_Accel_Y','thigh_Accel_Z'}};
    accel_log(i,:) = [accelerometer(3), -accelerometer(1), -accelerometer(2)];
    % accel_log(i,:) = accelerometer;

    error = log{i,4};
    % Need to rethink this a bit, perhaps only log to-heel error in the
    % first place so that we don't have to deal with in-contact vs. not
    if log{i,5}
        error_log(i,:) = error([1:3,9:11]);  % first 3 elements of bd after first 8 (bp2)
    else
        error_log(i,:) = [NaN, NaN, NaN, NaN, NaN, NaN];
    end

    comp_log(i,:) = [rot_diff,p_diff'];

    gt_traj(i,:) = p_gt;
    filt_traj(i,:) = p_filt;
    foot_traj{i,1} = p_foot(1);
    foot_traj{i,2} = p_foot(2);
    foot_traj{i,3} = p_foot(3);

    thigh2foot{i,1} = p_foot_relative(1);
    thigh2foot{i,2} = p_foot_relative(2);
    thigh2foot{i,3} = p_foot_relative(3);
end
foot_traj = cell2mat(foot_traj);
thigh2foot = cell2mat(thigh2foot);

figure;

subplot(2,3,1);
% plot(1:len,comp_log(:,1:3))
% legend('z','y','x')
plot(1:len,comp_log(:,2))
legend('y')
title('Difference in Euler angles between ground truth & predicted')

subplot(2,3,2);
plot(1:len,comp_log(:,4:6))
legend('x','y','z')
title('Delta between predicted and actual femur point')

subplot(2,3,3);
% plot(1:len,accel_log)
% legend('z (x)', '-x (y)', '-y (z)')
plot(1:len,error_log(:,4:6))  % just error to heel
legend('x','y','z')
title('Estimated error over time')

subplot(2,3,4);
plot3(gt_traj(:,1),gt_traj(:,2),gt_traj(:,3),'Color','g');
hold on; axis equal;
plot3(filt_traj(:,1),filt_traj(:,2),filt_traj(:,3),'Color','b');
scatter3(foot_traj(:,1),foot_traj(:,2),foot_traj(:,3),'Color','r');
% plot initial points
plot3(gt_traj(end,1),gt_traj(end,2),gt_traj(end,3),'Color','g','Marker','*')
plot3(filt_traj(end,1),filt_traj(end,2),filt_traj(end,3),'Color','b','Marker','*')
xlabel('x')
ylabel('y')
zlabel('z')
title('Trajectories: ground truth, pred. thigh, foot')
legend('GT','Filt','Filt (Heel)')

subplot(2,3,5);
plot(1:len,thigh2foot)
legend('x','y','z')
title('Thigh to Calcaneus Components Over Time')
