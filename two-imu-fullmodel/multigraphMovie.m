run load_dataset.m;
run matrices.m;

imu_vars = {'thigh_Accel_X','thigh_Accel_Y','thigh_Accel_Z'};
femur_init = cell2mat(fkTable{1,'femur_r'});
femur_p = femur_init(1:3,4);
accelerometer = imu_data{1,imu_vars};
ax1 = subplot(1,2,1);
femur_traj = animatedline(ax1);
ax2 = subplot(1,2,2);
accel_x = animatedline(ax2, 'Color', 'r');
accel_y = animatedline(ax2, 'Color', 'g');
accel_z = animatedline(ax2, 'Color', 'b');
for i = 1:3068  % 3068 is number of timesteps for which we have IMU
    % time difference from last step

    femur_T = cell2mat(fkTable{i,'femur_r'});
    femur_p = femur_T(1:3,4);
    subplot(1,2,1);
    axis equal;
    view(3);
    addpoints(femur_traj, femur_p(1), femur_p(2), femur_p(3))
    % plot3(femur_traj(:,1),femur_traj(:,2),femur_traj(:,3))

    accelerometer = imu_data{i,imu_vars};
    subplot(1,2,2);
    addpoints(accel_x, i, accelerometer(1));
    addpoints(accel_y, i, accelerometer(2));
    addpoints(accel_z, i, accelerometer(3));

    drawnow limitrate

end
