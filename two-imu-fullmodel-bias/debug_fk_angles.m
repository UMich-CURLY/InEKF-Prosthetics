run load_dataset.m;
run matrices.m;
thigh_time = zeros(2,height(imu_data));
knee_time = zeros(2,height(imu_data));
ankle_time = zeros(2,height(imu_data));
for i = 1:height(imu_data)
    t = table2array(fkTable(i,'Header'));
    T1 = cell2mat(fkTable{i,'femur_r'});
    T2 = cell2mat(fkTable{i,'tibia_r'});
    T3 = cell2mat(fkTable{i,'calcn_r'});

    goniometer_row = table2array(gon_data(:,'Header'))==t;
    angles = gon_data{goniometer_row,angle_vars};
    thigh = angles(1)*-pi/180;  % This is nearly accurate
    knee = angles(2)*pi/180;
    ankle = angles(3)*-pi/180;

    thigh_mocap = rotm2eul(T1(1:3,1:3));
    thigh_mocap = thigh_mocap(2);
    % Knee mocap angle is coming out to 0 -- wonder why?
    knee_mocap = rotm2eul(T1(1:3,1:3)\T2(1:3,1:3));
    knee_mocap = knee_mocap(2);
    ankle_mocap = rotm2eul(T2(1:3,1:3)\T3(1:3,1:3));
    ankle_mocap = ankle_mocap(2);

    thigh_time(:,i) = [thigh; thigh_mocap];
    knee_time(:,i) = [knee; knee_mocap];
    ankle_time(:,i) = [ankle; ankle_mocap];
end

figure;
subplot(1,3,1)
plot(1:height(imu_data),thigh_time)
title("Thigh")
legend(["Gon","MoCap"])
subplot(1,3,2)
plot(1:height(imu_data),knee_time)
title("Knee")
legend(["Gon","MoCap"])
subplot(1,3,3)
plot(1:height(imu_data),ankle_time)
title("Ankle")
legend(["Gon","MoCap"])