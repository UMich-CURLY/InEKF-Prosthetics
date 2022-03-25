filepre = "example/";
motfile = "ik/levelground_ccw_fast_01_01.mat";
imufile = "imu/levelground_ccw_fast_01_01.mat";
fpfile = "fp/levelground_ccw_fast_01_01.mat";
imu_data = load(filepre+imufile).data;
fp_data = load(filepre+fpfile).data;
% osimfile = "osimxml/AB06.osim";
vis = 0;
% options used to generate the fkTable
%fkTable = FK(filepre+motfile,filepre+osimfile, ...
%    'OutputType','H', ...
%    'Transform','zup');
fkTable = load('fkTable.mat','fkTable').fkTable;
loc_rot_vars = {'torso_x','torso_y','torso_z';
    'pelvis_x','pelvis_y','pelvis_z';
    'femur_r_x','femur_r_y','femur_r_z';
    'tibia_r_x','tibia_r_y','tibia_r_z';
    'talus_r_x','talus_r_y','talus_r_z';
    'calcn_r_x','calcn_r_y','calcn_r_z';
    'toes_r_x','toes_r_y','toes_r_z';
    'femur_l_x','femur_l_y','femur_l_z';
    'tibia_l_x','tibia_l_y','tibia_l_z';
    'talus_l_x','talus_l_y','talus_l_z';
    'calcn_l_x','calcn_l_y','calcn_l_z';
    'toes_l_x','toes_l_y','toes_l_z'};
H_vars = {'torso','pelvis',...
    'femur_r','tibia_r','talus_r','calcn_r','toes_r',...
    'femur_l','tibia_l','talus_l','calcn_l','toes_l'};
input_vars = {'thigh_Gyro_X','thigh_Gyro_Y','thigh_Gyro_Z',...
    'thigh_Accel_X','thigh_Accel_Y','thigh_Accel_Z'};
meas_vars = {'FP1_px','FP1_py','FP1_pz','FP1_vx','FP1_vy','FP1_vz',...
    'FP6_px','FP6_py','FP6_pz','FP6_vx','FP6_vy','FP6_vz'};
run matrices.m;  % initialize matrices

% Initialize first rotation based on IMU instead?
X = blkdiag(cell2mat(fkTable{1,'femur_r'}),eye(2));  % adding two extra columns
P = eye(12);  % 3 for rotation, 3x3 more for p1,v1,d
Q = eye(12);
N = eye(6);
fp_base = table2array(fp_data(1,meas_vars));
log = {fkTable{1,'Header'},X,P};
for i = 2:3068  % 3068 is number of timesteps for which we have IMU
    % time difference from last step
    t = fkTable{i,'Header'};
    dt = fkTable{i,'Header'}-fkTable{i-1,'Header'};
    % fkTable1 = fkTable{t,loc_rot_vars'};
    % fkxyz = reshape(fkTable1,3,12);
    fkTable1 = fkTable{i,H_vars};
    rleg = 1:7;
    lleg = [1,2,8:12];
    if vis
        vis_tforms(fkTable1(lleg),'r','.')
        axis equal;
        hold on;
        vis_tforms(fkTable1(rleg),'b','.')
        hold off;
        pause(.005)
    end
    % get inputs from imu
    imu_row = table2array(imu_data(:,'Header'))==t;
    inputs = table2array(imu_data(imu_row,input_vars))';
    [X,P] = predict(inputs, dt, X, P, A, Q);
    fp_row = table2array(fp_data(:,'Header'))==t;
    fp_meas = table2array(fp_data(fp_row,meas_vars));
    % for ccw, want fp1 and fp6 (guessing these, though fp1 is a solid
    % guess)
    if norm(fp_meas-fp_base) > 0.1  % arbitrary threshold
        Nt = N;
        % manually set the contact point, need to fix this later to do it
        % only once
        heel = cell2mat(fkTable{i,'calcn_r'});
        X(1:3,6) = heel(1:3,4);
        % perhaps need to add expansion/contraction of 
        'not here'
    else
        Nt = 1000*N;  % don't trust contact point if not in contact
    end
    % actual measurement is the vector to the contact point found in
    % forward kinematics (in world frame or body frame?)
    T1 = cell2mat(fkTable{i,'femur_r'});
    T2 = cell2mat(fkTable{i,'calcn_r'});
    v_fc = T1(1:3,1:3)\(T2(1:3,4) - T1(1:3,4)); % Just want to rotate
    meas = [v_fc; 1; 0; -1];
    [X,P] = update(meas,X,P,Hd,bd,Nt);
    log{i,1} = t;
    log{i,2} = X;
    log{i,3} = P;
end
