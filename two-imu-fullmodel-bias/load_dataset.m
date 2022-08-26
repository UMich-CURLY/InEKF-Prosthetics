check_params = exist('dataset_params','var');
if ~check_params
    filepre = "../../AB06/AB06/";
    motfile = "10_09_18/levelground/ik/levelground_ccw_slow_01_01.mat";
    imufile = "10_09_18/levelground/imu/levelground_ccw_slow_01_01.mat";
    fpfile = "10_09_18/levelground/fp/levelground_ccw_slow_01_01.mat";
    gonfile = "10_09_18/levelground/gon/levelground_ccw_slow_01_01.mat";
    imu_data = load(filepre+imufile).data;
    fp_data = load(filepre+fpfile).data;
    gon_data = load(filepre+gonfile).data;
    osimfile = "osimxml/AB06.osim";
else
    filepre = convertCharsToStrings(dataset_params("filepre"));
    motfile = convertCharsToStrings(dataset_params("motfile"));
    imufile = convertCharsToStrings(dataset_params("imufile"));
    fpfile = convertCharsToStrings(dataset_params("fpfile"));
    gonfile = convertCharsToStrings(dataset_params("gonfile"));
    imu_data = load(filepre+imufile).data; % for some reason, numbering scheme was flipped, add exception? No, check if file exists when generating the file names
    fp_data = load(filepre+fpfile).data;
    gon_data = load(filepre+gonfile).data;
    osimfile = convertCharsToStrings(dataset_params("osimfile"));
end
% options used to generate the fkTable
check = exist('fkTable','var');
if ~check
    fkTable = FK(filepre+motfile,filepre+osimfile, ...
        'OutputType','H', ...
        'Transform','zup');
    disp('FK Table loaded from Osim')
else
    disp('FK Table present')
end
% fkTable = load('fkTable.mat','fkTable').fkTable;
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
    'thigh_Accel_X','thigh_Accel_Y','thigh_Accel_Z', ...
    'shank_Accel_X','shank_Accel_Y','shank_Accel_Z'};
shank_vars = {'shank_Gyro_X','shank_Gyro_Y','shank_Gyro_Z'};
meas_vars = {'FP1_px','FP1_py','FP1_pz','FP1_vx','FP1_vy','FP1_vz',...
    'FP6_px','FP6_py','FP6_pz','FP6_vx','FP6_vy','FP6_vz'};
angle_vars = {'hip_sagittal','knee_sagittal','ankle_sagittal'};