filepre = "../../AB06/AB06/";
motfile = "10_09_18/levelground/ik/levelground_ccw_slow_01_01.mat";
imufile = "10_09_18/levelground/imu/levelground_ccw_slow_01_01.mat";
fpfile = "10_09_18/levelground/fp/levelground_ccw_slow_01_01.mat";
imu_data = load(filepre+imufile).data;
fp_data = load(filepre+fpfile).data;
osimfile = "osimxml/AB06.osim";
vis = 0;
% options used to generate the fkTable
fkTable = FK(filepre+motfile,filepre+osimfile, ...
    'OutputType','H', ...
    'Transform','zup');
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
meas_vars = {'FP1_px','FP1_py','FP1_pz','FP1_vx','FP1_vy','FP1_vz',...
    'FP6_px','FP6_py','FP6_pz','FP6_vx','FP6_vy','FP6_vz'};
run matrices.m;  % initialize matrices

% Initialize first rotation based on IMU instead?
X = blkdiag(cell2mat(fkTable{1,'femur_r'}),eye(4));  % adding four extra columns
% Initialize contact point? Still no help
calcn_init = cell2mat(fkTable{1,'calcn_r'});
tibia_init = cell2mat(fkTable{1,'tibia_r'});
imu1_p = X(1:3,1:3)'*(tibia_init(1:3,4)-X(1:3,4))/2;  % Transpose instead of \?
X(1:3,6) = calcn_init(1:3,4);
P0 = blkdiag(0.1*eye(3),0.1*eye(3),0.1*eye(3),0.1*eye(3));  % 3 for rotation, 3x3 more for p1,v1,d
P = P0;
Q = blkdiag(0.1*eye(3),0.1*eye(3),0.01*eye(3),0.1*eye(3));  % 3 for rotation, next 3 for position, next 3 for velocity
N = 0.01*eye(6);  % Do the last three matter?
N_reduced = N(1:3,1:3);
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
    % Need to align axes properly. FK axes for femur are y up, z forward.
    % IMU axes are z forward, y medial (inward), so need:
    % x_imu = -y_fk, y_imu = x_fk, z_imu = z_fk
    % x_fk = y_imu, y_fk = -x_imu, z_fk = z_imu
    inputs = [inputs(2);-inputs(1);inputs(3);...
            inputs(5);-inputs(4);inputs(6)];
    % compensate linear velocity w/ displacement of IMU
    % This line is sus, am I doing it right?
    inputs(4:6) = inputs(4:6) + skew3x3(inputs(1:3))*imu1_p;
    % Check with and without this
    % inputs(1:3) = inputs(1:3)*pi/180; 
    [X,P] = predict(inputs, dt, X, P, A, Q);
    if sum(isnan(P(:))) > 0
        warning('Detected NaN in P')
    end
    % Use mocap for contact events
    

    % actual measurement is the vector to the contact point found in
    % forward kinematics (in world frame or body frame?)
    % Since we are getting relative position of the foot, not using the
    % state in this calculation is suitable.
    T1 = cell2mat(fkTable{i,'femur_r'});
    T2 = cell2mat(fkTable{i,'calcn_r'});
    % use T2 to determine contact
    % Could possibly include toes as extra contact point in future
    if T2(3,4) > 0.008  % heuristic, but a good one, (8cm?)
        Nt = 1000*N;  % don't trust contact point if not in contact
        % Going back to "skipping" strategy?
        log{i,1} = t;
        log{i,2} = X;
        log{i,3} = P;
    else
        % Need to be careful with this, if setting at each timestep then it
        % may implicitly violate zero-velocity constraint, or we may be
        % getting unrealistically accurate measurements.
        % Shouldn't be too much a problem though, since we're taking the
        % femur->calcaneus vector from FK, and (estimate)->calcaneus vector
        % within the correction step
        Nt = N;
        % Do I want to zero-out the z? would that help?
    end
    % Setting it regardless of whether or not we have contact, just to not
    % blow up the variance
    X(1:3,6) = T2(1:3,4);
    v_fc = T1(1:3,1:3)'*(T2(1:3,4) - T1(1:3,4)); % Just want to rotate, transpose instead of \?
    % Try without rotation? Similar result
    % v_fc = T2(1:3,4) - T1(1:3,4);
    meas = [v_fc; 1; 0; -1];
    [X,P] = update(meas,X,P,Hd,bd,Nt);
    log{i,1} = t;
    log{i,2} = X;
    log{i,3} = P;
end
