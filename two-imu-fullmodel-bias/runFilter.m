run load_dataset.m;
run matrices.m;  % initialize matrices

% Next step - low-pass filter on IMU data + distance corrections for IMUs
% being placed mid-link
% run frequency analysis on IMU data to see if it's already been filtered
% (paper says 100Hz)
% Extension - move to using factor graphs? Like
% https://arxiv.org/pdf/1803.07531.pdf ?

% TODO: adjust P and Q to include bias covariance in prediction & update - may
% need a couple passes

% TODO: implement z = 0 constraint via "measurement". Do it as
% 1. a right-invariant measurement
% 2. a left-invariant measurement, transfering the error to the left and
% back once left-invariant measurement step is done separately.
% Also: work on better visualization for attitude tracking, perhaps just
% have the one plot with angles (+ angular and linear velocities?) tracking
% Results:
% 1. Unclear if there is any improvement. If anything, there is a worsening
% of trajectory tracking.
% 2. 

initial=1;
% Initialize first rotation based on IMU instead?
femur_init = cell2mat(fkTable{initial,'femur_r'});
X = blkdiag(femur_init,eye(3));  % adding four extra columns, for v1, p2, v2
% Initialize contact point in the filter when contact is found
tibia_init = cell2mat(fkTable{initial,'tibia_r'});
ankle_init = cell2mat(fkTable{initial,'talus_r'});
calcn_init = cell2mat(fkTable{initial,'calcn_r'});

% Currently unused imu points, for screw corrections if necessary
imu1_p = X(1:3,1:3)'*(tibia_init(1:3,4)-X(1:3,4))/2;  % Transpose instead of \?
imu2_p = tibia_init(1:3,1:3)'*(ankle_init(1:3,4)-tibia_init(1:3,4));

fk_params = rough_fk_estimate(femur_init,tibia_init,ankle_init,calcn_init);
X(1:3,6) = tibia_init(1:3,4);

bias = zeros(9,1);
P = blkdiag(P0,bias_cov);
Q = blkdiag(Q0,bias_cov);
contact = 0;
log = {fkTable{1,'Header'},X,P,zeros(16,1),contact,[NaN;NaN;NaN],bias,0};
contact = 0;
for i = (initial+1):height(imu_data)  % 3068 is number of timesteps for which we have IMU
    % time difference from last step
    t = fkTable{i,'Header'};
    dt = fkTable{i,'Header'}-fkTable{i-1,'Header'};
    % get inputs from imu
    imu_row = table2array(imu_data(:,'Header'))==t;
    inputs = table2array(imu_data(imu_row,input_vars))';
    % Need to align axes properly. FK axes for femur are y up, x forward.
    % IMU axes are z forward, y medial (inward), and x down, so need:
    % x_imu = -y_fk, y_imu = -z_fk, z_imu = x_fk
    % x_fk = z_imu, y_fk = -x_imu, z_fk = -y_imu
    % I'm still not sold on this...
    inputs = [inputs(3);-inputs(1);-inputs(2);...
            inputs(6);-inputs(4);-inputs(5);...
            inputs(9);-inputs(7);-inputs(8)];
    % And yet, the results weren't any better...
    % compensate linear velocity w/ displacement of IMU
    % This line is sus, am I doing it right?
    % inputs(4:6) = inputs(4:6) - skew3x3(inputs(1:3))*imu1_p;  % omega x OA is linear velocity due to acceleration, so this needs to change
    shank_gyro = table2array(imu_data(imu_row,shank_vars));
    shank_gyro = [shank_gyro(3);-shank_gyro(1);-shank_gyro(2)];
    % inputs(7:9) = inputs(7:9) - skew3x3(shank_gyro)*imu2_p; 
    % Check how to do with tibia accelerometer, may just be the same
    % Check with and without this
    % Add calculation of forward kinematics rotation to second joint -
    % ideally using goniometer and not ground truth, if possible
    T1 = cell2mat(fkTable{i,'femur_r'});
    T2 = cell2mat(fkTable{i,'tibia_r'});
    T3 = cell2mat(fkTable{i,'calcn_r'});
    fk2 = T1(1:3,1:3)'*T2(1:3,1:3);  % Right order?
    T1to3 = T1\T3;
    
    % TODO: go back to fullmodel w/o bias and move this section to the
    % front - how much does order matter here? This is a valid question to
    % ask
    % Use mocap for contact events
    

    % actual measurement is the vector to the contact point found in
    % forward kinematics (in world frame or body frame?)
    % Since we are getting relative position of the foot, not using the
    % state in this calculation is suitable.
    % use T2 to determine contact
    % Could possibly include toes as extra contact point in future
    angles = gon_data{i,angle_vars};
    fk_params{1,2} = angles(1)*-pi/180;  % Only using this first one for debugging
    fk_params{2,2} = angles(2)*-pi/180;
    fk_params{3,2} = angles(3)*-pi/180;
    J = JacobianFK(fk_params);
    if T3(3,4) > -1000*0.004  % Override for debugging sake
        % See if we are going into contact
        if contact
            contact = 0;
            [X,P] = remove_contact(X,P);
        end
    else
        if ~contact
            contact = 1;
            % What if we do the above at every timestep when not in contact?
            [X,P] = add_contact(X,P,T1to3(1:3,4),J,fk_cov);
            % X(1:3,8) = T3(1:3,4);  % setting to mocap, for checking
            % X(3,8) = 0  % set to 0 z
        end
    end
    
    % [X,P] = predict(inputs, dt, fk2, imu1_p, imu2_p, shank_gyro, X, P, A, Q);
    if contact
        [X,P] = predict(inputs, bias, dt, fk2, X, P, A, Q);
    else
        [X,P] = predict_nocontact(inputs, bias, ...
            dt, fk2, X, P, ...
            A([1:15,19:27],[1:15,19:27]), ...
            Q([1:15,19:27],[1:15,19:27]));
    end

    if sum(isnan(P(:))) > 0
        warning('Detected NaN in P')
        disp(i)
        return;
    end

    v_ft = T1(1:3,1:3)\(T2(1:3,4) - T1(1:3,4));
    v_fc = T1(1:3,1:3)\(T3(1:3,4) - T1(1:3,4));
    if contact
        v_constraint = X(1:3,1:3)\[X(1:2,8); 0];  % Does this need to be transformed into world coordinates? According to the math: no, just rotated
        b = [bp2; bd; bz];
        measp2 = [v_ft; 1; 0; -1; 0; 0];
        measd = [v_fc; 1; 0; 0; 0; -1];
        measz = [v_constraint(1:3); 0; 0; 0; 0; -1];
        meas = [measp2; measd; measz];
        H = [Hp2(1:3,:); Hd(1:3,:); Hz_r(1:3,:)];
        % We know model so this is fine for now
        N = blkdiag(fk_cov,fk_cov,fk_cov);  % only apply cov increase to contact point
        log{i-initial+1,4} = blkdiag(X,X,X)*meas-b;
        [X,P] = update(meas,X,bias,P,H,b,N,J);
    else
        J = J(:,1);  % only take the first angle, even in right-invariant case
        meas = [v_ft; 1; 0; -1; 0];
        H = Hp2(1:3,[1:15,19:27]);
        b = bp2(1:7);

        N = fk_cov(1:2,1:2);  % fine for debugging, but should change to stacked 1x1 when going back to right-invariant
        b = [b_gps1; b_gps2];
        H = [H_gps1(1:3,:); H_gps2(1:3,:)];
        meas_gps1 = [T1(1:3,4); 1; 0; 0; 0];
        meas_gps2 = [T2(1:3,4); 0; 0; 1; 0];
        meas = [meas_gps1; meas_gps2];
        log{i-initial+1,4} = blkdiag(X,X)\meas-b;  % divide because left-invariant
        [X,P,bias] = update_nocontact_leftgps(meas,X,bias,P,H,b,N,J);
    end
    % P will come out bigger, need to reshape it down?
    log{i-initial+1,1} = t;
    log{i-initial+1,2} = X;
    log{i-initial+1,3} = P;
    log{i-initial+1,5} = contact;
    log{i-initial+1,6} = T1to3(1:3,4);
    log{i-initial+1,7} = bias;

    % calculate y angle error and store in 8th column
    euler_error = rotm2eul(X(1:3,1:3)) - rotm2eul(T1(1:3,1:3));
    log{i-initial+1,8} = euler_error(2);
end
