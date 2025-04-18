run load_dataset.m;
run matrices.m;  % initialize matrices

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

P = P0;
contact = 0;
log = {fkTable{1,'Header'},X,P,zeros(16,1),contact,[NaN;NaN;NaN]};
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

    % Use mocap for contact events
    

    % actual measurement is the vector to the contact point found in
    % forward kinematics (in world frame or body frame?)
    % Since we are getting relative position of the foot, not using the
    % state in this calculation is suitable.
    % use T2 to determine contact
    % Could possibly include toes as extra contact point in future
    angles = gon_data{i,angle_vars};
    fk_params{1,2} = angles(1);
    fk_params{2,2} = angles(2);
    fk_params{3,2} = angles(3);
    J = JacobianFK(fk_params);
    if T3(3,4) > 0.008  % heuristic, but a good one, (8cm?)
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
            X(1:3,8) = T3(1:3,4);  % setting to mocap, for checking
            % X(3,8) = 0  % set to 0 z
        end
    end

    % [X,P] = predict(inputs, dt, fk2, imu1_p, imu2_p, shank_gyro, X, P, A, Q);
    if contact
        [X,P] = predict(inputs, dt, fk2, X, P, A, Q);
    else
        [X,P] = predict_nocontact(inputs, ...
            dt, fk2, X, P, A(1:15,1:15), Q(1:15,1:15));
    end

    if sum(isnan(P(:))) > 0
        warning('Detected NaN in P')
        disp(i)
        return;
    end
    
    v_ft = T1(1:3,1:3)\(T2(1:3,4) - T1(1:3,4));
    v_fc = T1(1:3,1:3)\(T3(1:3,4) - T1(1:3,4));
    if contact
        b = [bp2; bd];
        measp2 = [v_ft; 1; 0; -1; 0; 0];
        measd = [v_fc; 1; 0; 0; 0; -1];
        meas = [measp2; measd];
        H = [Hp2; Hd];
        % We know model so this is fine for now
        H = H([1:3,9:11],:);
        N = blkdiag(fk_cov,fk_cov);  % only apply cov increase to contact point
        log{i-initial+1,4} = blkdiag(X,X)*meas-b;
        [X,P] = update(meas,X,P,H,b,N,J);
    else
        J = J(:,1:2);
        meas = [v_ft; 1; 0; -1; 0];
        H = Hp2(1:3,1:15);
        b = bp2(1:7);
        N = fk_cov(1:2,1:2);
        log{i-initial+1,4} = X*meas-b;
        [X,P] = update_nocontact(meas,X,P,H,b,N,J);
    end
    log{i-initial+1,1} = t;
    log{i-initial+1,2} = X;
    log{i-initial+1,3} = P;
    log{i-initial+1,5} = contact;
    log{i-initial+1,6} = T1to3(1:3,4);
end
