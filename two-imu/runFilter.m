run load_dataset.m;
run matrices.m;  % initialize matrices

initial=1657;
% Initialize first rotation based on IMU instead?
femur_init = cell2mat(fkTable{initial,'femur_r'};
X = blkdiag(femur_init,eye(4));  % adding four extra columns
% Initialize contact point? Still no help
tibia_init = cell2mat(fkTable{initial,'tibia_r'});
ankle_init = cell2mat(fkTable{initial,'talus_r'});
calcn_init = cell2mat(fkTable{initial,'calcn_r'});
imu1_p = X(1:3,1:3)'*(tibia_init(1:3,4)-X(1:3,4))/2;  % Transpose instead of \?
imu2_p = tibia_init(1:3,1:3)'*(ankle_init(1:3,4)-tibia_init(1:3,4));
fk_params = rough_fk_param_estimate(femur_init,tibia_init,ankle_init,calcn_init);
X(1:3,6) = tibia_init(1:3,4);
X(1:3,8) = calcn_init(1:3,4);
P0 = blkdiag(0.01*eye(3),0.00001*eye(3),0.00001*eye(3),0.01*eye(3),0.01*eye(3),0.0001*eye(3));  % 3 for rotation, 3x3 more for p1,v1,d
% Changing last one causes large shifts in accuracy - try for yourself
% -- e.g. smaller -> less drift over time. Further decreases past 0.0001
% don't affect x/y as much as z
% -- but why when smaller do the trajectories still get closer at the end?
% Decreasing p1 cov makes initial downward spike bigger -- why? but no
% major change under 0.0001. Trusting less (0.1) causes slightly more x
% drift, slightly? 0.01 seems to be a sweet spot
% P2 cov shows same trend as p1, but to a similar-if-slightly-lesser degree
% Velocity covariances: 
% v1 higher -> a somewhat flattening effect in the z direction? Not much
% difference when very small
% v2 higher -> same as v1
% Rotation: 
% Higher: much larger initial drift, but actually performs better in x and kind
% of in y as well over time
% Lower: Just more of the same
% Pairwise:
% Decreasing p1 & v1 just makes it look like p1 was decreased
% Is the initial orientation frame different than expected?
P = P0;
Q = blkdiag(0.0001*eye(3),0.0001*eye(3),0.1*eye(3),0.0001*eye(3),0.1*eye(3),0.0001*eye(3));  % 3 for rotation, next 3 for position, next 3 for velocity
% Covariance adjustment effects:

N = 0.001*eye(8);  % Do the last five matter? Only first three go into measurements currently
N_reduced = N(1:3,1:3);
log = {fkTable{1,'Header'},X,P};
contact = 0;
for i = (initial+1):3068  % 3068 is number of timesteps for which we have IMU
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
    % [X,P] = predict(inputs, dt, fk2, imu1_p, imu2_p, shank_gyro, X, P, A, Q);
    [X,P] = predict(inputs, dt, fk2, X, P, A, Q);
    if sum(isnan(P(:))) > 0
        warning('Detected NaN in P')
    end
    % Use mocap for contact events
    

    % actual measurement is the vector to the contact point found in
    % forward kinematics (in world frame or body frame?)
    % Since we are getting relative position of the foot, not using the
    % state in this calculation is suitable.
    % use T2 to determine contact
    % Could possibly include toes as extra contact point in future
    if T3(3,4) > 0.008  % heuristic, but a good one, (8cm?)
        % See if we are going into contact
        if contact
            contact = 0;
        end
        % only have this apply to calcaneus fk
        Nt = 1000*N;  % don't trust contact point if not in contact
        % Going back to "skipping" strategy?
        log{i-initial+1,1} = t;
        log{i-initial+1,2} = X;
        log{i-initial+1,3} = P;
    else
        if ~contact
            contact = 1;
            % What if we do the above at every timestep when not in contact?
            temp = X(1:4,1:4)*T1to3(1:4,4);
            X(1:3,8) = temp(1:3);
            % X(1:3,8) = T3(1:3,4);
        end
        % Need to be careful with this, if setting at each timestep then it
        % may implicitly violate zero-velocity constraint, or we may be
        % getting unrealistically accurate measurements.
        % Shouldn't be too much a problem though, since we're taking the
        % femur->calcaneus vector from FK, and (estimate)->calcaneus vector
        % within the correction step

        % Only have this apply to calcaneus fk
        Nt = N;
        % Do I want to zero-out the z? would that help?
    end
    % X(1:3,8) = T3(1:3,4);  % Use transformation from current estimated origin instead?
    v_ft = T1(1:3,1:3)\(T2(1:3,4) - T1(1:3,4)); % Just want to rotate, transpose instead of \?
    v_fc = T1(1:3,1:3)\(T3(1:3,4) - T1(1:3,4));
    % Try without rotation? Similar result
    % v_fc = T2(1:3,4) - T1(1:3,4);
    measp2 = [v_ft; 1; 0; -1; 0; 0];
    measd = [v_fc; 1; 0; 0; 0; -1];
    meas = [measp2; measd];
    H = [Hp2; Hd];
    b = [bp2; bd];
    Nt = blkdiag(100*N,Nt);  % only apply cov increase to contact point
    [X,P] = update(meas,X,P,H,b,Nt);
    log{i-initial+1,1} = t;
    log{i-initial+1,2} = X;
    log{i-initial+1,3} = P;
end
