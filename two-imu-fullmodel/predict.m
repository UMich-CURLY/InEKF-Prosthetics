function [state,cov] = predict(inputs,dt,fk2,X,P,A,Q)
    g = [0; 0; -9.81];
    omega = skew3x3(inputs(1:3));
    a1 = inputs(4:6);
    a2 = inputs(7:9);
    % needs to be rotation from first frame to second frame, so 
    % fk2 = R_{WF}\R_{WT}, world->femur and world->tibia gives femur->tibia
    % assume we already rotate the acceleration via forward kinematics for
    % the second link
    % dX = [R*omega*dt, X(1:3,5), R*inputs(4:6)*dt+g, X(1:3,7), R*inputs(7:9)*dt, zeros(3,1); zeros(5,8)];
    % state = X*expm(dX);
    R = X(1:3,1:3);
    p1 = X(1:3,4);
    v1 = X(1:3,5);
    p2 = X(1:3,6);
    v2 = X(1:3,7);
    d = X(1:3,8);
    
    RdX = expm(omega*dt);
    v1dX = (R*a1 + g)*dt;
    % These lines don't appear to do much
    % v1dX = v1dX - omega*imu1_p*dt;  % May need to remove these lines if it does nothing, if only for clarity
    p1dX = v1*dt + 0.5*v1dX*dt;
    v2dX = (R*fk2*a2 + g)*dt;
    % The below likely needs to be shank gyroscope
    % v2dX = v2dX - skew3x3(shank_gyro)*imu2_p*dt;
    p2dX = v2*dt + 0.5*v2dX*dt;

    state = eye(size(X));
    state(1:3,1:3) = R*RdX;
    state(1:3,4) = p1 + p1dX;
    state(1:3,5) = v1 + v1dX;
    state(1:3,6) = p2 + p2dX;
    state(1:3,7) = v2 + v2dX;
    state(1:3,8) = d;

    phi = expm(A*dt);
    cov = phi*P*phi' + Adj(X)*(phi*Q*phi'*dt)*Adj(X)';
end