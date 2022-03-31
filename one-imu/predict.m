function [state,cov] = predict(inputs,dt,X,P,A,Q)
    g = [0; 0; -9.81];
    % assume we already rotate the acceleration via forward kinematics for
    % the second link
    R = X(1:3,1:3);
    p1 = X(1:3,4);
    v1 = X(1:3,5);
    d = X(1:3,6);
    % We tried this, didn't work
    %RdX = skew3x3(inputs(1:3))*dt;
    %vdX = R*inputs(4:6)+g;
    %dX = [RdX, v1*dt, vdX*dt, zeros(3,1);
    %      zeros(3,3),zeros(3,1),zeros(3,1),zeros(3,1)];
    %state = X*expm(dX);
    % Do discrete Euler integrations, see Hartley IV.E
    RdX = expm(skew3x3(inputs(1:3))*dt);
    vdX = R*inputs(4:6)*dt + g*dt;
    v2 = v1 + vdX;
    p2 = p1 + v1*dt + 0.5*vdX*dt;
    state = eye(6);
    state(1:3,1:3) = R*RdX;
    state(1:3,4) = p2;
    state(1:3,5) = v2;
    state(1:3,6) = d;
    'before'
    phi = expm(A*dt);
    'after'
    cov = phi*P*phi' + Adj(X)*(phi*Q*phi'*dt)*Adj(X)';
end