function [state,cov] = predict(inputs,dt,X,P,A,Q)
    g = [0, 0, -9.81];
    omega = [0, -inputs(3), inputs(2);
            inputs(3), 0, -inputs(1);
            -inputs(2), inputs(1), 0];
    % assume we already rotate the acceleration via forward kinematics for
    % the second link
    dX = [R*omega*dt, X(1:3,5), R*inputs(4:6)*dt+g, zeros(3,1);
          zeros(3,6)];
    state = X*expm(dX);

    phi = expm(A*dt);
    cov = phi*P*phi' + Adj(X)*(phi*Q*phi'*dt)*Adj(X)';
end