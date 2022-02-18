function [state,cov] = predict(inputs,dt,X,P,A,Q)
    g = [0, 0, -9.81];
    omega = [0, -inputs(3), inputs(2);
            inputs(3), 0, -inputs(1);
            -inputs(2), inputs(1), 0];
    % assume we already rotate the acceleration via forward kinematics for
    % the second link
    dX = [R*omega, X(1:3,5), R*inputs(4:6)+g, X(1:3,7), R*inputs(7:9), zeros(3,1);
          zeros(5,8)];
    state = X+dX*dt;

    phi = expm(A*dt);
    cov = phi*P*phi' + Adj(X)*(phi*Q*phi'*dt)*Adj(X)';
end