function [state,cov,bias] = update(meas,X,zeta,P,H,b,N,Jac)
    % Requires reduced form of H to make work - we should do that in this
    % function to maintain clarity. Or...
    % ^ hard-coded sizes is not the best from software engineering, but
    % this is more math. Besides, we have the model set. Flexibility is for
    % the addition/removal of contact points.
    % Need to make into block diagonal form to account for stacked
    % measurements
    % disp(size(J))
    % disp(size(X))
    % disp(size(N))
    R = X(1:3,1:3);
    Rk = blkdiag(R,R,R);  % does it work this way?  
    Jk = blkdiag(Jac,Jac,Jac);
    Nk = Rk*Jk*N*Jk'*Rk';
    % Nk = Nk([1:6,9],[1:6,9]);  % removing all but the z variable
    S = H*P*H' + Nk;
    L = P*H'/S;

    % Assumes measurement is formatted correctly, also assumes stacked
    % measurement beforehand
    error = blkdiag(X,X,X)*meas - b;
    error = error([1:3,9:11,17:19]);
    innovation = L*error;
    innovation_state = innovation(1:18,:);
    innovation_bias = innovation(19:27,:);
    bias = zeta + innovation_bias;
    innovation_state = lie_groupify(innovation_state);
    state = expm(innovation_state)*X;
    LH = L*H;
    cov = (eye(size(LH))-LH)*P*(eye(size(LH))-LH)' + L*Nk*L';
end