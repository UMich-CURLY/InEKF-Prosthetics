function [state,cov] = update(meas,X,P,H,b,N,Jac)
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
    N = N([1:3,9:11],[1:3,9:11]);
    R = X(1:3,1:3);
    % We know model so this is fine for now
    H = H([1:3,9:11],:);
    Rk = blkdiag(R,R);
    Jk = blkdiag(Jac,Jac);
    Nk = Rk*Jk*N*Jk'*Rk';
    S = H*P*H' + Nk;
    L = P*H'/S;

    % Assumes measurement is formatted correctly, also assumes stacked
    % measurement beforehand
    error = blkdiag(X,X)*meas - b;
    innovation = lie_groupify(L*error([1:3,9:11]));
    state = expm(innovation)*X;
    LH = L*H;
    cov = (eye(size(LH))-LH)*P*(eye(size(LH))-LH)' + L*Nk*L';
end