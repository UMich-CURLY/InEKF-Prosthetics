function [state,cov] = update(meas,X,P,H,b,N) % dt necessary?
    S = H*P*H' + N;
    L = P*H'/S;

    % Assumes measurement is formatted correctly, also assumes stacked
    % measurement beforehand
    error = blkdiag(X,X)*meas - b;
    innovation = lie_groupify(L*error);
    state = expm(innovation)*X;
    LH = L*H;
    Xk = blkdiag(X,X);
    Nk = Xk*N*Xk';
    cov = (eye(size(LH))-LH)*P*(eye(size(LH))-LH)' + L*Nk*L';
end