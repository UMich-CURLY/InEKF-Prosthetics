function [state,cov] = update(meas,X,P,H,b,N) % dt necessary?
    S = H*P*H' + N;
    L = P*H'/S;

    % Assumes measurement is formatted correctly, also assumes stacked
    % measurement beforehand
    state = expm(L*(blkdiag(X,X)*meas-b))*X;
    LH = L*H;
    cov = (eye(size(LH))-LH)*P*(eye(size(LH))-LH)' + L*N*L';
end