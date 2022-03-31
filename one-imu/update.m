function [state,cov] = update(meas,X,P,H,b,N) % dt necessary?
    S = H*P*H' + N;
    L = (P*H')/S;

    % Assumes measurement is formatted correctly, needs to be skewified
    error = L*(X*meas-b);
    state = expm(lie_groupify(error))*X;
    LH = L*H;
    cov = (eye(size(LH))-LH)*P*(eye(size(LH))-LH) + L*N*L';
end