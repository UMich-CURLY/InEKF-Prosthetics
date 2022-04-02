function [state,cov] = update(meas,X,P,H,b,N) % dt necessary?
    S = H*P*H' + N;
    L = (P*H')/S;

    % Assumes measurement is formatted correctly, needs to be skewified
    % innovation = L*(X*meas-b);
    % use reduced version
    error = X*meas;  % should just be 0
    innovation = L*error(1:3);
    state = expm(lie_groupify(innovation))*X;
    LH = L*H;
    cov = (eye(size(LH))-LH)*P*(eye(size(LH))-LH) + L*N*L';
end