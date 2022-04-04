function [state,cov] = update(meas,X,P,H,b,N) % dt necessary?
    S = H*P*H' + N;
    L = (P*H')/S;

    % Assumes measurement is formatted correctly, needs to be skewified
    % innovation = L*(X*meas-b);
    % use reduced version
    error = X*meas - b;  % should just be 0 for our model
    innovation = L*error;  % for reduction: use error(1:3)
    state = expm(lie_groupify(innovation))*X;
    LH = L*H;
    % Do the below as opposed to (I-LH)*P for numerical accuracy
    % Nk = X*N*X', see slides from Mobile Robotics for this explanation, 
    % this seems like a Right InEKF thing. Left uses inv(X).
    Nk = X*N*X';
    cov = (eye(size(LH))-LH)*P*(eye(size(LH))-LH)' + L*Nk*L';
end