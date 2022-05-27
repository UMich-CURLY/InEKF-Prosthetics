function [state,cov] = update_nocontact(meas,X,P,H,b,N,Jac) % dt necessary?
    R = X(1:3,1:3);
    Nhat = R*Jac*N*Jac'*R';
    S = H*P*H' + Nhat;
    L = P*H'/S;

    % Assumes measurement is formatted correctly, also assumes single
    % measurement
    error = X*meas-b;
    innovation = lie_groupify_nocontact(L*error);
    state = expm(innovation)*X;
    LH = L*H;
    Xk = X;
    Nk = Xk*Nhat*Xk';
    % more stable form of covariance update
    cov = (eye(size(LH))-LH)*P*(eye(size(LH))-LH)' + L*Nk*L';
end