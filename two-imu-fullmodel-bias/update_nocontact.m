function [state,cov,bias] = update_nocontact(meas,X,zeta,P,H,b,N,Jac)
    R = X(1:3,1:3);
    Nk = R*Jac*N*Jac'*R';
    S = H*P*H' + Nk;
    L = P*H'/S;

    L_bias = L(16:24,1:end);
    L_state = L(1:15,1:end);

    % Assumes measurement is formatted correctly, also assumes single
    % measurement
    error = X*meas-b;
    error = error(1:3);
    bias = zeta + L_bias*error;
    innovation = lie_groupify_nocontact(L_state*error);
    state = expm(innovation)*X;
    LH = L*H;
    % more stable form of covariance update
    cov = (eye(size(LH))-LH)*P*(eye(size(LH))-LH)' + L*Nk*L';
end