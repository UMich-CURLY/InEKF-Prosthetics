function [state,cov,bias] = update_nocontact_leftgps(meas,X,zeta,P,H,b,N,Jac)
    R = X(1:3,1:3);
    Rk = blkdiag(R,R);
    Jk = blkdiag(Jac,Jac);
    Nk = Rk\Jk*N*Jk'/(Rk');
    % Inverse X for left-invariant case
    % Try replacing this with an arbitrary covariance and see how it does

    S = H*P*H' + Nk;
    L = P*H'/S;

    L_bias = L(16:24,1:end);
    L_state = L(1:15,1:end);

    % Assumes measurement is formatted correctly
    error = blkdiag(X,X)\meas-b;
    error = error([1:3,8:10]);
    bias = zeta + L_bias*error;
    innovation = lie_groupify_nocontact(L_state*error);
    state = X*expm(innovation);
    LH = L*H;
    % more stable form of covariance update
    cov = (eye(size(LH))-LH)*P*(eye(size(LH))-LH)' + L*Nk*L';
end