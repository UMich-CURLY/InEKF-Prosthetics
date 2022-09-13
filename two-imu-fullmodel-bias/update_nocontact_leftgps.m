function [state,cov,bias] = update_nocontact_leftgps(meas,X,zeta,P,H,b,N,Jac)
    R = X(1:3,1:3);
    Rk = blkdiag(R,R);
    Jk = blkdiag(Jac,Jac);
    Nk = Rk\Jk*N*Jk'/(Rk');
    % Inverse X for left-invariant case
    % Try replacing this with an arbitrary covariance and see how it does

    % Make P left-invariant form, then turn back at the end
    AdjXinv = Adj_nocontact(X\eye(size(X)));
    AdjXinv = blkdiag(AdjXinv,eye(9));
    Pl = AdjXinv*P*AdjXinv';

    S = H*Pl*H' + Nk;
    L = Pl*H'/S;

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
    cov = (eye(size(LH))-LH)*Pl*(eye(size(LH))-LH)' + L*Nk*L';

    AdjX = Adj_nocontact(state);
    AdjX = blkdiag(AdjX,eye(9));
    cov = AdjX*cov*AdjX';
end