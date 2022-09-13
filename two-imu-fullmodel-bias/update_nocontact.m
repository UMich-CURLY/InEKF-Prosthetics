function [state,cov,bias] = update_nocontact(meas,X,zeta,P,H,b,N,Jac)
    R = X(1:3,1:3);
    % Nk = R*Jac*N*Jac'*R';
    Nk = R*blkdiag(N,N,N)*R'; % HACK, once again, to see about singularity of covariance
    S = H*P*H' + Nk;
    L = P*H'/S;

    % Assumes measurement is formatted correctly, also assumes single
    % measurement
    error = X*meas-b;
    error = error(1:3);
    innovation = L*error;
    innovation_state = innovation(1:15,:);
    innovation_bias = innovation(16:24,:);
    bias = zeta + innovation_bias;
    innovation_state = lie_groupify_nocontact(innovation_state);
    state = expm(innovation_state)*X;
    LH = L*H;
    % more stable form of covariance update
    cov = (eye(size(LH))-LH)*P*(eye(size(LH))-LH)' + L*Nk*L';
end