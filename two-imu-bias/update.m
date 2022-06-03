function [state,cov,bias] = update(meas,zeta,X,P,H,b,N) % dt necessary?
    % rotation + jacobians? Would be much easier with reduced form N/H/etc.
    S = H*P*H' + N;
    L = P*H'/S;

    L_bias = L(19:27,1:end);  % 1:end accommodates stacked measurements
    L = L(1:18,1:end);

    % Assumes measurement is formatted correctly, also assumes stacked
    % measurement beforehand
    % The below is PI(XY) in rhartley-2018b paper
    error = blkdiag(X,X)*meas - b;
    bias = zeta + L_bias*error;

    innovation = lie_groupify(L*error);
    state = expm(innovation)*X;
    LH = L*H;
    Xk = blkdiag(X,X);
    Nk = Xk*N*Xk';
    % This sizes down P to 18 by 18 from 27 by 27
    cov = (eye(size(LH))-LH)*P*(eye(size(LH))-LH)' + L*Nk*L';
end