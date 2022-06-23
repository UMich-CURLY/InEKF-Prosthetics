function [state,cov] = add_contact(X,P,fkd,J,fk_cov)
    state = eye(size(X)+[1,1]);
    d = X(1:3,4)+X(1:3,1:3)*fkd;
    state(1:3,end) = d;

    % augmentation to capture p1 contribution
    F = zeros(27,24);
    F(1:15,1:15) = eye(15);  % keeping what we have going
    F(16:18,4:6) = eye(3);  % p1 contributes to d error
    F(19:27,16:24) = eye(9);  % bias portion
    G = [zeros(15,3);
        X(1:3,1:3)*J;
        zeros(9,3)];  % extra row included for biases

    cov = F*P*F' + G*fk_cov*G';
end