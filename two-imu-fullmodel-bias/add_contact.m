function [state,cov] = add_contact(X,P,fkd,J,fk_cov)
    state = eye(size(X)+[1,1]);
    d = X(1:3,4)+X(1:3,1:3)*fkd;
    state(1:3,end) = d;

    % augmentation to capture p1 contribution
    point_augmentation = [zeros(3,3), eye(3), zeros(3,9)];
    F = [eye(15); 
        point_augmentation];
    G = [zeros(15,3);
        X(1:3,1:3)*J];

    cov = F*P*F' + G*fk_cov*G';
end