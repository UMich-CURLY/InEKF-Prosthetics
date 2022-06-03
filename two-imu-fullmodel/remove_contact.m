function [state,cov] = remove_contact(X,P)
    state = X(1:7,1:7);
    % cov = P(1:end-3,1:end-3);  % -3 because of 3D coords for contact point
    F = [eye(15), zeros(15,3)];
    cov = F*P*F';
end