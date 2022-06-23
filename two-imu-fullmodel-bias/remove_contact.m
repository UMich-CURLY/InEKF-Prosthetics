function [state,cov] = remove_contact(X,P)
    state = X(1:7,1:7);
    % cov = P(1:end-3,1:end-3);  % -3 because of 3D coords for contact point
    F = zeros(24,27);
    F(1:15,1:15) = eye(15);
    F(16:24,19:27) = eye(9);  % for keeping biases the same
    cov = F*P*F';
end