function m = lie_se3(vector)
    % Assumes 6x1 vector
    w = vector(1:3);
    v = vector(4:6);
    m = zeros(4,4);
    m(1:3,1:3) = skew3x3(w);
    m(1:3,4) = v;
end