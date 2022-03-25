function g = lie_groupify(vec)
    g = zeros(6,6);
    g(1:3,1:3) = skew3x3(vec(1:3));
    g(1:3,4) = vec(4:6);
    g(1:3,5) = vec(7:9);
    g(1:3,6) = vec(10:12);
end