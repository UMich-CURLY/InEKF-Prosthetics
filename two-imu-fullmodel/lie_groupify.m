function g = lie_groupify(vec)
    g = zeros(8,8);
    g(1:3,1:3) = skew3x3(vec(1:3));
    g(1:3,4) = vec(4:6);
    g(1:3,5) = vec(7:9);
    g(1:3,6) = vec(10:12);
    g(1:3,7) = vec(13:15);
    g(1:3,8) = vec(16:18);
end