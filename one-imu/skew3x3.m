function v = skew3x3(vec)
    v = [0 -vec(3) vec(2);
        vec(3) 0 -vec(1);
        -vec(2) vec(1) 0];
end