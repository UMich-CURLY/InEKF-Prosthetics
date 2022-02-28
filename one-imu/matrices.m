X = eye(6);
R = X(1:3,1:3);
p1 = X(1:3,4);
v1 = X(1:3,5);
d = X(1:3,6);

g = [0 0 -9.81]; % should this be negative? I always get this backwards

A = zeros(12,12);
A(4:6,7:9) = eye(3);
A(7:9,1:3) = skew3x3(g);

bd = [0; 0; 0; 1; 0; -1];

Hd = zeros(6,12);
Hd(1:3,4:6) = eye(3);
Hd(1:3,10:12) = -eye(3);


function v = skew3x3(vec)
    v = [0 -vec(3) vec(2);
        vec(3) 0 -vec(1);
        -vec(2) vec(1) 0];
end

