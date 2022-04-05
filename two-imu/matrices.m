X = eye(8);
R = X(1:3,1:3);
p1 = X(1:3,4);
v1 = X(1:3,5);
p2 = X(1:3,6);
v2 = X(1:3,7);
d = X(1:3,8);

g = [0 0 -9.81]; % should this be negative? I always get this backwards

A = zeros(18,18);
A(4:6,7:9) = eye(3);
A(10:12,13:15) = eye(3);
A(7:9,1:3) = skew3x3(g);
A(13:15,1:3) = skew3x3(g);

bp2 = [0; 0; 0; 1; 0; -1; 0; 0];
bd = [0; 0; 0; 1; 0; 0; 0; -1];

Hp2 = zeros(8,18);
Hp2(1:3,4:6) = eye(3);
Hp2(1:3,10:12) = -eye(3);

Hd = zeros(8,18);
Hd(1:3,4:6) = eye(3);
Hd(1:3,16:18) = -eye(3);

