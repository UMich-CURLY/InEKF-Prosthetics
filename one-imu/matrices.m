g = [0; 0; -9.81]; % should this be negative? I always get this backwards

A = zeros(12,12);
A(4:6,7:9) = eye(3);
A(7:9,1:3) = skew3x3(g);

bd = [0; 0; 0; 1; 0; -1];

Hd = zeros(6,12);
Hd(1:3,4:6) = eye(3);
Hd(1:3,10:12) = -eye(3);

