dt = 0.005;

A = zeros(18,18);
A(4:6,7:9) = eye(3);
A(10:12,13:15) = eye(3);
A(7:9,1:3) = skew3x3(g);
A(13:15,1:3) = skew3x3(g);

phi = expm(A*dt);

Hp2 = zeros(8,18);
Hp2(1:3,4:6) = -eye(3);
Hp2(1:3,10:12) = eye(3);
Hp2 = Hp2(1:3,:);

Hd = zeros(8,18);
Hd(1:3,4:6) = -eye(3);
Hd(1:3,16:18) = eye(3);
Hd = Hd(1:3,:);

Op2 = [Hp2; Hp2*phi; Hp2*phi*phi];

Od = [Hd; Hd*phi; Hd*phi*phi];