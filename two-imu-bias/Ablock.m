function A_block = Ablock(X,A)
% Helper function for generating the block form of A for the dynamics of
% two-imu InEKF with bias corrections
A_block = zeros(27,27);  % 18 + 9
A_block(1:18,1:18) = A;  % from matrices.m
R = X(1:3,1:3);
p1 = X(1:3,4);
v1 = X(1:3,5);
p2 = X(1:3,6);
v2 = X(1:3,7);
d = X(1:3,8);
A_gyro = [-R;
          -skew3x3(p1)*R;
          -skew3x3(v1)*R;
          -skew3x3(p2)*R;
          -skew3x3(v2)*R;
          -skew3x3(d)*R
          ];
A_a1 = [zeros(3,3);
        zeros(3,3);
        -R;
        zeros(3,3);
        zeros(3,3);
        zeros(3,3)
        ];
A_a2 = [zeros(3,3);
        zeros(3,3);
        zeros(3,3);
        zeros(3,3);
        -R;
        zeros(3,3)
        ];
A_block(1:18,19:21) = A_gyro;
A_block(1:18,22:24) = A_a1;
A_block(1:18,25:27) = A_a2;
end