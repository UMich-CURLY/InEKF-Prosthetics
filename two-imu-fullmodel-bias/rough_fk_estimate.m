function fk_params = rough_fk_estimate(T1,T2,T3,T4)
    R1 = T1(1:3,1:3);
    p1 = T1(1:3,4);
    R2 = T2(1:3,1:3);
    p2 = T2(1:3,4);
    R3 = T3(1:3,1:3);
    p3 = T3(1:3,4);
    R4 = T4(1:3,1:3);
    p4 = T4(1:3,4);

    l1 = norm(p2-p1);
    l2 = norm(p3-p2);
    l3 = norm(p4-p3);

    eul1 = rotm2eul(R1);
    eul2 = rotm2eul(R2);
    eul3 = rotm2eul(R3);
    eul4 = rotm2eul(R4);

    % z axis rotations
    theta1 = eul2(1) - eul1(1);
    theta2 = eul3(1) - eul2(1);
    theta3 = eul4(1) - eul3(1);

    fk_params = {l1, theta1;
                 l2, theta2;
                 l3, theta3};
end