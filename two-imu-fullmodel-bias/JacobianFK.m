function J = JacobianFK(fk_params)
    % Assume input comes in exactly as desired
    l1 = fk_params{1,1};
    theta1 = fk_params{1,2};
    l2 = fk_params{2,1};
    theta2 = fk_params{2,2};
    l3 = fk_params{3,1};
    theta3 = fk_params{3,2};

    % Using planar three-link manipulator model. Axes are FK at rest axes
    dxd3 = l3*cos(theta1+theta2+theta3);
    dxd2 = l2*cos(theta1+theta2) + dxd3;
    dxd1 = l1*cos(theta1) + dxd2;

    dyd3 = l3*sin(theta1+theta2+theta3);
    dyd2 = l2*sin(theta1+theta2) + dyd3;
    dyd1 = l1*sin(theta1) + dyd2;

    dzd3 = 0;
    dzd2 = 0 + dzd3;
    dzd1 = 0 + dzd2;
    % TODO: figure out how to use "Station Jacobian" (likely not System or
    % Frame Jacobian) from Osim: 
    % https://simbody.github.io/simbody-3.6-doxygen/api/classSimTK_1_1SimbodyMatterSubsystem.html
    J = [dxd1, dxd2, dxd3;
         dyd1, dyd2, dyd3;
         dzd1, dzd2, dzd3];
    J = eye(3);  % HACK, fix once a better solution is found

    % Instead, make unit twists to calculate Jacobian
    % How to find the unit vector in the direction of the Y axis of the base? Assume second column of R for now
    % Need to incorporate X into this calculation if we do this... is there any problem in doing this? Could instead have omega be fixed from the start, but what about q?
    % Technically hip is a ball joint, but we are assuming revolute joint for the sake of simplicity
    omega = X(1:3,2);
    q1 = X(1:3,4);
    q2 = X(1:3,6);
    if size(X,1) > 7
        q3 = x(1:3,8);
    end
    twist1 = [omega; -skew3x3(omega)*q1];
    twist2 = [omega; -skew3x3(omega)*q2];
    twist3 = [omega; -skew3x3(omega)*q3];

    exp1 = expm(lie_se3(twist1)*theta1);
    exp2 = expm(lie_se3(twist2)*theta2);
    exp3 = expm(lie_se3(twist3)*theta3);

    inv1 = expm(-lie_se3(twist1)*theta1);    
    inv2 = expm(-lie_se3(twist2)*theta2);    
    inv3 = expm(-lie_se3(twist3)*theta3);

    % Are inverses just negating the exponents? Yes, but be careful of order. May be easier to multiply exps, then invert all at once

    
end