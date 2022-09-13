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
end