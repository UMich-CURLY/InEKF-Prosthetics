X = eye(8);
R = X(1:3,1:3);
p1 = X(1:3,4);
v1 = X(1:3,5);
p2 = X(1:3,6);
v2 = X(1:3,7);
d = X(1:3,8);

g = [0 0 -9.81]; % should this be negative? I always get this backwards

A = zeros(27,27);
A(4:6,7:9) = eye(3);
A(10:12,13:15) = eye(3);
A(7:9,1:3) = skew3x3(g);
A(13:15,1:3) = skew3x3(g);
% rest needs to be constructed online

bp2 = [0; 0; 0; 1; 0; -1; 0; 0];
bd = [0; 0; 0; 1; 0; 0; 0; -1];
bz_r = [0; 0; 0; 0; 0; 0; 0; -1];
bz_l = [0; 0; 0; 0; 0; 0; 0; 1];

Hp2 = zeros(8,27);
Hp2(1:3,4:6) = -eye(3);
Hp2(1:3,10:12) = eye(3);

Hd = zeros(8,27);
Hd(1:3,4:6) = -eye(3);
Hd(1:3,16:18) = eye(3);

Hz_r = zeros(8,27);
Hz_r(1:3,16:18) = eye(3);

% double-check the left-invariant forms
Hz_l = zeros(8,27);
Hz_l(1:3,16:18) = -eye(3);

rotation_variance = 0.01;
p1_variance = 0.01;
v1_variance = 0.001;
p2_variance = 0.01;
v2_variance = 0.001;
P0 = blkdiag(rotation_variance*eye(3),...
             p1_variance*eye(3),...
             v1_variance*eye(3),...
             p2_variance*eye(3),...
             v2_variance*eye(3));  % 3 for rotation, 3x3 more for p1,v1,d
% Changing last one causes large shifts in accuracy - try for yourself
% -- e.g. smaller -> less drift over time. Further decreases past 0.0001
% don't affect x/y as much as z
% -- but why when smaller do the trajectories still get closer at the end?
% Decreasing p1 cov makes initial downward spike bigger -- why? but no
% major change under 0.0001. Trusting less (0.1) causes slightly more x
% drift, slightly? 0.01 seems to be a sweet spot
% P2 cov shows same trend as p1, but to a similar-if-slightly-lesser degree
% Velocity covariances: 
% v1 higher -> a somewhat flattening effect in the z direction? Not much
% difference when very small
% v2 higher -> same as v1
% Rotation: 
% Higher: much larger initial drift, but actually performs better in x and kind
% of in y as well over time
% Lower: Just more of the same
% Pairwise:
% Decreasing p1 & v1 just makes it look like p1 was decreased
% Is the initial orientation frame different than expected?

gyro_variance = (0.009*pi/180)^2;  % assuming variance is noise density squared
accelerometer_variance_low = (99*(10^-6)*9.81)^2;  % 99ug/sqrt(Hz)
accelerometer_variance_mid = (650*(10^-6)*9.81)^2; % 650ug/sqrt(Hz)
accelerometer_variance_high = (15*(10^-3)*9.81)^2;  % 15mg/sqrt(Hz)
accelerometer_variance = accelerometer_variance_high;
Q0 = blkdiag(gyro_variance*eye(3),...
             accelerometer_variance^2*eye(3),...
             accelerometer_variance*eye(3),...
             accelerometer_variance^2*eye(3),...
             accelerometer_variance*eye(3),...
             accelerometer_variance^2*eye(3));
% The above actually looks incredibly smooth

% Q = blkdiag(0.0001*eye(3),0.001*eye(3),0.00001*eye(3),0.001*eye(3),0.00001*eye(3),0.0001*eye(3));  % 3 for rotation, next 3 for position, next 3 for velocity
% Q = blkdiag(0.0001*eye(3),0.01*eye(3),0.01*eye(3),0.001*eye(3),0.01*eye(3),0.01*eye(3)) <- appears smoothest for euler angles, perhaps not most precise though
% blkdiag(0.0001*eye(3),0.001*eye(3),0.00001*eye(3),0.001*eye(3),0.00001*eye(3),0.0001*eye(3)) <- second best or new best?
% Covariance adjustment effects:
% R: bigger -> bigger difference in euler angles vs. gt, more loop-de-loop.
% Decreasing past 0.001 doesn't give much
% p1: bigger -> subtle differences, not sure what they are
% v1: smaller -> major difference in how far off trajectory was. Bigger ->
% may produce better results? On the order of 1 or higher?
% p2: bigger -> flatter trajectory? Maybe. Slightly worse Euler angle
% deltas maybe
% v2: bigger -> flatter trajectory that doesn't go as far, decreasing
% doesn't gain much past 0.01
% d: bigger -> more trajectory drift, but no visual gains past 0.001 to
% 0.0001

Np2 = 0.1*eye(8);  % Do the last five matter? Only first three go into measurements currently
% bigger -> trajectory drifts downward faster, more spiky
% smaller -> trajectory also goes down, but is smoother
Nd = 0.1*eye(8);
% bigger -> trajectory goes straight down then stays in place for a time
% smaller -> 

goniometer_variance = (2*pi/180)^2;  % 2 deg over +/- 90 deg as Accuracy was all that was given on their website, and only then for the two-axis goniometers
fk_cov = goniometer_variance*eye(3);
% fk_cov = 0.1*eye(3);

gyro1b_variance = (2.5*pi/180)^2;
% bias stability: 2.5 deg/hr per @ 25 C
accel1b_variance = 0.01;
% ±0.008%/°C, ±0.01%/°C(HH, H3)
accel2b_variance = 0.01;
% same as above
bias_cov = blkdiag(gyro1b_variance*eye(3), ...
                   accel1b_variance*eye(3), ...
                   accel2b_variance*eye(3));
