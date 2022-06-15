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
Hp2(1:3,4:6) = -eye(3);
Hp2(1:3,10:12) = eye(3);

Hd = zeros(8,18);
Hd(1:3,4:6) = -eye(3);
Hd(1:3,16:18) = eye(3);

fk_cov = 0.01*eye(3);  % three angles, three covariances

P0 = blkdiag(0.001*eye(3),0.01*eye(3),0.001*eye(3),0.01*eye(3),0.001*eye(3));  % 3 for rotation, 3x3 more for p1,v1,d
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

Q = blkdiag(0.001*eye(3),0.0001*eye(3),10*eye(3),0.0001*eye(3),0.1*eye(3),0.01*eye(3));  % 3 for rotation, next 3 for position, next 3 for velocity
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
Nd = 0.001*eye(8);
% bigger -> trajectory goes straight down then stays in place for a time
% smaller -> 
fk_cov = 0.01*eye(3);

