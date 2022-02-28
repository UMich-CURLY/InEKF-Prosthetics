% Assumes contact point is present, otherwise this doesn't work. Need
% separate versions with and without contact points.
function Ad = Adj(X)
    R = X(1:3,1:3);
    p1 = X(1:3,4);
    v1 = X(1:3,5);
    d = X(1:3,6);
    Ad = blkdiag(R,R,R,R);
    Ad(4:6,1:3) = skew3x3(p1)*R;
    Ad(7:9,1:3) = skew3x3(v1)*R;
    Ad(10:12,1:3) = skew3x3(d)*R;
end