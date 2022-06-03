% Assumes contact point is not present, otherwise this doesn't work. Need
% separate versions with and without contact points.
function Ad = Adj_nocontact(X)
    R = X(1:3,1:3);
    p1 = X(1:3,4);
    v1 = X(1:3,5);
    p2 = X(1:3,6);
    v2 = X(1:3,7);
    Ad = blkdiag(R,R,R,R,R);
    Ad(4:6,1:3) = skew3x3(p1)*R;
    Ad(7:9,1:3) = skew3x3(v1)*R;
    Ad(10:12,1:3) = skew3x3(p2)*R;
    Ad(13:15,1:3) = skew3x3(v2)*R;
end