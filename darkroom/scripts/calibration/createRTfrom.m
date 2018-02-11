function RT = createRTfrom(x)
% createRtfrom 
% constructs a 4x4 RT matrix from 6 coefficients [r p y x y z]
    alpha_squared = (x(1)^2 + x(2)^2 + x(3)^2)^2;
    q = [(1-alpha_squared)/(alpha_squared+1), 2*x(1)/(alpha_squared+1), 2*x(2)/(alpha_squared+1), 2*x(3)/(alpha_squared+1)];
    q = q/norm(q);
    qw = q(1); qx = q(2); qy = q(3); qz = q(4);
    tx = 2*qx;
    ty = 2*qy;
    tz = 2*qz;
    twx = tx*qw;
    twy = ty*qw;
    twz = tz*qw;
    txx = tx*qx;
    txy = ty*qx;
    txz = tz*qx;
    tyy = ty*qy;
    tyz = tz*qy;
    tzz = tz*qz;

    RT = [1-(tyy+tzz),	txy-twz, txz+twy, x(4);
          txy+twz,	1-(txx+tzz), tyz-twx, x(5);
          txz-twy,	tyz+twx , 1-(txx+tyy), x(6);
          0, 0, 0, 1];
end

