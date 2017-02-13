function T = drag(rpm, diam, elev, c)
    %T = c*lift(rpm, diam, elev, 1)*diam*0.75;
    w = rpm/60;
    R = diam/2;
    C = c*1.225*R/15;
    F = 6.6*C*R^3*w.^2;
    T = 0.84*R*F;
end