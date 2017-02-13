function F = lift(rpm, diam, elev, c)
    w = rpm/60;
    airspeed = w * elev;
    area = (diam/2)^2 * pi;
    volume = airspeed * area;
    mass = volume * 1.225;
    F = mass .* airspeed * c;
end