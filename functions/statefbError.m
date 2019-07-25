function en = statefbError(X,np,bike_m, dat)


out = modelSimlinkInverse(X,bike_m,dat);
output=[out.roll_angle,out.steer_angle];

e = (output - np.y);
en2 = ((sum(e.^2)) * 1 / np.N);
en=en2(2);
end