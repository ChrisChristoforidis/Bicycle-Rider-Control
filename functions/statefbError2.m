function en = statefbError2(X,np,bike_m, dat)

K=X;
out = modelSim(K,bike_m,dat);
y_mod= [out.roll_angle out.steer_angle];
e = (y_mod- np.y);
en = ((sum(e.^2)) * 1 / np.N);
en=en(1);
end