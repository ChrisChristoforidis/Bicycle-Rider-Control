function en = statefbError(X,np,bike_m, dat,n,sim_method)

if (sim_method=="simulink_predict")
  out = modelSimlinkPredict(X,bike_m,dat);
elseif (sim_method=="simulink_position")
  out = modelSimlinkPredict(X,bike_m,dat);
else
  out= modelSim(X,bike_m,dat);
end
y_mod=[out.roll_rate out.roll_angle  out.steer_angle out.heading];

e = (y_mod - np.y);
e(isnan(e))=inf;
en2 = ((sum(e.^2)) * 1 / np.N);
en=en2(n);
end