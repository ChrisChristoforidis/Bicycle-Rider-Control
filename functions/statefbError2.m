function en = statefbError2(X,np,rider, dat,n,k)

if (k==1)
  out = modelSimOLD(X,rider,dat);
else 
  out = modelSim3(X,rider,dat);
end
y_mod= [out.roll_rate out.roll_angle  out.steer_angle out.heading];

e = (y_mod- np.y);
en = ((sum(e.^2)) * 1 / length(np.y));
en= en(n);
end