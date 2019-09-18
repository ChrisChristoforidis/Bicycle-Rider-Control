function en = statefbError3(X,np,rider, dat,n,k)

if (k==1)
  out = modelSim3(X,rider,dat,"on");
elseif (k==2)
  out = modelSim3(X,rider,dat,"off");
elseif (k==3)
  out = modelSimOLD(X,rider,dat);
elseif (k==4)
  out = modelSim4(X,rider,dat,"on");
elseif (k==5)
  out = modelSim4(X,rider,dat,"off");  
end
y_mod= [ out.roll_angle  out.steer_angle ];
r = ((sum(out.steer_torque.^2)) * 1 / length(np.y));
e = (y_mod- np.y);
en = ((sum(e.^2)) * 1 / length(np.y));
en= en(n)+0.0008*r;
end