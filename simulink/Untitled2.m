in.pullforce=[dat.t',dat.w];
in.leantorque=[dat.t',zeros(dat.N,1)];

out = modelSimlink(K,bike,dat);


in.pullforce=[dat.t',dat.w];
in.leantorque=[dat.t',zeros(dat.N,1)];
in.steer_rate=[dat.t',out.steer_rate];
in.steer_angle=[dat.t',out.steer_angle];
in.roll_angle=[dat.t',out.roll_angle];
in.roll_rate=[dat.t',out.roll_rate];
in.a =[dat.t',out.a];
options = simset('SrcWorkspace','current');


endtime=dat.t(end);
bike_m=bike;
open_system('state_estimator');
out= sim('state_estimator',[],options);
