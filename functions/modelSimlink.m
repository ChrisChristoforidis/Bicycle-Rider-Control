function out = modelSimlink(X,bike_m,dat)
% Simulates the response of the measured input with the full closed loop
% model. (using simulink)
% Inputs 
%     K      :   The gains of the controller
%     bike_m :   The EOM of the whipple model
%     dat    :   The full measured dataset
% Outputs
%     out :  contains all 4 states of bicycle.(dphi,ddelta,phi,delta) +
%     Rider Torque


options = simset('SrcWorkspace','current');

omegac= 2 * pi * 2.17;
Gp=plantModel(bike_m,omegac);
I=eye(7);
Gp=ss(Gp.A,Gp.B(:,2:3),I(1:7,:),zeros(7,2));

Gpd=c2d(Gp,0.001);
K=[X(1:6) 0]
in.pullforce = [dat.t',dat.w];
in.leantorque = [dat.t',zeros(dat.N,1)];

if max(dat.w)<20
bike_m.B(:,3)=bike_m.B(:,2);
end

endtime=dat.t(end);
% timedelay=0.00;
try 
  output= sim('state_fb_model_v2',[],options);
  out.roll_angle=output.State(:,3);
  out.steer_angle=output.State(:,4);
  out.heading=output.State(:,5);
  out.steer_torque=output.State(:,6);
  out.roll_rate = output.State(:,1);


catch 
  out.roll_angle=ones(length(dat.N),1)*inf;
  out.steer_angle=ones(length(dat.N),1)*inf;
end

end