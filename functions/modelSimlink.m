function out = modelSimlink(K,bike_m,dat)
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


in.pullforce = [dat.t',dat.w];
in.leantorque = [dat.t',zeros(dat.N,1)];

if max(dat.w)<20
bike_m.B(:,3)=bike_m.B(:,2);
end

endtime=dat.t(end);
% timedelay=0.00;
try 
  out= sim('state_fb_model_v2',[],options);
  out.steer_torque=out.SteerTorque;

catch 
  out.roll_angle=ones(length(dat.N),1)*inf;
  out.steer_angle=ones(length(dat.N),1)*inf;
end

end