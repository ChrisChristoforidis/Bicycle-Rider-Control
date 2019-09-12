function out = modelSimlinkPredict(K,bike_m,dat)
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

% whipple.M0(2,2)=K(7);
% whipple.K0(2,2)=K(8);

in.pullforce = [dat.t',dat.w];
in.leantorque = [dat.t',zeros(dat.N,1)];

if max(dat.w)<20
bike_m.B(:,3)=bike_m.B(:,2);
end

% Process
omegac= 2 * pi * 2.17;
Gp=plantModel(bike_m,omegac);
I=eye(7);
%  Q=diag([1/max(dat.Rates(:,1))^2 , 1/max(dat.Rates(:,2))^2 ,1/max(dat.y(:,1))^2 ,1/max(dat.y(:,2))^2 0 0 0]);
%  R=1/max(dat.Tdelta)^2;
% Q=diag(K(1:6));
% R=K(7);
Gp=ss(Gp.A,Gp.B(:,2:3),I(1:7,:),zeros(7,2));
% [K,~,~]=lqr(Gp.A,Gp.B(:,1),Q,R);
Gpd=c2d(Gp,0.001);
endtime=dat.t(end);
K=[K(1:6)  0 ];
% out=lsim(Gp.A-Gp.B(:,1)*K,Gp.B(:,2),Gp.C,Gp.D(:,1),dat.w,dat.t)
try 
  tic
  output= sim('tapped_delay_line',[],options);
  toc
  out.roll_angle=output.State(:,3);
  out.steer_angle=output.State(:,4);
  out.heading=output.State(:,5);
  out.steer_torque=output.State(:,6);
  out.roll_rate = output.State(:,1);
  %out.steer_torque=out.SteerTorque;

catch 
  out.roll_angle=ones(length(dat.N),1)*inf;
  out.steer_angle=ones(length(dat.N),1)*inf;
end
end