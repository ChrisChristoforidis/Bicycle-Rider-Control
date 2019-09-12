function out = modelSimOLD(X,rider,dat)
% Simulates the response of the measured input with the full closed loop model.
% Inputs 
%     K   :   The gains of the controller
%     bike: The EOM of the whipple model
%     dat :  The full measured dataset
% Outputs
%     out :  contains all 4 states of bicycle.(dphi,ddelta,phi,delta) +
%     Rider Torque


% Get combined plant model (bicycle+neuromuscular dyanmics)
K=[X(1:6) 0];
% Process
Gpd=rider.Gpd;
% Q=eye(6);
% R=0.01;
% [K,~,~]=lqr(Gp.A,Gp.B(:,1),Q,R);
Cd=ss([],[],[],-K,1/dat.Fs,'InputDelay',0.000);
Cd.u='y_un';
Cd.y='a';
input={'w'};
%Gcl_delayed=connect(Gp2,Cd,input,'y');
Gcl=connect(Gpd,Cd,input,'y_un');
% Gclx = pade(Gcl,3);
output=lsim(Gcl,dat.w,dat.t);
%Simulation of closed loop system
%output = lsim(Gp.A-Gp.B(:,2)*[K 0 0],Gp.B(:,3),eye(6),zeros(6,1),dat.w*1.,dat.t);
% Gpd=c2d(Gp,0.001);
% Gpd_delayed=delayStateSpace(Gpd,0.022,K);

% output1=lsim(Gpd,[zeros(length(dat.w),1) steer_torque dat.w],dat.t)
% plot(output1(:,4))
% ylim([-0.5 0.5])
% output = lsim(Gp_d.A-Gp_d.B(:,2)*Gp_d.K ,Gp_d.B(:,3),eye(length(Gp_d.A)),zeros(length(Gp_d.A),1),dat.w,dat.t);



% Output assignment
if ( size(Gpd.A,1)==7)
  out.heading=output(:,5);
  out.steer_torque = output(:,6);
else
out.steer_torque = output(:,5);
end
out.steer_angle = output(:,4);
out.roll_angle = output(:,3);
out.roll_rate = output(:,1);
out.steer_rate = output(:,2);
out.Input= -K*output(:,1:length(K)).';

end