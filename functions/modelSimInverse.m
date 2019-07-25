function out = modelSimInverse(K,bike,dat)
% Simulates the response of the measured input with the full closed loop model.
% Inputs 
%     K   :   The gains of the controller
%     bike: The EOM of the whipple model
%     dat :  The full measured dataset
% Outputs
%     out :  contains all 4 states of bicycle.(dphi,ddelta,phi,delta) +
%     Rider Torque


% Get combined plant model (bicycle+neuromuscular dyanmics)
% whipple.M0(2,2)=K(5);
% whipple.K0(2,2)=K(6);
% whipple.M0(1,2)=K(7);
% whipple.M0(2,1)=whipple.M0(1,2);
% whipple.K0(2,2)=K(8);
% whipple.K0(2,1)=whipple.K0(1,2);

% bike = delftbike(dat.v,whipple); 
% K=K(1:4);
% Process
omegac= 2 * pi * 2.17;
Gp=plantModel(bike,omegac);

%Gpd=ss(Gp.A,Gp.B,Gp.C,Gp.D,'OutputDelay',0.022);

if max(abs(dat.w)) < 20
  Gp.B(:,3)=[bike.B(:,2) ;0 ;0];
end
% Q=eye(6);
% R=0.01;
% [K,~,~]=lqr(Gp.A,Gp.B(:,1),Q,R);
K=[K(1:4) 0 0];
Gp2=ss(Gp.A,Gp.B(:,2:3),eye(6),zeros(6,2));
Gp2.u={'a','w'};
Gp2.y='y';
Cd=ss([],[],[],-K,'InputDelay',0.000);
Cd.u='y';
Cd.y='a';
C=ss([],[],[],K);
C.u='y';
C.y='a';
input={'w'};

%Gcl_delayed=connect(Gp2,Cd,input,'y');
Gcl=connect(Gp2,Cd,input,'y');
Gclx = pade(Gcl,3);
output=lsim(Gclx,dat.w,dat.t);
%Simulation of closed loop system
%output = lsim(Gp.A-Gp.B(:,2)*[K 0 0],Gp.B(:,3),eye(6),zeros(6,1),dat.w*1.,dat.t);
% Gpd=c2d(Gp,0.001);
% Gpd_delayed=delayStateSpace(Gpd,0.022,K);

% output1=lsim(Gpd,[zeros(length(dat.w),1) steer_torque dat.w],dat.t)
% plot(output1(:,4))
% ylim([-0.5 0.5])
% output = lsim(Gp_d.A-Gp_d.B(:,2)*Gp_d.K ,Gp_d.B(:,3),eye(length(Gp_d.A)),zeros(length(Gp_d.A),1),dat.w,dat.t);



% Output assignment
out.steer_torque = output(:,5);
out.steer_angle = output(:,4);
out.roll_angle = output(:,3);
out.roll_rate = output(:,1);
out.steer_rate = output(:,2);
out.Input= -K*output(:,1:length(K)).';

end