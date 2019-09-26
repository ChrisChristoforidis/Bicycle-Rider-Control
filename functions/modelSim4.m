function out = modelSim4(X,rider,dat,fb_status)
% Simulates the response of the measured input with the full closed loop model.
% Inputs 
%     K   :   The gains of the controller
%     bike: The EOM of the whipple model
%     dat :  The full measured dataset
% Outputs
%     out :  contains all 4 states of bicycle.(dphi,ddelta,phi,delta) +
%     Rider Torque

if (fb_status=="on")
  Gpd=rider.Gpd3;
elseif (fb_status=="off")
  Gpd=rider.Gpd4;
end
if (length(X)==5)
  X(6)=0;
end

Del=rider.Del;

K=[X(1:4) X(5) 0 0 0];
%Q=diag(X(1:7));
% 0
%  Q=eye(7);
%  K=lqr(rider.Gp2.A,rider.Gp2.B(:,1),Q,0.5);
%  K=[K 0];
%  K=zeros(1,8);
%  K=[K 0 0];
Cd=ss([],[],[],-K);
Cd.u='y';
Cd.y='a';

C=ss([],[],[],[0 -X(6) 0 -X(7) 0 0 0 0]);
% C=ss([],[],[],[0 -0 0 -0 0 0 0 0]);

C.u={'y_un'};
C.y='Tint';


Gcl=connect(Gpd,Del,Cd,C,'w',{'y_un','y','Tint'});


output=lsim(Gcl,dat.w,dat.t);


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