function out = modelSim3(X,rider,dat,fb_status)
% Simulates the response of the measured input with the full closed loop model.
% Inputs 
%     K   :   The gains of the controller
%     bike: The EOM of the whipple model
%     dat :  The full measured dataset
% Outputs
%     out :  contains all 4 states of bicycle.(dphi,ddelta,phi,delta) +
%     Rider Torque

if (fb_status=="on")
  Gpd=rider.Gpd2;
elseif (fb_status=="off")
  Gpd=rider.Gpd4;
end
Predictor=rider.Predictor;
Del=rider.Del;
K=[X(1:4) 0 X(5) 0 0];
%Q=diag(X(1:7));
% 0
%  Q=eye(7);
%  K=lqr(rider.Gp2.A,rider.Gp2.B(:,1),Q,0.5);
%  K=[K 0];
%  K=zeros(1,8);
%   
%K=[K 0 0];
Cd=ss([],[],[],-K);
Cd.u={Predictor.OutputName{1}(1:end-3)};
Cd.InputName(8)={'y(8)'};
Cd.y='a';


input={'w'};
% Sum=sumblk('a=a1+a2');
% C_total=connect(Cd,C,Sum,'a','y');
Del2=rider.Del2;

Sum1=sumblk('ye=y-ym_d',7);

Sum2=sumblk(convertStringsToChars("y_f="+Predictor.OutputName{1}(1:end-3)+"+ye"),7);
array={'y_f(1)','y_f(2)','y_f(3)','y_f(4)',convertStringsToChars(convertCharsToStrings(Predictor.OutputName{1}(1:end-3))+"(5)"),'y_f(6)','y(7)','y(8)'};
Cd.u=array;


Gcl=connect(Gpd,Del,Sum2,Del2,Sum1,Predictor,Cd,input,{'y_un','y',Predictor.OutputName{1}(1:end-3),'ye','y_f'});


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